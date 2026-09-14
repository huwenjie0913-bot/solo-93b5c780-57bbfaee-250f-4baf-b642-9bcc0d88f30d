"""审查引擎：逐行重建运动轨迹并产出诊断。

坐标约定：内部一律 mm。工件坐标（work）经 工件零点偏置 + G52/G92 附加偏置 +
刀具长度（z）换算为机床物理坐标（machine）后与包络比较。
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .collision import (aabb_gap, densify_polyline, first_crossing,
                        last_crossing, lerp3, obstacle_aabb, obstacle_usable,
                        swept_aabb, swept_clearance)
from .config import WORK_OFFSET_KEYS, normalize_config, tool_length, cutter_radius
from .geometry import PLANE_AXES, PLANE_CENTER_WORDS, resolve_arc
from .parser import parse_program
from .types import Diagnostic, Move

_AXES = ("x", "y", "z")
_AXIS_WORDS = {"x": "X", "y": "Y", "z": "Z"}
_INCH = 25.4

# 刀具组件（自刀尖向上堆叠）：(组件名, 直径字段, 长度字段, 中文名)
_TOOL_COMPONENTS = (
    ("flute", "flute_diameter", "flute_length", "刃部"),
    ("shank", "shank_diameter", "shank_length", "刀杆"),
    ("holder", "holder_diameter", "holder_length", "刀柄"),
)
_COMPONENT_CN = {name: cn for name, _d, _l, cn in _TOOL_COMPONENTS}
_KIND_CN = {"rapid": "快移", "linear": "直线", "arc": "圆弧"}


def _fmt(v: Optional[float]) -> str:
    if v is None:
        return "—"
    return f"{v:.4g}"


def _zero3() -> Dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": 0.0}


@dataclass
class _Bounds:
    min: Dict[str, float] = field(default_factory=lambda: {a: math.inf for a in _AXES})
    max: Dict[str, float] = field(default_factory=lambda: {a: -math.inf for a in _AXES})

    def add(self, p: Dict[str, float]) -> None:
        for a in _AXES:
            v = p.get(a, 0.0)
            if v < self.min[a]:
                self.min[a] = v
            if v > self.max[a]:
                self.max[a] = v

    def to_dict(self) -> Dict[str, Any]:
        return {
            "min": {a: (None if self.min[a] == math.inf else round(self.min[a], 4))
                    for a in _AXES},
            "max": {a: (None if self.max[a] == -math.inf else round(self.max[a], 4))
                    for a in _AXES},
        }


class ReviewEngine:
    """给定一份配置，对一段 G-code 程序做静态审查。"""

    def __init__(self, config: Dict[str, Any]):
        self.cfg = normalize_config(config)
        self.diags: List[Diagnostic] = []
        self.moves: List[Move] = []
        self.machine_bounds = _Bounds()
        self.work_bounds = _Bounds()
        self.assumptions: List[str] = []

        # ---- 模态状态（默认值按 RS-274/Fanuc 习惯）----
        self.units: Optional[str] = None          # 'mm' / 'inch'
        self.distance_mode = "G90"
        self.arc_distance_mode = "G91.1"
        self.plane = "G17"
        self.feed_mode = "G94"
        self.motion: Optional[str] = None
        self.cutter_mode = "G40"
        self.tool_length_mode = "G49"
        self.work_offset = "G54"
        self.return_mode = "G98"
        self.canned: Optional[str] = None
        self.canned_params: Dict[str, Any] = {}
        self.spindle: Optional[str] = None        # M03/M04/M05
        self.spindle_rpm: Optional[float] = None
        self.feed_native: Optional[float] = None  # 当前单位下的 F
        self.tool_no: Optional[str] = None
        self.h_code: Optional[str] = None
        self.d_code: Optional[str] = None
        self.base_offset = _zero3()
        self.g52 = _zero3()
        self.g92_shift = _zero3()
        # 工件坐标当前点（mm）
        self.work_pos = _zero3()
        self.known_pos = False
        self.first_move_done = False

        # 计时（s）
        self.t_rapid = 0.0
        self.t_cut = 0.0
        self.t_dwell = 0.0
        self.t_toolchange = 0.0

        # 去重/一次性提示
        self._spindle_warned = False
        self._g93_noted = False
        self._g921_noted = False
        self._start_noted = False
        self._unit_noted = False
        self._h_warned: Optional[str] = None
        self._d_warned: Optional[str] = None
        self._no_tool_m6_warned = False
        # 起始点相关（_init_position 会覆盖）
        self._default_start_used = True
        self._start_margin = 0.0
        self._start_machine = _zero3()
        # 本行待生效的 H/D（补偿切换时区分旧值/新值）
        self._pending_h: Optional[str] = None
        self._pending_d: Optional[str] = None

    # ------------------------------------------------------------------ #
    # 坐标换算
    # ------------------------------------------------------------------ #
    def _offset_total(self) -> Dict[str, float]:
        return {a: self.base_offset[a] + self.g52[a] + self.g92_shift[a]
                for a in _AXES}

    def _active_length(self) -> float:
        if self.tool_length_mode == "G43":
            return tool_length(self.cfg, self.h_code, self.tool_no)
        if self.tool_length_mode == "G44":
            return -tool_length(self.cfg, self.h_code, self.tool_no)
        return 0.0

    def _to_machine(self, work: Dict[str, float]) -> Dict[str, float]:
        p = {a: work[a] + self._offset_total()[a] for a in _AXES}
        p["z"] += self._active_length()
        return p

    def _to_work(self, machine: Dict[str, float]) -> Dict[str, float]:
        p = {a: machine[a] - self._offset_total()[a] for a in _AXES}
        p["z"] -= self._active_length()
        return p

    # ------------------------------------------------------------------ #
    # 主入口
    # ------------------------------------------------------------------ #
    def review(self, text: str, filename: Optional[str] = None) -> Dict[str, Any]:
        blocks, parse_diags = parse_program(text)
        self.diags.extend(parse_diags)
        self._init_position()

        for block in blocks:
            self._process_block(block)

        self._end_of_program_checks(last_line=blocks[-1].line_no if blocks else None)

        return self._build_report(text, filename, blocks)

    def _init_position(self) -> None:
        env = self.cfg["envelope"]
        sp = self.cfg.get("start_position") or {}
        # 未显式给定起始点时，选一个对“已配置的最大刀具实体”也安全的包络内角点：
        # XY 从 min 向内侧让出最大刀具半径，Z 取最高点。这样首段运动（含刀具包络检查）
        # 不会因为假设的初始点本身贴边/出界而对安全程序误报越界。
        max_radius = 0.0
        for tool in self.cfg.get("tools", {}).values():
            max_radius = max(max_radius, float(tool.get("diameter", 0.0)) / 2.0)
        for dval in self.cfg.get("d_offsets", {}).values():
            max_radius = max(max_radius, float(dval))
        margin = max_radius + 1e-6
        machine = {
            "x": float(sp["x"]) if sp.get("x") is not None
            else float(env["xmin"]) + margin,
            "y": float(sp["y"]) if sp.get("y") is not None
            else float(env["ymin"]) + margin,
            "z": float(sp["z"]) if sp.get("z") is not None
            else float(env["zmax"]),
        }
        self._default_start_used = not bool(sp)
        self._start_margin = margin
        self.base_offset = dict(self.cfg["work_offsets"]["G54"])
        self.work_pos = self._to_work(machine)
        self.known_pos = True
        self.machine_bounds.add(machine)
        self.work_bounds.add(self.work_pos)
        self._start_machine = machine

    # ------------------------------------------------------------------ #
    # 行处理
    # ------------------------------------------------------------------ #
    def _process_block(self, block) -> None:
        b = block
        words = b.words

        # 0) F / S / T / H / D 参数字必须先于 G 代码处理：
        #    同段 "G00 G43 H1 Z100" 中，G43 分支重映射坐标时就要用到本行的 H，
        #    否则会按旧 H（多为 0）重映射一次，段末再换用新 H，凭空产生一个刀长跳变。
        if "F" in words:
            self.feed_native = words["F"].value
        if "S" in words:
            self.spindle_rpm = float(words["S"].value)
        if "T" in words:
            new_tool = str(int(words["T"].value)) if words["T"].value == int(
                words["T"].value) else str(words["T"].value)
            if self.tool_no is not None and new_tool != self.tool_no and "M06" not in b.m_codes:
                self._diag("TOOL_WITHOUT_CHANGE", "info", b.line_no,
                           f"第 {b.line_no} 行 T{new_tool} 预选刀具但本行无 M06，"
                           f"当前刀具仍为 T{self.tool_no}",
                           "T 仅刀库预选；刀长/直径要等 M06 后才生效")
            self.tool_no = new_tool
        # H/D 字是本行待生效的寄存器选择，先暂存；补偿切换时要先用“旧 H”
        # 反算物理位置，切到新补偿后再提交新 H，避免切换瞬间混用语义。
        self._pending_h = self._code_word(words, "H")
        self._pending_d = self._code_word(words, "D")

        # 1) 模态 G 代码分组处理（顺序：单位/平面/距离等先于运动）
        motion_requested: Optional[str] = None
        g53_active = False
        length_comp_pending: Optional[str] = None
        for g in b.g_codes:
            if g in ("G20", "G21"):
                new_units = "inch" if g == "G20" else "mm"
                if self.units is not None and new_units != self.units and self.first_move_done:
                    self._diag("UNITS_SWITCH_MIDPROGRAM", "warning", b.line_no,
                               f"第 {b.line_no} 行程序中途切换为 {g}（{new_units}），"
                               "此前坐标/进给已按旧单位解释",
                               f"G20=英寸/G21=毫米；切换前单位={self.units}")
                self.units = new_units
            elif g in ("G90", "G91"):
                self.distance_mode = g
            elif g in ("G90.1", "G91.1"):
                self.arc_distance_mode = g
            elif g in ("G17", "G18", "G19"):
                self.plane = g
            elif g in ("G93", "G94", "G95"):
                self.feed_mode = g
            elif g in ("G00", "G01", "G02", "G03"):
                motion_requested = g
                self.motion = g
                self.canned = None  # 01 组运动取消固定循环
            elif g in ("G40", "G41", "G42"):
                self.cutter_mode = g
                if g == "G40":
                    self.d_code = None
                elif self._pending_d is not None:
                    self.d_code = self._pending_d
            elif g in ("G43", "G44", "G49"):
                # 补偿代码的处理延迟到 G 循环后（见 length_comp_pending）
                length_comp_pending = g
            elif g in WORK_OFFSET_KEYS:
                self._switch_work_offset(g, b.line_no)
            elif g in ("G98", "G99"):
                self.return_mode = g
            elif g == "G80":
                if self.canned:
                    self._diag("CANNED_CYCLE_CANCELED", "info", b.line_no,
                               f"第 {b.line_no} 行 G80 取消固定循环 {self.canned}",
                               "G80 清除固定循环模态，后续段不再继承钻孔动作")
                self.canned = None
                self.canned_params = {}
            elif g in ("G73", "G74", "G76") or g in {
                    f"G{n}" for n in range(81, 90)}:
                self.canned = g
                self.motion = None
            elif g in ("G61", "G61.1", "G64"):
                pass  # 路径控制模式，不影响几何
            elif g == "G04":
                self._handle_dwell(b)
            elif g in ("G28", "G30"):
                self._handle_g28(g, b)
            elif g == "G53":
                g53_active = True
            elif g == "G52":
                self._handle_g52(b)
            elif g in ("G92", "G92.1"):
                self._handle_g92(g, b)
            elif g == "G09":
                pass  # 段内精确停止，不影响几何

        # 2) 长度补偿切换（G43/G44/G49）：
        #    当前点始终按“当前生效补偿”的工件坐标表示，故切换时物理位置不动、
        #    先按新补偿重映射当前点；本行的 Z 目标若存在，则按 Fanuc 语义视为
        #    “新补偿生效后的工件坐标”，在运动执行前覆盖重映射后的当前 z。
        if length_comp_pending is not None:
            self._apply_length_comp(length_comp_pending, b,
                                    has_z="Z" in words)
        elif self._pending_h is not None and self.tool_length_mode in ("G43", "G44"):
            # 补偿已激活、本行仅换 H 号：物理位置不动，按新刀重重映射 work z
            self._apply_length_comp(self.tool_length_mode, b, has_z="Z" in words)

        if self._pending_d is not None and self.cutter_mode in ("G41", "G42"):
            # G41/G42 已激活时本行仅换 D 号（无位置重映射，刀径不改变刀位点）
            self.d_code = self._pending_d

        # 补偿号校验（F/S/T/H/D 已登记）
        if self.tool_length_mode in ("G43", "G44"):
            self._check_h_defined(b.line_no)
        if self.cutter_mode in ("G41", "G42"):
            self._check_d_defined(b.line_no)

        # 3) M 代码
        for m in b.m_codes:
            self._handle_m(m, b)

        # 4) 运动 / 固定循环
        axis_words_present = any(w in words for w in ("X", "Y", "Z"))
        if motion_requested:
            self._execute_motion(b, motion_requested, g53_active)
        elif self.canned is not None and axis_words_present:
            self._execute_canned(b)

        # 仅有坐标词、没有任何运动模态也无固定循环
        if (motion_requested is None and self.canned is None and axis_words_present
                and "G04" not in b.g_codes and "G52" not in b.g_codes
                and "G92" not in b.g_codes and not g53_active
                and not any(g in ("G28", "G30") for g in b.g_codes)):
            self._diag(
                "IMPLICIT_MOTION_MODE", "warning", b.line_no,
                f"第 {b.line_no} 行有轴坐标但无 G00/G01/G02/G03，按 RS-274 默认 G01 处理",
                "01 组运动模态上电状态未定义；缺少显式运动代码属于危险的模态继承",
                pre_state=self._snapshot(),
            )
            self._execute_motion(b, "G01", g53_active)

    # ------------------------------------------------------------------ #
    # 坐标目标解析
    # ------------------------------------------------------------------ #
    def _to_mm(self, value: float) -> float:
        return value * _INCH if self.units == "inch" else value

    @staticmethod
    def _code_word(words, letter) -> Optional[str]:
        w = words.get(letter)
        if w is None:
            return None
        return str(int(w.value)) if w.value == int(w.value) else str(w.value)

    def _apply_length_comp(self, code: str, block, has_z: bool) -> None:
        """切换 G43/G44/G49：物理位置不动，当前工件坐标 z 按新补偿重映射。

        反算物理点时仍用“旧 H/旧模式”，提交模式后再用“新 H”重映射，
        避免同段 G43 Hn 提前覆盖 H 号造成刀长跳变。
        随后的 Z 目标解析（绝对 G90 给新坐标，增量 G91 累加）符合 Fanuc 行为：
        "G43 H1 Z100" 物理终点 = 零点偏置 + 100 + 新刀长。
        """
        machine_z = self._to_machine(self.work_pos)["z"]  # 旧补偿
        self.tool_length_mode = code
        if code == "G49":
            self.h_code = None
        elif self._pending_h is not None:
            self.h_code = self._pending_h
        new_work_z = machine_z - self._offset_total()["z"] - self._active_length()
        self.work_pos["z"] = new_work_z

    def _target(self, block, axes: Tuple[str, ...], g53: bool = False) -> Dict[str, float]:
        """根据 G90/G91 与单位解析轴目标（返回出现的轴）。"""
        out: Dict[str, float] = {}
        for ax in axes:
            w = block.words.get(_AXIS_WORDS[ax])
            if w is None:
                continue
            v = self._to_mm(w.value)
            if g53:
                # G53：给定值就是机床坐标，反算成内部工件坐标
                out[ax] = v - self._offset_total()[ax] - (
                    self._active_length() if ax == "z" else 0.0)
            elif self.distance_mode == "G91":
                out[ax] = self.work_pos[ax] + v
            else:
                out[ax] = v
        return out

    # ------------------------------------------------------------------ #
    # 运动执行
    # ------------------------------------------------------------------ #
    def _first_move_guard(self, block) -> None:
        if self.first_move_done:
            return
        if self.units is None and not self._unit_noted:
            self._diag("UNITS_ASSUMED_MM", "warning", block.line_no,
                       f"第 {block.line_no} 行首次运动前未指定 G20/G21，按毫米 G21 解释",
                       "单位上电模态不确定；首件前缺少 G21 会导致英制程序被放大 25.4 倍",
                       pre_state=self._snapshot())
            self._unit_noted = True
            self.units = "mm"
            self.assumptions.append("程序未声明单位，按 G21 毫米处理")
        if not self._start_noted:
            self._start_noted = True
            if self._default_start_used:
                self._diag("INITIAL_POSITION_ASSUMED", "info", block.line_no,
                           "程序未用 G28/G53 建立初始位置，起始点取机床包络内的安全角点",
                           f"假设初始机床坐标 X{_fmt(self._start_machine['x'])} "
                           f"Y{_fmt(self._start_machine['y'])} Z{_fmt(self._start_machine['z'])}"
                           f"（XY 已内缩最大刀具半径 {_fmt(self._start_margin)}mm，"
                           "Z 取 zmax；可在 config.start_position 中指定）")
                self.assumptions.append(
                    f"初始机床位置取包络安全角点（内缩刀具半径 {_fmt(self._start_margin)}mm，"
                    "可用 start_position 覆盖）")

    def _execute_motion(self, block, code: str, g53: bool) -> None:
        self._first_move_guard(block)
        if not self.known_pos:
            self._diag("POSITION_UNKNOWN", "error", block.line_no,
                       f"第 {block.line_no} 行在 G28/G30 回零后以增量方式运动，参考点未知，"
                       "无法保证轨迹正确",
                       "G28/G30 终点为机床参考点，静态审查不知道其坐标",
                       pre_state=self._snapshot())

        target_axes = self._target(block, _AXES, g53=g53)
        has_center = any(w in block.words for w in ("I", "J", "K"))
        if code in ("G02", "G03") and not target_axes and not has_center:
            self._diag("ARC_WITHOUT_ENDPOINT", "warning", block.line_no,
                       f"第 {block.line_no} 行 {code} 无终点轴字也无 I/J/K，未产生运动",
                       "圆弧段必须给出终点轴字（整圆可仅给 I/J/K）")
            return
        # 补齐未给轴 -> 保持当前
        end_work = {**self.work_pos, **target_axes}
        start_work = dict(self.work_pos)
        pre = self._snapshot()

        if code == "G00":
            self._emit_linear(block, "rapid", start_work, end_work, pre, g53=g53)
        elif code == "G01":
            if self.feed_mode == "G93":
                if not self._g93_noted:
                    self._diag("INVERSE_TIME_FEED", "info", block.line_no,
                               "程序使用 G93 反比时间进给，逐段按 1/F 秒估算",
                               "G93 下 F 单位为 1/min；静态审查无法校验实际速度上限")
                    self._g93_noted = True
            self._check_cutting_preconditions(block, pre, length=None)
            self._emit_linear(block, "linear", start_work, end_work, pre, g53=g53)
        else:  # G02/G03
            geom = self._build_arc(block, code, start_work, end_work, pre)
            if geom is None:
                return
            self._check_cutting_preconditions(block, pre, length=geom.path_length)
            self._emit_arc(block, geom, start_work, end_work, pre, g53=g53)

        self.work_pos = end_work
        self.known_pos = True
        self.first_move_done = True

    def _build_arc(self, block, code, start_work, end_work, pre):
        h_ax, v_ax, n_ax = PLANE_AXES[self.plane]
        h_word, v_word = PLANE_CENTER_WORDS[self.plane]
        words = block.words
        clockwise = code == "G02"

        has_r = "R" in words
        center_letters = [w for w in ("I", "J", "K") if w in words]
        has_center = bool(center_letters)

        if has_r and has_center:
            self._diag("ARC_BOTH_R_IJK", "warning", block.line_no,
                       f"第 {block.line_no} 行同时给出 R 与 {'/'.join(center_letters)}，"
                       "按多数控制器以 I/J/K 为准",
                       "RS-274 规定同段 R 与 IJK 同时出现时 IJK 优先",
                       pre_state=pre)

        center_work = None
        r_val = None
        if has_center:
            center_work = dict(start_work)  # 未给的补偿轴等保持
            for word_letter, ax in (("I", "x"), ("J", "y"), ("K", "z")):
                if word_letter in words:
                    v = self._to_mm(words[word_letter].value)
                    if self.arc_distance_mode == "G91.1":
                        center_work[ax] = start_work[ax] + v
                    else:
                        center_work[ax] = v
            # 补偿轴圆心无意义，保持与起终点一致，避免螺旋误判
            center_work[n_ax] = start_work[n_ax]
            end_work.setdefault(n_ax, start_work[n_ax])
        elif has_r:
            r_val = self._to_mm(words["R"].value)
        else:
            self._diag("ARC_GEOMETRY_ERROR", "error", block.line_no,
                       f"第 {block.line_no} 行 {code} 缺少 R 或 I/J/K，无法重建圆弧",
                       "圆弧必须给出半径 R 或圆心 I/J/K",
                       pre_state=pre)
            return None

        # 圆弧不能在补偿轴方向出现圆心式偏移外的非法情况：交给几何求解判定
        geom = resolve_arc(
            start_work, end_work, self.plane, clockwise,
            center=center_work, radius_r=r_val,
            tolerance=float(self.cfg["arc_tolerance"]),
        )
        if not geom.ok:
            self._diag("ARC_GEOMETRY_ERROR", "error", block.line_no,
                       f"第 {block.line_no} 行圆弧几何错误：{geom.reason}",
                       (f"平面 {self.plane}（{self.plane}={h_ax.upper()}/{v_ax.upper()}），"
                        f"起点 ({_fmt(start_work[h_ax])},{_fmt(start_work[v_ax])})，"
                        f"终点 ({_fmt(end_work[h_ax])},{_fmt(end_work[v_ax])})，"
                        f"圆心方式={'IJK' if has_center else 'R'}，"
                        f"单位={self.units or 'mm(假定)'}，容差={self.cfg['arc_tolerance']:g}mm"),
                       pre_state=pre)
            return None
        return geom

    def _effective_feed(self, block, pre, length: Optional[float]) -> Tuple[float, float]:
        """返回 (进给 mm/min, 本段耗时 s)。"""
        if self.feed_mode == "G93":
            if self.feed_native and self.feed_native > 0:
                return (0.0, 60.0 / self.feed_native)
            self._diag("FEED_MISSING", "warning", block.line_no,
                       f"第 {block.line_no} 行 G93 反比时间进给但缺 F，按默认进给估算",
                       f"default_feed={self.cfg['default_feed']:g} mm/min",
                       pre_state=pre)
            return float(self.cfg["default_feed"]), 0.0

        feed_mm_min: Optional[float] = None
        if self.feed_native is not None:
            if self.feed_mode == "G95":
                f_rev = self._to_mm(self.feed_native)  # mm/rev
                rpm = self.spindle_rpm or 0.0
                if rpm <= 0:
                    self._diag("G95_NO_SPINDLE_SPEED", "warning", block.line_no,
                               f"第 {block.line_no} 行每转进给 G95 但无有效 S 转速，"
                               f"按默认 {self.cfg['default_spindle']:g} rpm 估算",
                               f"F={_fmt(self.feed_native)} {('in/rev' if self.units=='inch' else 'mm/rev')}"
                               f" × S({self.cfg['default_spindle']:g} rpm)",
                               pre_state=pre)
                    rpm = float(self.cfg["default_spindle"])
                feed_mm_min = f_rev * rpm
            else:  # G94
                feed_mm_min = self._to_mm(self.feed_native)

        if not feed_mm_min or feed_mm_min <= 0:
            self._diag("FEED_MISSING", "warning", block.line_no,
                       f"第 {block.line_no} 行切削运动缺少有效 F，按默认进给 "
                       f"{self.cfg['default_feed']:g} mm/min 估算耗时",
                       f"当前进给模态 {self.feed_mode}，F={_fmt(self.feed_native)}",
                       pre_state=pre)
            feed_mm_min = float(self.cfg["default_feed"])
        duration = 0.0 if length is None else length / feed_mm_min * 60.0
        return feed_mm_min, duration

    def _check_cutting_preconditions(self, block, pre, length: Optional[float]) -> None:
        if self.spindle not in ("M03", "M04"):
            if not self._spindle_warned:
                self._diag("SPINDLE_OFF_CUTTING", "warning", block.line_no,
                           f"第 {block.line_no} 行切削进给时主轴未启动"
                           f"（当前 {self.spindle or '无 M03/M04'}）",
                           "G01/G02/G03 为切削运动；未启动主轴直接进给会顶刀/崩刃",
                           pre_state=pre)
                self._spindle_warned = True
        else:
            self._spindle_warned = False
        if self.tool_no is None:
            self._diag("MOTION_WITHOUT_TOOL", "warning", block.line_no,
                       f"第 {block.line_no} 行切削运动前未用 T/M06 装刀",
                       "当前无刀具号，无法核对刀长与直径，行程检查未计入刀具实体",
                       pre_state=pre)

    # ------------------------------------------------------------------ #
    # 段发射（记录段 + 几何检查）
    # ------------------------------------------------------------------ #
    def _emit_linear(self, block, kind, start_work, end_work, pre,
                     g53: bool = False, note: str = "", synthetic: bool = False,
                     feed_override: Optional[float] = None) -> Optional[Move]:
        length = math.sqrt(sum((end_work[a] - start_work[a]) ** 2 for a in _AXES))
        if length < 1e-12:
            return None  # 零位移段不发射（起点已在前段检查过）

        # 采样：线性段取两端（安全平面/包络沿线性插值极值在端点）
        samples_work = [dict(start_work), dict(end_work)]

        feed = duration = None
        if kind == "rapid":
            duration = length / float(self.cfg["rapid_speed"]) * 60.0
            self.t_rapid += duration
        elif kind == "linear":
            if feed_override is not None:
                feed, duration = feed_override, length / feed_override * 60.0
            else:
                feed, duration = self._effective_feed(block, pre, length)
            self.t_cut += duration

        move = self._record_move(block.line_no, kind, samples_work, pre,
                                 duration=duration or 0.0, feed=feed,
                                 note=note, synthetic=synthetic, g53=g53)
        return move

    def _emit_arc(self, block, geom, start_work, end_work, pre,
                  g53: bool = False) -> Move:
        feed, duration = self._effective_feed(block, pre, geom.path_length)
        self.t_cut += duration
        # geom.samples 为工件坐标
        move = self._record_move(
            block.line_no, "arc", geom.samples, pre,
            duration=duration, feed=feed, geom=geom, g53=g53)
        return move

    def _record_move(self, line_no, kind, samples_work, pre, duration=0.0,
                     feed=None, note="", synthetic=False, g53=False, geom=None) -> Move:
        samples_machine = [self._to_machine(p) for p in samples_work]
        start_m, end_m = samples_machine[0], samples_machine[-1]
        start_w, end_w = samples_work[0], samples_work[-1]

        idx = len(self.moves)
        center_m = sweep = radius = helical = None
        if geom is not None:
            center_m = self._to_machine({**start_w, **geom.center}) if geom.center else None
            # 圆心仅平面轴有效
            sweep = round(geom.sweep_deg, 6)
            radius = geom.radius
            helical = geom.helical_depth

        move = Move(
            index=idx, kind=kind, line_no=line_no,
            start=start_m, end=end_m, plane=self.plane,
            radius=radius, center=center_m, sweep_deg=sweep,
            helical_depth=helical, feed_mm_min=feed, duration_s=duration,
            samples=samples_machine, note=note, synthetic=synthetic,
        )
        self.moves.append(move)
        for p in samples_machine:
            self.machine_bounds.add(p)
        for p in samples_work:
            self.work_bounds.add(p)

        self._check_envelope(move, samples_machine, pre, line_no)
        self._check_obstacle_collision(move, samples_machine, pre, line_no)
        if kind == "rapid":
            self._check_rapid_safety(samples_work, pre, line_no, idx, note)
        return move

    def _check_envelope(self, move, samples, pre, line_no) -> None:
        env = self.cfg["envelope"]
        radius = 0.0
        tool_len_margin = 0.0
        tool = None
        if self.tool_no is not None:
            tool = self.cfg["tools"].get(str(self.tool_no))
        if tool is not None and self.cfg.get("check_tool_envelope", True):
            if self.cutter_mode in ("G41", "G42") and self.d_code is not None:
                radius = cutter_radius(self.cfg, self.d_code, self.tool_no)
            else:
                radius = float(tool.get("diameter", 0.0)) / 2.0
        # 逐轴找最差采样
        checks = {
            "x": (("xmin", -1), ("xmax", 1)),
            "y": (("ymin", -1), ("ymax", 1)),
            "z": (("zmin", -1), ("zmax", 1)),
        }
        for ax, sides in checks.items():
            for env_key, sign in sides:
                limit = float(env[env_key])
                margin = radius if ax in ("x", "y") else 0.0
                worst = None
                for p in samples:
                    val = p[ax] - margin if sign < 0 else p[ax] + margin
                    if (worst is None or
                            (sign < 0 and val < worst) or
                            (sign > 0 and val > worst)):
                        worst = val
                if worst is None:
                    continue
                if (sign < 0 and worst < limit - 1e-9) or (sign > 0 and worst > limit + 1e-9):
                    tool_desc = (f"计入 T{self.tool_no} 半径 {radius:g}mm"
                                 if margin else "未计入刀具尺寸")
                    self._diag(
                        "ENVELOPE_VIOLATION", "error", line_no,
                        f"第 {line_no} 行 {move.kind} 段 {ax.upper()} 轴越界："
                        f"{_fmt(worst)} mm 超出 {env_key}={_fmt(limit)} mm",
                        f"{ax.upper()}{'最小' if sign < 0 else '最大'}物理值={_fmt(worst)}"
                        f"（机床坐标{'+/-刀具' if margin else ''}），"
                        f"行程限制 {_fmt(limit)}mm，超差 {_fmt(abs(worst-limit))}mm；{tool_desc}",
                        pre_state=pre, segment_index=move.index)

    def _check_rapid_safety(self, samples_work, pre, line_no, seg_idx, note) -> None:
        clearance = float(self.cfg["safety_clearance"])
        zs = [p["z"] for p in samples_work]
        zmin, zmax = min(zs), max(zs)
        if zmin + 1e-9 >= clearance:
            return
        xy_moving = any(
            abs(samples_work[0][a] - samples_work[-1][a]) > 1e-9
            for a in ("x", "y")
        )
        descending = samples_work[-1]["z"] + 1e-9 < samples_work[0]["z"]
        origin = "（固定循环接近 R/孔底）" if note else ""
        if xy_moving:
            severity = "error"
            risk = "带 XY 联动的低空快移极易撞上夹具/毛坯"
        elif descending:
            severity = "warning"
            risk = "快移垂直下插到安全平面以下，可能直接冲入毛坯/夹具"
        else:
            severity = "info"
            risk = "切削后的垂直快移抬刀属常规动作，但部分工艺要求抬出安全平面前用 G01"
        self._diag(
            "RAPID_BELOW_SAFETY", severity, line_no,
            f"第 {line_no} 行 G00 快移{origin}低于安全平面：最低 z={_fmt(zmin)} mm "
            f"< 安全平面 {_fmt(clearance)} mm",
            f"安全高度取自 config.safety_clearance={_fmt(clearance)}mm（工件坐标）；"
            f"该快移段 z 范围 {_fmt(zmin)}~{_fmt(zmax)}mm，"
            f"XY{'有' if xy_moving else '无'}联动、方向{'下插' if descending else '上抬'}；{risk}",
            pre_state=pre, segment_index=seg_idx)

    # ------------------------------------------------------------------ #
    # 障碍物碰撞检查（刀具组件扫掠体）
    # ------------------------------------------------------------------ #
    def _tool_assembly(self) -> Optional[Dict[str, Any]]:
        """当前刀具的组件圆柱模型。

        参考点 = 引擎跟踪的机床坐标点（G43/G44 激活时即主轴端面/规线）。
        刀尖 = 参考点 z − 生效长度补偿；组件自刀尖向上堆叠：刃 → 刀杆 → 刀柄。
        无刀具或未定义刀具时返回 None（无法建模，跳过碰撞检查）。
        """
        if self.tool_no is None:
            return None
        tool = self.cfg.get("tools", {}).get(str(self.tool_no))
        if not isinstance(tool, dict):
            return None
        tip = -self._active_length()
        comps = []
        z = tip
        for name, dkey, lkey, _cn in _TOOL_COMPONENTS:
            length = tool.get(lkey) or 0.0
            dia = tool.get(dkey) or 0.0
            if length > 1e-9 and dia > 1e-9:
                comps.append({"name": name, "radius": dia / 2.0,
                              "z_lo": z, "z_hi": z + length})
            z += length
        if not comps:
            return None
        return {"tip_offset": tip, "components": comps}

    def _check_obstacle_collision(self, move, samples, pre, line_no) -> None:
        """按可配置最大步长细分轨迹，逐段求组件扫掠体与障碍物的相交/最小间隙。

        同一运动段内、同一 (障碍物, 组件) 的连续命中合并为一条诊断，
        记录首次/末次接触的机床坐标（刀具参考点）。
        """
        if not self.cfg.get("check_obstacle_collision", True):
            return
        raw_obstacles = self.cfg.get("obstacles") or []
        if not raw_obstacles or move.kind == "dwell" or len(samples) < 2:
            return
        assembly = self._tool_assembly()
        if assembly is None:
            return
        obstacles = [ob for ob in raw_obstacles if obstacle_usable(ob)]
        if not obstacles:
            return
        step = float(self.cfg.get("collision_max_step") or 2.0)
        step = min(max(step, 1e-3), 1000.0)
        warn = max(0.0, float(self.cfg.get("collision_clearance_warn") or 0.0))

        pts = densify_polyline(samples, step)
        ob_bounds = [(ob, obstacle_aabb(ob)) for ob in obstacles]
        open_hits: Dict[tuple, dict] = {}   # (障碍物id, 组件) -> 进行中的命中
        closed: List[dict] = []
        for a, b in zip(pts, pts[1:]):
            current: Dict[tuple, tuple] = {}
            for comp in assembly["components"]:
                bb = swept_aabb(a, b, comp["radius"], comp["z_lo"], comp["z_hi"])
                for ob, obb in ob_bounds:
                    key = (ob["id"], comp["name"])
                    if key in current:
                        continue
                    # 宽相位：包围盒间距已超告警阈值则不必精算
                    if aabb_gap(bb, obb) > warn + 1e-9:
                        continue
                    d, t = swept_clearance(a, b, comp["radius"],
                                           comp["z_lo"], comp["z_hi"], ob)
                    if d <= warn + 1e-9:
                        current[key] = (d, t, a, b)
            # 未继续命中的连续段在此结束
            for key in list(open_hits):
                if key not in current:
                    closed.append(open_hits.pop(key))
            for key, (d, t, a, b) in current.items():
                hit = open_hits.get(key)
                if hit is None:
                    open_hits[key] = {"ob": key[0], "comp": key[1], "min": d,
                                      "first_seg": (a, b, t), "last_seg": (a, b, t)}
                else:
                    if d < hit["min"]:
                        hit["min"] = d
                    hit["last_seg"] = (a, b, t)
        closed.extend(open_hits.values())

        if not closed:
            return
        ob_by_id = {ob["id"]: ob for ob in obstacles}
        comp_by_name = {c["name"]: c for c in assembly["components"]}
        for hit in closed:
            oid, cname = hit["ob"], hit["comp"]
            comp = comp_by_name[cname]
            ob = ob_by_id[oid]
            intersect = hit["min"] <= 1e-6
            clearance = round(max(hit["min"], 0.0), 4)
            severity = "error" if intersect else "warning"
            # 首次/末次接触：进入/离开「间隙 ≤ 告警阈值」区域的精确位置
            fa, fb, ft = hit["first_seg"]
            la, lb, lt = hit["last_seg"]
            t_first = first_crossing(fa, fb, ft, comp["radius"],
                                     comp["z_lo"], comp["z_hi"], ob, warn)
            t_last = last_crossing(la, lb, lt, comp["radius"],
                                   comp["z_lo"], comp["z_hi"], ob, warn)
            first = {a: round(v, 4) for a, v in lerp3(fa, fb, t_first).items()}
            last = {a: round(v, 4) for a, v in lerp3(la, lb, t_last).items()}
            kind_cn = _KIND_CN.get(move.kind, move.kind)
            comp_cn = _COMPONENT_CN[cname]
            if intersect:
                msg = (f"第 {line_no} 行{kind_cn}段刀具{comp_cn}（T{self.tool_no}）"
                       f"与障碍物 {oid} 相交")
            else:
                msg = (f"第 {line_no} 行{kind_cn}段刀具{comp_cn}（T{self.tool_no}）"
                       f"与障碍物 {oid} 最小间隙 {clearance}mm，"
                       f"低于告警阈值 {warn:g}mm")
            basis = (
                f"刀具 T{self.tool_no} {comp_cn}组件：半径 {comp['radius']:g}mm，"
                f"轴向区间 [{comp['z_lo']:g}, {comp['z_hi']:g}]mm"
                f"（相对刀具参考点，刀尖位于 {assembly['tip_offset']:g}mm）；"
                f"轨迹按最大步长 {step:g}mm 细分后逐段求扫掠体与障碍物"
                f"（{ob['type']}）的最小间隙={clearance}mm"
                f"（{'≤0 判定相交' if intersect else f'告警阈值 collision_clearance_warn={warn:g}mm'}）；"
                f"首次接触机床坐标 ({first['x']:g}, {first['y']:g}, {first['z']:g})，"
                f"末次接触 ({last['x']:g}, {last['y']:g}, {last['z']:g})"
                f"（进入/离开间隙告警阈值带的刀具参考点位置；阈值设为 0 时即精确接触点）")
            self._diag(
                "OBSTACLE_COLLISION", severity, line_no, msg, basis,
                pre_state=pre, segment_index=move.index,
                details={
                    "obstacle_id": oid,
                    "obstacle_type": ob["type"],
                    "component": cname,
                    "tool": self.tool_no,
                    "move_kind": move.kind,
                    "intersection": intersect,
                    "min_clearance_mm": clearance,
                    "first_contact_machine": first,
                    "last_contact_machine": last,
                })

    # ------------------------------------------------------------------ #
    # 固定循环
    # ------------------------------------------------------------------ #
    def _execute_canned(self, block) -> None:
        self._first_move_guard(block)
        code = self.canned
        pre = self._snapshot()

        # 参数更新（模态保持）；R/Z 各自独立按 G90/G91 与单位换算
        if "R" in block.words:
            r = self._to_mm(block.words["R"].value)
            if self.distance_mode == "G91":
                self.canned_params["r"] = self.work_pos["z"] + r
            else:
                self.canned_params["r"] = r
        z_target = self._target(block, ("z",)).get("z")
        if z_target is not None:
            self.canned_params["z"] = z_target
        xy_targets = self._target(block, ("x", "y"))
        if "Q" in block.words:
            self.canned_params["q"] = self._to_mm(block.words["Q"].value)
        if "P" in block.words:
            self.canned_params["p"] = self._to_mm(block.words["P"].value)
        x = xy_targets.get("x", self.canned_params.get("x", self.work_pos["x"]))
        y = xy_targets.get("y", self.canned_params.get("y", self.work_pos["y"]))
        self.canned_params["x"] = x
        self.canned_params["y"] = y

        if "z" not in self.canned_params or "r" not in self.canned_params:
            self._diag("CANNED_CYCLE_INCOMPLETE", "error", block.line_no,
                       f"第 {block.line_no} 行 {code} 缺少 Z/R 参数，无法重建钻孔轨迹",
                       "固定循环必须给出孔底 Z 与 R 点（模态保持也行）",
                       pre_state=pre)
            return

        z_bottom = self.canned_params["z"]
        r_point = self.canned_params["r"]
        q = self.canned_params.get("q")
        initial_z = self.work_pos["z"]

        if r_point < z_bottom - 1e-9:
            self._diag("CANNED_CYCLE_INCOMPLETE", "error", block.line_no,
                       f"第 {block.line_no} 行 {code} 的 R 点({_fmt(r_point)})"
                       f"低于孔底 Z({_fmt(z_bottom)})",
                       "R 点必须在孔底之上，否则快移会直接撞入工件",
                       pre_state=pre)
            return

        feed, _ = self._effective_feed(block, pre, length=0.0)
        self._check_cutting_preconditions(block, pre, length=None)

        def rapid(to: Dict[str, float], n: str):
            if self._emit_linear(block, "rapid", dict(self.work_pos), to, pre,
                                 note=f"canned:{code}:{n}", synthetic=True) is not None:
                self.work_pos = to

        def feed_to(to: Dict[str, float], n: str):
            if self._emit_linear(block, "linear", dict(self.work_pos), to, pre,
                                 note=f"canned:{code}:{n}", synthetic=True,
                                 feed_override=feed) is not None:
                self.work_pos = to

        # 1) XY 定位（z 保持当前高度）
        xy = {"x": x, "y": y, "z": self.work_pos["z"]}
        if any(abs(self.work_pos[a] - xy[a]) > 1e-9 for a in ("x", "y")):
            rapid(xy, "position")
        # 2) 快移到 R
        if self.work_pos["z"] > r_point + 1e-9:
            rapid({"x": x, "y": y, "z": r_point}, "to-R")
        # 3) 加工到孔底
        bottom = {"x": x, "y": y, "z": z_bottom}
        peck_codes = {"G73", "G83"}
        if code in peck_codes and q and q > 0:
            z = self.work_pos["z"]
            step = 0
            while z > z_bottom + 1e-9:
                step += 1
                next_z = max(z - q, z_bottom)
                feed_to({"x": x, "y": y, "z": next_z}, f"peck{step}")
                z = next_z
                if z > z_bottom + 1e-9:
                    if code == "G83":
                        rapid({"x": x, "y": y, "z": r_point}, f"retract{step}")
                        rapid({"x": x, "y": y, "z": z}, f"reapproach{step}")
                    else:  # G73 微小退刀
                        rapid({"x": x, "y": y, "z": min(z + 1.0, r_point)},
                              f"chip{step}")
        else:
            feed_to(bottom, "drill")

        # 4) 孔底动作：G82/G89 暂停；G74/G84 反转；G86 主轴停；G76/G87/G88 近似
        dwell_s = 0.0
        if code in ("G82", "G89"):
            p = self.canned_params.get("p")
            if p is not None:
                dwell_s = p / 1000.0
                if self.cfg.get("dwell_units_seconds"):
                    dwell_s = p
        if dwell_s:
            self._emit_dwell(block, dwell_s, pre, note=f"canned:{code}:dwell")
        if code in ("G84", "G74"):
            self.spindle = "M04" if code == "G84" else "M03"  # 攻丝回退转向
        elif code == "G86":
            self.spindle = "M05"

        # 5) 回退
        if code in ("G85", "G89"):
            feed_to({"x": x, "y": y, "z": r_point}, "feed-retract")
        elif code in ("G84", "G74"):
            # 攻丝以进给速度回 R，恢复正转
            feed_to({"x": x, "y": y, "z": r_point}, "tap-retract")
            self.spindle = "M03"
        else:
            # G73/G76/G81/G82/G83/G86/G87/G88 快移回退
            retract_z = initial_z if self.return_mode == "G98" else r_point
            if self.return_mode == "G98" and initial_z < r_point:
                retract_z = r_point
            rapid({"x": x, "y": y, "z": retract_z},
                  "G98-initial" if self.return_mode == "G98" else "G99-R")

        if code in ("G76", "G87", "G88"):
            self._diag("CANNED_CYCLE_APPROX", "info", block.line_no,
                       f"第 {block.line_no} 行 {code} 含让刀/镗孔定向动作，"
                       "静态轨迹按通用钻孔模板近似",
                       "Q 让刀量、主轴定向等不产生可静态确定的轴移动，未建模",
                       pre_state=pre)
        self.first_move_done = True
        self.known_pos = True

    # ------------------------------------------------------------------ #
    # 特殊 G/M
    # ------------------------------------------------------------------ #
    def _handle_dwell(self, block) -> None:
        pre = self._snapshot()
        words = block.words
        seconds = None
        if "P" in words:
            p = float(words["P"].value)
            seconds = p if self.cfg.get("dwell_units_seconds") else p / 1000.0
        elif "X" in words:
            seconds = float(words["X"].value)
        if seconds is not None and seconds > 0:
            self._emit_dwell(block, seconds, pre)

    def _emit_dwell(self, block, seconds, pre, note=""):
        self.t_dwell += seconds
        move = Move(
            index=len(self.moves), kind="dwell", line_no=block.line_no,
            start=self._to_machine(self.work_pos),
            end=self._to_machine(self.work_pos),
            plane=self.plane, duration_s=seconds, note=note,
        )
        self.moves.append(move)

    def _handle_g28(self, g, block) -> None:
        pre = self._snapshot()
        targets = self._target(block, _AXES)
        if targets:
            intermediate = {**self.work_pos, **targets}
            moved = self._emit_linear(block, "rapid", dict(self.work_pos), intermediate,
                                      pre, note=f"{g}:intermediate")
            if moved is not None:
                self.work_pos = intermediate
                self.first_move_done = True
        self._diag("UNKNOWN_REFERENCE_POSITION", "warning", block.line_no,
                   f"第 {block.line_no} 行 {g} 回参考点：参考点坐标未知，"
                   "回零段及之后的绝对位置审查存在盲区",
                   "机床参考点取决于各轴回零设定，未包含在机床配置中",
                   pre_state=pre)
        self.known_pos = False

    def _handle_g52(self, block) -> None:
        pre = self._snapshot()
        machine_before = self._to_machine(self.work_pos)
        vals_present = False
        # G52 的值始终是绝对局部偏置（与 G90/G91 无关）
        for ax in ("x", "y", "z"):
            w = block.words.get(_AXIS_WORDS[ax])
            if w is not None:
                vals_present = True
                self.g52[ax] = self._to_mm(w.value)
        if vals_present:
            # 物理位置不变，工件坐标按新的总偏置重新解释
            self.work_pos = self._to_work(machine_before)
        if all(abs(v) < 1e-12 for v in self.g52.values()):
            self._diag("LOCAL_OFFSET_RESET", "info", block.line_no,
                       f"第 {block.line_no} 行 G52 取消局部坐标系",
                       "局部工件偏置清零", pre_state=pre)
        elif vals_present:
            self._diag("LOCAL_OFFSET_SET", "info", block.line_no,
                       f"第 {block.line_no} 行 G52 设置局部偏置 "
                       f"X{_fmt(self.g52['x'])} Y{_fmt(self.g52['y'])} "
                       f"Z{_fmt(self.g52['z'])}",
                       "G52 在当前工件坐标系上叠加局部偏置，跨设备移植时易被遗忘",
                       pre_state=pre)

    def _handle_g92(self, g, block) -> None:
        pre = self._snapshot()
        if g == "G92.1":
            self.g92_shift = _zero3()
            if not self._g921_noted:
                self._diag("G92_SHIFT_RESET", "info", block.line_no,
                           f"第 {block.line_no} 行 G92.1 取消坐标设定偏置",
                           "G92 设定的坐标偏移清零", pre_state=pre)
                self._g921_noted = True
            return
        vals = {}
        for ax in ("x", "y", "z"):
            w = block.words.get(_AXIS_WORDS[ax])
            if w is not None:
                vals[ax] = self._to_mm(w.value)
        if not vals:
            return
        # 当前物理位置 = 给定坐标 + base+g52+shift+刀长  =>  反解 shift，
        # 随后用新偏置把当前工件坐标重映射为 G92 所宣告的坐标
        cur_m = self._to_machine(self.work_pos)
        for ax, coord in vals.items():
            self.g92_shift[ax] = cur_m[ax] - self.base_offset[ax] - self.g52[ax] - (
                self._active_length() if ax == "z" else 0.0) - coord
        remapped = self._to_work(cur_m)
        self.work_pos = remapped
        self._diag("G92_COORD_SET", "warning", block.line_no,
                   f"第 {block.line_no} 行 G92 重设坐标系（"
                   + " ".join(f"{a.upper()}={_fmt(v)}" for a, v in vals.items())
                   + "），后续绝对坐标继承该临时偏置",
                   "G92 不移动轴、只改坐标偏置；换机后实际位置不同会造成系统性错位，"
                   f"反解附加偏置 z={_fmt(self.g92_shift['z'])}mm",
                   pre_state=pre)

    def _switch_work_offset(self, g, line_no) -> None:
        if g == self.work_offset:
            return
        old = self.work_offset
        pre = self._snapshot()
        machine = self._to_machine(self.work_pos)
        self.work_offset = g
        self.base_offset = dict(self.cfg["work_offsets"][g])
        self.work_pos = self._to_work(machine)
        self._diag("WORK_OFFSET_SWITCH", "info", line_no,
                   f"第 {line_no} 行切换工件坐标系 {old} -> {g}，"
                   f"零点偏置 X{_fmt(self.base_offset['x'])} "
                   f"Y{_fmt(self.base_offset['y'])} Z{_fmt(self.base_offset['z'])}",
                   f"物理位置不变，工件坐标按 {g} 的零点重新解释；"
                   "两套设备零点标定不同会直接改变机床坐标",
                   pre_state=pre)

    def _handle_m(self, m, block) -> None:
        line_no = block.line_no
        if m in ("M03", "M04", "M05"):
            self.spindle = m
        elif m in ("M07", "M08"):
            self.coolant = m  # type: ignore[attr-defined]
        elif m == "M09":
            self.coolant = None  # type: ignore[attr-defined]
        elif m == "M06":
            pre = self._snapshot()
            if self.tool_no is None:
                self._diag("UNDEFINED_TOOL", "error", line_no,
                           f"第 {line_no} 行 M06 换刀但未指定 T 刀号",
                           "换刀指令必须有 T 号；无法核对刀长/直径",
                           pre_state=pre)
            elif self.cfg.get("tools") and str(self.tool_no) not in self.cfg["tools"]:
                self._diag("UNDEFINED_TOOL", "error", line_no,
                           f"第 {line_no} 行 M06 调用 T{self.tool_no}，"
                           "但机床配置刀具表中没有该刀",
                           f"config.tools 中已定义 {sorted(self.cfg['tools'].keys())}；"
                           "未定义刀具的长度/直径未知，首件存在撞机风险",
                           pre_state=pre)
            else:
                tool_row = self.cfg["tools"].get(str(self.tool_no), {})
                self._diag("TOOL_CHANGE", "info", line_no,
                           f"第 {line_no} 行换刀 T{self.tool_no}，"
                           f"刀长 {_fmt(tool_row.get('length', 0.0))}mm、"
                           f"直径 {_fmt(tool_row.get('diameter', 0.0))}mm"
                           + (f"（{tool_row.get('description')}）"
                              if tool_row.get("description") else ""),
                           "刀具几何取自 config.tools；长度补偿需后续 G43 H 才生效",
                           pre_state=pre)
            self.t_toolchange += float(self.cfg["tool_change_time"])
            # 换刀后长度/半径补偿模态按 Fanuc 习惯取消，等待重新 G43/G41
            self.tool_length_mode = "G49"
            self.h_code = None
        elif m in ("M02", "M30"):
            self.program_end = (m, line_no)  # type: ignore[attr-defined]
        elif m in ("M00", "M01"):
            pass  # 计划停止，无轴运动

    def _check_h_defined(self, line_no) -> None:
        h = self.h_code
        if h is None:
            if self._h_warned != "NONE":
                self._h_warned = "NONE"
                self._diag("UNDEFINED_TOOL_LENGTH", "warning", line_no,
                           f"第 {line_no} 行 {self.tool_length_mode} 未给 H 号，"
                           "长度补偿按 0 处理",
                           "无 H 的刀长补偿在多数控制器上报警；物理 z 与预期可能差整个刀长",
                           pre_state=self._snapshot())
            return
        in_h = h in self.cfg.get("h_offsets", {})
        in_tool = self.tool_no is not None and h == str(self.tool_no) \
            and str(self.tool_no) in self.cfg.get("tools", {})
        if not in_h and not in_tool and self._h_warned != h:
            self._h_warned = h
            self._diag("UNDEFINED_TOOL_LENGTH", "warning", line_no,
                       f"第 {line_no} 行 H{h} 在 h_offsets 与同号刀具表中均无定义，"
                       "长度补偿按 0 处理",
                       "刀长未标定；换机/换刀后 z 向超差或撞机风险高",
                       pre_state=self._snapshot())
        elif (in_h or in_tool) and self._h_warned == h:
            self._h_warned = None

    def _check_d_defined(self, line_no) -> None:
        d = self.d_code
        if d is None:
            if self._d_warned != "NONE":
                self._d_warned = "NONE"
                self._diag("UNDEFINED_CUTTER_RADIUS", "warning", line_no,
                           f"第 {line_no} 行 {self.cutter_mode} 未给 D 号，"
                           "半径补偿按 0 处理",
                           "无 D 的刀径补偿在控制器上通常报警",
                           pre_state=self._snapshot())
            return
        in_d = d in self.cfg.get("d_offsets", {})
        in_tool = self.tool_no is not None and d == str(self.tool_no) \
            and str(self.tool_no) in self.cfg.get("tools", {})
        if not in_d and not in_tool and self._d_warned != d:
            self._d_warned = d
            self._diag("UNDEFINED_CUTTER_RADIUS", "warning", line_no,
                       f"第 {line_no} 行 D{d} 在 d_offsets 与同号刀具表中均无定义",
                       "刀径补偿值未知，侧向轨迹与包络检查按半径 0 处理",
                       pre_state=self._snapshot())
        elif (in_d or in_tool) and self._d_warned == d:
            self._d_warned = None

    # ------------------------------------------------------------------ #
    # 结束检查 / 报告
    # ------------------------------------------------------------------ #
    def _end_of_program_checks(self, last_line) -> None:
        line = last_line
        if not getattr(self, "program_end", None):
            self._diag("NO_PROGRAM_END", "warning", line,
                       "程序缺少 M30/M02 结束指令",
                       "无结束码时执行完最后一行后的控制器行为不确定",
                       pre_state=self._snapshot())
        if self.spindle == "M03" or self.spindle == "M04":
            self._diag("SPINDLE_ON_AT_END", "warning", line,
                       f"程序结束时主轴仍处于 {self.spindle} 旋转状态",
                       "未见 M05；模态跨程序继承到下一工件有安全隐患",
                       pre_state=self._snapshot())
        if getattr(self, "coolant", None):
            self._diag("COOLANT_ON_AT_END", "warning", line,
                       f"程序结束时冷却 {self.coolant} 仍开",
                       "未见 M09，冷却状态被继承", pre_state=self._snapshot())
        if self.cutter_mode != "G40":
            self._diag("CUTTER_COMP_ACTIVE_AT_END", "warning", line,
                       f"程序结束时刀径补偿 {self.cutter_mode} 仍激活"
                       f"（D{self.d_code or '?'}）",
                       "未见 G40；下个程序首段移动会继承补偿矢量",
                       pre_state=self._snapshot())
        if self.canned is not None:
            self._diag("CANNED_ACTIVE_AT_END", "warning", line,
                       f"程序结束时固定循环 {self.canned} 仍为模态",
                       "未见 G80；后续任何带轴字的程序段都会触发钻孔动作",
                       pre_state=self._snapshot())
        if any(abs(v) > 1e-9 for v in self.g92_shift.values()):
            self._diag("G92_SHIFT_ACTIVE_AT_END", "warning", line,
                       "程序结束时 G92 临时坐标偏置仍未清零",
                       "未见 G92.1；该偏置会被后续程序继承",
                       pre_state=self._snapshot())

    def _snapshot(self) -> Dict[str, Any]:
        machine = self._to_machine(self.work_pos) if self.known_pos else None
        return {
            "line_context": "该行执行前",
            "units": self.units or ("mm(假定)" if self._unit_noted else "未指定"),
            "distance_mode": self.distance_mode,
            "arc_distance_mode": self.arc_distance_mode,
            "plane": self.plane,
            "motion_mode": self.motion,
            "feed_mode": self.feed_mode,
            "feed": self.feed_native,
            "spindle": self.spindle,
            "spindle_rpm": self.spindle_rpm,
            "tool": self.tool_no,
            "tool_length_comp": (None if self.tool_length_mode == "G49"
                                 else f"{self.tool_length_mode} H{self.h_code or '?'} "
                                      f"= {_fmt(self._active_length())}mm"),
            "cutter_comp": (None if self.cutter_mode == "G40"
                            else f"{self.cutter_mode} D{self.d_code or '?'} "
                                 f"r={_fmt(cutter_radius(self.cfg, self.d_code, self.tool_no))}mm"),
            "work_offset": self.work_offset,
            "return_mode": self.return_mode,
            "canned_cycle": self.canned,
            "work_position": {a: round(self.work_pos[a], 6) for a in _AXES},
            "machine_position": ({a: round(machine[a], 6) for a in _AXES}
                                 if machine else None),
            "position_known": self.known_pos,
        }

    def _diag(self, code, severity, line_no, message, basis,
              pre_state=None, segment_index=None, details=None) -> None:
        self.diags.append(Diagnostic(
            code=code, severity=severity, line_no=line_no,
            message=message, basis=basis,
            preceding_state=pre_state or {}, segment_index=segment_index,
            details=details or {},
        ))

    def _build_report(self, text, filename, blocks) -> Dict[str, Any]:
        counts = {"error": 0, "warning": 0, "info": 0}
        for d in self.diags:
            counts[d.severity] = counts.get(d.severity, 0) + 1
        severity_order = {"error": 0, "warning": 1, "info": 2}
        diags = sorted(self.diags,
                       key=lambda d: (severity_order.get(d.severity, 9),
                                      d.line_no or 0, d.code))
        total_time = self.t_rapid + self.t_cut + self.t_dwell + self.t_toolchange
        coll_diags = [d for d in self.diags if d.code == "OBSTACLE_COLLISION"]
        clearances = [d.details.get("min_clearance_mm") for d in coll_diags
                      if d.details.get("min_clearance_mm") is not None]
        return {
            "filename": filename or "inline",
            "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest()[:16],
            "lines_total": len(text.splitlines()),
            "blocks_executed": len(blocks),
            "units": self.units or "mm(假定)",
            "config": {
                "name": self.cfg.get("name"),
                "version": self.cfg.get("version"),
                "safety_clearance": self.cfg["safety_clearance"],
                "envelope": self.cfg["envelope"],
            },
            "risk": "error" if counts["error"] else (
                "warning" if counts["warning"] else "clean"),
            "diagnostic_counts": counts,
            "diagnostics": [d.to_dict() for d in diags],
            "collisions": {
                "obstacles_configured": len(self.cfg.get("obstacles") or []),
                "incidents": len(coll_diags),
                "intersections": sum(1 for d in coll_diags if d.severity == "error"),
                "min_clearance_mm": (round(min(clearances), 4)
                                     if clearances else None),
                "max_step_mm": self.cfg.get("collision_max_step"),
                "clearance_warn_mm": self.cfg.get("collision_clearance_warn"),
            },
            "trajectory": {
                "segments": len(self.moves),
                "bounds_machine": self.machine_bounds.to_dict(),
                "bounds_work": self.work_bounds.to_dict(),
                "initial_machine_position": {
                    a: round(self._start_machine[a], 6) for a in _AXES},
            },
            "time_estimate_s": {
                "total": round(total_time, 3),
                "rapid": round(self.t_rapid, 3),
                "cutting": round(self.t_cut, 3),
                "dwell": round(self.t_dwell, 3),
                "tool_change": round(self.t_toolchange, 3),
            },
            "time_basis": (
                f"快移 {self.cfg['rapid_speed']:g} mm/min；"
                f"切削距离/进给（G94）或 F×S（G95）；"
                f"换刀 {self.cfg['tool_change_time']:g}s/次；"
                "未计入加减速"),
            "assumptions": self.assumptions,
            "moves": [self._move_dict(m) for m in self.moves],
        }

    def _move_dict(self, m: Move) -> Dict[str, Any]:
        return {
            "index": m.index,
            "kind": m.kind,
            "line_no": m.line_no,
            "synthetic": m.synthetic,
            "note": m.note,
            "plane": m.plane,
            "work_start": {a: round(m.start[a] - self._offset_total()[a]
                                    - (self._active_length() if a == "z" else 0.0), 6)
                           for a in _AXES},
            "start_machine": {a: round(m.start[a], 6) for a in _AXES},
            "end_machine": {a: round(m.end[a], 6) for a in _AXES},
            "radius": None if m.radius is None else round(m.radius, 6),
            "center_machine": m.center and {a: round(m.center[a], 6) for a in _AXES},
            "sweep_deg": m.sweep_deg,
            "helical_depth": (None if m.helical_depth is None
                              else round(m.helical_depth, 6)),
            "feed_mm_min": None if m.feed_mm_min is None else round(m.feed_mm_min, 4),
            "duration_s": round(m.duration_s, 4),
        }


def review_program(text: str, config: Dict[str, Any],
                   filename: Optional[str] = None) -> Dict[str, Any]:
    """便捷入口：对一段程序文本按给定配置完成静态审查。"""
    return ReviewEngine(config).review(text, filename=filename)
