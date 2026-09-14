"""断点续跑审查：重放恢复点之前的程序，重建续跑所依赖的模态状态。

换刀、断刀检测或 M00/M01 临时停机后，操作员若直接从某个程序段续跑，上电/跨程序
默认模态（G54、G49、G40、进给、主轴、冷却等）与“该段原本执行时已生效的模态”
可能不一致。本模块：

1. 解析整份程序，把恢复行之前的程序段（前缀）交给 :class:`ReviewEngine` 重放，
   逐段记录单位、平面、G90/G91、G54–G59、刀具/H/D、进给、主轴、冷却等模态的
   来源行（逐段依据）；
2. 给出恢复行依赖的状态清单、与可选实测机床坐标的状态差异；
3. 按“先退刀（Z 向抬到安全平面）→ 再 XY 定位 → 最后进给接近切入点”的最短
   顺序生成恢复前导段（G-code 文本）；
4. 把前导段 + 恢复行起的剩余程序拼回完整程序，交回现有审查引擎复核，
   返回逐段依据、越界与碰撞风险。

位置未知（G28/G30 回零后）、固定循环仍活动、刀径补偿 G41/G42 激活、G92 临时
偏置未清、运动/单位/进给模态无法确定等情形下**不生成可执行前导段**，只返回
阻断原因与需要补充的信息。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from .config import (WORK_OFFSET_KEYS, normalize_config, cutter_radius,
                     tool_length)
from .engine import ReviewEngine
from .parser import parse_program

_AXES = ("x", "y", "z")
_INCH = 25.4

# 探针字段 -> (依赖 field 名, 中文标签)
_PROBE_FIELDS: Dict[str, Tuple[str, str]] = {
    "units": ("units", "单位 G20/G21"),
    "distance_mode": ("distance_mode", "绝对/增量模式 G90/G91"),
    "plane": ("plane", "加工平面 G17/G18/G19"),
    "feed_mode": ("feed_mode", "进给模式 G93/G94/G95"),
    "motion": ("motion_mode", "运动模态 G00/G01/G02/G03"),
    "cutter_mode": ("cutter_mode", "刀径补偿 G40/G41/G42"),
    "tool_length_mode": ("tool_length_mode", "刀长补偿 G43/G44/G49"),
    "work_offset": ("work_offset", "工件坐标系 G54–G59"),
    "return_mode": ("return_mode", "固定循环返回平面 G98/G99"),
    "canned": ("canned", "固定循环模态"),
    "spindle": ("spindle", "主轴状态 M03/M04/M05"),
    "spindle_rpm": ("spindle_rpm", "主轴转速 S"),
    "coolant": ("coolant", "冷却 M07/M08/M09"),
    "feed_native": ("feed_native", "进给 F"),
    "tool_no": ("tool_no", "刀号 T"),
    "h_code": ("h_code", "刀长补偿号 H"),
    "d_code": ("d_code", "刀径补偿号 D"),
}

# 阻断代码 -> 中文处置提示
_BLOCKER_NEED = {
    "RESUME_POSITION_UNKNOWN":
        "提供停机时机床实测坐标 measured_position（机床坐标 x/y/z，mm），"
        "或改选 G28/G30 回零之前、位置确定的程序段恢复",
    "RESUME_CANNED_ACTIVE":
        "先手动 G80 取消固定循环，或选择 G80（或 01 组运动）之后的程序段恢复",
    "RESUME_CUTTER_COMP_ACTIVE":
        "在 G40 取消刀径补偿后的程序段恢复，或手动完成补偿引入/退出动作后再续跑",
    "RESUME_G92_SHIFT_ACTIVE":
        "选择 G92.1 清零之后的程序段恢复，或手动恢复坐标系并核对实测位置",
    "RESUME_MOTION_MODE_UNKNOWN":
        "在恢复行显式写出 G00/G01/G02/G03，或在其之前补一行明确的运动模态",
    "RESUME_ARC_MODAL_AMBIGUOUS":
        "圆弧段续跑需在恢复行显式写出 G02/G03 及终点与 I/J/K（或 R）",
    "RESUME_UNITS_UNDECLARED":
        "确认程序单位并在恢复行之前补写 G21（毫米）或 G20（英寸）",
    "RESUME_TOOL_MISSING":
        "先完成换刀（Tn M06）并在机床配置中登记刀具，或选择换刀之后的程序段恢复",
    "RESUME_TOOL_UNDEFINED":
        "在机床配置 tools 中补登该刀的 length/diameter 后重新审查",
    "RESUME_H_MISSING":
        "在恢复行之前补写 G43/G44 H 号，或改选刀长补偿明确建立后的程序段",
    "RESUME_H_UNDEFINED":
        "在机床配置 h_offsets（或同号刀具 length）中补登该 H 补偿值",
    "RESUME_FEED_MISSING":
        "在恢复行之前给出有效 F 进给（当前单位下），或手动控制接近进给",
    "RESUME_G95_RPM_MISSING":
        "G95 每转进给需要 S 转速：补写 S 值或手动设定主轴转速后续跑",
    "RESUME_G93_FEED_UNDETERMINED":
        "G93 反比时间进给的接近段时间无法静态确定：改选给出 F 的程序段，"
        "或手动完成接近",
    "RESUME_PREFIX_ERRORS":
        "修正恢复点之前程序中的上述错误后再申请续跑审查",
    "RESUME_MEASURED_OUT_OF_ENVELOPE":
        "核对实测坐标或机床配置行程；实测点本身已在行程外，禁止自动退刀",
}


class ResumeValidationError(ValueError):
    """请求级参数错误（恢复行非法、实测坐标非数值等），对应 HTTP 400。"""


# ---------------------------------------------------------------------- #
# 公共入口
# ---------------------------------------------------------------------- #
def review_resume(content: str, config: Dict[str, Any], resume_line: int,
                  measured_position: Optional[Dict[str, Any]] = None,
                  position_tolerance: float = 0.01,
                  filename: Optional[str] = None) -> Dict[str, Any]:
    """断点续跑审查主入口。

    :param content: 完整 G-code 程序文本（必须包含恢复行及其前缀）
    :param config: 机床配置（mm），与即时审查接口同一结构
    :param resume_line: 计划恢复执行的物理行号（从 1 开始）
    :param measured_position: 可选实测机床坐标 {x,y,z}（mm）
    :param position_tolerance: 实测位置与重放位置的一致性告警容差（mm）
    """
    cfg = normalize_config(config)
    raw_lines = content.splitlines()
    blocks, parse_diags = parse_program(content)

    resume_block = _validate_resume_line(resume_line, raw_lines, blocks)
    idx = next(i for i, b in enumerate(blocks) if b.line_no == resume_line)
    prefix_blocks = blocks[:idx]
    resume_idx0 = resume_line - 1

    # ---- 1) 重放恢复点之前的程序，逐段记录模态来源 ---- #
    eng = ReviewEngine(cfg)
    eng._init_position()
    history, origins = [], {}

    def _probe() -> Dict[str, Any]:
        p = {attr: getattr(eng, attr, None) for attr in _PROBE_FIELDS}
        p["coolant_dyn"] = getattr(eng, "coolant", None)
        p["g52"] = dict(eng.g52)
        p["g92_shift"] = dict(eng.g92_shift)
        return p

    prev = _probe()
    for block in prefix_blocks:
        eng._process_block(block)
        cur = _probe()
        changes = _diff_probe(prev, cur)
        if "M06" in block.m_codes:
            changes.append(("tool_change", "实际换刀 M06", None, eng.tool_no))
        if changes:
            entry = {"line_no": block.line_no, "source": block.source,
                     "changes": [
                         {"field": f, "label": lab, "old": _jsonable(o),
                          "new": _jsonable(n)} for f, lab, o, n in changes]}
            history.append(entry)
            for f, _lab, _o, _n in changes:
                origins[f] = entry
        prev = cur

    # 恢复行自身若显式给出单位/取消补偿，则恢复帧在本行即被确定。
    # 注意：引擎重放时若首段运动前缺单位会“假定毫米”，这里必须另行确认前缀/恢复行
    # 是否**显式**出现过 G20/G21，否则单位模态对续跑仍是不可确定的。
    # 单位：以 G20/G21 显式出现为准；引擎在缺单位首段运动时会假定毫米，
    # 对续跑而言该假定不成立，恢复为“未指定”
    units_explicit = any(g in ("G20", "G21") for b in prefix_blocks for g in b.g_codes) \
        or any(g in ("G20", "G21") for g in resume_block.g_codes)
    if not units_explicit:
        eng.units = None
    else:
        for g in resume_block.g_codes:
            if g in ("G20", "G21") and eng.units is None:
                eng.units = "inch" if g == "G20" else "mm"

    # 解析诊断按物理行过滤到前缀（解析器是整份程序解析的）
    prefix_parse_diags = [d for d in parse_diags if d.line_no < resume_line]
    prefix_diags = prefix_parse_diags + list(eng.diags)
    prefix_errors = [d for d in prefix_diags if d.severity == "error"]

    # ---- 2) 恢复行所依赖的状态 ---- #
    entry_work = dict(eng.work_pos)
    expected_machine = eng._to_machine(entry_work)
    measured = _validate_measured(measured_position)
    position_diffs, _measured_seed = _position_diffs(
        expected_machine, measured, float(position_tolerance))

    dependencies = _build_dependencies(eng, cfg, origins, expected_machine,
                                       entry_work)

    # ---- 3) 阻断判定 ---- #
    blockers: List[Dict[str, Any]] = []

    def block(code: str, reason: str, related: Optional[int] = None) -> None:
        blockers.append({
            "code": code,
            "reason": reason,
            "need": _BLOCKER_NEED.get(code, ""),
            "related_line_no": related,
        })

    if prefix_errors:
        block("RESUME_PREFIX_ERRORS",
              f"恢复点之前的程序重放出 {len(prefix_errors)} 个 error 级问题，"
              "续跑依据不可信："
              + "；".join(f"第 {d.line_no} 行 {d.message}" for d in prefix_errors))
    if not eng.known_pos:
        block("RESUME_POSITION_UNKNOWN",
              "前缀程序以 G28/G30 回参考点结束，参考点坐标不在机床配置中，"
              "恢复行执行前的轴位置无法静态确定",
              related=_last_unknown_line(prefix_blocks))
    if not units_explicit:
        block("RESUME_UNITS_UNDECLARED",
              "恢复行之前（含恢复行自身）未出现 G20/G21，单位模态不确定，"
              "所有坐标与进给的物理尺度无法确定")
    if eng.canned is not None:
        cancel_in_line = ("G80" in resume_block.g_codes
                          or any(g in ("G00", "G01", "G02", "G03")
                                 for g in resume_block.g_codes))
        if not cancel_in_line:
            block("RESUME_CANNED_ACTIVE",
                  f"固定循环 {eng.canned} 仍为模态：恢复行一旦带轴字就会触发钻孔动作，"
                  "其内部退刀/接近序列依赖循环参数与 R 点状态",
                  related=origins.get("canned", {}).get("line_no"))
    if eng.cutter_mode in ("G41", "G42") and "G40" not in resume_block.g_codes:
        block("RESUME_CUTTER_COMP_ACTIVE",
              f"刀径补偿 {eng.cutter_mode} D{eng.d_code or '?'} 已激活：续跑首段的"
              "补偿矢量取决于此前整段轮廓的偏置建立过程，无法从前导段可靠重建",
              related=origins.get("cutter_mode", {}).get("line_no"))
    if any(abs(v) > 1e-9 for v in eng.g92_shift.values()) \
            and "G92.1" not in resume_block.g_codes:
        block("RESUME_G92_SHIFT_ACTIVE",
              "G92 设定的临时坐标偏置仍生效：该偏置依赖设定时机床的实际位置，"
              "跨停机续跑无法保证其仍然成立",
              related=origins.get("g92_shift", {}).get("line_no"))
    # 恢复行自身是否显式给出运动代码
    line_motion = next((g for g in resume_block.g_codes
                        if g in ("G00", "G01", "G02", "G03")), None)
    line_has_axes = any(w in resume_block.words for w in ("X", "Y", "Z"))
    if line_motion is None and line_has_axes and eng.canned is None:
        if eng.motion in ("G02", "G03"):
            block("RESUME_ARC_MODAL_AMBIGUOUS",
                  "恢复行带轴字但未显式给出 G02/G03，继承的运动模态是圆弧："
                  "续跑起点处的 I/J/K 模态不确定，圆弧无法重建")
        elif eng.motion is None:
            block("RESUME_MOTION_MODE_UNKNOWN",
                  "恢复行带轴字但未显式给出 G00/G01/G02/G03，此前也未建立运动模态，"
                  "01 组上电状态不确定")
    if eng.tool_no is None:
        block("RESUME_TOOL_MISSING",
              "前缀程序未建立当前刀具（无 T/M06），刀长、刀径与碰撞模型均无法确定")
    elif str(eng.tool_no) not in cfg.get("tools", {}):
        block("RESUME_TOOL_UNDEFINED",
              f"当前刀具 T{eng.tool_no} 未在机床配置中登记，"
              "长度/直径未知，无法核对行程与碰撞",
              related=origins.get("tool_change", origins.get("tool_no", {})
                                  ).get("line_no"))
    # H 刀长：恢复行若以 G49 取消补偿则恢复帧回到无补偿
    if eng.tool_length_mode in ("G43", "G44") \
            and "G49" not in resume_block.g_codes:
        if eng.h_code is None:
            block("RESUME_H_MISSING",
                  f"刀长补偿 {eng.tool_length_mode} 激活但无 H 号，"
                  "控制器通常报警且物理 z 可能差整个刀长",
                  related=origins.get("tool_length_mode", {}).get("line_no"))
        elif not _h_defined(cfg, eng.h_code, eng.tool_no):
            block("RESUME_H_UNDEFINED",
                  f"H{eng.h_code} 在 h_offsets 与同号刀具表中均无定义，"
                  "有效刀长未知，z 向轨迹不可信",
                  related=origins.get("h_code", {}).get("line_no"))

    # ---- 实测位置：差异、包络、作为前导段起点 ---- #
    if measured is not None:
        env = cfg["envelope"]
        for ax in _AXES:
            if measured[ax] < float(env[f"{ax}min"]) - 1e-9 \
                    or measured[ax] > float(env[f"{ax}max"]) + 1e-9:
                block("RESUME_MEASURED_OUT_OF_ENVELOPE",
                      f"实测机床坐标 {ax.upper()}={measured[ax]:g}mm 超出机床行程 "
                      f"[{env[f'{ax}min']:g}, {env[f'{ax}max']:g}]mm")
    start_machine = measured if measured is not None else expected_machine

    # ---- 4) 生成前导段（无阻断时）---- #
    preamble: Dict[str, Any] = {"generated": False}
    prerequisites: List[str] = list(eng.assumptions)
    warnings: List[str] = []
    review: Optional[Dict[str, Any]] = None

    if "M06" in resume_block.m_codes:
        warnings.append(
            f"恢复行（第 {resume_line} 行）本身是换刀行：前导段不会替操作员换刀，"
            "续跑前确认刀库/刀位状态安全")
    if any(m in resume_block.m_codes for m in ("M00", "M01")):
        warnings.append(f"恢复行（第 {resume_line} 行）为计划停止，无轴运动")

    if blockers:
        preamble = {"generated": False, "lines": [], "segments": [],
                    "text": "", "blocked_codes": [b["code"] for b in blockers]}
    else:
        # 实测位置先换算到恢复帧，再注入引擎作为当前点
        measured_work = eng._to_work(start_machine)
        eng.work_pos = measured_work
        eng.known_pos = True

        approach_needed = entry_work["z"] < float(cfg["safety_clearance"]) - 1e-9
        # 接近段进给可行性（必须在生成前判定）
        if approach_needed:
            if eng.feed_mode == "G93":
                block("RESUME_G93_FEED_UNDETERMINED",
                      "G93 反比时间进给下接近段的 F 语义依赖程序段时长，"
                      "无法为合成接近段确定安全进给")
            elif eng.feed_native is None:
                block("RESUME_FEED_MISSING",
                      "恢复模态中没有有效 F，切入点低于安全平面需要 G01 进给接近，"
                      "进给量无法确定")
            elif eng.feed_mode == "G95" and not (eng.spindle_rpm or 0) > 0:
                block("RESUME_G95_RPM_MISSING",
                      "G95 每转进给但无有效 S 转速，接近进给 mm/min 无法换算")

        if blockers:
            preamble = {"generated": False, "lines": [], "segments": [],
                        "text": "",
                        "blocked_codes": [b["code"] for b in blockers]}
        else:
            preamble = _build_preamble(
                eng, cfg, entry_work, measured_work, resume_block)
            # 人工确认项（即便生成可执行前导段也必须给出）
            prerequisites += _prerequisites(
                eng, cfg, measured is not None, position_diffs,
                approach_needed, preamble)
            # ---- 5) 前导段 + 剩余程序交回现有审查引擎复核 ---- #
            suffix_text = "\n".join(raw_lines[resume_idx0:])
            # 不再额外插空行：拼接后第 N+1 行恰为原程序的 resume_line 行
            combined = preamble["text"] + "\n" + suffix_text
            review_cfg = dict(cfg)
            review_cfg["start_position"] = {a: start_machine[a] for a in _AXES}
            raw_report = ReviewEngine(review_cfg).review(
                combined, filename=filename or "resume")
            review = _remap_review(raw_report, preamble["line_count"], resume_line)
            _attach_review_moves(preamble, review)

    # ---- 6) 汇总 ---- #
    resume_dep_warnings = [
        f"恢复行为 {resume_block.source.strip()!r}，续跑前请逐行核对上述模态"]
    return {
        "filename": filename or "inline",
        "resume_line": resume_line,
        "resume_source": resume_block.source,
        "resumable": not blockers,
        "blockers": blockers,
        "dependencies": dependencies,
        "state_history": history,
        "state_differences": {
            "measured_position_provided": measured is not None,
            "position_tolerance_mm": float(position_tolerance),
            "expected_machine": _round3(expected_machine),
            "measured_machine": _round3(measured),
            "axes": position_diffs,
        },
        "warnings": warnings + resume_dep_warnings,
        "prerequisites": _dedup(prerequisites),
        "preamble": preamble,
        "prefix_review": {
            "diagnostic_counts": _counts(prefix_diags),
            "diagnostics": [d.to_dict() for d in prefix_diags],
        },
        "review": review,
        "verdict": _verdict(blockers, review, preamble),
    }


# ---------------------------------------------------------------------- #
# 参数校验
# ---------------------------------------------------------------------- #
def _validate_resume_line(resume_line, raw_lines, blocks) -> Any:
    if not isinstance(resume_line, int) or isinstance(resume_line, bool) \
            or resume_line < 1:
        raise ResumeValidationError("resume_line 必须是从 1 开始的正整数行号")
    if resume_line > len(raw_lines):
        raise ResumeValidationError(
            f"resume_line={resume_line} 超出程序总行数 {len(raw_lines)}")
    target = next((b for b in blocks if b.line_no == resume_line), None)
    if target is None:
        near = [b.line_no for b in blocks if abs(b.line_no - resume_line) <= 5]
        hint = f"；附近的有效程序段行号：{near}" if near else "；程序中没有有效程序段"
        raise ResumeValidationError(
            f"第 {resume_line} 行为空行或纯注释，不含可执行语句{hint}")
    return target


def _validate_measured(measured: Any) -> Optional[Dict[str, float]]:
    if measured is None:
        return None
    if not isinstance(measured, dict):
        raise ResumeValidationError("measured_position 必须是含 x/y/z 的对象")
    out = {}
    for ax in _AXES:
        v = measured.get(ax)
        if not isinstance(v, (int, float)) or isinstance(v, bool) \
                or not math.isfinite(float(v)):
            raise ResumeValidationError(
                f"measured_position.{ax} 必须是有限数值（拒绝 NaN/无穷）")
        out[ax] = float(v)
    return out


# ---------------------------------------------------------------------- #
# 状态探针 / 依赖清单
# ---------------------------------------------------------------------- #
def _diff_probe(prev: Dict[str, Any], cur: Dict[str, Any]) -> List[tuple]:
    changes = []
    for attr, (field, label) in _PROBE_FIELDS.items():
        if attr == "coolant":
            old, new = prev["coolant_dyn"], cur["coolant_dyn"]
        else:
            old, new = prev[attr], cur[attr]
        if old != new:
            changes.append((field, label, old, new))
    if prev["g52"] != cur["g52"]:
        changes.append(("g52", "局部坐标偏置 G52",
                        dict(prev["g52"]), dict(cur["g52"])))
    if prev["g92_shift"] != cur["g92_shift"]:
        changes.append(("g92_shift", "G92 临时坐标偏置",
                        dict(prev["g92_shift"]), dict(cur["g92_shift"])))
    return changes


def _origin(origins: Dict[str, Any], key: str) -> Optional[Dict[str, Any]]:
    e = origins.get(key)
    if not e:
        return None
    return {"line_no": e["line_no"], "source": e["source"]}


def _build_dependencies(eng: ReviewEngine, cfg: Dict[str, Any],
                        origins: Dict[str, Any],
                        machine_pos: Dict[str, float],
                        work_pos: Dict[str, float]) -> List[Dict[str, Any]]:
    deps: List[Dict[str, Any]] = []

    def add(field, label, value, origin_key=None, importance="required",
            detail="", note=""):
        deps.append({"field": field, "label": label, "value": _jsonable(value),
                     "origin": _origin(origins, origin_key) if origin_key else None,
                     "importance": importance, "detail": detail, "note": note})

    add("units", "单位", eng.units or "未指定",
        "units", importance=("required" if eng.units else "warning"),
        note="G21=毫米/G20=英寸；内部统一换算为 mm" if eng.units
        else "前缀未声明单位，续跑尺度无依据")
    add("distance_mode", "绝对/增量模式", eng.distance_mode, "distance_mode")
    add("plane", "加工平面", eng.plane, "plane", importance="info")
    wo = eng.work_offset
    off = cfg["work_offsets"].get(wo, {})
    add("work_offset", "工件坐标系", wo, "work_offset",
        detail=f"零点偏置（机床坐标 mm）：X{_fmt(off.get('x'))} "
               f"Y{_fmt(off.get('y'))} Z{_fmt(off.get('z'))}")
    # 刀具
    tool = cfg.get("tools", {}).get(str(eng.tool_no)) if eng.tool_no else None
    add("tool", "当前刀具",
        (f"T{eng.tool_no}" if eng.tool_no else "无刀具"),
        "tool_change" if origins.get("tool_change") else "tool_no",
        importance=("required" if tool else "warning"),
        detail=("刀长 {:g}mm、直径 {:g}mm{}".format(
            float(tool.get("length", 0.0)), float(tool.get("diameter", 0.0)),
            f"（{tool.get('description')}）" if tool.get("description") else "")
            if tool else "该刀未在机床配置中登记"),
        note="前导段不发 M06：续跑前必须确认主轴上确为该刀")
    # H 刀长
    if eng.tool_length_mode in ("G43", "G44"):
        length = eng._active_length()
        add("tool_length_comp", "刀长补偿",
            f"{eng.tool_length_mode} H{eng.h_code or '?'}", "h_code",
            detail=(f"生效长度 {_fmt(length)}mm（G44 取负）"
                    + ("" if _h_defined(cfg, eng.h_code, eng.tool_no)
                       else "；H 值未定义，按 0 处理")),
            importance=("required"
                        if _h_defined(cfg, eng.h_code, eng.tool_no)
                        else "warning"))
    else:
        add("tool_length_comp", "刀长补偿", "G49（取消）",
            "tool_length_mode", importance="info",
            note="恢复帧无刀长补偿；程序若随后 G43 需重新建立")
    # D 刀径
    if eng.cutter_mode in ("G41", "G42"):
        add("cutter_comp", "刀径补偿",
            f"{eng.cutter_mode} D{eng.d_code or '?'}", "cutter_mode",
            detail=f"生效半径 {_fmt(cutter_radius(cfg, eng.d_code, eng.tool_no))}mm",
            importance="warning",
            note="G41/G42 激活态不可直接续跑（见 blockers）")
    else:
        add("cutter_comp", "刀径补偿", "G40（取消）", "cutter_mode",
            importance="info")
    # 进给
    feed_mm_min = None
    if eng.feed_native is not None:
        if eng.feed_mode == "G94":
            feed_mm_min = eng._to_mm(eng.feed_native)
        elif eng.feed_mode == "G95" and (eng.spindle_rpm or 0) > 0:
            feed_mm_min = eng._to_mm(eng.feed_native) * eng.spindle_rpm
    add("feed_mode", "进给模式", eng.feed_mode, "feed_mode", importance="info")
    add("feed", "进给 F",
        (eng.feed_native if eng.feed_native is not None else "未指定"),
        "feed_native",
        importance=("info" if eng.feed_native is not None else "warning"),
        detail=(f"约合 {_fmt(feed_mm_min)} mm/min"
                if feed_mm_min is not None
                else ("G95 且无 S，mm/min 不可换算"
                      if eng.feed_mode == "G95" else "接近段需要 F 时将阻断")))
    # 主轴 / 冷却
    add("spindle", "主轴状态",
        f"{eng.spindle or '停止'}"
        + (f" S{_fmt(eng.spindle_rpm)}" if eng.spindle_rpm else ""),
        "spindle", importance=("required" if eng.spindle in ("M03", "M04")
                               else "warning"),
        note="前导段会重发 M03/M04 与 S，但停机后必须实际确认主轴已按要求旋转")
    coolant = getattr(eng, "coolant", None)
    add("coolant", "冷却", coolant or "关闭（M09）", "coolant",
        importance="info",
        note="前导段会在接近前重发 M07/M08" if coolant else "")
    add("return_mode", "固定循环返回平面", eng.return_mode, "return_mode",
        importance="info")
    if any(abs(v) > 1e-9 for v in eng.g52.values()):
        add("g52", "局部坐标偏置 G52",
            {a: round(eng.g52[a], 6) for a in _AXES}, "g52",
            importance="warning",
            note="前导段会重发 G52；跨设备标定时易被遗忘")
    if any(abs(v) > 1e-9 for v in eng.g92_shift.values()):
        add("g92_shift", "G92 临时坐标偏置",
            {a: round(eng.g92_shift[a], 6) for a in _AXES}, "g92_shift",
            importance="warning", note="该偏置下不可直接续跑（见 blockers）")
    add("entry_position", "恢复行切入点（工件坐标 mm）",
        _round3(work_pos), importance="required",
        note="即恢复行执行前的刀位点；前导段以此为接近终点")
    add("entry_machine_position", "恢复行切入点（机床坐标 mm）",
        _round3(machine_pos), importance="required",
        detail="工件零点 + G52 + 刀长补偿换算所得")
    return deps


def _h_defined(cfg: Dict[str, Any], h: Optional[str],
               tool_no: Optional[str]) -> bool:
    if h is None:
        return False
    if h in cfg.get("h_offsets", {}):
        return True
    return tool_no is not None and h == str(tool_no) \
        and str(tool_no) in cfg.get("tools", {})


# ---------------------------------------------------------------------- #
# 状态差异
# ---------------------------------------------------------------------- #
def _position_diffs(expected: Dict[str, float],
                    measured: Optional[Dict[str, float]],
                    tol: float) -> Tuple[List[Dict[str, Any]], bool]:
    if measured is None:
        return [], False
    out = []
    for ax in _AXES:
        delta = measured[ax] - expected[ax]
        out.append({
            "axis": ax.upper(),
            "expected_mm": round(expected[ax], 6),
            "measured_mm": round(measured[ax], 6),
            "delta_mm": round(delta, 6),
            "within_tolerance": abs(delta) <= tol + 1e-12,
        })
    return out, True


# ---------------------------------------------------------------------- #
# 前导段生成
# ---------------------------------------------------------------------- #
def _build_preamble(eng: ReviewEngine, cfg: Dict[str, Any],
                    entry_work: Dict[str, float],
                    current_work: Dict[str, float],
                    resume_block) -> Dict[str, Any]:
    """生成 退刀 → 定位 →（辅助）→ 接近 的最短前导段。

    所有坐标按恢复帧（工件坐标系 + G52 + 当前刀长补偿）以绝对方式给出，
    结束前按需恢复 G91。返回文本与逐段元数据。
    """
    units = eng.units or "mm"

    def dim(v: float) -> str:
        v = v / _INCH if units == "inch" else v
        return f"{v:.6g}"

    clearance = float(cfg["safety_clearance"])
    z_safe = max(clearance, entry_work["z"])

    lines: List[str] = []
    segments: List[Dict[str, Any]] = []

    def emit(text: str, phase: str, purpose: str) -> int:
        lines.append(text)
        line_no = len(lines)
        segments.append({"phase": phase, "line_no": line_no, "source": text,
                         "purpose": purpose})
        return line_no

    emit("(resume preamble: 由断点续跑审查合成，执行前必须逐行人工核对)",
         "comment", "前导段说明（注释不执行）")
    # 模态头：单位/强制绝对/平面/工件坐标系
    g_unit = "G20" if units == "inch" else "G21"
    emit(f"{g_unit} G90 {eng.plane} {eng.work_offset}", "modal",
         f"重建单位/绝对模式/平面 {eng.plane}/工件坐标系 {eng.work_offset}")
    # 当前刀具预选（不发 M06；复核引擎据此选刀长/刀径与碰撞模型）
    if eng.tool_no is not None:
        emit(f"T{eng.tool_no}", "modal",
             f"声明当前刀具 T{eng.tool_no}（不换刀；续跑前人工确认主轴上确为该刀）")
    # G52 局部偏置（G92 偏置在阻断阶段已排除）
    if any(abs(v) > 1e-9 for v in eng.g52.values()):
        emit(f"G52 X{dim(eng.g52['x'])} Y{dim(eng.g52['y'])} "
             f"Z{dim(eng.g52['z'])}", "modal",
             "重建前缀中生效的 G52 局部坐标偏置")
    # 刀长补偿（物理位置不动，仅重建补偿帧）
    if eng.tool_length_mode in ("G43", "G44") and eng.h_code is not None:
        emit(f"{eng.tool_length_mode} H{eng.h_code}", "modal",
             f"重建刀长补偿 {eng.tool_length_mode} H{eng.h_code}（不发 M06，"
             "确认当前刀具与补偿值）")
    # 进给模式 + 当前 F（供恢复行/接近段继承）。
    # 注意 feed_native 是当前单位下的原生值（G94 in/min、G95 in/rev），不做 mm 换算。
    def _num(v: float) -> str:
        return f"{v:.6g}"

    feed_line = eng.feed_mode
    if eng.feed_native is not None:
        feed_line += f" F{_num(eng.feed_native)}"
    emit(feed_line, "modal",
         f"重建进给模式 {eng.feed_mode}"
         + (f" 与 F{dim(eng.feed_native)}" if eng.feed_native is not None else ""))
    # 固定循环返回平面模态
    if eng.return_mode != "G98":
        emit(eng.return_mode, "modal", f"重建固定循环返回平面模态 {eng.return_mode}")

    # 1) 退刀：先 Z 向垂直抬到安全高度（安全平面以上无低空快移风险）
    if abs(current_work["z"] - z_safe) > 1e-9:
        higher = current_work["z"] < z_safe
        emit(f"G00 Z{dim(z_safe)}", "retract",
             ("垂直退刀到安全平面 "
              if higher else "Z 向调整到切入点高度（切入点本身在安全平面之上）")
             + f"z={dim(z_safe)}")
    # 2) 定位：安全高度上 XY 联动到切入点正上方
    if abs(current_work["x"] - entry_work["x"]) > 1e-9 \
            or abs(current_work["y"] - entry_work["y"]) > 1e-9:
        emit(f"G00 X{dim(entry_work['x'])} Y{dim(entry_work['y'])}",
             "position", f"安全平面高度上定位到切入点 XY 正上方")
    # 3) 辅助模态：主轴 / 冷却（定位完成、接近之前）
    if eng.spindle in ("M03", "M04"):
        s = f" S{_num(eng.spindle_rpm)}" if eng.spindle_rpm else ""
        emit(f"{eng.spindle}{s}", "auxiliary",
             f"恢复主轴 {eng.spindle}{('，转速 ' + _fmt(eng.spindle_rpm) + ' rpm') if eng.spindle_rpm else ''}"
             "；确认机床主轴实际已旋转且转向正确")
    coolant = getattr(eng, "coolant", None)
    if coolant:
        emit(coolant, "auxiliary", f"恢复冷却 {coolant}")
    # 4) 接近：仅当切入点低于安全平面时，以进给速度垂直下到切入点
    approach_needed = entry_work["z"] < z_safe - 1e-9
    if approach_needed:
        f_word = f" F{_num(eng.feed_native)}" if eng.feed_native is not None else ""
        emit(f"G01 Z{dim(entry_work['z'])}{f_word}", "approach",
             f"以当前 {eng.feed_mode} 进给"
             + (f"（F{_num(eng.feed_native)}）" if eng.feed_native is not None else "")
             + "垂直接近切入点；必须手动确认残料/夹具净空")
    # 5) 恢复增量模态（若恢复帧本为 G91，续跑行可能依赖该模态）
    if eng.distance_mode == "G91":
        emit("G91", "modal", "恢复前缀生效的增量编程模式，供剩余程序继承")

    return {
        "generated": True,
        "text": "\n".join(lines),
        "lines": lines,
        "line_count": len(lines),
        "segments": segments,
        "motion_phase_count": sum(
            1 for s in segments if s["phase"] in ("retract", "position", "approach")),
        "approach_needed": approach_needed,
        "entry_work": _round3(entry_work),
        "z_safe_work": round(z_safe, 6),
        "current_work": _round3(current_work),
        "blocked_codes": [],
    }


def _prerequisites(eng: ReviewEngine, cfg: Dict[str, Any], measured_given: bool,
                   diffs: List[Dict[str, Any]], approach_needed: bool,
                   preamble: Dict[str, Any]) -> List[str]:
    items = [
        f"确认主轴上安装的是 T{eng.tool_no}，且 H{eng.h_code or '-'}/"
        f"D{eng.d_code or '-'} 与机床配置一致（前导段不含换刀动作）",
    ]
    if measured_given:
        bad = [d for d in diffs if not d["within_tolerance"]]
        if bad:
            items.append(
                "实测位置与程序重放位置不一致（"
                + "，".join(f"{d['axis']} 偏差 {d['delta_mm']:g}mm" for d in bad)
                + "）：前导段已按实测点重算，确认停机未发生碰撞/丢步")
        else:
            items.append("实测位置与程序重放位置一致（容差内），可按前导段续跑")
    else:
        items.append("未提供实测机床坐标：续跑前必须手动回读各轴位置，"
                     "确认其与重放切入点一致，否则禁止执行前导段")
    if approach_needed:
        items.append("接近段为 G01 垂直下刀：单段/手轮方式执行，确认切入处残料、"
                     "台阶与夹具净空后再恢复自动")
    if eng.spindle not in ("M03", "M04"):
        items.append("恢复模态中主轴为停止：前导段不会凭空启动主轴，"
                     "若恢复行进行切削，先手动 M03/M04 S___ 并确认转速")
    elif not eng.spindle_rpm:
        items.append("主轴旋转但无 S 转速记录，手动确认当前转速")
    if not getattr(eng, "coolant", None):
        items.append("冷却为关闭状态，按工艺需要手动开启")
    if any(abs(v) > 1e-9 for v in eng.g52.values()):
        items.append("G52 局部偏置仍生效（前导段已重发），确认该偏置与当前装夹一致")
    if preamble.get("motion_phase_count") == 0:
        items.append("当前点与切入点重合且位于安全平面之上：前导段仅重建模态，无轴运动")
    return items


# ---------------------------------------------------------------------- #
# 复核结果回填
# ---------------------------------------------------------------------- #
def _remap_review(report: Dict[str, Any], preamble_lines: int,
                  resume_line: int) -> Dict[str, Any]:
    """给复核报告的诊断/运动段标注来自前导段还是剩余程序，并还原原始行号。"""
    def region_of(line_no: Optional[int]) -> Tuple[str, Optional[int]]:
        if line_no is None:
            return "preamble", None
        if line_no <= preamble_lines:
            return "preamble", line_no
        # 拼接后第 N+1 行 = 原程序第 resume_line 行
        return "suffix", resume_line + line_no - preamble_lines - 1

    for d in report["diagnostics"]:
        region, orig = region_of(d.get("line_no"))
        d["region"] = region
        d["orig_line_no"] = orig
    for m in report["moves"]:
        region, orig = region_of(m.get("line_no"))
        m["region"] = region
        m["orig_line_no"] = orig

    pre_diags = [d for d in report["diagnostics"] if d["region"] == "preamble"]
    suf_diags = [d for d in report["diagnostics"] if d["region"] == "suffix"]
    pre_moves = [m for m in report["moves"] if m["region"] == "preamble"]
    return {
        "risk": report["risk"],
        "diagnostic_counts": report["diagnostic_counts"],
        "preamble_diagnostic_counts": _counts(pre_diags),
        "suffix_diagnostic_counts": _counts(suf_diags),
        "diagnostics": report["diagnostics"],
        "trajectory": report["trajectory"],
        "collisions": report["collisions"],
        "time_estimate_s": report["time_estimate_s"],
        "preamble_segments": len(pre_moves),
        "suffix_segments": report["trajectory"]["segments"] - len(pre_moves),
        "moves": report["moves"],
        "assumptions": report["assumptions"],
    }


def _attach_review_moves(preamble: Dict[str, Any],
                         review: Dict[str, Any]) -> None:
    """把复核引擎重建出的前导段运动（物理坐标/耗时/进给）回填到前导段。"""
    by_line: Dict[int, List[Dict[str, Any]]] = {}
    for m in review["moves"]:
        if m["region"] == "preamble":
            by_line.setdefault(m["line_no"], []).append(m)
    for seg in preamble["segments"]:
        ms = by_line.get(seg["line_no"], [])
        if not ms:
            seg.update({"move_indices": [], "moves": []})
            continue
        seg["move_indices"] = [m["index"] for m in ms]
        seg["moves"] = [{
            "index": m["index"], "kind": m["kind"],
            "start_machine": m["start_machine"], "end_machine": m["end_machine"],
            "feed_mm_min": m["feed_mm_min"], "duration_s": m["duration_s"],
            "synthetic": m["synthetic"], "note": m["note"],
        } for m in ms]
    preamble["segments"] = preamble["segments"]


# ---------------------------------------------------------------------- #
# 小工具
# ---------------------------------------------------------------------- #
def _last_unknown_line(prefix_blocks) -> Optional[int]:
    for b in reversed(prefix_blocks):
        if any(g in ("G28", "G30") for g in b.g_codes):
            return b.line_no
    return prefix_blocks[-1].line_no if prefix_blocks else None


def _counts(diags: List[Any]) -> Dict[str, int]:
    c = {"error": 0, "warning": 0, "info": 0}
    for d in diags:
        sev = d.severity if hasattr(d, "severity") else d.get("severity")
        c[sev] = c.get(sev, 0) + 1
    return c


def _round3(p: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
    if p is None:
        return None
    return {a: round(p[a], 6) for a in _AXES}


def _jsonable(v: Any) -> Any:
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


def _fmt(v: Optional[float]) -> str:
    if v is None:
        return "—"
    return f"{v:.4g}"


def _dedup(items: List[str]) -> List[str]:
    out, seen = [], set()
    for it in items:
        if it and it not in seen:
            seen.add(it)
            out.append(it)
    return out


def _verdict(blockers: List[Dict[str, Any]], review: Optional[Dict[str, Any]],
             preamble: Dict[str, Any]) -> str:
    if blockers:
        return (f"不可断点续跑：{len(blockers)} 项阻断，未生成可执行前导段。"
                "请按 blockers[].need 补充信息或改选恢复行")
    counts = review["diagnostic_counts"] if review else {"error": 0}
    n_seg = preamble.get("motion_phase_count", 0)
    if counts.get("error"):
        return (f"已生成前导段（{n_seg} 个运动段：退刀→定位→接近），"
                f"但复核发现 {counts['error']} 个 error 级风险，不得直接续跑，"
                "详见 review.diagnostics")
    tail = f"；复核有 {counts.get('warning', 0)} 个 warning，请按 prerequisites 人工确认" \
        if review and counts.get("warning") else "；复核未发现越界/碰撞风险"
    return f"可以断点续跑：前导段含 {n_seg} 个运动段（先退刀、再定位、最后接近切入点）{tail}"
