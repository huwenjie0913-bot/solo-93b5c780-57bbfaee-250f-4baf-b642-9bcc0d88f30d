"""机床/工件配置：默认值、校验与归一化。

配置全部以 **mm** 为单位。程序自身若用 G20（英寸），审查器在重建时把坐标和
进给换算成 mm 再与配置比较。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

WORK_OFFSET_KEYS = [f"G5{n}" for n in range(4, 10)]  # G54..G59
_AXES = ("x", "y", "z")
_ENVELOPE_KEYS = {f"{a}{b}" for a in _AXES for b in ("min", "max")}

DEFAULT_CONFIG: Dict[str, Any] = {
    "name": "default",
    "version": 1,
    # 机床行程包络（机床坐标，mm）
    "envelope": {
        "xmin": 0.0, "xmax": 500.0,
        "ymin": 0.0, "ymax": 400.0,
        "zmin": 0.0, "zmax": 450.0,
    },
    # 工件零点（各工件坐标系相对机床原点的偏置，mm）
    "work_offsets": {key: {"x": 0.0, "y": 0.0, "z": 0.0} for key in WORK_OFFSET_KEYS},
    # 安全平面（工件坐标 z，mm）：G00 快移 z 不得低于此值
    "safety_clearance": 50.0,
    # 刀具表：键为刀号字符串；length 为长度补偿几何值，diameter 为刀具直径。
    # 可选组件化描述（碰撞检测用）：flute_diameter/flute_length（刃，缺省
    # 刃径=diameter、刃长=length−刀杆−刀柄）、shank_diameter/shank_length（刀杆）、
    # holder_diameter/holder_length（刀柄）；旧版仅 length/diameter 的配置仍可用。
    "tools": {
        # "1": {"length": 100.0, "diameter": 10.0, "description": "立铣刀",
        #       "flute_length": 30.0,
        #       "shank_diameter": 10.0, "shank_length": 40.0,
        #       "holder_diameter": 45.0, "holder_length": 30.0}
    },
    # H 号 -> 长度补偿值（mm）；缺省时回退到同号刀具的 length
    "h_offsets": {},
    # D 号 -> 半径补偿值（mm）；缺省时回退到同号刀具半径
    "d_offsets": {},
    # 静态障碍物（机床坐标系，mm）：轴对齐长方体 box 或竖直圆柱 cylinder，
    # id 必须唯一。示例：
    # {"id": "vice", "type": "box",
    #  "min": {"x": 100, "y": 50, "z": 0}, "max": {"x": 200, "y": 150, "z": 80}}
    # {"id": "column", "type": "cylinder",
    #  "center": {"x": 300, "y": 200}, "radius": 25, "zmin": 0, "zmax": 120}
    "obstacles": [],
    # 碰撞检测（刀具组件扫掠体 vs 障碍物）
    "check_obstacle_collision": True,  # 是否启用障碍物干涉检查
    "collision_max_step": 2.0,         # 轨迹细分最大步长 mm
    "collision_clearance_warn": 2.0,   # 最小间隙告警阈值 mm（≤0 为相交，报 error）
    # 速度参数
    "rapid_speed": 8000.0,     # 快移速度 mm/min
    "default_feed": 500.0,     # 程序缺 F 时采用的保守进给 mm/min
    "default_spindle": 1000.0, # G95 每转进给缺 S 时采用的转速 rpm
    "tool_change_time": 8.0,   # M06 估算换刀耗时 s
    "dwell_units_seconds": False,  # True: G4 P 为秒；False: P 为毫秒（Fanuc 习惯）
    # 几何判定
    "arc_tolerance": 0.01,     # 起终点半径一致性容差 mm
    "check_tool_envelope": True,  # 行程检查是否计入刀具半径/长度
    "notes": "",
}


def normalize_config(raw: Dict[str, Any]) -> Dict[str, Any]:
    """用默认值补全并深拷贝出一份完整配置（不校验数值，仅补默认）。"""
    cfg = _deep_merge(_deepcopy_default(), raw or {})
    # 刀具/H/D 键统一成字符串，数值转 float
    cfg["tools"] = {str(k): _normalize_tool(v) for k, v in (cfg.get("tools") or {}).items()}
    cfg["h_offsets"] = {str(k): float(v) for k, v in (cfg.get("h_offsets") or {}).items()}
    cfg["d_offsets"] = {str(k): float(v) for k, v in (cfg.get("d_offsets") or {}).items()}
    cfg["obstacles"] = _normalize_obstacles(cfg.get("obstacles"))
    return cfg


def validate_config(raw: Dict[str, Any]) -> Tuple[Dict[str, Any], List[str]]:
    """校验配置。返回 (归一化配置, 错误消息列表)；列表为空表示通过。"""
    errors: List[str] = []
    cfg = normalize_config(raw)

    env = cfg["envelope"]
    for key in _ENVELOPE_KEYS:
        if not isinstance(env.get(key), (int, float)) or math.isnan(float(env[key])):
            errors.append(f"envelope.{key} 必须是数值")
    for ax in _AXES:
        lo, hi = env.get(f"{ax}min"), env.get(f"{ax}max")
        if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and lo > hi:
            errors.append(f"envelope.{ax}min({lo}) 不得大于 {ax}max({hi})")

    if not isinstance(cfg["safety_clearance"], (int, float)):
        errors.append("safety_clearance 必须是数值")
    elif cfg["safety_clearance"] < env.get("zmin", -1e18) - 1e-9:
        errors.append("safety_clearance 低于机床 zmin，安全平面无意义")

    for key in WORK_OFFSET_KEYS:
        off = cfg["work_offsets"].get(key)
        if not isinstance(off, dict) or any(a not in off for a in _AXES):
            errors.append(f"work_offsets.{key} 必须含 x/y/z")
        elif any(not isinstance(off[a], (int, float)) for a in _AXES):
            errors.append(f"work_offsets.{key} 的 x/y/z 必须是数值")

    for tnum, tool in cfg["tools"].items():
        if not isinstance(tool, dict):
            errors.append(f"tools.{tnum} 必须是对象")
            continue
        for fld in ("length", "diameter", "flute_diameter", "flute_length",
                    "shank_diameter", "shank_length",
                    "holder_diameter", "holder_length"):
            v = tool.get(fld)
            if not _ok_num(v) or v < 0:
                errors.append(f"tools.{tnum}.{fld} 必须为非负数值")
        # 刀杆/刀柄的直径与轴向长度必须成对出现
        if _pos(tool.get("shank_length")) != _pos(tool.get("shank_diameter")):
            errors.append(f"tools.{tnum} 的 shank_length 与 shank_diameter 必须同时为正（刀杆）")
        if _pos(tool.get("holder_length")) != _pos(tool.get("holder_diameter")):
            errors.append(f"tools.{tnum} 的 holder_length 与 holder_diameter 必须同时为正（刀柄）")
        # 组件轴向长度之和不得超过刀具总长（刀柄顶端即主轴端面）
        total_len = tool.get("length")
        if _ok_num(total_len):
            comp_sum = sum(v for v in (tool.get("flute_length"), tool.get("shank_length"),
                                       tool.get("holder_length")) if _ok_num(v))
            if comp_sum > total_len + 1e-6:
                errors.append(
                    f"tools.{tnum} 刃/刀杆/刀柄轴向长度之和 {comp_sum:g}mm "
                    f"超过刀具总长 length={total_len:g}mm")
        dia = tool.get("diameter")
        if _ok_num(dia) and dia > max(
            env["xmax"] - env["xmin"], env["ymax"] - env["ymin"]
        ):
            errors.append(f"tools.{tnum}.diameter 大于机床行程，配置可疑")

    # ---- 障碍物：尺寸、坐标、唯一标识 ----
    if "obstacles" in (raw or {}) and not isinstance(raw.get("obstacles"), list):
        errors.append("obstacles 必须是数组（box/cylinder 对象列表）")
    seen_obstacle_ids = set()
    for i, ob in enumerate(cfg.get("obstacles") or []):
        oid = ob.get("id") or ""
        if not oid:
            errors.append(f"obstacles[{i}].id 不能为空")
        elif oid in seen_obstacle_ids:
            errors.append(f"obstacles[{i}].id '{oid}' 重复，障碍物标识必须唯一")
        seen_obstacle_ids.add(oid)
        otype = ob.get("type")
        if otype == "box":
            for key in ("min", "max"):
                for a in _AXES:
                    if not _ok_num((ob.get(key) or {}).get(a)):
                        errors.append(f"obstacles[{i}].{key}.{a} 必须是数值坐标")
            for a in _AXES:
                lo, hi = (ob.get("min") or {}).get(a), (ob.get("max") or {}).get(a)
                if _ok_num(lo) and _ok_num(hi) and lo > hi:
                    errors.append(
                        f"obstacles[{i}] 的 {a} 轴 min({lo:g}) 大于 max({hi:g})")
        elif otype == "cylinder":
            center = ob.get("center") or {}
            for a in ("x", "y"):
                if not _ok_num(center.get(a)):
                    errors.append(f"obstacles[{i}].center.{a} 必须是数值坐标")
            if not _ok_num(ob.get("radius")) or ob.get("radius") <= 0:
                errors.append(f"obstacles[{i}].radius 必须为正数")
            for key in ("zmin", "zmax"):
                if not _ok_num(ob.get(key)):
                    errors.append(f"obstacles[{i}].{key} 必须是数值坐标")
            if _ok_num(ob.get("zmin")) and _ok_num(ob.get("zmax")) \
                    and ob["zmin"] > ob["zmax"]:
                errors.append(
                    f"obstacles[{i}] 的 zmin({ob['zmin']:g}) 大于 zmax({ob['zmax']:g})")
        else:
            errors.append(f"obstacles[{i}].type 必须是 box（轴对齐长方体）"
                          f"或 cylinder（竖直圆柱），当前为 {otype!r}")

    # ---- 碰撞检测参数 ----
    v = cfg.get("collision_max_step")
    if not _ok_num(v) or v <= 0:
        errors.append("collision_max_step 必须为正数（轨迹细分最大步长 mm）")
    elif v > 1000:
        errors.append("collision_max_step 超过 1000mm，细分失去意义")
    v = cfg.get("collision_clearance_warn")
    if not _ok_num(v) or v < 0:
        errors.append("collision_clearance_warn 必须为非负数值（最小间隙告警阈值 mm）")

    for fld in ("rapid_speed", "default_feed", "default_spindle",
                "tool_change_time", "arc_tolerance"):
        v = cfg.get(fld)
        if not isinstance(v, (int, float)) or v <= 0:
            errors.append(f"{fld} 必须为正数")

    for name in ("rapid_speed", "default_feed", "default_spindle"):
        if isinstance(cfg.get(name), (int, float)) and cfg[name] > 120000:
            errors.append(f"{name}={cfg[name]} 超出合理上限 120000")

    return cfg, errors


def tool_length(cfg: Dict[str, Any], h_code: str, tool_no: str = None) -> float:
    """取生效长度补偿：优先 h_offsets[H]；H 号与刀号相同时回退到该刀 length；否则 0。"""
    if h_code is not None and h_code in cfg.get("h_offsets", {}):
        return float(cfg["h_offsets"][h_code])
    if tool_no is not None and h_code == str(tool_no) \
            and str(tool_no) in cfg.get("tools", {}):
        return float(cfg["tools"][str(tool_no)].get("length", 0.0))
    return 0.0


def cutter_radius(cfg: Dict[str, Any], d_code: str, tool_no: str = None) -> float:
    """取生效半径补偿：优先 d_offsets[D]；D 号与刀号相同时回退到该刀半径；否则 0。"""
    if d_code is not None and d_code in cfg.get("d_offsets", {}):
        return float(cfg["d_offsets"][d_code])
    if tool_no is not None and d_code == str(tool_no) \
            and str(tool_no) in cfg.get("tools", {}):
        return float(cfg["tools"][str(tool_no)].get("diameter", 0.0)) / 2.0
    return 0.0


def _num(v: Any):
    """尽力转 float；失败返回 None（由 validate 统一报错，normalize 不抛异常）。"""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _ok_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def _pos(v: Any) -> bool:
    return _ok_num(v) and v > 0


def _normalize_tool(v: Any) -> Dict[str, Any]:
    if not isinstance(v, dict):
        v = {}
    length = _num(v.get("length", 0.0))
    diameter = _num(v.get("diameter", 0.0))
    shank_len = _num(v.get("shank_length", 0.0))
    holder_len = _num(v.get("holder_length", 0.0))
    if "flute_length" in v:
        flute_len = _num(v.get("flute_length"))
    else:
        # 缺省：刃部占满总长中未被刀杆/刀柄占用的部分（旧版行为 = 整支刀皆为刃）
        flute_len = max(0.0, (length or 0.0) - (shank_len or 0.0) - (holder_len or 0.0))
    # 缺省刃径 = 旧版 diameter
    flute_dia = _num(v.get("flute_diameter")) if "flute_diameter" in v else diameter
    return {
        "length": length,
        "diameter": diameter,
        "flute_diameter": flute_dia,
        "flute_length": flute_len,
        "shank_diameter": _num(v.get("shank_diameter", 0.0)),
        "shank_length": shank_len,
        "holder_diameter": _num(v.get("holder_diameter", 0.0)),
        "holder_length": holder_len,
        "description": str(v.get("description", "")),
    }


def _normalize_obstacles(raw: Any) -> List[Dict[str, Any]]:
    if not isinstance(raw, list):
        return []
    return [_normalize_obstacle(item) for item in raw]


def _normalize_obstacle(item: Any) -> Dict[str, Any]:
    if not isinstance(item, dict):
        return {"id": "", "type": "", "description": ""}
    ob = {
        "id": str(item.get("id", "")),
        "type": str(item.get("type", "")).strip().lower(),
        "description": str(item.get("description", "")),
    }
    if ob["type"] == "box":
        mn = item.get("min") if isinstance(item.get("min"), dict) else {}
        mx = item.get("max") if isinstance(item.get("max"), dict) else {}
        ob["min"] = {a: _num(mn.get(a)) for a in _AXES}
        ob["max"] = {a: _num(mx.get(a)) for a in _AXES}
    elif ob["type"] == "cylinder":
        c = item.get("center") if isinstance(item.get("center"), dict) else {}
        ob["center"] = {"x": _num(c.get("x")), "y": _num(c.get("y"))}
        ob["radius"] = _num(item.get("radius"))
        ob["zmin"] = _num(item.get("zmin"))
        ob["zmax"] = _num(item.get("zmax"))
    return ob


def _deepcopy_default() -> Dict[str, Any]:
    import json
    return json.loads(json.dumps(DEFAULT_CONFIG))


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base
