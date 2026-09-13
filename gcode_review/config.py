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
    # 刀具表：键为刀号字符串；length 为长度补偿几何值，diameter 为刀具直径
    "tools": {
        # "1": {"length": 100.0, "diameter": 10.0, "description": "立铣刀"}
    },
    # H 号 -> 长度补偿值（mm）；缺省时回退到同号刀具的 length
    "h_offsets": {},
    # D 号 -> 半径补偿值（mm）；缺省时回退到同号刀具半径
    "d_offsets": {},
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
        for fld in ("length", "diameter"):
            v = tool.get(fld)
            if not isinstance(v, (int, float)) or v < 0:
                errors.append(f"tools.{tnum}.{fld} 必须为非负数值")
        if tool.get("diameter", 0) > max(
            env["xmax"] - env["xmin"], env["ymax"] - env["ymin"]
        ):
            errors.append(f"tools.{tnum}.diameter 大于机床行程，配置可疑")

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


def _normalize_tool(v: Any) -> Dict[str, Any]:
    if not isinstance(v, dict):
        return {"length": 0.0, "diameter": 0.0}
    return {
        "length": float(v.get("length", 0.0)),
        "diameter": float(v.get("diameter", 0.0)),
        "description": str(v.get("description", "")),
    }


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
