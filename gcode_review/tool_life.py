"""刀具寿命预测：磨损记录校验、磨损趋势拟合与剩余寿命估算。

模型：对历史磨损记录（累计切削时间 → 后刀面磨损量）做一元线性最小二乘拟合，
外推到磨钝标准（wear_limit_mm）得到预计总寿命；再按本次工况（转速/进给/切削
深度）相对历史平均工况的偏离，用扩展 Taylor 经验公式修正剩余寿命：

    寿命倍率 = (n_ref/n_new)^a · (f_ref/f_new)^b · (ap_ref/ap_new)^c · k_material

指数默认 a=4 > b=2 > c=1（转速影响最大、切削深度最小），可在 life_config 调整。
置信区间由回归残差经 delta 法传播到寿命点。仅使用标准库。
"""

from __future__ import annotations

import math
from statistics import NormalDist
from typing import Any, Dict, List, Optional, Tuple

# 工况参数：(字段名, 中文名, 单位, Taylor 指数配置键, 安全阈值配置键)
_CONDITION_PARAMS = (
    ("speed_rpm", "主轴转速", "rpm", "speed_exponent", "max_speed_rpm"),
    ("feed_mm_min", "进给速度", "mm/min", "feed_exponent", "max_feed_mm_min"),
    ("depth_mm", "切削深度", "mm", "depth_exponent", "max_depth_mm"),
)

DEFAULT_LIFE_CONFIG: Dict[str, Any] = {
    "wear_limit_mm": 0.3,       # 磨钝标准（后刀面磨损带 VB，mm）
    "min_records": 3,           # 趋势拟合建议的最少记录数
    "confidence_level": 0.95,   # 置信区间置信水平
    "speed_exponent": 4.0,      # 扩展 Taylor 指数：转速
    "feed_exponent": 2.0,       # 进给
    "depth_exponent": 1.0,      # 切削深度
    "max_speed_rpm": None,      # 工况安全阈值（None 表示不检查）
    "max_feed_mm_min": None,
    "max_depth_mm": None,
    "min_remaining_min": 30.0,  # 剩余寿命低于该值提示尽快换刀
    "min_r_squared": 0.6,       # 拟合优度下限，低于则提示趋势不可信
}


class LifeValidationError(ValueError):
    """刀具寿命数据校验失败：携带结构化错误列表，便于 API 返回 details。"""

    def __init__(self, errors):
        self.errors = [str(e) for e in errors]
        super().__init__("刀具寿命数据校验失败：" + "；".join(self.errors))


# ---------------------------------------------------------------------- #
# 数据校验
# ---------------------------------------------------------------------- #
def validate_life_config(raw: Any) -> Tuple[Dict[str, Any], List[str]]:
    """校验寿命预测参数。返回 (补全后的配置, 错误消息列表)。"""
    if raw is None:
        return dict(DEFAULT_LIFE_CONFIG), []
    if not isinstance(raw, dict):
        return dict(DEFAULT_LIFE_CONFIG), ["life_config 必须是对象（JSON object）"]
    cfg = dict(DEFAULT_LIFE_CONFIG)
    for key in DEFAULT_LIFE_CONFIG:
        if key in raw:
            cfg[key] = raw[key]
    errors: List[str] = []
    if not _ok_num(cfg["wear_limit_mm"]) or cfg["wear_limit_mm"] <= 0:
        errors.append("wear_limit_mm 必须为正数（磨钝标准 mm）")
    if not isinstance(cfg["min_records"], int) or cfg["min_records"] < 2:
        errors.append("min_records 必须为不小于 2 的整数")
    cl = cfg["confidence_level"]
    if not _ok_num(cl) or not 0.5 < cl < 1.0:
        errors.append("confidence_level 必须在 (0.5, 1) 之间，如 0.95")
    for key in ("speed_exponent", "feed_exponent", "depth_exponent"):
        if not _ok_num(cfg[key]) or cfg[key] <= 0:
            errors.append(f"{key} 必须为正数（扩展 Taylor 寿命指数）")
    for _n, label, _u, _e, max_key in _CONDITION_PARAMS:
        v = cfg[max_key]
        if v is not None and (not _ok_num(v) or v <= 0):
            errors.append(f"{max_key} 若提供必须为正数（{label}安全阈值），"
                          "不需要检查可设为 null")
    if not _ok_num(cfg["min_remaining_min"]) or cfg["min_remaining_min"] < 0:
        errors.append("min_remaining_min 必须为非负数值（剩余寿命安全余量 min）")
    if not _ok_num(cfg["min_r_squared"]) \
            or not 0.0 <= cfg["min_r_squared"] <= 1.0:
        errors.append("min_r_squared 必须在 [0, 1] 之间")
    return cfg, errors


def validate_records(raw: Any, min_count: int = 2
                     ) -> Tuple[List[Dict[str, Any]], List[str], List[str]]:
    """校验并清洗磨损记录。返回 (按切削时间排序的记录, 错误, 可解释异常提示)。

    记录字段：cutting_time_min（累计切削时间，必传）、wear_mm（磨损量，必传）、
    speed_rpm/feed_mm_min/depth_mm（该段历史工况，可选）。
    """
    errors: List[str] = []
    anomalies: List[str] = []
    if not isinstance(raw, list) or not raw:
        return [], ["records 必须是非空数组（磨损记录列表）"], anomalies
    clean: List[Dict[str, Any]] = []
    for i, rec in enumerate(raw):
        if not isinstance(rec, dict):
            errors.append(f"records[{i}] 必须是对象（JSON object）")
            continue
        t, w = rec.get("cutting_time_min"), rec.get("wear_mm")
        if not _ok_num(t) or t <= 0:
            errors.append(
                f"records[{i}].cutting_time_min 必须为正数（累计切削时间 min）")
            continue
        if not _ok_num(w) or w < 0:
            errors.append(f"records[{i}].wear_mm 必须为非负数值（磨损量 mm）")
            continue
        entry = {"cutting_time_min": float(t), "wear_mm": float(w)}
        for name, label, unit, _e, _m in _CONDITION_PARAMS:
            v = rec.get(name)
            if v is None:
                entry[name] = None
            elif _ok_num(v) and v > 0:
                entry[name] = float(v)
            else:
                errors.append(
                    f"records[{i}].{name} 若提供必须为正数（{label} {unit}）")
        clean.append(entry)
    if errors:
        return [], errors, anomalies
    if len(clean) < min_count:
        return [], [f"至少需要 {min_count} 条有效磨损记录才能拟合磨损趋势"], anomalies
    clean.sort(key=lambda r: r["cutting_time_min"])
    # 可解释异常：时间重复 / 磨损回退（物理上磨损不可减小）
    for prev, cur in zip(clean, clean[1:]):
        if cur["cutting_time_min"] == prev["cutting_time_min"]:
            anomalies.append(
                f"切削时间 {cur['cutting_time_min']:g}min 存在多条记录，"
                "疑似重复录入，已保留并参与拟合")
        if cur["wear_mm"] < prev["wear_mm"] - 1e-9:
            anomalies.append(
                f"t={cur['cutting_time_min']:g}min 磨损量 {cur['wear_mm']:g}mm "
                f"小于前一点 t={prev['cutting_time_min']:g}min 的 "
                f"{prev['wear_mm']:g}mm，磨损物理上不可回退，"
                "疑似测量误差或中途换刀/重磨后未重置计时")
    return clean, [], anomalies


def validate_condition(raw: Any) -> Tuple[Dict[str, Any], List[str]]:
    """校验本次工况。返回 (condition, 错误列表)。字段可缺省（缺省=与历史相同）。"""
    if raw is None:
        return {}, []
    if not isinstance(raw, dict):
        return {}, ["condition 必须是对象（JSON object）"]
    errors: List[str] = []
    cond: Dict[str, Any] = {}
    for name, label, unit, _e, _m in _CONDITION_PARAMS:
        v = raw.get(name)
        if v is None:
            continue
        if not _ok_num(v) or v <= 0:
            errors.append(f"condition.{name} 必须为正数（{label} {unit}）")
        else:
            cond[name] = float(v)
    if raw.get("material") is not None:
        cond["material"] = str(raw["material"])
    mf = raw.get("material_factor")
    if mf is not None:
        if not _ok_num(mf) or mf <= 0:
            errors.append("condition.material_factor 若提供必须为正数"
                          "（相对历史材料的寿命系数）")
        else:
            cond["material_factor"] = float(mf)
    return cond, errors


# ---------------------------------------------------------------------- #
# 磨损趋势与剩余寿命
# ---------------------------------------------------------------------- #
def _linear_fit(xs: List[float], ys: List[float]) -> Optional[Dict[str, float]]:
    """一元线性最小二乘。返回截距/斜率/R²/残差标准差及参数方差（delta 法用）。"""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = my - slope * mx
    resid = [y - (intercept + slope * x) for x, y in zip(xs, ys)]
    sse = sum(r * r for r in resid)
    sst = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - sse / sst if sst > 0 else (1.0 if sse == 0 else 0.0)
    s2 = sse / (n - 2) if n > 2 else 0.0
    return {
        "intercept": intercept,
        "slope": slope,
        "r_squared": r2,
        "residual_std": math.sqrt(s2),
        "var_intercept": s2 * (1.0 / n + mx * mx / sxx),
        "var_slope": s2 / sxx,
        "cov": -mx * s2 / sxx,
    }


def predict_tool_life(records: Any, condition: Any = None,
                      life_config: Any = None, tool_id: Optional[str] = None,
                      material: Optional[str] = None) -> Dict[str, Any]:
    """刀具寿命预测主入口。数据非法抛 LifeValidationError（携带 details）。"""
    cfg, cfg_errors = validate_life_config(life_config)
    if cfg_errors:
        raise LifeValidationError(cfg_errors)
    clean, rec_errors, anomalies = validate_records(records)
    if rec_errors:
        raise LifeValidationError(rec_errors)
    cond, cond_errors = validate_condition(condition)
    if cond_errors:
        raise LifeValidationError(cond_errors)

    limit = cfg["wear_limit_mm"]
    xs = [r["cutting_time_min"] for r in clean]
    ys = [r["wear_mm"] for r in clean]
    fit = _linear_fit(xs, ys)
    if fit is None:
        raise LifeValidationError(["所有记录的切削时间相同，无法拟合磨损趋势"])

    t_last, w_last = xs[-1], ys[-1]
    slope, r2 = fit["slope"], fit["r_squared"]

    # ---- 数据质量提示（可解释） ----
    if len(clean) < cfg["min_records"]:
        anomalies.append(
            f"有效记录仅 {len(clean)} 条，少于建议的 {cfg['min_records']} 条，"
            "趋势与置信区间仅供参考")
    if r2 < cfg["min_r_squared"]:
        anomalies.append(
            f"磨损趋势拟合优度 R²={r2:.2f} 低于 {cfg['min_r_squared']:g}，"
            "数据离散度大、外推不确定性高，建议检查磨损测量的一致性")

    # ---- 历史参考工况（有工况字段的记录取平均） ----
    ref: Dict[str, Optional[float]] = {}
    for name, _l, _u, _e, _m in _CONDITION_PARAMS:
        vals = [r[name] for r in clean if r.get(name) is not None]
        ref[name] = sum(vals) / len(vals) if vals else None

    # ---- 工况修正系数（扩展 Taylor）与主要影响因素 ----
    factors: List[Dict[str, Any]] = []
    factor_total = 1.0
    for name, label, unit, exp_key, _m in _CONDITION_PARAMS:
        new_v, ref_v = cond.get(name), ref[name]
        entry: Dict[str, Any] = {
            "parameter": name, "label": label, "unit": unit,
            "reference": _r(ref_v), "current": _r(new_v),
        }
        if new_v is None:
            entry.update(applied=False, factor=1.0,
                         note=f"未提供本次{label}，按与历史工况相同处理")
        elif ref_v is None:
            entry.update(applied=False, factor=1.0,
                         note=f"历史记录缺少{label}，该项未做修正")
        else:
            f = (ref_v / new_v) ** cfg[exp_key]
            entry.update(applied=True, factor=_r(f, 4), _log=abs(math.log(f)),
                         effect_pct=round((f - 1.0) * 100.0, 1))
            factor_total *= f
        factors.append(entry)

    # 材料修正：显式 material_factor 优先；材料不一致且未给系数时提示
    tool_material = (material or "").strip()
    cond_material = (cond.get("material") or "").strip()
    if cond.get("material_factor") is not None:
        factor_total *= cond["material_factor"]
    elif tool_material and cond_material and tool_material != cond_material:
        anomalies.append(
            f"本次加工材料（{cond_material}）与历史磨损记录材料"
            f"（{tool_material}）不一致且未提供 material_factor，"
            "未做材料修正，预测结果仅供参考")

    # 影响占比：|ln 系数| 归一化，占比最高者即主要影响因素
    log_sum = sum(e["_log"] for e in factors if e.get("applied"))
    for e in factors:
        if e.get("applied"):
            e["share"] = round(e.pop("_log") / log_sum, 3) if log_sum > 0 else 0.0
            if e["factor"] > 1:
                effect = f"延长寿命 {e['effect_pct']:g}%"
            elif e["factor"] < 1:
                effect = f"缩短寿命 {abs(e['effect_pct']):g}%"
            else:
                effect = "与历史工况相同，不影响寿命"
            e["note"] = (f"{e['label']}由 {e['reference']:g}{e['unit']} 变为 "
                         f"{e['current']:g}{e['unit']}，{effect}")
        else:
            e["share"] = 0.0
            e.pop("_log", None)
    factors.sort(key=lambda e: e["share"], reverse=True)

    # ---- 寿命外推与置信区间（delta 法） ----
    if w_last >= limit:
        anomalies.append(
            f"当前实测磨损 {w_last:g}mm 已达到磨钝标准 {limit:g}mm，"
            "继续使用存在崩刃与尺寸超差风险")
        prediction = _prediction_dict(t_last, 0.0, 0.0, 0.0, factor_total, cfg)
    elif slope <= 0:
        anomalies.append(
            f"拟合磨损速率为 {slope:.3g}mm/min（非增长），无法外推寿命；"
            "若数据无误，说明刀具处于稳定低磨损阶段，建议继续观测后再评估")
        prediction = _prediction_dict(t_last, None, None, None, factor_total, cfg)
    else:
        x0 = (limit - fit["intercept"]) / slope  # 拟合线触及磨钝标准的时间
        g_a, g_b = -1.0 / slope, -x0 / slope     # x0=(L-a)/b 对 a,b 的偏导
        var_x0 = (g_a * g_a * fit["var_intercept"]
                  + g_b * g_b * fit["var_slope"]
                  + 2 * g_a * g_b * fit["cov"])
        se = math.sqrt(var_x0) if var_x0 > 0 else 0.0
        z = NormalDist().inv_cdf(0.5 + cfg["confidence_level"] / 2.0)
        rem = max(0.0, x0 - t_last)
        rem_lo = max(0.0, x0 - z * se - t_last)
        rem_hi = max(0.0, x0 + z * se - t_last)
        prediction = _prediction_dict(t_last, rem, rem_lo, rem_hi,
                                      factor_total, cfg)
        if rem * factor_total < cfg["min_remaining_min"]:
            anomalies.append(
                f"按本次工况预计剩余寿命仅 {rem * factor_total:.1f}min，"
                f"低于安全余量 {cfg['min_remaining_min']:g}min，"
                "建议本班次内安排换刀")

    # ---- 超出安全阈值的工况 ----
    violations: List[Dict[str, Any]] = []
    for name, label, unit, _e, max_key in _CONDITION_PARAMS:
        lim, v = cfg[max_key], cond.get(name)
        if v is not None and _ok_num(lim) and v > lim:
            violations.append({
                "parameter": name, "label": label, "unit": unit,
                "value": v, "limit": lim,
                "message": f"本次{label} {v:g}{unit} 超出安全阈值 {lim:g}{unit}，"
                           "刀具磨损将显著加速，建议降低参数或缩短换刀周期",
            })
    hist_exceed = sum(
        1 for r in clean
        if any(r.get(n) is not None and _ok_num(cfg[mk]) and r[n] > cfg[mk]
               for n, _l, _u, _e, mk in _CONDITION_PARAMS))
    if hist_exceed:
        anomalies.append(
            f"历史记录中有 {hist_exceed} 条工况超出当前安全阈值，"
            "对应区段磨损可能已加速，趋势外推偏保守")

    return {
        "tool_id": tool_id,
        "material": tool_material or None,
        "record_count": len(clean),
        "condition": cond,
        "life_config": cfg,
        "wear": {
            "current_wear_mm": _r(w_last),
            "wear_limit_mm": _r(limit),
            "wear_rate_mm_per_min": _r(slope, 6),
            "r_squared": _r(r2),
            "fit": {"intercept": _r(fit["intercept"], 6),
                    "slope": _r(fit["slope"], 6),
                    "residual_std": _r(fit["residual_std"], 6)},
            "reference_condition": {k: _r(v) for k, v in ref.items()},
        },
        "prediction": prediction,
        "factors": factors,
        "threshold_violations": violations,
        "anomalies": anomalies,
        "verdict": _verdict(w_last, limit, prediction, factors,
                            len(violations), cfg),
    }


def _prediction_dict(t_last: float, rem, rem_lo, rem_hi,
                     factor: float, cfg: Dict[str, Any]) -> Dict[str, Any]:
    """组装预测结果；rem 为 None 表示无法外推。置信区间随工况系数同步缩放。"""
    if rem is None:
        return {
            "elapsed_cutting_min": _r(t_last),
            "remaining_base_min": None,
            "condition_factor": _r(factor, 4),
            "remaining_min": None,
            "total_life_min": None,
            "confidence_level": cfg["confidence_level"],
            "remaining_ci_min": None,
            "total_life_ci_min": None,
        }
    return {
        "elapsed_cutting_min": _r(t_last),
        "remaining_base_min": _r(rem),          # 历史工况下的剩余寿命
        "condition_factor": _r(factor, 4),
        "remaining_min": _r(rem * factor),      # 本次工况修正后的剩余寿命
        "total_life_min": _r(t_last + rem * factor),
        "confidence_level": cfg["confidence_level"],
        "remaining_ci_min": [_r(rem_lo * factor), _r(rem_hi * factor)],
        "total_life_ci_min": [_r(t_last + rem_lo * factor),
                              _r(t_last + rem_hi * factor)],
    }


def _verdict(w_last: float, limit: float, prediction: Dict[str, Any],
             factors: List[Dict[str, Any]], n_violations: int,
             cfg: Dict[str, Any]) -> str:
    if w_last >= limit:
        return (f"磨损已达磨钝标准（{w_last:g}mm ≥ {limit:g}mm），"
                "应立即换刀并检查已加工尺寸")
    if prediction["remaining_min"] is None:
        return "磨损趋势非增长，暂无法外推剩余寿命，建议增加测量频次后继续观测"
    parts = [
        f"预计剩余安全切削时间约 {prediction['remaining_min']:g}min"
        f"（{cfg['confidence_level']:.0%} 置信区间 "
        f"{prediction['remaining_ci_min'][0]:g}–"
        f"{prediction['remaining_ci_min'][1]:g}min）"
    ]
    top = next((e for e in factors if e.get("applied")), None)
    if top:
        parts.append(f"主要影响因素：{top['label']}")
    if n_violations:
        parts.append(f"{n_violations} 项工况超出安全阈值")
    return "；".join(parts)


# ---------------------------------------------------------------------- #
def _ok_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def _r(v: Any, n: int = 4):
    """有限数值四舍五入；None/非数值原样返回。"""
    return round(v, n) if _ok_num(v) else v
