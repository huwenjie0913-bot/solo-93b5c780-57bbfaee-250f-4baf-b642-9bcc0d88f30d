"""同一程序在两套机床配置下的审查结果比对。"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .engine import ReviewEngine


def _fingerprint(diag: Dict[str, Any]) -> str:
    """诊断指纹：同代码+同行+同段视为同一类问题。"""
    return f"{diag['code']}@L{diag['line_no']}#S{diag['segment_index']}"


def compare_reviews(program: str, config_a: Dict[str, Any],
                    config_b: Dict[str, Any],
                    filename: Optional[str] = None) -> Dict[str, Any]:
    report_a = ReviewEngine(config_a).review(program, filename=filename)
    report_b = ReviewEngine(config_b).review(program, filename=filename)

    map_a = {_fingerprint(d): d for d in report_a["diagnostics"]}
    map_b = {_fingerprint(d): d for d in report_b["diagnostics"]}

    added: List[Dict[str, Any]] = []
    removed: List[Dict[str, Any]] = []
    severity_changed: List[Dict[str, Any]] = []
    shared: List[Dict[str, Any]] = []

    for fp, db in map_b.items():
        da = map_a.get(fp)
        if da is None:
            added.append(db)
        elif da["severity"] != db["severity"]:
            severity_changed.append({
                "fingerprint": fp,
                "code": db["code"],
                "line_no": db["line_no"],
                "severity_a": da["severity"],
                "severity_b": db["severity"],
                "message_b": db["message"],
                "basis_b": db["basis"],
            })
        else:
            shared.append({"fingerprint": fp, "code": db["code"],
                           "line_no": db["line_no"], "severity": db["severity"]})
    for fp, da in map_a.items():
        if fp not in map_b:
            removed.append(da)

    bounds_a = report_a["trajectory"]["bounds_machine"]
    bounds_b = report_b["trajectory"]["bounds_machine"]
    time_a = report_a["time_estimate_s"]["total"]
    time_b = report_b["time_estimate_s"]["total"]

    def _env_status(report):
        return [d for d in report["diagnostics"] if d["code"] == "ENVELOPE_VIOLATION"]

    def _safety_status(report):
        return [d for d in report["diagnostics"] if d["code"] == "RAPID_BELOW_SAFETY"]

    return {
        "filename": report_a["filename"],
        "config_a": report_a["config"],
        "config_b": report_b["config"],
        "summary": {
            "counts_a": report_a["diagnostic_counts"],
            "counts_b": report_b["diagnostic_counts"],
            "risk_a": report_a["risk"],
            "risk_b": report_b["risk"],
            "added": len(added),
            "removed": len(removed),
            "severity_changed": len(severity_changed),
            "shared": len(shared),
            "envelope_violations_a": len(_env_status(report_a)),
            "envelope_violations_b": len(_env_status(report_b)),
            "rapid_below_safety_a": len(_safety_status(report_a)),
            "rapid_below_safety_b": len(_safety_status(report_b)),
        },
        "added_in_b": added,
        "removed_from_a": removed,
        "severity_changed": severity_changed,
        "trajectory_diff": {
            "bounds_machine_a": bounds_a,
            "bounds_machine_b": bounds_b,
            "time_total_a_s": time_a,
            "time_total_b_s": time_b,
            "time_delta_s": round(time_b - time_a, 3),
            "time_delta_basis": "换刀/快移/进给参数随配置变化；程序几何不变",
        },
        "verdict": _verdict(added, removed, severity_changed,
                            report_a, report_b),
    }


def _verdict(added, removed, changed, ra, rb) -> str:
    if ra["risk"] == "clean" and rb["risk"] != "clean":
        return f"配置 A 无风险而配置 B 出现 {rb['risk']} 级问题，程序不能直接换到 B 设备"
    if rb["risk"] == "clean" and ra["risk"] != "clean":
        return "配置 B 无风险，配置 A 的问题在 B 上消失"
    if not added and not removed and not changed:
        return "两套配置下风险一致（注意仍可能存在共同问题）"
    parts = []
    if added:
        parts.append(f"B 新增 {len(added)} 项")
    if removed:
        parts.append(f"B 消除 {len(removed)} 项")
    if changed:
        parts.append(f"{len(changed)} 项严重度变化")
    return "；".join(parts) + "，详见各项定位与依据"
