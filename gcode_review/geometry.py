"""圆弧几何求解：支持 I/J/K 与 R 两种形式、整圆、螺旋线采样。

平面坐标约定（沿补偿轴正方向朝原点看，屏幕水平 h、垂直 v）：

* G17：h=X, v=Y，补偿轴 Z，I→X、J→Y
* G18：h=Z, v=X，补偿轴 Y，K→Z、I→X
* G19：h=Y, v=Z，补偿轴 X，J→Y、K→Z

该视角下 G02 为顺时针（扫略角取负），G03 为逆时针（取正），
与主流控制器（Fanuc/三菱/LinuxCNC 的常规图示）一致。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# 平面 -> (水平轴, 垂直轴, 补偿轴)
PLANE_AXES: Dict[str, Tuple[str, str, str]] = {
    "G17": ("x", "y", "z"),
    "G18": ("z", "x", "y"),
    "G19": ("y", "z", "x"),
}
# 平面 -> (水平轴圆心词, 垂直轴圆心词)
PLANE_CENTER_WORDS: Dict[str, Tuple[str, str]] = {
    "G17": ("I", "J"),
    "G18": ("K", "I"),
    "G19": ("J", "K"),
}

_ARC_SAMPLES = 64
_EPS = 1e-9


@dataclass
class ArcGeom:
    ok: bool
    reason: str = ""                       # 失败原因（中文）
    center: Optional[Dict[str, float]] = None
    radius: Optional[float] = None
    sweep_deg: Optional[float] = None      # 带符号扫略角
    helical_depth: float = 0.0             # 补偿轴方向螺旋深度
    arc_length: float = 0.0
    path_length: float = 0.0
    samples: List[Dict[str, float]] = field(default_factory=list)
    plane: Optional[str] = None

    def extents(self) -> Optional[Tuple[float, float, float, float]]:
        """采样包围：(hmin, hmax, vmin, vmax)。"""
        if not self.samples or self.plane is None:
            return None
        h_axis, v_axis, _ = PLANE_AXES[self.plane]
        hs = [p[h_axis] for p in self.samples]
        vs = [p[v_axis] for p in self.samples]
        return min(hs), max(hs), min(vs), max(vs)


def resolve_arc(
    start: Dict[str, float],
    end: Dict[str, float],
    plane: str,
    clockwise: bool,
    center: Optional[Dict[str, float]] = None,
    radius_r: Optional[float] = None,
    tolerance: float = 0.01,
    samples: int = _ARC_SAMPLES,
) -> ArcGeom:
    """求解一段圆弧。

    :param start/end: 三维物理起点/终点（含补偿轴，用于螺旋）
    :param center: 三维圆心（I/J/K 经 G90.1/G91.1 换算后的绝对位置）
    :param radius_r: R 半径；与 center 同时给定时调用方应先告警，此处优先 IJK
    """
    h_ax, v_ax, n_ax = PLANE_AXES[plane]
    s = (start[h_ax], start[v_ax])
    e = (end[h_ax], end[v_ax])

    if center is not None:
        geom = _resolve_ijk(s, e, center, plane, clockwise, tolerance)
    elif radius_r is not None:
        geom = _resolve_r(s, e, radius_r, plane, clockwise, tolerance)
    else:
        return ArcGeom(ok=False, reason="圆弧段缺少 R 或 I/J/K 圆心定义")

    if not geom.ok:
        return geom

    # 采样（补偿轴线性螺旋插值）
    n0 = start.get(n_ax, 0.0)
    n1 = end.get(n_ax, 0.0)
    cu, cv = geom.center[h_ax], geom.center[v_ax]  # type: ignore[index]
    # 起点角与终点角都按标准数学角 atan2(dv,du) 计算；sweep 已带 G02/G03 符号
    theta0 = math.atan2(s[1] - cv, s[0] - cu)
    theta1 = math.atan2(e[1] - cv, e[0] - cu)
    sweep_rad = math.radians(geom.sweep_deg)  # type: ignore[arg-type]
    full = abs(abs(geom.sweep_deg) - 360.0) < 1e-9  # type: ignore[arg-type]
    pts: List[Dict[str, float]] = []
    steps = max(2, samples)
    for i in range(steps + 1):
        t = i / steps
        # 非整圆末端强制 theta1，避免大弧角度回卷误差；整圆首尾同点
        if not full and i == steps:
            a = theta1
        else:
            a = theta0 + sweep_rad * t
        pts.append({
            h_ax: cu + geom.radius * math.cos(a),  # type: ignore[operator]
            v_ax: cv + geom.radius * math.sin(a),  # type: ignore[operator]
            n_ax: n0 + (n1 - n0) * t,
        })
    geom.samples = pts
    geom.helical_depth = n1 - n0
    geom.arc_length = abs(geom.radius * sweep_rad)  # type: ignore[arg-type]
    geom.path_length = math.hypot(geom.arc_length, geom.helical_depth)
    geom.plane = plane
    return geom


def _resolve_ijk(s, e, center, plane, clockwise, tolerance) -> ArcGeom:
    h_ax, v_ax, _ = PLANE_AXES[plane]
    c = (center[h_ax], center[v_ax])
    r0 = math.hypot(s[0] - c[0], s[1] - c[1])
    r1 = math.hypot(e[0] - c[0], e[1] - c[1])

    if r0 < _EPS:
        return ArcGeom(ok=False, reason="起点与圆心重合，半径为 0，无法构成圆弧")
    if abs(r0 - r1) > tolerance:
        return ArcGeom(
            ok=False,
            reason=(f"起终点到圆心距离不一致：r0={r0:.4f}、r1={r1:.4f}，"
                    f"偏差 {abs(r0 - r1):.4f} mm 超过容差 {tolerance:g} mm"),
        )

    full_circle = abs(e[0] - s[0]) <= _EPS and abs(e[1] - s[1]) <= _EPS
    if full_circle:
        sweep = -360.0 if clockwise else 360.0
    else:
        theta0 = math.atan2(s[1] - c[1], s[0] - c[0])
        theta1 = math.atan2(e[1] - c[1], e[0] - c[0])
        ccw = math.degrees(theta1 - theta0) % 360.0
        sweep = (ccw - 360.0) if clockwise else ccw

    return ArcGeom(
        ok=True,
        center={h_ax: c[0], v_ax: c[1]},
        radius=r0,
        sweep_deg=sweep,
    )


def _resolve_r(s, e, r, plane, clockwise, tolerance) -> ArcGeom:
    h_ax, v_ax, _ = PLANE_AXES[plane]
    if abs(r) < _EPS:
        return ArcGeom(ok=False, reason="R 半径为 0，无法构成圆弧")

    d = (e[0] - s[0], e[1] - s[1])
    chord = math.hypot(d[0], d[1])
    if chord < _EPS:
        return ArcGeom(
            ok=False,
            reason="R 编程时圆弧起终点重合：整圆必须用 I/J/K 而不能用 R 描述",
        )

    half = chord / 2.0
    h2 = r * r - half * half
    if h2 < -tolerance:
        return ArcGeom(
            ok=False,
            reason=(f"弦长 {chord:.4f} mm 大于直径 {2 * abs(r):.4f} mm，"
                    f"R 圆弧无解（差额 {math.sqrt(-h2) * 2:.4f} mm）"),
        )
    h2 = max(0.0, h2)
    a = math.sqrt(h2)
    mid = ((s[0] + e[0]) / 2.0, (s[1] + e[1]) / 2.0)
    # 弦的逆时针法向（指向左侧圆心 c_plus）
    perp = (-d[1] / chord, d[0] / chord)
    c_plus = (mid[0] + perp[0] * a, mid[1] + perp[1] * a)
    c_minus = (mid[0] - perp[0] * a, mid[1] - perp[1] * a)

    major = r < 0.0
    # 逆时针小弧圆心是 c_plus；顺时针小弧是 c_minus。大弧取对面。
    if clockwise:
        c = c_plus if major else c_minus
    else:
        c = c_minus if major else c_plus

    theta0 = math.atan2(s[1] - c[1], s[0] - c[0])
    theta1 = math.atan2(e[1] - c[1], e[0] - c[0])
    ccw = math.degrees(theta1 - theta0) % 360.0
    sweep = (ccw - 360.0) if clockwise else ccw

    return ArcGeom(
        ok=True,
        center={h_ax: c[0], v_ax: c[1]},
        radius=abs(r),
        sweep_deg=sweep,
    )
