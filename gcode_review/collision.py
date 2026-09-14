"""刀具组件扫掠体与静态障碍物的间隙/相交计算。

模型
----
* 障碍物（机床坐标系，mm）：
  - ``box``：轴对齐长方体 ``{"min": {x,y,z}, "max": {x,y,z}}``
  - ``cylinder``：竖直圆柱 ``{"center": {x,y}, "radius": r, "zmin", "zmax"}``
* 刀具组件：竖直圆柱，半径 r、相对刀具参考点的 z 区间 [z_lo, z_hi]。
  参考点沿细分后的轨迹段 p→q 刚性平移，组件扫掠体 = 圆柱 ⊕ 线段
  （凸体的闵可夫斯基和，仍为凸体）。

距离等价变换（C 空间）：

    dist(组件扫掠体, 障碍物) = min_t dist(参考点(t), 障碍物 ⊕ (−组件圆柱))

膨胀后的 C 空间障碍物是「XY 区域 × Z 区间」的笛卡尔积：
* box：XY = 矩形 footprint 按半径 r 圆角膨胀；Z = [zmin−z_hi, zmax−z_lo]
* cylinder：XY = 半径 R+r 的圆盘；Z 区间同理

f(t) = hypot(d_xy(t), d_z(t)) 在 t∈[0,1] 上是凸函数（凸集距离复合仿射映射
仍为凸；非负凸函数经非减凸范数复合保持凸），故用黄金分割搜索即可求得
精确最小间隙及取到最小值的参数 t*。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

_AXES = ("x", "y", "z")


def _finite(v: Any) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def obstacle_usable(ob: Any) -> bool:
    """障碍物定义是否具备参与计算的全部数值字段（容错直接传入未校验配置）。"""
    if not isinstance(ob, dict):
        return False
    otype = ob.get("type")
    try:
        if otype == "box":
            return all(_finite(ob["min"][a]) and _finite(ob["max"][a]) for a in _AXES)
        if otype == "cylinder":
            return (_finite(ob["center"]["x"]) and _finite(ob["center"]["y"])
                    and _finite(ob["radius"]) and ob["radius"] > 0
                    and _finite(ob["zmin"]) and _finite(ob["zmax"]))
    except (KeyError, TypeError):
        return False
    return False


def obstacle_aabb(ob: Dict[str, Any]) -> Tuple[float, float, float, float, float, float]:
    """障碍物的轴对齐包围盒 (xmin, ymin, zmin, xmax, ymax, zmax)。"""
    if ob["type"] == "box":
        return (ob["min"]["x"], ob["min"]["y"], ob["min"]["z"],
                ob["max"]["x"], ob["max"]["y"], ob["max"]["z"])
    cx, cy, r = ob["center"]["x"], ob["center"]["y"], ob["radius"]
    return (cx - r, cy - r, ob["zmin"], cx + r, cy + r, ob["zmax"])


def lerp3(a: Dict[str, float], b: Dict[str, float], t: float) -> Dict[str, float]:
    return {k: a[k] + (b[k] - a[k]) * t for k in _AXES}


def densify_polyline(points: List[Dict[str, float]],
                     max_step: float) -> List[Dict[str, float]]:
    """把折线采样按最大步长细分：任意相邻输出点间距 ≤ max_step。"""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        dist = math.sqrt(sum((b[k] - a[k]) ** 2 for k in _AXES))
        n = max(1, int(math.ceil(dist / max_step - 1e-9)))
        for i in range(1, n + 1):
            out.append(lerp3(a, b, i / n))
    return out


def swept_aabb(p: Dict[str, float], q: Dict[str, float], radius: float,
               z_lo: float, z_hi: float
               ) -> Tuple[float, float, float, float, float, float]:
    """组件（半径 r、相对参考点 z 区间 [z_lo,z_hi]）沿线段 p→q 扫掠体的包围盒。"""
    return (min(p["x"], q["x"]) - radius, min(p["y"], q["y"]) - radius,
            min(p["z"], q["z"]) + z_lo,
            max(p["x"], q["x"]) + radius, max(p["y"], q["y"]) + radius,
            max(p["z"], q["z"]) + z_hi)


def aabb_gap(a: Tuple[float, float, float, float, float, float],
             b: Tuple[float, float, float, float, float, float]) -> float:
    """两个 AABB 之间的间距（相交为 0）。用于宽相位剔除。"""
    g2 = 0.0
    for i in range(3):
        d = max(a[i] - b[i + 3], b[i] - a[i + 3], 0.0)
        g2 += d * d
    return math.sqrt(g2)


def swept_distance_fn(p: Dict[str, float], q: Dict[str, float],
                      radius: float, z_lo: float, z_hi: float,
                      ob: Dict[str, Any]):
    """返回 f(t)：组件扫掠体与障碍物的间隙在轨迹段参数 t∈[0,1] 上的函数（凸）。"""

    if ob["type"] == "box":
        xmin, ymin = ob["min"]["x"], ob["min"]["y"]
        xmax, ymax = ob["max"]["x"], ob["max"]["y"]
        cz_lo = ob["min"]["z"] - z_hi
        cz_hi = ob["max"]["z"] - z_lo

        def dxy(x: float, y: float) -> float:
            dx = max(xmin - x, 0.0, x - xmax)
            dy = max(ymin - y, 0.0, y - ymax)
            return max(0.0, math.hypot(dx, dy) - radius)
    else:  # cylinder
        cx, cy = ob["center"]["x"], ob["center"]["y"]
        rr = ob["radius"] + radius
        cz_lo = ob["zmin"] - z_hi
        cz_hi = ob["zmax"] - z_lo

        def dxy(x: float, y: float) -> float:
            return max(0.0, math.hypot(x - cx, y - cy) - rr)

    dx, dy, dz = q["x"] - p["x"], q["y"] - p["y"], q["z"] - p["z"]

    def f(t: float) -> float:
        z = p["z"] + dz * t
        if z < cz_lo:
            d_z = cz_lo - z
        elif z > cz_hi:
            d_z = z - cz_hi
        else:
            d_z = 0.0
        return math.hypot(dxy(p["x"] + dx * t, p["y"] + dy * t), d_z)

    return f


def swept_clearance(p: Dict[str, float], q: Dict[str, float],
                    radius: float, z_lo: float, z_hi: float,
                    ob: Dict[str, Any]) -> Tuple[float, float]:
    """组件圆柱沿 p→q 的扫掠体与障碍物的最小间隙。

    :returns: (最小间隙 mm, 取到最小值的线段参数 t*)。相交时间隙为 0。
    """
    return _golden_min(swept_distance_fn(p, q, radius, z_lo, z_hi, ob))


def first_crossing(p: Dict[str, float], q: Dict[str, float], t_star: float,
                   radius: float, z_lo: float, z_hi: float,
                   ob: Dict[str, Any], threshold: float) -> float:
    """段内首次进入「间隙 ≤ threshold」区域的参数 t（起点已在区域内时为 0）。"""
    f = swept_distance_fn(p, q, radius, z_lo, z_hi, ob)
    if f(0.0) <= threshold:
        return 0.0
    lo, hi = 0.0, t_star      # 不变式：f(lo) > threshold，f(hi) ≤ threshold
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if f(mid) <= threshold:
            hi = mid
        else:
            lo = mid
    return hi


def last_crossing(p: Dict[str, float], q: Dict[str, float], t_star: float,
                  radius: float, z_lo: float, z_hi: float,
                  ob: Dict[str, Any], threshold: float) -> float:
    """段内末次离开「间隙 ≤ threshold」区域的参数 t（终点仍在区域内时为 1）。"""
    f = swept_distance_fn(p, q, radius, z_lo, z_hi, ob)
    if f(1.0) <= threshold:
        return 1.0
    lo, hi = t_star, 1.0      # 不变式：f(lo) ≤ threshold，f(hi) > threshold
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if f(mid) <= threshold:
            lo = mid
        else:
            hi = mid
    return lo


def _golden_min(f, a: float = 0.0, b: float = 1.0,
                iters: int = 60) -> Tuple[float, float]:
    """凸函数 f 在 [a,b] 上的最小值与极小点（黄金分割搜索）。"""
    invphi = (math.sqrt(5.0) - 1.0) / 2.0
    c = b - invphi * (b - a)
    d = a + invphi * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - invphi * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + invphi * (b - a)
            fd = f(d)
    t = 0.5 * (a + b)
    return f(t), t
