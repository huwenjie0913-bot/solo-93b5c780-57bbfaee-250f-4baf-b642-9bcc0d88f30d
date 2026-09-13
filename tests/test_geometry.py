"""圆弧几何求解测试。"""

import math

from gcode_review.geometry import resolve_arc


def test_ijk_semicircle_ccw():
    """起点 (0,0) 在左、终点 (20,0) 在右：逆时针走下半圆。"""
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 20.0, "y": 0.0, "z": 0.0}
    c = {"x": 10.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, e, "G17", clockwise=False, center=c)
    assert g.ok
    assert abs(g.radius - 10.0) < 1e-9
    assert abs(g.sweep_deg - 180.0) < 1e-9
    # 弧底应在 (10,-10)
    bottom = min(g.samples, key=lambda p: p["y"])
    assert abs(bottom["x"] - 10.0) < 1e-6 and abs(bottom["y"] + 10.0) < 1e-6


def test_ijk_semicircle_cw_goes_above():
    """同起终点：顺时针走上半圆。"""
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 20.0, "y": 0.0, "z": 0.0}
    c = {"x": 10.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, e, "G17", clockwise=True, center=c)
    assert g.ok
    assert abs(g.sweep_deg + 180.0) < 1e-9
    top = max(g.samples, key=lambda p: p["y"])
    assert abs(top["y"] - 10.0) < 1e-6


def test_full_circle():
    s = {"x": 10.0, "y": 0.0, "z": 0.0}
    c = {"x": 0.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, s, "G17", clockwise=False, center=c)
    assert g.ok
    assert abs(abs(g.sweep_deg) - 360.0) < 1e-9
    assert abs(g.path_length - 2 * math.pi * 10) < 1e-6


def test_radius_mismatch_detected():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 10.0, "y": 0.0, "z": 0.0}
    c = {"x": 5.0, "y": 0.0, "z": 0.0}  # r0=5, r1 应=5 但给偏差端点
    e2 = {"x": 11.0, "y": 0.5, "z": 0.0}
    g = resolve_arc(s, e2, "G17", clockwise=True, center=c, tolerance=0.01)
    assert not g.ok
    assert "距离不一致" in g.reason


def test_zero_radius_center_at_start():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, {"x": 5.0, "y": 5.0, "z": 0.0}, "G17",
                    clockwise=False, center={"x": 0.0, "y": 0.0, "z": 0.0})
    assert not g.ok and "半径为 0" in g.reason


def test_r_form_minor_ccw():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 20.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, e, "G17", clockwise=False, radius_r=10.0)
    assert g.ok
    assert abs(g.center["x"] - 10.0) < 1e-9
    assert abs(g.center["y"] - 0.0) < 1e-9
    assert abs(g.sweep_deg - 180.0) < 1e-9


def test_r_form_negative_major():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 10.0, "y": 0.0, "z": 0.0}
    g_minor = resolve_arc(s, e, "G17", clockwise=False, radius_r=10.0)
    g_major = resolve_arc(s, e, "G17", clockwise=False, radius_r=-10.0)
    assert g_minor.ok and g_major.ok
    assert abs(g_minor.sweep_deg - 60.0) < 1e-9
    assert abs(g_major.sweep_deg - 300.0) < 1e-9


def test_r_chord_exceeds_diameter():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 25.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, e, "G17", clockwise=False, radius_r=10.0)
    assert not g.ok and "弦长" in g.reason


def test_r_with_coincident_endpoints_rejected():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, s, "G17", clockwise=False, radius_r=10.0)
    assert not g.ok and "I/J/K" in g.reason


def test_helical_arc_depth_and_length():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 20.0, "y": 0.0, "z": -5.0}
    c = {"x": 10.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, e, "G17", clockwise=False, center=c)
    assert g.ok
    assert abs(g.helical_depth + 5.0) < 1e-9
    assert g.path_length > g.arc_length
    assert abs(g.samples[-1]["z"] + 5.0) < 1e-9


def test_g18_plane_axes_and_center_words():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    e = {"x": 0.0, "y": 0.0, "z": 20.0}
    # G18: h=Z, v=X，圆心 (h=10,v=0)
    c = {"x": 0.0, "y": 0.0, "z": 10.0}
    g = resolve_arc(s, e, "G18", clockwise=False, center=c)
    assert g.ok
    assert abs(g.radius - 10.0) < 1e-9
    assert abs(g.sweep_deg - 180.0) < 1e-9


def test_missing_center_and_radius():
    s = {"x": 0.0, "y": 0.0, "z": 0.0}
    g = resolve_arc(s, {"x": 5.0, "y": 5.0, "z": 0.0}, "G17", clockwise=True)
    assert not g.ok and "缺少" in g.reason
