"""三项可用性缺陷的回归测试。

1. 服务可安装、入口可启动（依赖声明由 requirements/pyproject 保证，这里验 create_app）；
2. 缺省起始点对最大刀具实体安全，安全程序首段不报越界；
3. "G00 G43 H1 Z100" 同行补偿不再凭空产生刀长跳变。
"""

import math

from gcode_review.app import create_app
from gcode_review.engine import review_program


# ---------- 缺陷 3：G43 同行 H 的刀长跳变 ---------- #
def test_g43_same_line_h1_no_phantom_jump(base_config):
    """work z=100、刀长 120、G54 z=-300 => 机床 z 必须恰为 -80，不多不少。"""
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z100.
M30
"""
    rep = review_program(prog, base_config)
    move = [m for m in rep["moves"] if m["line_no"] == 3][0]
    assert abs(move["end_machine"]["z"] - (-80.0)) < 1e-9
    assert abs(move["start_machine"]["z"] - 520.0) < 1e-9
    # 行程 = 520 - (-80) = 600mm，快移 8000mm/min
    assert abs(move["duration_s"] - 600.0 / 8000.0 * 60.0) < 1e-9
    env = [d for d in rep["diagnostics"] if d["code"] == "ENVELOPE_VIOLATION"]
    assert env == []


def test_g43_z100_no_tool_offset_when_tool_length_50():
    """用户场景：刀长 50，G54 z=0，包络 z 0..450：work100 -> machine150，无 50mm 跳变。"""
    cfg = {
        "envelope": {"xmin": 0, "xmax": 500, "ymin": 0, "ymax": 400,
                     "zmin": 0, "zmax": 450},
        "work_offsets": {f"G5{n}": {"x": 0.0, "y": 0.0, "z": 0.0}
                         for n in range(4, 10)},
        "safety_clearance": 50.0,
        "tools": {"1": {"length": 50.0, "diameter": 10.0}},
    }
    prog = """G21 G90 G17 G54
T1 M06
G00 G43 H1 Z100.
M30
"""
    rep = review_program(prog, cfg)
    move = [m for m in rep["moves"] if m["line_no"] == 3][0]
    assert abs(move["end_machine"]["z"] - 150.0) < 1e-9
    assert not any(d["code"] == "ENVELOPE_VIOLATION" for d in rep["diagnostics"])


def test_g43_switch_h_remaps_in_same_block(base_config):
    """同行 G43 H 必须在执行 Z 运动前生效；切换补偿不凭空产生刀长跳变。"""
    cfg = {**base_config,
           "h_offsets": {"1": 120.0, "2": 80.0}}
    prog = """G21 G90 G54
T2 M06
G00 G43 H2 Z100.
G00 G43 H1 Z90.
M30
"""
    rep = review_program(prog, cfg)
    # L3: H2 刀长 80，work100 -> machine -120
    # L4: 先从 H2 切到 H1（物理不动），work 坐标重映射为 140；
    #     再 G00 到 work90 -> machine -90。物理位移恰为 -120 -> -90 = 30mm。
    m3 = [m for m in rep["moves"] if m["line_no"] == 3][0]
    m4 = [m for m in rep["moves"] if m["line_no"] == 4][0]
    assert abs(m3["end_machine"]["z"] + 120.0) < 1e-9
    assert abs(m4["start_machine"]["z"] + 120.0) < 1e-9
    assert abs(m4["end_machine"]["z"] + 90.0) < 1e-9
    assert abs((m4["end_machine"]["z"] - m4["start_machine"]["z"]) - 30.0) < 1e-9
    assert not any(d["code"] == "ENVELOPE_VIOLATION" for d in rep["diagnostics"])


# ---------- 缺陷 2：安全初始点 ---------- #
def test_default_start_is_inside_envelope_with_tool(base_config):
    """安全程序首段不得因假设起点贴边而误报越界。"""
    prog = """G21 G90 G17 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X10 Y10 Z55
G01 Z-5 F200
G00 Z80
M05
G49
M30
"""
    rep = review_program(prog, base_config)
    env = [d for d in rep["diagnostics"] if d["code"] == "ENVELOPE_VIOLATION"]
    assert env == [], [d["message"] for d in env]


def test_default_start_inset_by_largest_tool_radius():
    cfg = {
        "envelope": {"xmin": 0, "xmax": 500, "ymin": 0, "ymax": 400,
                     "zmin": 0, "zmax": 450},
        "work_offsets": {f"G5{n}": {"x": 0.0, "y": 0.0, "z": 0.0}
                         for n in range(4, 10)},
        "safety_clearance": 50.0,
        "tools": {"1": {"length": 100.0, "diameter": 16.0}},   # 半径 8
        "d_offsets": {"2": 12.0},                              # 最大半径 12
    }
    rep = review_program("G21 G90 G54\nT1 M06\nG00 G43 H1 Z100.\nM30\n", cfg)
    p = rep["trajectory"]["initial_machine_position"]
    # 仅声明位置、不产生段也要查边界；初始点 x/y 内缩 12mm，z 取 zmax
    assert abs(p["x"] - 12.0) < 1e-6
    assert abs(p["y"] - 12.0) < 1e-6
    assert abs(p["z"] - 450.0) < 1e-6


def test_explicit_start_position_honored(base_config):
    cfg = {**base_config, "start_position": {"x": 10.0, "y": 20.0, "z": 300.0}}
    rep = review_program("G21 G90 G54\nT1 M06\nG00 G43 H1 Z80.\nM30\n", cfg)
    p = rep["trajectory"]["initial_machine_position"]
    assert p == {"x": 10.0, "y": 20.0, "z": 300.0}
    # 用了显式起点则不再有“初始点假设”提示
    assert not any(d["code"] == "INITIAL_POSITION_ASSUMED"
                   for d in rep["diagnostics"])


# ---------- 缺陷 1：应用入口可启动 ---------- #
def test_create_app_health(tmp_path):
    app = create_app(str(tmp_path / "test.db"))
    client = app.test_client()
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.get_json()["status"] == "ok"


def test_entrypoint_main_callable():
    from gcode_review.app import main
    from gcode_review import __main__ as mod  # noqa: F401
    assert callable(main)
