"""审查引擎：轨迹重建与五类核心诊断 + 危险模态继承。"""

from gcode_review.engine import review_program


def codes(report, line=None):
    out = [d["code"] for d in report["diagnostics"] if line is None or d["line_no"] == line]
    return out


def test_clean_program_has_no_errors(base_config):
    prog = """G21 G90 G17 G54
T1 M06
G43 H1 Z80.
M03 S2000
G00 X10 Y10 Z55
G01 Z-5 F200
G02 X30 Y10 I10 J0 F150
G01 Y30
G00 Z80
M05
G40 G49
M30
"""
    rep = review_program(prog, base_config, "ok.nc")
    assert rep["risk"] in ("clean", "warning")
    assert rep["diagnostic_counts"]["error"] == 0
    assert codes(rep).count("ENVELOPE_VIOLATION") == 0
    assert rep["trajectory"]["segments"] >= 6


def test_machine_coordinate_transform(base_config):
    """G54 偏置 (-150,-100,-300) + 刀长 120：work (0,0,0) -> machine (-150,-100,-180)。"""
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0
G01 Z0 F200
M05
G49
M30
"""
    rep = review_program(prog, base_config)
    cutting = [m for m in rep["moves"] if m["kind"] == "linear"]
    z0 = [m for m in cutting if abs(m["end_machine"]["z"] + 180.0) < 1e-6]
    assert z0, "work z=0 应映射为 machine z = -300+120 = -180"
    xy = [m for m in rep["moves"] if m["kind"] == "rapid"
          and abs(m["end_machine"]["x"] + 150.0) < 1e-6]
    assert xy


def test_envelope_violation_x(base_config):
    # G54 偏置 x=-150；work x=200 -> machine 50 -> 改 work x=500 -> machine 350 越界
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X500 Y0
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    env = [d for d in rep["diagnostics"] if d["code"] == "ENVELOPE_VIOLATION"]
    assert any("X 轴越界" in d["message"] for d in env)
    d = next(d for d in env if "X 轴" in d["message"])
    assert d["segment_index"] is not None
    assert d["preceding_state"]["work_offset"] == "G54"
    assert "xmax=300" in d["basis"]


def test_tool_diameter_counts_in_envelope(base_config):
    """刀具半径 5mm：work x=456 -> machine 306，加半径 311 越界；不加半径则 306 也越界。"""
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X451 Y0
M05
G49 M30
"""
    # work x=451 -> machine 301；加半径 5 -> 306 越界
    rep = review_program(prog, base_config)
    assert any(d["code"] == "ENVELOPE_VIOLATION" and "X" in d["message"]
               for d in rep["diagnostics"])
    base_config["check_tool_envelope"] = False
    rep2 = review_program(prog, base_config)
    # 301 仍越界（xmax=300）；改成 450 -> 300 边界：计入半径越界，关闭则不越界
    prog_edge = prog.replace("X451", "X450")
    rep3 = review_program(prog_edge, {**base_config, "check_tool_envelope": True})
    rep4 = review_program(prog_edge, {**base_config, "check_tool_envelope": False})
    assert any(d["code"] == "ENVELOPE_VIOLATION" for d in rep3["diagnostics"])
    assert not any(d["code"] == "ENVELOPE_VIOLATION" for d in rep4["diagnostics"])


def test_undefined_tool_on_m06(base_config):
    prog = """G21 G90 G54
T9 M06
G00 G43 H9 Z80.
M30
"""
    rep = review_program(prog, base_config)
    undef = [d for d in rep["diagnostics"] if d["code"] == "UNDEFINED_TOOL"]
    assert undef and undef[0]["severity"] == "error"
    assert "T9" in undef[0]["message"]


def test_m06_without_t_is_error(base_config):
    rep = review_program("G21\nM06\nM30\n", base_config)
    assert any(d["code"] == "UNDEFINED_TOOL" for d in rep["diagnostics"])


def test_undefined_h_offset(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H7 Z80.
M30
"""
    rep = review_program(prog, base_config)
    d = [x for x in rep["diagnostics"] if x["code"] == "UNDEFINED_TOOL_LENGTH"]
    assert d and "H7" in d[0]["message"]


def test_rapid_below_safety_with_xy_is_error(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X20 Y20 Z10
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    hit = [d for d in rep["diagnostics"] if d["code"] == "RAPID_BELOW_SAFETY"]
    assert hit and hit[0]["severity"] == "error"
    assert "XY有联动" in hit[0]["basis"]


def test_rapid_vertical_down_below_safety_is_warning(base_config):
    # 先安全高度定位 XY，再单独 Z 向快移下探
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X20 Y20
G00 Z10
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    hit = [d for d in rep["diagnostics"] if d["code"] == "RAPID_BELOW_SAFETY"
           and d["line_no"] == 6]
    assert hit and hit[0]["severity"] == "warning"


def test_rapid_up_retract_is_info(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X20 Y20 Z55
G01 Z5 F200
G00 Z55
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    hit = [d for d in rep["diagnostics"] if d["code"] == "RAPID_BELOW_SAFETY"
           and d["line_no"] == 7]
    assert hit and hit[0]["severity"] == "info"


def test_arc_geometry_error_radius_mismatch(base_config):
    # I5 对应半径 5，但终点 (11,0) 到圆心 (5,0) 是 6 -> 不一致
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0 Z55
G01 Z-5 F200
G02 X11 Y0 I5 J0 F150
M05
G00 Z80
G49 M30
"""
    rep = review_program(prog, base_config)
    arcs = [d for d in rep["diagnostics"] if d["code"] == "ARC_GEOMETRY_ERROR"]
    assert arcs and "距离不一致" in arcs[0]["message"]
    assert arcs[0]["preceding_state"]["plane"] == "G17"


def test_arc_chord_too_big_with_r(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0 Z55
G01 Z-5 F200
G02 X25 Y0 R10 F150
M05
G00 Z80
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "ARC_GEOMETRY_ERROR" and "弦长" in d["message"]
               for d in rep["diagnostics"])


def test_arc_r_and_ijk_prefers_ijk(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0 Z55
G01 Z-5 F200
G02 X20 Y0 I10 J0 R99 F150
M05
G00 Z80
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "ARC_BOTH_R_IJK" for d in rep["diagnostics"])
    arc_moves = [m for m in rep["moves"] if m["kind"] == "arc"]
    assert arc_moves and abs(arc_moves[0]["radius"] - 10.0) < 1e-6


def test_incremental_and_inch_conversion(base_config):
    # G20 英寸：X1 in = 25.4mm；增量累计
    prog = """G20 G91 G17 G54
T1 M06
G00 G43 H1 Z5.
M03 S1000
G00 X1 Y1
G01 X-0.5 F10.
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    assert rep["units"] == "inch"
    moves = rep["moves"]
    # 初始 machine x=xmin=-300；+1in(25.4mm) 后 -274.6
    assert any(abs(m["end_machine"]["x"] + 274.6) < 1e-6 for m in moves)
    # 切削 -0.5in -> -287.3
    cut = [m for m in moves if m["kind"] == "linear"]
    assert cut and abs(cut[0]["end_machine"]["x"] + 287.3) < 1e-6
    # F10 in/min = 254 mm/min
    assert cut[0]["feed_mm_min"] == 254.0


def test_units_assumed_warning_when_missing(base_config):
    rep = review_program("G90 G54\nT1 M06\nG00 G43 H1 Z80.\nM03 S1000\nG01 X10 F200\nM30\n",
                         base_config)
    assert any(d["code"] == "UNITS_ASSUMED_MM" for d in rep["diagnostics"])
    assert "未声明单位" in " ".join(rep["assumptions"])


def test_implicit_motion_mode_warning(base_config):
    rep = review_program("G21 G90 G54\nX10 Y10\nM30\n", base_config)
    d = [x for x in rep["diagnostics"] if x["code"] == "IMPLICIT_MOTION_MODE"]
    assert d and d[0]["preceding_state"]["motion_mode"] is None


def test_spindle_off_cutting_warning(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
G01 X10 F200
M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "SPINDLE_OFF_CUTTING" for d in rep["diagnostics"])


def test_feed_missing_defaults_and_time(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0 Z55
G01 Z0
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "FEED_MISSING" for d in rep["diagnostics"])
    # 缺 F 按 500mm/min；work z55->0 即 machine -125->-180 距离 55mm
    lin = [m for m in rep["moves"] if m["line_no"] == 6]
    assert lin and abs(lin[0]["duration_s"] - 55 / 500 * 60) < 1e-6


def test_g95_feed_uses_spindle_rpm(base_config):
    # F0.1 mm/rev * S2000 = 200 mm/min，走 10mm 应 3s
    prog = """G21 G90 G95 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X0 Y0 Z55
G01 X10 Z0 F0.1
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    lin = [m for m in rep["moves"] if m["kind"] == "linear"]
    assert lin and abs(lin[0]["feed_mm_min"] - 200.0) < 1e-6


def test_g95_without_spindle_warns(base_config):
    prog = """G21 G90 G95 G54
T1 M06
G00 G43 H1 Z80.
G01 X10 F0.1
M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "G95_NO_SPINDLE_SPEED" for d in rep["diagnostics"])


def test_no_program_end_warning(base_config):
    rep = review_program("G21 G90\nT1 M06\nG00 G43 H1 Z80.\nM03 S1000\nG01 X1 F200\nM05\n",
                         base_config)
    assert any(d["code"] == "NO_PROGRAM_END" for d in rep["diagnostics"])


def test_spindle_on_at_end_warning(base_config):
    rep = review_program("G21 G90\nM03 S1000\nG00 X1\nM30\n", base_config)
    # 末尾 M30 前主轴仍 M03 -> 报警
    assert any(d["code"] == "SPINDLE_ON_AT_END" for d in rep["diagnostics"])


def test_cutter_comp_active_at_end(base_config):
    rep = review_program("G21 G90\nT1 M06\nG00 G41 D1 X10 Y10\nM30\n", base_config)
    assert any(d["code"] == "CUTTER_COMP_ACTIVE_AT_END" for d in rep["diagnostics"])


def test_canned_cycle_modal_inheritance(base_config):
    """第二个孔位行只有 XY，必须继承 G81 产生完整钻孔动作；程序末尾 G81 仍模态要报警。"""
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X10 Y10
G81 R2 Z-10 F150
X30 Y30
G80
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    syn = [m for m in rep["moves"] if m.get("note", "").startswith("canned:G81")]
    # 两个孔：定位/to-R/drill/抬刀
    lines = {m["line_no"] for m in syn}
    assert lines == {6, 7}
    # 不应有 CANNED_ACTIVE_AT_END（已 G80）
    assert not any(d["code"] == "CANNED_ACTIVE_AT_END" for d in rep["diagnostics"])


def test_canned_cycle_active_at_end(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G81 X10 Y10 R2 Z-10 F150
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "CANNED_ACTIVE_AT_END" for d in rep["diagnostics"])


def test_canned_cycle_missing_zr_is_error(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G81 X10 Y10 F150
M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "CANNED_CYCLE_INCOMPLETE" for d in rep["diagnostics"])


def test_peck_cycle_g83_retracts(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X10 Y10 Z5
G83 R2 Z-8 Q3 F150
G80
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    rapid_notes = [m["note"] for m in rep["moves"]
                   if m["kind"] == "rapid" and "canned:G83" in m.get("note", "")]
    assert any("retract" in n for n in rapid_notes)
    assert any("reapproach" in n for n in rapid_notes)


def test_g92_shift_warns_and_remaps(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G92 X0 Y0 Z0
G01 X5 F200
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "G92_COORD_SET" for d in rep["diagnostics"])
    # 不崩且有轨迹
    assert rep["trajectory"]["segments"] >= 2


def test_work_offset_switch_keeps_machine_position(base_config):
    cfg = {**base_config}
    cfg["work_offsets"] = {**cfg["work_offsets"],
                           "G55": {"x": -50.0, "y": -20.0, "z": -300.0}}
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G00 X0 Y0
G55
G01 X0 Y0 F200
M05
G49 M30
"""
    rep = review_program(prog, cfg)
    sw = [d for d in rep["diagnostics"] if d["code"] == "WORK_OFFSET_SWITCH"]
    assert sw and "G54 -> G55" in sw[0]["message"]
    # G55 下 work (0,0) 的 machine = (-50,-20)；切换后立即 G01 X0 Y0 应移动到该物理点
    cut = [m for m in rep["moves"] if m["kind"] == "linear"]
    assert cut
    assert abs(cut[0]["end_machine"]["x"] + 50.0) < 1e-6
    assert abs(cut[0]["end_machine"]["y"] + 20.0) < 1e-6


def test_g52_local_offset(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G52 X10 Y0 Z0
G00 X0 Y0
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    assert any(d["code"] == "LOCAL_OFFSET_SET" for d in rep["diagnostics"])
    # work(局部) x=0 -> 全局 work x=10 -> machine -140
    hit = [m for m in rep["moves"]
           if m["kind"] == "rapid" and abs(m["end_machine"]["x"] + 140.0) < 1e-6]
    assert hit


def test_tool_change_time_counted(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
T2 M06
G00 G43 H2 Z80.
M30
"""
    rep = review_program(prog, base_config)
    assert rep["time_estimate_s"]["tool_change"] == 16.0


def test_dwell_time(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S1000
G04 P500
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    dwell = [m for m in rep["moves"] if m["kind"] == "dwell"]
    assert dwell and abs(dwell[0]["duration_s"] - 0.5) < 1e-9
    assert abs(rep["time_estimate_s"]["dwell"] - 0.5) < 1e-9


def test_trajectory_bounds_and_time_report(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X0 Y0 Z55
G01 Z0 F300
G00 Z80
M05
G49 M30
"""
    rep = review_program(prog, base_config)
    bm = rep["trajectory"]["bounds_machine"]
    bw = rep["trajectory"]["bounds_work"]
    # work bounds: x/y 0, z 从初始 (zmax+300-120=680?) 到 0
    assert bw["min"]["z"] <= 0.0
    assert bw["max"]["z"] >= 80.0
    # machine z 最低点 = 0-300+120 = -180
    assert abs(bm["min"]["z"] + 180.0) < 1e-6
    assert rep["time_estimate_s"]["total"] > 0
    assert "rapid" in rep["time_basis"]


def test_preceding_state_attached(base_config):
    prog = """G21 G90 G54
T9 M06
M30
"""
    rep = review_program(prog, base_config)
    d = next(d for d in rep["diagnostics"] if d["code"] == "UNDEFINED_TOOL")
    pre = d["preceding_state"]
    assert pre["units"] == "mm"
    assert pre["distance_mode"] == "G90"
    assert pre["work_offset"] == "G54"


def test_helical_arc_g17_z_movement(base_config):
    prog = """G21 G90 G54
T1 M06
G00 G43 H1 Z80.
M03 S2000
G00 X0 Y0 Z5
G02 X0 Y0 Z-5 I10 J0 F300
M05
G00 Z80
G49 M30
"""
    rep = review_program(prog, base_config)
    arc = [m for m in rep["moves"] if m["kind"] == "arc"]
    assert arc
    assert abs(abs(arc[0]["sweep_deg"]) - 360.0) < 1e-6
    assert abs(arc[0]["helical_depth"] + 10.0) < 1e-6
