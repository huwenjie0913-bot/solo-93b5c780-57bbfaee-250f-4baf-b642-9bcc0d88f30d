"""词法/语法解析器测试。"""

from gcode_review.parser import parse_program


def test_comments_and_blank_lines():
    text = "(头部注释)\nG21 G90\n\n  (纯注释行)\nG00 X1\n"
    blocks, diags = parse_program(text)
    assert [b.line_no for b in blocks] == [2, 5]
    assert blocks[1].words["X"].value == 1.0
    assert diags == []


def test_inline_parenthesized_comment():
    blocks, diags = parse_program("G1 X10.5 F300 (切到 10.5)\n")
    assert len(blocks) == 1
    assert blocks[0].words["X"].value == 10.5
    assert blocks[0].words["F"].value == 300.0
    assert diags == []


def test_g_code_canonical_and_modal_conflict():
    blocks, diags = parse_program("G01 G02 X1\n")
    codes = [d.code for d in diags]
    assert "MODAL_GROUP_CONFLICT" in codes
    assert blocks[0].g_codes == ["G01", "G02"]


def test_duplicate_axis_word():
    blocks, diags = parse_program("G01 X1 X2\n")
    assert blocks[0].words["X"].value == 2.0
    assert any(d.code == "DUPLICATE_WORD" for d in diags)


def test_unknown_g_code_and_m_code():
    _, diags = parse_program("G999 X1\nM47\n")
    codes = {d.code for d in diags}
    assert "UNKNOWN_GCODE" in codes
    assert "UNKNOWN_MCODE" in codes


def test_lexical_error_on_garbage():
    _, diags = parse_program("G01 X@10\n")
    assert any(d.code == "LEXICAL_ERROR" for d in diags)


def test_block_delete_marker_is_info():
    _, diags = parse_program("/G00 X1\n")
    assert any(d.code == "BLOCK_DELETE_IGNORED" and d.severity == "info"
               for d in diags)


def test_decimal_and_scientific_numbers():
    blocks, _ = parse_program("G01 X.5 Y-10. Z1e2\n")
    w = blocks[0].words
    assert w["X"].value == 0.5
    assert w["Y"].value == -10.0
    assert w["Z"].value == 100.0


def test_g90_1_preserved():
    blocks, _ = parse_program("G90.1 G02 X0 Y10 I-5 J0\n")
    assert "G90.1" in blocks[0].g_codes
