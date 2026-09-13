"""G-code 词法/语法解析。

输入程序文本，输出 :class:`Block` 列表。本模块只做解析，不做运动学/模态推导，
也不做风险判定；解析层面的异常（坏词、未知 G/M 代码等）以 Diagnostic 返回，
由引擎统一汇总。
"""

from __future__ import annotations

import re
from typing import List, Tuple

from .types import Block, Diagnostic, Word, CODE_GROUP

# 词法单元：字母 + 带符号数值。允许 1. 或 .5 这类写法，也允许 G90.1。
_WORD_RE = re.compile(
    r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)"
)
# 合法词字母白名单（N/O 单独处理）
_WORD_LETTERS = set("XYZABCUVWIJKRFHSDTLP")
_AXIS_LETTERS = set("XYZABCUVW")
_PARAM_LETTERS = set("IJKRFHSDTLP")


def _strip_comment(line: str) -> Tuple[str, List[str]]:
    """去掉圆括号注释；返回（干净文本, 注释列表）。行内 () 直接剔除。"""
    comments: List[str] = []
    out = []
    depth = 0
    buf: List[str] = []
    for ch in line:
        if ch == "(":
            depth += 1
            buf = []
            continue
        if ch == ")":
            depth = max(0, depth - 1)
            comments.append("".join(buf))
            buf = []
            continue
        if depth:
            buf.append(ch)
        else:
            out.append(ch)
    # 未闭合括号：把残留当注释内容，保证不崩
    if depth:
        comments.append("".join(buf))
    return "".join(out), comments


def parse_program(text: str) -> Tuple[List[Block], List[Diagnostic]]:
    """解析整个程序文本。

    :returns: (blocks, diagnostics)。空白/纯注释行不出现在 blocks 中，
        但诊断行号仍按物理行计算。
    """
    blocks: List[Block] = []
    diags: List[Diagnostic] = []

    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        clean, _comments = _strip_comment(raw_line)
        tokens = list(_WORD_RE.finditer(clean))
        if not tokens:
            continue

        words: dict = {}
        g_codes: List[str] = []
        m_codes: List[str] = []
        labels: List[Word] = []
        parsed_spans = []

        for tok in tokens:
            letter = tok.group(1).upper()
            value = float(tok.group(2))
            parsed_spans.append(tok.span())
            word = Word(letter=letter, value=value, raw=tok.group(0))

            if letter == "N":
                labels.append(word)
                continue
            if letter == "O":
                # 程序号：行首 O0001 之类，忽略不参与运动
                continue

            if letter == "G":
                code = _canon_code(value, "G")
                g_codes.append(code)
            elif letter == "M":
                code = _canon_code(value, "M")
                m_codes.append(code)
            elif letter in _AXIS_LETTERS or letter in _PARAM_LETTERS:
                if letter in words:
                    diags.append(Diagnostic(
                        code="DUPLICATE_WORD",
                        severity="warning",
                        line_no=line_no,
                        message=f"第 {line_no} 行中 {letter} 字重复，以后值 {value:g} 为准",
                        basis=(f"同一程序段内 {letter} 出现多次：{words[letter].raw} 与 "
                               f"{tok.group(0)}，按 RS-274 取最后值"),
                    ))
                words[letter] = word
            else:
                diags.append(Diagnostic(
                    code="UNKNOWN_WORD",
                    severity="warning",
                    line_no=line_no,
                    message=f"第 {line_no} 行含未识别词 {tok.group(0)}",
                    basis=f"字母 {letter} 不在支持的字集合内，已忽略",
                ))

        # 检查是否有无法词法化的残留字符（剔除空白）
        residual = list(clean)
        for span in parsed_spans:
            residual[span[0]:span[1]] = [" "] * (span[1] - span[0])
        garbage = "".join(residual).replace(" ", "").replace("\t", "")
        # N 标签已被词法化；其余残留（如裸 '/' 跳段符）单独提示
        if "/" in garbage:
            diags.append(Diagnostic(
                code="BLOCK_DELETE_IGNORED",
                severity="info",
                line_no=line_no,
                message=f"第 {line_no} 行含跳段符 '/'，静态审查按该段生效处理",
                basis="跳段开关取决于机床面板状态，服务端无法获知，保守按段执行审查",
            ))
            garbage = garbage.replace("/", "")
        if garbage:
            diags.append(Diagnostic(
                code="LEXICAL_ERROR",
                severity="error",
                line_no=line_no,
                message=f"第 {line_no} 行存在无法解析的字符：{garbage!r}",
                basis=f"词法规则为 字母+数值；残留 {garbage!r} 不符合",
            ))

        # 同组 G 代码冲突（如 G01 G02 同行）
        seen_group: dict = {}
        for code in g_codes:
            group = CODE_GROUP.get(code)
            if group is None:
                diags.append(Diagnostic(
                    code="UNKNOWN_GCODE",
                    severity="warning",
                    line_no=line_no,
                    message=f"第 {line_no} 行含不支持的 G 代码 {code}，已忽略",
                    basis="审查器仅实现常用 G00-G99 子集，该代码不改变重建状态",
                ))
                continue
            if group in seen_group and seen_group[group] != code:
                diags.append(Diagnostic(
                    code="MODAL_GROUP_CONFLICT",
                    severity="error",
                    line_no=line_no,
                    message=(f"第 {line_no} 行同组模态冲突：{seen_group[group]} 与 {code}"
                             f"（{group} 组）"),
                    basis=f"RS-274 同一程序段内同组代码只能出现一个，运动语义不确定",
                ))
            seen_group[group] = code

        for code in m_codes:
            if code not in _SUPPORTED_M:
                diags.append(Diagnostic(
                    code="UNKNOWN_MCODE",
                    severity="info",
                    line_no=line_no,
                    message=f"第 {line_no} 行含不支持的 M 代码 {code}，不影响轨迹重建",
                    basis="M 代码不驱动坐标轴；仅记录，未纳入碰撞分析",
                ))

        if g_codes or m_codes or words or labels:
            blocks.append(Block(
                line_no=line_no,
                source=raw_line.rstrip("\r\n"),
                words=words,
                g_codes=g_codes,
                m_codes=m_codes,
                labels=labels,
            ))

    return blocks, diags


def _canon_code(value: float, prefix: str) -> str:
    """把 G/M 数值规范化成比较用字符串：整数去 .0，保留 G90.1 形式。"""
    if abs(value - round(value)) < 1e-9:
        return f"{prefix}{int(round(value)):02d}"
    return f"{prefix}{value:g}"


_SUPPORTED_M = {
    "M00", "M01", "M02", "M03", "M04", "M05",
    "M06", "M07", "M08", "M09", "M30",
}
