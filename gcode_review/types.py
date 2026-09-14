"""核心数据类型：词法单元、行、状态快照、运动段、诊断结果。"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional


# 模态/非模态 G 代码分组。非模态组 motion/nonmodal 内的代码在块结束后清空。
GROUPS: Dict[str, set] = {
    "motion": {"G00", "G01", "G02", "G03"},
    "plane": {"G17", "G18", "G19"},
    "distance": {"G90", "G91"},
    "arc_distance": {"G90.1", "G91.1"},
    "units": {"G20", "G21"},
    "feed_mode": {"G93", "G94", "G95"},
    "cutter_radius": {"G40", "G41", "G42"},
    "tool_length": {"G43", "G44", "G49"},
    "work_offset": {f"G5{n}" for n in range(4, 10)},  # G54..G59
    "return_mode": {"G98", "G99"},
    "canned_cycle": {
        "G73", "G74", "G76",
        "G81", "G82", "G83", "G84", "G85", "G86", "G87", "G88", "G89",
    },
    "canned_cancel": {"G80"},
    "path_control": {"G61", "G61.1", "G64"},
    "nonmodal": {"G04", "G09", "G28", "G30", "G52", "G53", "G92", "G92.1"},
}

# 反向索引：G 代码 -> 组名
CODE_GROUP: Dict[str, str] = {
    code: group for group, codes in GROUPS.items() for code in codes
}


@dataclass
class Word:
    """一个词法单元，如 X12.5、G01、F300。"""

    letter: str
    value: float
    raw: str


@dataclass
class Block:
    """一行有效 G-code（去掉空行/纯注释后）。"""

    line_no: int                 # 物理行号，从 1 开始
    source: str                  # 原始行（去尾部换行）
    words: Dict[str, Word]       # 轴/参数字 -> Word（重复字母保留最后一个）
    g_codes: list                # list[str]，按出现顺序，如 ["G01", "G90"]
    m_codes: list                # list[str]，如 ["M03"]
    labels: list                 # N 行号词（记录但不参与运动）


@dataclass
class Diagnostic:
    """单项诊断。"""

    code: str                            # 诊断代码（英文枚举）
    severity: str                        # error / warning / info
    line_no: Optional[int]               # 定位行号
    message: str                         # 中文描述
    basis: str                           # 判定依据（算式、模态、阈值）
    preceding_state: Dict[str, Any] = field(default_factory=dict)  # 前置状态
    segment_index: Optional[int] = None  # 关联运动段（若有）
    details: Dict[str, Any] = field(default_factory=dict)  # 结构化附加数据（如碰撞详情）

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Move:
    """重建出的一段运动（物理机床坐标，mm）。"""

    index: int
    kind: str          # rapid / linear / arc / dwell
    line_no: int
    start: Dict[str, float]   # 物理起点 {x,y,z}
    end: Dict[str, float]     # 物理终点（dwell 时等于起点）
    plane: str                # G17/G18/G19
    radius: Optional[float] = None
    center: Optional[Dict[str, float]] = None   # 圆弧圆心（物理坐标）
    sweep_deg: Optional[float] = None          # 带符号扫略角
    helical_depth: Optional[float] = None      # 非插补平面的螺旋深度
    feed_mm_min: Optional[float] = None        # 生效进给（rapid/dwell 为 None）
    duration_s: float = 0.0
    samples: list = field(default_factory=list)  # 物理轨迹采样点（越界检测用）
    note: str = ""                             # 如 canned:G81 合成段
    synthetic: bool = False                    # 固定循环合成段

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
