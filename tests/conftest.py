import os
import sys

sys.path.insert(0, "/tmp/pylibs")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gcode_review.config import DEFAULT_CONFIG
import pytest


@pytest.fixture
def base_config():
    """一份自洽的小机床配置：G54 零点在 (-150,-100,-300)。"""
    cfg = {
        **DEFAULT_CONFIG,
        # 立式机床：Z 零点（主轴端面）在台面之上，工作 z=0 经 G54(-300)+刀长(120)
        # 后为机床 z=-180，故 zmin 必须低于台面
        "envelope": {"xmin": -320.0, "xmax": 320.0,
                     "ymin": -220.0, "ymax": 220.0,
                     "zmin": -400.0, "zmax": 520.0},
        "work_offsets": {
            **{f"G5{n}": {"x": 0.0, "y": 0.0, "z": 0.0} for n in range(4, 10)},
            "G54": {"x": -150.0, "y": -100.0, "z": -300.0},
        },
        "safety_clearance": 50.0,
        "tools": {"1": {"length": 120.0, "diameter": 10.0, "description": "10mm 立铣刀"},
                  "2": {"length": 80.0, "diameter": 6.0}},
        "h_offsets": {},
        "d_offsets": {},
        "rapid_speed": 8000.0,
        "default_feed": 500.0,
        "default_spindle": 1000.0,
        "tool_change_time": 8.0,
        "arc_tolerance": 0.01,
        "check_tool_envelope": True,
    }
    return cfg
