"""G-code 静态审查服务：解析、轨迹重建与撞机风险诊断。"""

from .app import create_app

__all__ = ["create_app"]
