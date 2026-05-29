"""Global safety state service.

全局安全状态服务。

当前阶段只管理软件层面的急停状态。后续加入运动控制时，所有运动命令都应先检查
这里的急停状态。
"""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class SafetyState:
    """Current global safety state."""

    emergency_stop: bool = False
    message: str = "系统正常，只读模式"
    updated_at: str = ""


class SafetyService:
    """Manage global safety state."""

    def __init__(self) -> None:
        self.state = SafetyState(updated_at=self._now())

    def trigger_emergency_stop(self, reason: str = "用户触发急停") -> SafetyState:
        """Enter emergency-stop state."""

        self.state.emergency_stop = True
        self.state.message = reason
        self.state.updated_at = self._now()
        return self.state

    def clear_emergency_stop(self) -> SafetyState:
        """Clear emergency-stop state.

        这里只清除软件急停标记。真实运动功能加入后，清除急停前还需要确认硬件状态。
        """

        self.state.emergency_stop = False
        self.state.message = "系统正常，只读模式"
        self.state.updated_at = self._now()
        return self.state

    def status_text(self) -> str:
        """Return a user-facing status string."""

        prefix = "急停中" if self.state.emergency_stop else "安全"
        return f"{prefix}: {self.state.message} ({self.state.updated_at})"

    @staticmethod
    def _now() -> str:
        return time.strftime("%H:%M:%S")
