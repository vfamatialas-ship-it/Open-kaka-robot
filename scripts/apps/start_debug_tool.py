"""Start the desktop debug tool.

启动上位机调试工具。
"""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.apps.debug_tool_gui import main


if __name__ == "__main__":
    main()
