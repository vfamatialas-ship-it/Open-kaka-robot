"""Read pink master arm status.

读取粉色主臂状态，只读。
"""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import open_arm_connection


def main() -> None:
    connection = open_arm_connection("pink_master")
    try:
        print("\n".join(connection.read_once()))
    finally:
        connection.close()


if __name__ == "__main__":
    main()
