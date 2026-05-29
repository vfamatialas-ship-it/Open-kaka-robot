"""Generic read-only arm status CLI.

通用机械臂只读状态读取命令行入口。

示例：
  python scripts/read_arms/read_arm_status.py --arm pink_master --once
  python scripts/read_arms/read_arm_status.py --arm gray_slave --cycles 10
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ARM_PROFILES, open_arm_connection


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="读取指定机械臂状态，只读。")
    parser.add_argument("--arm", choices=ARM_PROFILES.keys(), required=True)
    parser.add_argument("--port", default=None, help="串口号；默认使用机械臂配置")
    parser.add_argument("--baudrate", type=int, default=None, help="波特率；默认使用机械臂配置")
    parser.add_argument("--once", action="store_true", help="只读取一次")
    parser.add_argument("--cycles", type=int, default=0, help="读取轮数；0 表示一直循环")
    parser.add_argument("--interval", type=float, default=0.2, help="读取间隔，单位秒")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    connection = open_arm_connection(args.arm, args.port, args.baudrate)
    try:
        cycle = 0
        while True:
            cycle += 1
            print("\n".join(connection.read_once()))
            print()

            if args.once or (args.cycles > 0 and cycle >= args.cycles):
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("Stopped by user / 用户停止。")
    finally:
        connection.close()


if __name__ == "__main__":
    main()
