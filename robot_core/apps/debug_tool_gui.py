"""Desktop debug tool for the dual-arm robot.

双臂机器人上位机调试工具。

布局原则：
  - 左侧：固定的连接设置和基础操作按钮
  - 右侧：功能页签，不同功能显示在不同区域

当前已实现功能：
  - 机械臂状态读取
  - 机械臂状态表格显示
  - 状态日志显示
  - 连续读取 / 停止连续读取
  - 零点管理
  - 关节限位示教
  - 单关节低速测试

安全约束：
  - 默认只读，不自动使能，不自动运动
  - 校准页运动测试只能单关节操作
  - 所有运动目标必须通过软件关节限位
  - 急停状态下禁止写参数和运动测试
"""

from __future__ import annotations

import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_core.services.arm_read_service import ARM_PROFILES, ArmReadConnection
from robot_core.services.arm_read_service import ArmStatusSnapshot
from robot_core.services.arm_read_service import get_arm_profile, list_serial_ports, open_arm_connection
from robot_core.services.hardware_zero_service import write_hardware_zero
from robot_core.services.joint_limit_service import check_snapshot_against_limits
from robot_core.services.joint_limit_service import default_limits_from_snapshot
from robot_core.services.joint_limit_service import format_limits_text
from robot_core.services.joint_limit_service import load_limits_text
from robot_core.services.joint_limit_service import parse_limits_text
from robot_core.services.joint_limit_service import save_limits_text
from robot_core.services.joint_limit_service import write_hardware_limits
from robot_core.services.safety_service import SafetyService
from robot_core.services.single_joint_motion_service import disable_arm_motion
from robot_core.services.single_joint_motion_service import hold_current_position
from robot_core.services.single_joint_motion_service import command_single_joint_target
from robot_core.services.single_joint_motion_service import step_single_joint
from robot_core.services.teleop_service import TeleopJointSetting, TeleopRuntimeSettings
from robot_core.services.teleop_service import TeleopState, build_anchor, compute_mapping_preview
from robot_core.services.teleop_service import make_default_joint_settings, target_status_is_commandable
from robot_core.services.teleop_service import targets_are_safe
from robot_core.services.teleop_service import update_continuous_position_map
from robot_core.services.zero_service import format_zero_snapshot, load_zero_text, save_zero_snapshot


class DebugToolApp(tk.Tk):
    """上位机调试工具主窗口。"""

    def __init__(self) -> None:
        super().__init__()
        self.title("Dual Arm Robot Debug Tool")
        self.geometry("1180x760")
        self.minsize(980, 620)

        self.profile_key = tk.StringVar(value="pink_master")
        self.port = tk.StringVar()
        self.baudrate = tk.StringVar()
        self.read_interval_ms = tk.StringVar(value="200")
        self.status = tk.StringVar(value="未连接")
        self.safety_status = tk.StringVar()
        self.limit_joint_name = tk.StringVar(value="joint1")
        self.motion_joint_name = tk.StringVar(value="joint1")
        self.motion_status = tk.StringVar(value="单关节测试未开始")
        self.teleop_master_key = tk.StringVar(value="pink_master")
        self.teleop_slave_key = tk.StringVar(value="pink_slave")
        self.teleop_master_port = tk.StringVar()
        self.teleop_master_baudrate = tk.StringVar()
        self.teleop_slave_port = tk.StringVar()
        self.teleop_slave_baudrate = tk.StringVar()
        self.teleop_joint_name = tk.StringVar(value="joint1")
        self.teleop_selected_enabled = tk.BooleanVar(value=False)
        self.teleop_selected_scale = tk.StringVar(value="1.0")
        self.teleop_selected_sign = tk.StringVar(value="+1")
        self.teleop_alpha = tk.StringVar(value="0.2")
        self.teleop_max_step = tk.StringVar(value="0.02")
        self.teleop_hz = tk.StringVar(value="20")
        self.teleop_damiao_wait = tk.StringVar(value="0.005")
        self.teleop_state_text = tk.StringVar(value=TeleopState.IDLE.value)
        self.teleop_state = TeleopState.IDLE

        self.connection: ArmReadConnection | None = None
        self.teleop_master_connection: ArmReadConnection | None = None
        self.teleop_slave_connection: ArmReadConnection | None = None
        self.teleop_lock = threading.Lock()
        self.teleop_running = False
        self.teleop_control_mode = "preview"
        self.teleop_control_joints: set[str] = set()
        self.teleop_master_anchor: dict[str, float] = {}
        self.teleop_slave_anchor: dict[str, float] = {}
        self.teleop_previous_targets: dict[str, float] = {}
        self.teleop_master_last_raw: dict[str, float] = {}
        self.teleop_master_continuous: dict[str, float] = {}
        self.teleop_joint_settings = make_default_joint_settings()
        self.read_lock = threading.Lock()
        self.continuous_reading = False
        self.last_snapshot: ArmStatusSnapshot | None = None
        self.taught_limits: dict[str, dict[str, float | None]] = {}
        self.safety_service = SafetyService()

        self._build_ui()
        self.update_safety_status()
        self.refresh_ports()
        self._apply_profile_defaults()
        self._apply_teleop_profile_defaults()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        self._build_sidebar(root)
        self._build_main_area(root)

    def _build_sidebar(self, parent: ttk.Frame) -> None:
        sidebar = ttk.Frame(parent, width=230)
        sidebar.grid(row=0, column=0, sticky=tk.NS, padx=(0, 10))
        sidebar.grid_propagate(False)

        connection_box = ttk.LabelFrame(sidebar, text="通讯设置", padding=10)
        connection_box.pack(fill=tk.X)

        ttk.Label(connection_box, text="机械臂").pack(anchor=tk.W)
        self.profile_combo = ttk.Combobox(
            connection_box,
            textvariable=self.profile_key,
            state="readonly",
            values=list(ARM_PROFILES.keys()),
        )
        self.profile_combo.pack(fill=tk.X, pady=(2, 8))
        self.profile_combo.bind("<<ComboboxSelected>>", lambda _event: self._apply_profile_defaults())

        ttk.Label(connection_box, text="串口号").pack(anchor=tk.W)
        self.port_combo = ttk.Combobox(connection_box, textvariable=self.port)
        self.port_combo.pack(fill=tk.X, pady=(2, 8))

        ttk.Label(connection_box, text="波特率").pack(anchor=tk.W)
        self.baud_combo = ttk.Combobox(
            connection_box,
            textvariable=self.baudrate,
            values=("115200", "500000", "921600", "1000000"),
        )
        self.baud_combo.pack(fill=tk.X, pady=(2, 10))

        ttk.Button(connection_box, text="刷新串口", command=self.refresh_ports).pack(
            fill=tk.X,
            pady=(0, 6),
        )
        ttk.Button(connection_box, text="打开串口", command=self.open_connection).pack(
            fill=tk.X,
            pady=(0, 6),
        )
        ttk.Button(connection_box, text="关闭串口", command=self.close_connection).pack(fill=tk.X)

        read_box = ttk.LabelFrame(sidebar, text="机械臂状态读取", padding=10)
        read_box.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(read_box, text="读取周期 ms").pack(anchor=tk.W)
        ttk.Entry(read_box, textvariable=self.read_interval_ms).pack(fill=tk.X, pady=(2, 8))
        ttk.Button(read_box, text="读取一次", command=self.read_once_async).pack(fill=tk.X, pady=(0, 6))
        ttk.Button(read_box, text="开始连续读取", command=self.start_continuous_read).pack(
            fill=tk.X,
            pady=(0, 6),
        )
        ttk.Button(read_box, text="停止连续读取", command=self.stop_continuous_read).pack(
            fill=tk.X,
            pady=(0, 6),
        )
        ttk.Button(read_box, text="清空日志", command=self.clear_log).pack(fill=tk.X)

        safety_box = ttk.LabelFrame(sidebar, text="安全状态", padding=10)
        safety_box.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(safety_box, text="当前版本：只读").pack(anchor=tk.W)
        ttk.Label(safety_box, text="不使能 / 不运动").pack(anchor=tk.W)
        ttk.Label(safety_box, textvariable=self.safety_status, foreground="#B00020").pack(
            anchor=tk.W,
            pady=(8, 8),
        )
        tk.Button(
            safety_box,
            text="急停",
            command=self.trigger_emergency_stop,
            bg="#B00020",
            fg="white",
            activebackground="#8A0018",
            activeforeground="white",
        ).pack(fill=tk.X, pady=(0, 6))
        ttk.Button(safety_box, text="解除急停", command=self.clear_emergency_stop).pack(
            fill=tk.X,
            pady=(0, 8),
        )
        ttk.Label(safety_box, textvariable=self.status, foreground="#005A9E").pack(
            anchor=tk.W,
            pady=(8, 0),
        )

    def _build_main_area(self, parent: ttk.Frame) -> None:
        main = ttk.Frame(parent)
        main.grid(row=0, column=1, sticky=tk.NSEW)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)

        self.notebook = ttk.Notebook(main)
        self.notebook.grid(row=0, column=0, sticky=tk.NSEW)

        status_tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(status_tab, text="机械臂状态")
        self._build_status_tab(status_tab)

        calibration_tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(calibration_tab, text="校准")
        self._build_calibration_tab(calibration_tab)

        teleop_tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(teleop_tab, text="主从遥操作")
        self._build_teleop_tab(teleop_tab)
        self._add_placeholder_tab("数据采集", "episode 录制、保存路径、采集状态会放在这里。")
        self._add_placeholder_tab("相机/触觉", "Gemini335 和 Paxini 预览会放在这里。")
        self._add_placeholder_tab("VLA", "VLA 推理入口和 action 输出会放在这里。")

    def _build_status_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=0)
        parent.rowconfigure(1, weight=1)

        table_frame = ttk.LabelFrame(parent, text="实时状态表 / Live Status Table", padding=8)
        table_frame.grid(row=0, column=0, sticky=tk.EW)
        table_frame.columnconfigure(0, weight=1)

        columns = ("joint", "id", "position", "velocity", "torque", "ticks", "status")
        self.status_table = ttk.Treeview(
            table_frame,
            columns=columns,
            show="headings",
            height=8,
        )
        headings = {
            "joint": "关节",
            "id": "ID",
            "position": "位置(rad)",
            "velocity": "速度(rad/s)",
            "torque": "力矩(Nm)",
            "ticks": "Ticks",
            "status": "状态",
        }
        widths = {
            "joint": 90,
            "id": 80,
            "position": 120,
            "velocity": 120,
            "torque": 120,
            "ticks": 90,
            "status": 260,
        }
        for column in columns:
            self.status_table.heading(column, text=headings[column])
            self.status_table.column(column, width=widths[column], anchor=tk.CENTER)
        self.status_table.grid(row=0, column=0, sticky=tk.EW)

        log_frame = ttk.LabelFrame(parent, text="状态输出 / Status Log", padding=8)
        log_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

        self.log = tk.Text(log_frame, height=18, wrap=tk.NONE)
        self.log.grid(row=0, column=0, sticky=tk.NSEW)
        scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        scroll.grid(row=0, column=1, sticky=tk.NS)
        self.log.configure(yscrollcommand=scroll.set)

    def _build_calibration_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(1, weight=1)
        parent.rowconfigure(2, weight=0)

        zero_controls = ttk.LabelFrame(parent, text="零点设置 / Zero Management", padding=10)
        zero_controls.grid(row=0, column=0, sticky=tk.EW, padx=(0, 6))
        zero_controls.columnconfigure(0, weight=1)
        zero_controls.columnconfigure(1, weight=1)

        ttk.Button(zero_controls, text="查看已保存零点", command=self.show_saved_zero).grid(
            row=0,
            column=0,
            sticky=tk.EW,
            padx=(0, 6),
            pady=(0, 6),
        )
        ttk.Button(zero_controls, text="读取当前状态作为零点预览", command=self.preview_current_zero).grid(
            row=0,
            column=1,
            sticky=tk.EW,
            pady=(0, 6),
        )
        ttk.Button(zero_controls, text="保存当前零点（二次确认）", command=self.save_current_zero).grid(
            row=1,
            column=0,
            sticky=tk.EW,
            padx=(0, 6),
        )
        tk.Button(
            zero_controls,
            text="写入电机/舵机零点",
            command=self.write_hardware_zero,
            bg="#B00020",
            fg="white",
            activebackground="#8A0018",
            activeforeground="white",
        ).grid(
            row=1,
            column=1,
            sticky=tk.EW,
        )

        limit_controls = ttk.LabelFrame(parent, text="关节限位 / Joint Limits", padding=10)
        limit_controls.grid(row=0, column=1, sticky=tk.EW, padx=(6, 0))
        limit_controls.columnconfigure(0, weight=1)
        limit_controls.columnconfigure(1, weight=1)
        limit_controls.columnconfigure(2, weight=1)

        ttk.Label(limit_controls, text="当前关节").grid(row=0, column=0, sticky=tk.W, pady=(0, 6))
        self.limit_joint_combo = ttk.Combobox(
            limit_controls,
            textvariable=self.limit_joint_name,
            state="readonly",
            values=[f"joint{i}" for i in range(1, 8)],
        )
        self.limit_joint_combo.grid(row=0, column=1, columnspan=2, sticky=tk.EW, pady=(0, 6))

        ttk.Button(limit_controls, text="加载限位", command=self.load_joint_limits).grid(
            row=1,
            column=0,
            sticky=tk.EW,
            padx=(0, 6),
            pady=(0, 6),
        )
        ttk.Button(limit_controls, text="记录当前为最小", command=self.record_current_limit_min).grid(
            row=1,
            column=1,
            sticky=tk.EW,
            padx=(0, 6),
            pady=(0, 6),
        )
        ttk.Button(limit_controls, text="记录当前为最大", command=self.record_current_limit_max).grid(
            row=1,
            column=2,
            sticky=tk.EW,
            pady=(0, 6),
        )
        ttk.Button(limit_controls, text="按当前位置生成±0.5rad", command=self.generate_default_limits).grid(
            row=2,
            column=0,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Button(limit_controls, text="保存软件限位", command=self.save_joint_limits).grid(
            row=2,
            column=1,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Button(limit_controls, text="检查当前状态", command=self.check_current_limits).grid(
            row=2,
            column=2,
            sticky=tk.EW,
        )
        tk.Button(
            limit_controls,
            text="写入硬件限位",
            command=self.write_joint_limits_to_hardware,
            bg="#8A5A00",
            fg="white",
            activebackground="#6E4700",
            activeforeground="white",
        ).grid(row=3, column=0, columnspan=3, sticky=tk.EW, pady=(6, 0))

        zero_frame = ttk.LabelFrame(parent, text="零点文件 / Zero File", padding=8)
        zero_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(10, 0), padx=(0, 6))
        zero_frame.columnconfigure(0, weight=1)
        zero_frame.rowconfigure(0, weight=1)
        zero_frame.rowconfigure(1, weight=0)

        self.zero_text = tk.Text(zero_frame, height=18, wrap=tk.NONE)
        self.zero_text.grid(row=0, column=0, sticky=tk.NSEW)
        zero_y_scroll = ttk.Scrollbar(zero_frame, orient=tk.VERTICAL, command=self.zero_text.yview)
        zero_y_scroll.grid(row=0, column=1, sticky=tk.NS)
        zero_x_scroll = ttk.Scrollbar(zero_frame, orient=tk.HORIZONTAL, command=self.zero_text.xview)
        zero_x_scroll.grid(row=1, column=0, sticky=tk.EW)
        self.zero_text.configure(yscrollcommand=zero_y_scroll.set, xscrollcommand=zero_x_scroll.set)

        limit_frame = ttk.LabelFrame(parent, text="限位采集 / Limit Teaching", padding=8)
        limit_frame.grid(row=1, column=1, sticky=tk.NSEW, pady=(10, 0), padx=(6, 0))
        limit_frame.columnconfigure(0, weight=1)
        limit_frame.rowconfigure(0, weight=0)
        limit_frame.rowconfigure(1, weight=1)
        limit_frame.rowconfigure(2, weight=0)

        limit_columns = ("joint", "min", "max")
        self.limit_table = ttk.Treeview(
            limit_frame,
            columns=limit_columns,
            show="headings",
            height=8,
        )
        self.limit_table.heading("joint", text="关节")
        self.limit_table.heading("min", text="最小位置(rad)")
        self.limit_table.heading("max", text="最大位置(rad)")
        self.limit_table.column("joint", width=90, anchor=tk.CENTER)
        self.limit_table.column("min", width=140, anchor=tk.CENTER)
        self.limit_table.column("max", width=140, anchor=tk.CENTER)
        self.limit_table.grid(row=0, column=0, sticky=tk.EW, pady=(0, 8))

        self.limit_text = tk.Text(limit_frame, height=18, wrap=tk.NONE)
        self.limit_text.grid(row=1, column=0, sticky=tk.NSEW)
        limit_y_scroll = ttk.Scrollbar(limit_frame, orient=tk.VERTICAL, command=self.limit_text.yview)
        limit_y_scroll.grid(row=1, column=1, sticky=tk.NS)
        limit_x_scroll = ttk.Scrollbar(limit_frame, orient=tk.HORIZONTAL, command=self.limit_text.xview)
        limit_x_scroll.grid(row=2, column=0, sticky=tk.EW)
        self.limit_text.configure(yscrollcommand=limit_y_scroll.set, xscrollcommand=limit_x_scroll.set)

        motion_frame = ttk.LabelFrame(parent, text="单关节低速测试 / Single Joint Jog", padding=10)
        motion_frame.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(10, 0))
        for column in range(7):
            motion_frame.columnconfigure(column, weight=1)

        ttk.Label(motion_frame, text="测试关节").grid(row=0, column=0, sticky=tk.W, pady=(0, 6))
        self.motion_joint_combo = ttk.Combobox(
            motion_frame,
            textvariable=self.motion_joint_name,
            state="readonly",
            values=[f"joint{i}" for i in range(1, 8)],
        )
        self.motion_joint_combo.grid(row=0, column=1, sticky=tk.EW, padx=(0, 8), pady=(0, 6))

        ttk.Label(
            motion_frame,
            text="默认小步长；不是回零；每次只控制一个关节",
        ).grid(row=0, column=2, columnspan=5, sticky=tk.W, pady=(0, 6))

        ttk.Button(
            motion_frame,
            text="-0.05 rad",
            command=lambda: self.step_current_joint(-0.05),
        ).grid(row=1, column=0, sticky=tk.EW, padx=(0, 6))
        ttk.Button(
            motion_frame,
            text="-0.02 rad",
            command=lambda: self.step_current_joint(-0.02),
        ).grid(row=1, column=1, sticky=tk.EW, padx=(0, 6))
        ttk.Button(
            motion_frame,
            text="回到当前位置保持",
            command=self.hold_current_joint,
        ).grid(row=1, column=2, columnspan=2, sticky=tk.EW, padx=(0, 6))
        ttk.Button(
            motion_frame,
            text="+0.02 rad",
            command=lambda: self.step_current_joint(0.02),
        ).grid(row=1, column=4, sticky=tk.EW, padx=(0, 6))
        ttk.Button(
            motion_frame,
            text="+0.05 rad",
            command=lambda: self.step_current_joint(0.05),
        ).grid(row=1, column=5, sticky=tk.EW, padx=(0, 6))
        ttk.Label(motion_frame, textvariable=self.motion_status).grid(
            row=1,
            column=6,
            sticky=tk.W,
        )

    def _build_teleop_tab(self, parent: ttk.Frame) -> None:
        """构建主从遥操作页签。"""

        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(1, weight=1)
        parent.rowconfigure(2, weight=0)

        master_profiles = [key for key, profile in ARM_PROFILES.items() if profile.kind == "feetech"]
        slave_profiles = [key for key, profile in ARM_PROFILES.items() if profile.kind == "damiao"]

        top = ttk.LabelFrame(parent, text="主从连接 / Master-Slave Connection", padding=10)
        top.grid(row=0, column=0, columnspan=2, sticky=tk.EW)
        for column in range(8):
            top.columnconfigure(column, weight=1)

        ttk.Label(top, text="主臂").grid(row=0, column=0, sticky=tk.W)
        self.teleop_master_combo = ttk.Combobox(
            top,
            textvariable=self.teleop_master_key,
            state="readonly",
            values=master_profiles,
        )
        self.teleop_master_combo.grid(
            row=0,
            column=1,
            sticky=tk.EW,
            padx=(0, 8),
        )
        self.teleop_master_combo.bind("<<ComboboxSelected>>", lambda _event: self._apply_teleop_master_defaults())
        ttk.Label(top, text="从臂").grid(row=0, column=2, sticky=tk.W)
        self.teleop_slave_combo = ttk.Combobox(
            top,
            textvariable=self.teleop_slave_key,
            state="readonly",
            values=slave_profiles,
        )
        self.teleop_slave_combo.grid(
            row=0,
            column=3,
            sticky=tk.EW,
            padx=(0, 8),
        )
        self.teleop_slave_combo.bind("<<ComboboxSelected>>", lambda _event: self._apply_teleop_slave_defaults())
        ttk.Button(top, text="刷新端口", command=self.refresh_teleop_ports).grid(
            row=0,
            column=4,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Button(top, text="连接主从", command=self.open_teleop_connections).grid(
            row=0,
            column=5,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Button(top, text="关闭主从", command=self.close_teleop_connections).grid(
            row=0,
            column=6,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Label(top, text="状态机").grid(row=0, column=7, sticky=tk.E)
        ttk.Label(top, textvariable=self.teleop_state_text, foreground="#005A9E").grid(
            row=1,
            column=7,
            sticky=tk.W,
        )
        ttk.Label(top, text="主臂串口").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        self.teleop_master_port_combo = ttk.Combobox(top, textvariable=self.teleop_master_port)
        self.teleop_master_port_combo.grid(row=1, column=1, sticky=tk.EW, padx=(0, 8), pady=(6, 0))
        ttk.Label(top, text="主臂波特率").grid(row=1, column=2, sticky=tk.W, pady=(6, 0))
        ttk.Combobox(
            top,
            textvariable=self.teleop_master_baudrate,
            values=["1000000", "921600", "115200"],
        ).grid(row=1, column=3, sticky=tk.EW, padx=(0, 8), pady=(6, 0))
        ttk.Label(top, text="从臂串口").grid(row=2, column=0, sticky=tk.W, pady=(6, 0))
        self.teleop_slave_port_combo = ttk.Combobox(top, textvariable=self.teleop_slave_port)
        self.teleop_slave_port_combo.grid(row=2, column=1, sticky=tk.EW, padx=(0, 8), pady=(6, 0))
        ttk.Label(top, text="从臂波特率").grid(row=2, column=2, sticky=tk.W, pady=(6, 0))
        ttk.Combobox(
            top,
            textvariable=self.teleop_slave_baudrate,
            values=["921600", "1000000", "115200"],
        ).grid(row=2, column=3, sticky=tk.EW, padx=(0, 8), pady=(6, 0))
        ttk.Label(
            top,
            text="主从遥操作使用这里的独立串口；左侧连接只用于单臂调试。",
            foreground="#666666",
        ).grid(row=2, column=4, columnspan=4, sticky=tk.W, pady=(6, 0))

        master_frame = ttk.LabelFrame(parent, text="主臂输入 / Master Input", padding=8)
        master_frame.grid(row=1, column=0, sticky=tk.NSEW, pady=(10, 0), padx=(0, 6))
        master_frame.columnconfigure(0, weight=1)
        master_frame.rowconfigure(0, weight=1)

        master_columns = ("joint", "raw", "ratio", "sign", "enabled")
        self.teleop_master_table = ttk.Treeview(master_frame, columns=master_columns, show="headings", height=8)
        for column, title in zip(master_columns, ("关节", "Raw Position(rad)", "Ratio", "Sign", "启用")):
            self.teleop_master_table.heading(column, text=title)
            self.teleop_master_table.column(column, anchor=tk.CENTER, width=110)
        self.teleop_master_table.grid(row=0, column=0, sticky=tk.NSEW)

        slave_frame = ttk.LabelFrame(parent, text="从臂输出 / Slave Output", padding=8)
        slave_frame.grid(row=1, column=1, sticky=tk.NSEW, pady=(10, 0), padx=(6, 0))
        slave_frame.columnconfigure(0, weight=1)
        slave_frame.rowconfigure(0, weight=1)

        slave_columns = ("joint", "current", "target", "limit", "sent")
        self.teleop_slave_table = ttk.Treeview(slave_frame, columns=slave_columns, show="headings", height=8)
        for column, title in zip(slave_columns, ("关节", "当前(rad)", "目标(rad)", "限位状态", "下发")):
            self.teleop_slave_table.heading(column, text=title)
            self.teleop_slave_table.column(column, anchor=tk.CENTER, width=110)
        self.teleop_slave_table.grid(row=0, column=0, sticky=tk.NSEW)

        settings = ttk.LabelFrame(parent, text="映射设置 / Mapping Settings", padding=10)
        settings.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(10, 0))
        for column in range(9):
            settings.columnconfigure(column, weight=1)

        ttk.Label(settings, text="单关节").grid(row=0, column=0, sticky=tk.W, pady=(0, 6))
        self.teleop_joint_combo = ttk.Combobox(
            settings,
            textvariable=self.teleop_joint_name,
            state="readonly",
            values=[f"joint{i}" for i in range(1, 8)],
        )
        self.teleop_joint_combo.grid(row=0, column=1, sticky=tk.EW, padx=(0, 6), pady=(0, 6))
        self.teleop_joint_combo.bind("<<ComboboxSelected>>", lambda _event: self.load_selected_teleop_joint())
        ttk.Checkbutton(
            settings,
            text="启用选中关节",
            variable=self.teleop_selected_enabled,
        ).grid(row=0, column=2, sticky=tk.W, padx=(0, 6), pady=(0, 6))
        ttk.Label(settings, text="scale").grid(row=0, column=3, sticky=tk.E, pady=(0, 6))
        ttk.Entry(settings, textvariable=self.teleop_selected_scale, width=8).grid(
            row=0,
            column=4,
            sticky=tk.EW,
            padx=(0, 6),
            pady=(0, 6),
        )
        ttk.Label(settings, text="sign").grid(row=0, column=5, sticky=tk.E, pady=(0, 6))
        ttk.Combobox(
            settings,
            textvariable=self.teleop_selected_sign,
            state="readonly",
            values=["+1", "-1"],
            width=6,
        ).grid(row=0, column=6, sticky=tk.EW, padx=(0, 6), pady=(0, 6))
        ttk.Button(settings, text="保存选中关节设置", command=self.save_selected_teleop_joint).grid(
            row=0,
            column=7,
            columnspan=2,
            sticky=tk.EW,
            pady=(0, 6),
        )

        self.teleop_enabled_vars: dict[str, tk.BooleanVar] = {}
        for index in range(1, 8):
            joint_name = f"joint{index}"
            var = tk.BooleanVar(value=False)
            self.teleop_enabled_vars[joint_name] = var
            ttk.Checkbutton(
                settings,
                text=joint_name,
                variable=var,
                command=self.sync_teleop_enabled_checkboxes,
            ).grid(row=1, column=index - 1, sticky=tk.W, pady=(0, 6))

        ttk.Label(settings, text="低通 alpha").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(settings, textvariable=self.teleop_alpha, width=8).grid(row=2, column=1, sticky=tk.EW, padx=(0, 6))
        ttk.Label(settings, text="max_step_rad").grid(row=2, column=2, sticky=tk.W)
        ttk.Entry(settings, textvariable=self.teleop_max_step, width=8).grid(
            row=2,
            column=3,
            sticky=tk.EW,
            padx=(0, 6),
        )
        ttk.Label(settings, text="控制频率 Hz").grid(row=2, column=4, sticky=tk.W)
        ttk.Entry(settings, textvariable=self.teleop_hz, width=8).grid(row=2, column=5, sticky=tk.EW, padx=(0, 6))
        ttk.Label(settings, text="达妙等待 s").grid(row=2, column=6, sticky=tk.W)
        ttk.Entry(settings, textvariable=self.teleop_damiao_wait, width=8).grid(
            row=2,
            column=7,
            sticky=tk.EW,
            padx=(0, 6),
        )

        ttk.Button(settings, text="Dry Run / 只读预览映射", command=self.start_teleop_preview).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky=tk.EW,
            pady=(8, 0),
            padx=(0, 6),
        )
        ttk.Button(settings, text="单关节跟随", command=self.start_single_joint_follow).grid(
            row=3,
            column=2,
            sticky=tk.EW,
            pady=(8, 0),
            padx=(0, 6),
        )
        ttk.Button(settings, text="全关节跟随", command=self.start_all_joint_follow).grid(
            row=3,
            column=3,
            sticky=tk.EW,
            pady=(8, 0),
            padx=(0, 6),
        )
        ttk.Button(settings, text="暂停跟随", command=self.pause_teleop).grid(
            row=3,
            column=4,
            sticky=tk.EW,
            pady=(8, 0),
            padx=(0, 6),
        )
        tk.Button(
            settings,
            text="急停",
            command=self.trigger_emergency_stop,
            bg="#B00020",
            fg="white",
            activebackground="#8A0018",
            activeforeground="white",
        ).grid(row=3, column=5, columnspan=2, sticky=tk.EW, pady=(8, 0))

        self.load_selected_teleop_joint()
        self.refresh_teleop_tables([])

    def _add_placeholder_tab(self, title: str, text: str) -> None:
        tab = ttk.Frame(self.notebook, padding=16)
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        ttk.Label(tab, text=text, foreground="#666666").grid(row=0, column=0, sticky=tk.NW)
        self.notebook.add(tab, text=title)

    def _current_profile(self):
        return get_arm_profile(self.profile_key.get())

    def _apply_profile_defaults(self) -> None:
        profile = self._current_profile()
        self.port.set(profile.default_port)
        self.baudrate.set(str(profile.default_baudrate))
        self.clear_status_table()
        if hasattr(self, "zero_text"):
            self.zero_text.delete("1.0", tk.END)
        if hasattr(self, "limit_text"):
            self.limit_text.delete("1.0", tk.END)
        self.taught_limits = {f"joint{i}": {"min": None, "max": None} for i in range(1, 8)}
        self.refresh_limit_table()

    def _apply_teleop_profile_defaults(self) -> None:
        """根据主从 profile 设置主从遥操作页签里的独立串口默认值。"""

        self._apply_teleop_master_defaults()
        self._apply_teleop_slave_defaults()
        self.refresh_teleop_ports()

    def _apply_teleop_master_defaults(self) -> None:
        profile = get_arm_profile(self.teleop_master_key.get())
        self.teleop_master_port.set(profile.default_port)
        self.teleop_master_baudrate.set(str(profile.default_baudrate))

    def _apply_teleop_slave_defaults(self) -> None:
        profile = get_arm_profile(self.teleop_slave_key.get())
        self.teleop_slave_port.set(profile.default_port)
        self.teleop_slave_baudrate.set(str(profile.default_baudrate))

    def refresh_ports(self) -> None:
        try:
            ports = list_serial_ports()
        except Exception as exc:  # noqa: BLE001
            self.append_log(f"刷新串口失败: {exc}")
            ports = []

        self.port_combo.configure(values=ports)
        if ports and not self.port.get():
            self.port.set(ports[0])
        self.append_log(f"串口列表: {', '.join(ports) if ports else '无'}")

    def refresh_teleop_ports(self) -> None:
        """刷新主从遥操作页签的主臂/从臂串口下拉框。"""

        try:
            ports = list_serial_ports()
        except Exception as exc:  # noqa: BLE001
            self.append_log(f"刷新主从串口失败: {exc}")
            ports = []
        if hasattr(self, "teleop_master_port_combo"):
            self.teleop_master_port_combo.configure(values=ports)
        if hasattr(self, "teleop_slave_port_combo"):
            self.teleop_slave_port_combo.configure(values=ports)

    def open_connection(self) -> None:
        if self.connection is not None:
            messagebox.showinfo("提示", "串口已经打开。")
            return

        try:
            profile = self._current_profile()
            connection = ArmReadConnection(profile, self.port.get(), int(self.baudrate.get()))
            connection.open()
        except Exception as exc:  # noqa: BLE001
            self.append_log(f"打开失败: {exc}")
            messagebox.showerror("打开失败", str(exc))
            return

        self.connection = connection
        self.status.set(f"已连接: {profile.label} @ {self.port.get()} {self.baudrate.get()}")
        self.append_log(f"已打开: {profile.label}, port={self.port.get()}, baudrate={self.baudrate.get()}")

    def close_connection(self) -> None:
        self.stop_continuous_read()
        if self.connection is None:
            return
        try:
            with self.read_lock:
                self.connection.close()
        finally:
            self.connection = None
            self.status.set("未连接")
            self.append_log("串口已关闭")

    def trigger_emergency_stop(self) -> None:
        """触发软件急停状态。"""

        self.stop_continuous_read()
        self.teleop_running = False
        self._set_teleop_state(TeleopState.EMERGENCY_STOP)
        self.safety_service.trigger_emergency_stop()
        self.update_safety_status()
        self.motion_status.set("急停已触发")
        self.append_log("急停已触发：已停止连续读取，正在尝试下发硬件失能。")
        threading.Thread(target=self._emergency_disable_worker, daemon=True).start()

    def _emergency_disable_worker(self) -> None:
        """急停后尽力下发硬件失能命令。"""

        connections = [
            connection
            for connection in (
                self.connection,
                self.teleop_master_connection,
                self.teleop_slave_connection,
            )
            if connection is not None
        ]
        if not connections:
            self.after(0, lambda: self.append_log("急停：当前没有打开的硬件连接。"))
            return

        all_lines: list[str] = []
        try:
            with self.read_lock, self.teleop_lock:
                for connection in connections:
                    all_lines.extend(disable_arm_motion(connection))
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
            self.after(0, lambda: self.append_log(f"急停硬件失能失败: {message}"))
            return

        self.after(0, lambda: self.append_log("\n".join(all_lines)))

    def clear_emergency_stop(self) -> None:
        """解除软件急停状态。"""

        self.safety_service.clear_emergency_stop()
        self.update_safety_status()
        if self.teleop_state == TeleopState.EMERGENCY_STOP:
            self._set_teleop_state(TeleopState.IDLE)
        self.append_log("急停状态已解除。")

    def update_safety_status(self) -> None:
        """刷新左侧安全状态显示。"""

        self.safety_status.set(self.safety_service.status_text())

    def read_once_async(self) -> None:
        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口。")
            return

        threading.Thread(target=self._read_once_worker, daemon=True).start()

    def _read_once_worker(self) -> None:
        connection = self.connection
        if connection is None:
            return

        try:
            with self.read_lock:
                snapshot = connection.read_snapshot()
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
            self.after(0, lambda: self.append_log(f"读取失败: {message}"))
            return

        self.after(0, lambda: self.handle_snapshot(snapshot))

    def start_continuous_read(self) -> None:
        """开始连续读取当前机械臂状态。"""

        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口。")
            return
        if self.continuous_reading:
            return

        try:
            interval_ms = int(self.read_interval_ms.get())
        except ValueError:
            messagebox.showerror("参数错误", "连续读取周期必须是整数毫秒。")
            return
        if interval_ms < 50:
            messagebox.showerror("参数错误", "连续读取周期建议不小于 50 ms。")
            return

        self.continuous_reading = True
        self.append_log(f"开始连续读取，周期 {interval_ms} ms")
        threading.Thread(
            target=self._continuous_read_worker,
            args=(interval_ms / 1000.0,),
            daemon=True,
        ).start()

    def stop_continuous_read(self) -> None:
        """停止连续读取。"""

        if self.continuous_reading:
            self.continuous_reading = False
            self.append_log("停止连续读取")

    def _continuous_read_worker(self, interval_s: float) -> None:
        """后台连续读取线程。"""

        while self.continuous_reading:
            connection = self.connection
            if connection is None:
                self.continuous_reading = False
                break

            try:
                with self.read_lock:
                    snapshot = connection.read_snapshot()
            except Exception as exc:  # noqa: BLE001
                self.continuous_reading = False
                message = str(exc)
                self.after(0, lambda: self.append_log(f"连续读取失败: {message}"))
                break

            self.after(0, lambda snapshot=snapshot: self.handle_snapshot(snapshot))
            time.sleep(interval_s)

    def handle_snapshot(self, snapshot: ArmStatusSnapshot) -> None:
        """刷新表格并把状态追加到日志。"""

        self.last_snapshot = snapshot
        self.update_status_table(snapshot)
        self.append_log("\n".join(snapshot.to_lines()))

    def show_saved_zero(self) -> None:
        """显示当前机械臂已保存的软件零点文件。"""

        profile_key = self.profile_key.get()
        text = load_zero_text(profile_key)
        self.zero_text.delete("1.0", tk.END)
        self.zero_text.insert(tk.END, text)
        self.notebook.select(1)

    def preview_current_zero(self) -> None:
        """把最新读取结果显示为零点预览。"""

        if self.last_snapshot is None:
            messagebox.showwarning("没有状态", "请先读取一次机械臂状态。")
            return

        profile_key = self.profile_key.get()
        text = format_zero_snapshot(profile_key, self.last_snapshot)
        self.zero_text.delete("1.0", tk.END)
        self.zero_text.insert(tk.END, text)
        self.notebook.select(1)

    def save_current_zero(self) -> None:
        """二次确认后保存当前状态为软件零点。"""

        if self.last_snapshot is None:
            messagebox.showwarning("没有状态", "请先读取一次机械臂状态。")
            return

        profile = self._current_profile()
        first_confirm = messagebox.askyesno(
            "确认保存零点",
            f"即将把当前读取到的位置保存为 {profile.label} 的软件零点。\n\n"
            "这不会写入电机，但会覆盖本地零点文件。\n\n"
            "是否继续？",
        )
        if not first_confirm:
            return

        typed = simpledialog.askstring(
            "二次确认",
            "请输入 SAVE ZERO 以确认保存当前零点：",
            parent=self,
        )
        if typed != "SAVE ZERO":
            messagebox.showinfo("已取消", "二次确认未通过，未保存零点。")
            return

        path = save_zero_snapshot(profile.key, self.last_snapshot)
        self.append_log(f"零点已保存: {path}")
        self.show_saved_zero()

    def write_hardware_zero(self) -> None:
        """多重确认后把当前姿态写入电机/舵机硬件零点。"""

        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口。")
            return
        if self.last_snapshot is None:
            messagebox.showwarning("没有状态", "请先读取一次机械臂状态。")
            return
        if self.safety_service.state.emergency_stop:
            messagebox.showerror("急停中", "当前处于急停状态，禁止写入硬件零点。")
            return

        profile = self._current_profile()
        first_confirm = messagebox.askyesno(
            "危险操作确认",
            f"即将把当前姿态写入 {profile.label} 的电机/舵机硬件零点。\n\n"
            "这会写入电机/舵机内部参数，不只是保存本地文件。\n"
            "请确认机械臂已经静止、无负载风险、当前位置就是你想要的零点。\n\n"
            "是否继续？",
        )
        if not first_confirm:
            return

        second_confirm = messagebox.askyesno(
            "再次确认",
            "再次确认：写入硬件零点可能影响后续控制坐标。\n\n"
            "确认继续写入？",
        )
        if not second_confirm:
            return

        typed = simpledialog.askstring(
            "最终确认",
            "请输入 YES 以确认写入硬件零点：",
            parent=self,
        )
        if typed != "YES":
            messagebox.showinfo("已取消", "最终确认未通过，未写入硬件零点。")
            return

        self.stop_continuous_read()
        try:
            with self.read_lock:
                lines = write_hardware_zero(self.connection)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("写入失败", str(exc))
            self.append_log(f"硬件零点写入失败: {exc}")
            return

        self.append_log("\n".join(lines))
        messagebox.showinfo("完成", "硬件零点写入命令已完成。建议断电重启后重新读取验证。")

    def load_joint_limits(self) -> None:
        """加载当前机械臂的软件限位文件。"""

        text = load_limits_text(self.profile_key.get())
        self.limit_text.delete("1.0", tk.END)
        self.limit_text.insert(tk.END, text)
        self._sync_taught_limits_from_text(silent=True)
        self.refresh_limit_table()
        self.notebook.select(1)

    def generate_default_limits(self) -> None:
        """按当前位置生成一个保守的默认限位模板。"""

        if self.last_snapshot is None:
            messagebox.showwarning("没有状态", "请先读取一次机械臂状态。")
            return
        limits = default_limits_from_snapshot(self.last_snapshot, margin_rad=0.5)
        self.limit_text.delete("1.0", tk.END)
        self.limit_text.insert(tk.END, format_limits_text(limits))
        self._sync_taught_limits_from_text(silent=True)
        self.refresh_limit_table()
        self.notebook.select(1)

    def record_current_limit_min(self) -> None:
        """记录选中关节当前姿态为最小限位。"""

        self.record_current_limit("min")

    def record_current_limit_max(self) -> None:
        """记录选中关节当前姿态为最大限位。"""

        self.record_current_limit("max")

    def record_current_limit(self, side: str) -> None:
        """记录选中关节当前姿态为 min 或 max。"""

        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口，再记录关节限位。")
            return

        # 记录限位时必须读一次最新硬件状态。
        # 如果只用界面缓存的 last_snapshot，用户移动机械臂后可能仍然记录到旧位置。
        try:
            with self.read_lock:
                snapshot = self.connection.read_snapshot()
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("读取失败", f"记录限位前读取当前状态失败：{exc}")
            return
        self.handle_snapshot(snapshot)

        joint_name = self.limit_joint_name.get()
        joint = next((item for item in self.last_snapshot.joints if item.name == joint_name), None)
        if joint is None or joint.position_rad is None:
            messagebox.showerror("读取失败", f"没有找到 {joint_name} 的有效位置。")
            return

        if joint_name not in self.taught_limits:
            self.taught_limits[joint_name] = {"min": None, "max": None}
        other_side = "max" if side == "min" else "min"
        other_value = self.taught_limits[joint_name].get(other_side)
        self.taught_limits[joint_name][side] = joint.position_rad
        self.refresh_limit_table()
        self.refresh_limit_text_from_taught()
        self.append_log(f"{joint_name} {side} 限位已记录: {joint.position_rad:.6f} rad")

        if other_value is not None and abs(other_value - joint.position_rad) < 1e-6:
            messagebox.showwarning(
                "限位相同",
                f"{joint_name} 的最小和最大位置相同：{joint.position_rad:.6f} rad。\n\n"
                "请把这个关节移动到另一个机械极限位置后，再记录另一侧限位。",
            )

    def save_joint_limits(self) -> None:
        """保存软件关节限位配置。"""

        self._sync_taught_limits_from_text(silent=True)
        text = self.limit_text.get("1.0", tk.END)
        try:
            path = save_limits_text(self.profile_key.get(), text)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("保存失败", str(exc))
            return
        self.append_log(f"软件关节限位已保存: {path}")

    def check_current_limits(self) -> None:
        """检查当前读取位置是否在软件限位内。"""

        if self.last_snapshot is None:
            messagebox.showwarning("没有状态", "请先读取一次机械臂状态。")
            return
        try:
            limits = parse_limits_text(self.limit_text.get("1.0", tk.END))
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("限位格式错误", str(exc))
            return
        lines = check_snapshot_against_limits(self.last_snapshot, limits)
        self.append_log("关节限位检查结果:\n" + "\n".join(lines))

    def write_joint_limits_to_hardware(self) -> None:
        """二次确认后写入硬件限位。"""

        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口。")
            return
        if self.safety_service.state.emergency_stop:
            messagebox.showerror("急停中", "当前处于急停状态，禁止写入硬件限位。")
            return
        try:
            limits = parse_limits_text(self.limit_text.get("1.0", tk.END))
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("限位格式错误", str(exc))
            return

        profile = self._current_profile()
        first_confirm = messagebox.askyesno(
            "危险操作确认",
            f"即将把关节限位写入 {profile.label} 的电机/舵机内部参数。\n\n"
            "Feetech 会写 Min/Max Angle Limit。\n"
            "Damiao 会写 PMAX；精确 min/max 仍由软件限位保证。\n\n"
            "是否继续？",
        )
        if not first_confirm:
            return
        typed = simpledialog.askstring(
            "最终确认",
            "请输入 YES 以确认写入硬件限位：",
            parent=self,
        )
        if typed != "YES":
            messagebox.showinfo("已取消", "最终确认未通过，未写入硬件限位。")
            return

        self.stop_continuous_read()
        try:
            with self.read_lock:
                lines = write_hardware_limits(self.connection, limits)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("写入失败", str(exc))
            self.append_log(f"硬件限位写入失败: {exc}")
            return
        self.append_log("\n".join(lines))
        messagebox.showinfo("完成", "硬件限位写入命令已完成。建议重新读取并检查。")

    def refresh_limit_table(self) -> None:
        """刷新限位采集表。"""

        if not hasattr(self, "limit_table"):
            return
        for item in self.limit_table.get_children():
            self.limit_table.delete(item)
        for index in range(1, 8):
            joint_name = f"joint{index}"
            values = self.taught_limits.get(joint_name, {"min": None, "max": None})
            self.limit_table.insert(
                "",
                tk.END,
                values=(
                    joint_name,
                    self._format_float(values.get("min")),
                    self._format_float(values.get("max")),
                ),
            )

    def refresh_limit_text_from_taught(self) -> None:
        """把采集表中的 min/max 同步到限位文本框。"""

        lines = [
            "# Joint limits taught from manual arm movement.",
            "# Move each joint to its mechanical min/max, then record it.",
            "# <joint_name> <min_rad> <max_rad>",
        ]
        for index in range(1, 8):
            joint_name = f"joint{index}"
            values = self.taught_limits.get(joint_name, {"min": None, "max": None})
            min_value = values.get("min")
            max_value = values.get("max")
            if min_value is None or max_value is None:
                lines.append(f"# {joint_name} <min_not_recorded> <max_not_recorded>")
                continue
            lo = min(min_value, max_value)
            hi = max(min_value, max_value)
            lines.append(f"{joint_name} {lo:.6f} {hi:.6f}")
        lines.append("")
        self.limit_text.delete("1.0", tk.END)
        self.limit_text.insert(tk.END, "\n".join(lines))

    def _sync_taught_limits_from_text(self, *, silent: bool) -> None:
        """从限位文本同步到采集表。"""

        try:
            limits = parse_limits_text(self.limit_text.get("1.0", tk.END))
        except Exception:
            if not silent:
                raise
            return
        for limit in limits:
            self.taught_limits[limit.name] = {"min": limit.min_rad, "max": limit.max_rad}

    def hold_current_joint(self) -> None:
        """单关节当前位置保持：读取 q_now，然后 q_des = q_now。"""

        self._start_single_joint_motion(delta_rad=None)

    def step_current_joint(self, delta_rad: float) -> None:
        """单关节小步进测试。"""

        self._start_single_joint_motion(delta_rad=delta_rad)

    def _start_single_joint_motion(self, delta_rad: float | None) -> None:
        """检查安全状态和软件限位，然后启动单关节运动线程。"""

        if self.connection is None:
            messagebox.showwarning("未连接", "请先打开串口。")
            return
        if self.safety_service.state.emergency_stop:
            messagebox.showerror("急停中", "当前处于急停状态，禁止单关节运动测试。")
            return
        try:
            limits = parse_limits_text(self.limit_text.get("1.0", tk.END))
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror(
                "限位未就绪",
                f"单关节运动前必须先完成并加载软件关节限位。\n\n{exc}",
            )
            return

        joint_name = self.motion_joint_name.get()
        if not joint_name:
            messagebox.showerror("参数错误", "请选择要测试的单个关节。")
            return

        self.stop_continuous_read()
        if delta_rad is None:
            self.motion_status.set(f"{joint_name}: 正在读取当前位置并保持...")
        else:
            self.motion_status.set(f"{joint_name}: 正在执行 {delta_rad:+.2f} rad 小步进...")

        threading.Thread(
            target=self._single_joint_motion_worker,
            args=(joint_name, delta_rad, limits),
            daemon=True,
        ).start()

    def _single_joint_motion_worker(self, joint_name: str, delta_rad: float | None, limits: list) -> None:
        """后台执行单关节运动，避免阻塞界面。"""

        connection = self.connection
        if connection is None:
            return

        try:
            with self.read_lock:
                if delta_rad is None:
                    result = hold_current_position(connection, joint_name, limits)
                else:
                    result = step_single_joint(connection, joint_name, delta_rad, limits)
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
            self.after(0, lambda: self._handle_motion_error(message))
            return

        self.after(0, lambda: self._handle_motion_result(result))

    def _handle_motion_result(self, result) -> None:
        """刷新界面中的运动测试结果。"""

        self.last_snapshot = result.snapshot
        self.update_status_table(result.snapshot)
        self.motion_status.set(
            f"{result.joint_name}: q_now={result.current_rad:.4f}, q_des={result.target_rad:.4f}"
        )
        self.append_log("\n".join(result.lines))

    def _handle_motion_error(self, message: str) -> None:
        """显示单关节运动错误。"""

        self.motion_status.set("单关节运动失败")
        self.append_log(f"单关节运动失败: {message}")
        messagebox.showerror("单关节运动失败", message)

    def open_teleop_connections(self) -> None:
        """打开主从遥操作需要的主臂和从臂连接。"""

        if self.teleop_master_connection is not None or self.teleop_slave_connection is not None:
            messagebox.showwarning("已连接", "主从遥操作连接已打开。")
            return
        if not self._resolve_left_connection_conflict():
            return
        try:
            master_key = self.teleop_master_key.get()
            slave_key = self.teleop_slave_key.get()
            master_port = self.teleop_master_port.get().strip()
            slave_port = self.teleop_slave_port.get().strip()
            master_baudrate = int(self.teleop_master_baudrate.get())
            slave_baudrate = int(self.teleop_slave_baudrate.get())
            if not master_port or not slave_port:
                raise ValueError("主臂串口和从臂串口都必须选择。")
            if master_port.upper() == slave_port.upper():
                raise ValueError(
                    f"主臂和从臂不能同时打开同一个串口 {master_port}。\n"
                    "请分别选择主臂串口和从臂 USB2CAN 串口。"
                )
            self.teleop_master_connection = open_arm_connection(
                master_key,
                port=master_port,
                baudrate=master_baudrate,
            )
            self.teleop_slave_connection = open_arm_connection(
                slave_key,
                port=slave_port,
                baudrate=slave_baudrate,
            )
        except Exception as exc:  # noqa: BLE001
            self.close_teleop_connections()
            messagebox.showerror("连接失败", str(exc))
            return

        self._set_teleop_state(TeleopState.IDLE)
        self.append_log(
            "主从遥操作连接已打开: "
            f"master={self.teleop_master_key.get()}@{self.teleop_master_port.get()}, "
            f"slave={self.teleop_slave_key.get()}@{self.teleop_slave_port.get()}"
        )

    def _resolve_left_connection_conflict(self) -> bool:
        """如果左侧单臂连接占用了主从串口，提示用户先关闭。"""

        if self.connection is None:
            return True
        left_port = self.connection.port.upper()
        teleop_ports = {
            self.teleop_master_port.get().strip().upper(),
            self.teleop_slave_port.get().strip().upper(),
        }
        if left_port not in teleop_ports:
            return True
        should_close = messagebox.askyesno(
            "串口已被左侧连接占用",
            f"左侧单臂调试当前已经打开 {self.connection.profile.label} @ {self.connection.port}。\n\n"
            "主从遥操作也需要使用这个串口，所以 Windows 不允许再次打开。\n\n"
            "是否先自动关闭左侧单臂连接，然后再连接主从？",
        )
        if should_close:
            self.close_connection()
        return should_close

    def close_teleop_connections(self) -> None:
        """关闭主从遥操作连接。"""

        self.pause_teleop(set_state=TeleopState.IDLE)
        for connection in (self.teleop_master_connection, self.teleop_slave_connection):
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:  # noqa: BLE001
                    self.append_log(f"关闭主从连接时出现错误: {exc}")
        self.teleop_master_connection = None
        self.teleop_slave_connection = None
        self.teleop_master_anchor = {}
        self.teleop_slave_anchor = {}
        self.teleop_previous_targets = {}
        self.refresh_teleop_tables([])

    def load_selected_teleop_joint(self) -> None:
        """把选中关节的映射参数加载到编辑控件。"""

        joint_name = self.teleop_joint_name.get()
        setting = self.teleop_joint_settings.get(joint_name, TeleopJointSetting(name=joint_name))
        self.teleop_selected_enabled.set(setting.enabled)
        self.teleop_selected_scale.set(f"{setting.scale:.3f}")
        self.teleop_selected_sign.set("+1" if setting.sign >= 0 else "-1")

    def save_selected_teleop_joint(self) -> bool:
        """保存选中关节的启用、比例和方向设置。"""

        joint_name = self.teleop_joint_name.get()
        try:
            scale = float(self.teleop_selected_scale.get())
        except ValueError:
            messagebox.showerror("参数错误", "scale 必须是数字。")
            return False
        sign = 1 if self.teleop_selected_sign.get() == "+1" else -1
        self.teleop_joint_settings[joint_name] = TeleopJointSetting(
            name=joint_name,
            enabled=self.teleop_selected_enabled.get(),
            scale=scale,
            sign=sign,
        )
        if hasattr(self, "teleop_enabled_vars"):
            self.teleop_enabled_vars[joint_name].set(self.teleop_selected_enabled.get())
        self.append_log(f"{joint_name} 映射设置已保存: enabled={self.teleop_selected_enabled.get()}, scale={scale}, sign={sign}")
        return True

    def sync_teleop_enabled_checkboxes(self) -> None:
        """从 7 个复选框同步启用状态。"""

        for joint_name, var in self.teleop_enabled_vars.items():
            setting = self.teleop_joint_settings.get(joint_name, TeleopJointSetting(name=joint_name))
            setting.enabled = var.get()
            self.teleop_joint_settings[joint_name] = setting
        self.load_selected_teleop_joint()

    def start_teleop_preview(self) -> None:
        """启动 Dry Run，只读预览映射，不向从臂发送目标。"""

        if not self._teleop_ready():
            return
        if not self.save_selected_teleop_joint():
            return
        self.sync_teleop_enabled_checkboxes()
        try:
            self._prepare_teleop_anchors()
            self._parse_teleop_runtime_settings()
            self._load_teleop_slave_limits()
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("预览启动失败", str(exc))
            return

        self.teleop_control_mode = "preview"
        self.teleop_control_joints = set()
        self.teleop_previous_targets = {}
        self._set_teleop_state(TeleopState.PREVIEW_MAPPING)
        self._start_teleop_loop()
        self.append_log("Dry Run 已启动：只读预览映射，不会向从臂发送运动命令。")

    def start_single_joint_follow(self) -> None:
        """单关节跟随。第一次点击只进入从臂当前位置保持，第二次点击才开始跟随。"""

        joint_name = self.teleop_joint_name.get()
        if not self.save_selected_teleop_joint():
            return
        if not self.teleop_joint_settings[joint_name].enabled:
            messagebox.showwarning("关节未启用", f"请先启用 {joint_name}。")
            return
        self._start_follow_with_state_gate({joint_name}, "single")

    def start_all_joint_follow(self) -> None:
        """全关节跟随。第一次点击只进入从臂当前位置保持，第二次点击才开始跟随。"""

        self.sync_teleop_enabled_checkboxes()
        enabled = {name for name, setting in self.teleop_joint_settings.items() if setting.enabled}
        if not enabled:
            messagebox.showwarning("没有启用关节", "请至少启用一个关节。")
            return
        self._start_follow_with_state_gate(enabled, "all")

    def _start_follow_with_state_gate(self, joints: set[str], mode: str) -> None:
        if not self._teleop_ready():
            return
        if self.safety_service.state.emergency_stop:
            messagebox.showerror("急停中", "当前处于急停状态，禁止进入主从跟随。")
            return

        try:
            limits = self._load_teleop_slave_limits()
            hold_is_ready = (
                self.teleop_state == TeleopState.ARM_SLAVE_HOLD_CURRENT
                and self.teleop_control_joints == set(joints)
                and self.teleop_control_mode == mode
            )
            if not hold_is_ready:
                self.teleop_running = False
                self._arm_slave_hold_current(joints, limits)
                self.teleop_control_joints = set(joints)
                self.teleop_control_mode = mode
                self._set_teleop_state(TeleopState.ARM_SLAVE_HOLD_CURRENT)
                messagebox.showinfo(
                    "已进入当前位置保持",
                    "从臂已保持当前位置。请先确认 Dry Run 方向和 target 正确，再次点击跟随按钮才会真正进入 TELEOP_ACTIVE。",
                )
                return
            self._prepare_teleop_anchors()
            self._parse_teleop_runtime_settings()
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("跟随启动失败", str(exc))
            return

        self.teleop_control_joints = set(joints)
        self.teleop_control_mode = mode
        self.teleop_previous_targets = {}
        self._set_teleop_state(TeleopState.TELEOP_ACTIVE)
        self._start_teleop_loop()
        self.append_log(f"进入 TELEOP_ACTIVE: mode={mode}, joints={sorted(joints)}")

    def pause_teleop(self, set_state: TeleopState = TeleopState.PAUSED) -> None:
        """暂停 Dry Run 或跟随循环。"""

        self.teleop_running = False
        self._set_teleop_state(set_state)

    def _start_teleop_loop(self) -> None:
        self.teleop_running = False
        time.sleep(0.02)
        self.teleop_running = True
        threading.Thread(target=self._teleop_loop_worker, daemon=True).start()

    def _teleop_loop_worker(self) -> None:
        """主从遥操作后台循环。"""

        try:
            hz = float(self.teleop_hz.get())
        except ValueError:
            hz = 20.0
        period = 1.0 / max(1.0, hz)

        while self.teleop_running:
            if self.safety_service.state.emergency_stop:
                self.teleop_running = False
                self.after(0, lambda: self._set_teleop_state(TeleopState.EMERGENCY_STOP))
                break

            try:
                with self.teleop_lock:
                    result = self._teleop_cycle()
            except Exception as exc:  # noqa: BLE001
                message = str(exc)
                self.teleop_running = False
                self.after(0, lambda: self._handle_teleop_error(message))
                break

            previews, sent_joints = result
            self.after(0, lambda previews=previews, sent_joints=sent_joints: self.refresh_teleop_tables(previews, sent_joints))
            time.sleep(period)

    def _teleop_cycle(self):
        master = self.teleop_master_connection
        slave = self.teleop_slave_connection
        if master is None or slave is None:
            raise RuntimeError("主从连接未打开")

        runtime_settings = self._parse_teleop_runtime_settings()
        limits = self._load_teleop_slave_limits()
        active_joints = (
            set(self.teleop_control_joints)
            if self.teleop_state == TeleopState.TELEOP_ACTIVE and self.teleop_control_joints
            else None
        )
        master_snapshot = master.read_snapshot(joint_names=active_joints)
        master_raw_map = {joint.name: joint.position_rad for joint in master_snapshot.joints}
        update_continuous_position_map(
            master_raw_map,
            self.teleop_master_last_raw,
            self.teleop_master_continuous,
        )
        slave_snapshot = slave.read_snapshot(
            joint_names=active_joints,
            damiao_response_wait=runtime_settings.damiao_response_wait,
        )

        previews, next_targets = compute_mapping_preview(
            master_snapshot=master_snapshot,
            slave_snapshot=slave_snapshot,
            joint_settings=self.teleop_joint_settings,
            runtime_settings=runtime_settings,
            slave_limits=limits,
            master_anchor=self.teleop_master_anchor,
            slave_anchor=self.teleop_slave_anchor,
            previous_targets=self.teleop_previous_targets,
            master_continuous_positions=self.teleop_master_continuous,
        )
        self.teleop_previous_targets = next_targets

        sent_joints: set[str] = set()
        if self.teleop_control_mode != "preview" and self.teleop_state == TeleopState.TELEOP_ACTIVE:
            if not targets_are_safe(previews):
                raise RuntimeError("目标超出限位，已停止 TELEOP_ACTIVE。请回到 Dry Run 检查方向和范围。")
            for preview in previews:
                if (
                    preview.enabled
                    and preview.name in self.teleop_control_joints
                    and preview.target_rad is not None
                    and target_status_is_commandable(preview.limit_status)
                ):
                    command_single_joint_target(
                        slave,
                        preview.name,
                        preview.target_rad,
                        limits,
                        enable_motor=False,
                    )
                    sent_joints.add(preview.name)
        return previews, sent_joints

    def _teleop_ready(self) -> bool:
        if self.teleop_master_connection is None or self.teleop_slave_connection is None:
            messagebox.showwarning("未连接", "请先点击“连接主从”。")
            return False
        return True

    def _prepare_teleop_anchors(self) -> None:
        master = self.teleop_master_connection
        slave = self.teleop_slave_connection
        if master is None or slave is None:
            raise RuntimeError("主从连接未打开")
        with self.teleop_lock:
            master_snapshot = master.read_snapshot()
            slave_snapshot = slave.read_snapshot()
        self.teleop_master_last_raw = {}
        self.teleop_master_continuous = {}
        master_raw_map = {joint.name: joint.position_rad for joint in master_snapshot.joints}
        update_continuous_position_map(
            master_raw_map,
            self.teleop_master_last_raw,
            self.teleop_master_continuous,
        )
        self.teleop_master_anchor = dict(self.teleop_master_continuous)
        self.teleop_slave_anchor = build_anchor(slave_snapshot)
        self.teleop_previous_targets = {}
        self.refresh_teleop_tables([])
        self.append_log("主从遥操作锚点已更新：master_anchor=当前主臂，slave_anchor=当前从臂。")

    def _arm_slave_hold_current(self, joints: set[str], limits) -> None:
        slave = self.teleop_slave_connection
        if slave is None:
            raise RuntimeError("从臂连接未打开")
        slave_snapshot = slave.read_snapshot()
        slave_map = {joint.name: joint.position_rad for joint in slave_snapshot.joints}
        lines = ["ARM_SLAVE_HOLD_CURRENT: 从臂当前位置保持"]
        for joint_name in sorted(joints):
            current = slave_map.get(joint_name)
            if current is None:
                raise RuntimeError(f"{joint_name}: 从臂当前位置无效，不能保持")
            command_single_joint_target(slave, joint_name, current, limits)
            lines.append(f"  {joint_name}: q_des=q_now={current:.6f} rad")
        self.append_log("\n".join(lines))

    def _load_teleop_slave_limits(self):
        text = load_limits_text(self.teleop_slave_key.get())
        return parse_limits_text(text)

    def _parse_teleop_runtime_settings(self) -> TeleopRuntimeSettings:
        alpha = float(self.teleop_alpha.get())
        max_step = float(self.teleop_max_step.get())
        hz = float(self.teleop_hz.get())
        damiao_wait = float(self.teleop_damiao_wait.get())
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("低通 alpha 必须在 0~1 之间。")
        if max_step <= 0.0 or max_step > 0.2:
            raise ValueError("max_step_rad 建议设置在 0~0.2 rad 之间。")
        if hz <= 0.0 or hz > 100.0:
            raise ValueError("控制频率 Hz 必须在 1~100 之间。")
        if damiao_wait < 0.0 or damiao_wait > 0.05:
            raise ValueError("达妙等待时间建议设置在 0~0.05 s 之间。")
        return TeleopRuntimeSettings(
            alpha=alpha,
            max_step_rad=max_step,
            damiao_response_wait=damiao_wait,
        )

    def refresh_teleop_tables(self, previews, sent_joints: set[str] | None = None) -> None:
        """刷新主从遥操作表格。"""

        sent_joints = sent_joints or set()
        if not hasattr(self, "teleop_master_table"):
            return
        for table in (self.teleop_master_table, self.teleop_slave_table):
            for item in table.get_children():
                table.delete(item)

        if not previews:
            for index in range(1, 8):
                joint_name = f"joint{index}"
                setting = self.teleop_joint_settings.get(joint_name, TeleopJointSetting(name=joint_name))
                self.teleop_master_table.insert(
                    "",
                    tk.END,
                    values=(joint_name, "", f"{abs(setting.scale):.2f}", setting.sign, "Y" if setting.enabled else "N"),
                )
                self.teleop_slave_table.insert("", tk.END, values=(joint_name, "", "", "", ""))
            return

        for preview in previews:
            self.teleop_master_table.insert(
                "",
                tk.END,
                values=(
                    preview.name,
                    self._format_float(preview.master_rad),
                    f"{preview.ratio:.2f}",
                    preview.sign,
                    "Y" if preview.enabled else "N",
                ),
            )
            self.teleop_slave_table.insert(
                "",
                tk.END,
                values=(
                    preview.name,
                    self._format_float(preview.slave_current_rad),
                    self._format_float(preview.target_rad),
                    preview.limit_status,
                    "YES" if preview.name in sent_joints else "NO",
                ),
            )

    def _set_teleop_state(self, state: TeleopState) -> None:
        self.teleop_state = state
        self.teleop_state_text.set(state.value)

    def _set_teleop_state_from_worker(self, state: TeleopState) -> None:
        self.after(0, lambda: self._set_teleop_state(state))

    def _handle_teleop_error(self, message: str) -> None:
        self._set_teleop_state(TeleopState.PAUSED)
        self.append_log(f"主从遥操作停止: {message}")
        messagebox.showerror("主从遥操作停止", message)

    def clear_status_table(self) -> None:
        """清空状态表格。"""

        if not hasattr(self, "status_table"):
            return
        for item in self.status_table.get_children():
            self.status_table.delete(item)

    def update_status_table(self, snapshot: ArmStatusSnapshot) -> None:
        """用最新读取结果刷新状态表格。"""

        self.clear_status_table()
        for joint in snapshot.joints:
            if joint.error is not None:
                values = (
                    joint.name,
                    self._format_device_id(joint.device_id),
                    "-",
                    "-",
                    "-",
                    "-",
                    joint.error,
                )
            else:
                values = (
                    joint.name,
                    self._format_device_id(joint.device_id),
                    self._format_float(joint.position_rad),
                    self._format_float(joint.velocity_rad_s),
                    self._format_float(joint.torque_nm),
                    "" if joint.ticks is None else str(joint.ticks),
                    "OK",
                )
            self.status_table.insert("", tk.END, values=values)

    def _format_device_id(self, device_id: int) -> str:
        profile = self._current_profile()
        if profile.kind == "damiao":
            return f"0x{device_id:02X}"
        return str(device_id)

    @staticmethod
    def _format_float(value: float | None) -> str:
        if value is None:
            return ""
        return f"{value:.4f}"

    def append_log(self, text: str) -> None:
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)

    def clear_log(self) -> None:
        self.log.delete("1.0", tk.END)

    def on_close(self) -> None:
        self.stop_continuous_read()
        self.close_connection()
        self.destroy()


def main() -> None:
    """启动上位机调试工具。"""

    app = DebugToolApp()
    app.mainloop()
