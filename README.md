# Open Kaka V0

Open Kaka V0 是一个面向双臂机器人原型的基础控制软件版本。当前版本聚焦在主从臂调试、零点/限位标定、粉色主臂到粉色从臂遥操作、上位机调试工具，以及基于 URDF 的 3D 可视化。

这个仓库暂时不包含触觉、LeRobot 数据集采集、VLA 推理、深度相机 SDK 接入等后续功能。那些能力会在后续版本中逐步加入。

## 当前功能

- Feetech STS3215 主臂位置读取
- Damiao 从臂状态读取
- 粉色主臂到粉色从臂遥操作
- 单关节和多关节遥操作调试
- 软件零点、关节限位、MIT 增益配置
- 急停/disable 安全接口
- Tkinter 上位机调试工具
- 粉色从臂 URDF/STL 3D 可视化
- 主臂控制虚拟从臂的网页映射调试工具

## 硬件配置

当前默认配置来自已调试的粉色机械臂系统：

| 设备 | 通信 | 默认端口 | 说明 |
| --- | --- | --- | --- |
| 粉色主臂 | Feetech STS3215 串口总线 | COM8 / 1000000 | 舵机 ID 8~14 |
| 粉色从臂 | Damiao USB2CAN | COM7 / 921600 | CAN ID 0x21~0x27 |

从臂电机：

| 关节 | CAN ID | Master ID | 电机类型 |
| --- | --- | --- | --- |
| joint1 | 0x21 | 0x31 | DM4340 |
| joint2 | 0x22 | 0x32 | DM4340 |
| joint3 | 0x23 | 0x33 | DM4340 |
| joint4 | 0x24 | 0x34 | DM4310 |
| joint5 | 0x25 | 0x35 | DM4310 |
| joint6 | 0x26 | 0x36 | DM4310 |
| joint7 | 0x27 | 0x37 | DM4310 |

## 安全原则

请先读完这一段再连接真实机械臂。

- 默认只读，不自动使能，不自动运动
- 真实运动必须显式添加 `--enable-motion YES`
- 启动时先读取当前位置，不直接回零
- 运动目标会经过软件限位检查
- Ctrl+C 后会尝试 disable 从臂
- 保存硬件零点/硬件限位必须二次确认
- 调试时必须准备物理急停或断电手段

## 安装

建议使用独立 Python 环境：

```powershell
cd open-kaka-V0
python -m pip install -e .
```

安装达妙电机 Python SDK：

```powershell
git clone https://github.com/cmjang/DM_Control_Python.git third_party\damiao\DM_Control_Python
```

如果系统没有 `git` 命令，也可以在浏览器下载 zip，解压到：

```text
third_party/damiao/DM_Control_Python/
```

网页 3D 可视化需要 Node.js：

```powershell
cd web_viewer\pink_slave_3d
npm install
cd ..\..
```

## 快速开始

读取粉色主臂：

```powershell
python scripts\read_arms\read_master_pink.py --once
```

读取粉色从臂：

```powershell
python scripts\read_arms\read_slave_pink.py --once
```

启动上位机调试工具：

```powershell
python scripts\apps\start_debug_tool.py
```

单关节遥操作测试：

```powershell
python scripts\teleop\06_pink_single_joint_zero_delta_follow.py --joint joint1 --master-port COM8 --slave-port COM7 --enable-motion YES
```

全关节遥操作：

```powershell
python scripts\teleop\07_pink_multi_joint_zero_delta_follow.py --joints joint1 joint2 joint3 joint4 joint5 joint6 joint7 --master-port COM8 --slave-port COM7 --test-scale 1.0 --hz 40 --alpha 0.55 --max-target-speed-rad-s 1.2 --master-deadband-rad 0.003 --target-deadband-rad 0.002 --slave-read-every 4 --enable-motion YES
```

启动主臂到虚拟从臂 3D 映射工具：

```powershell
python scripts\visualization\start_pink_master_virtual_slave_3d.py --master-port COM8
```

## 目录结构

```text
open-kaka-V0/
  configs/                 零点、限位、遥操作映射和增益配置
  robot_core/              核心 Python 包
    apps/                  Tkinter 上位机
    drivers/               Feetech / Damiao 驱动占位与适配入口
    services/              读取、标定、安全、遥操作服务层
    utils/                 硬件只读工具
    visualization/         URDF/socket 消息工具
  scripts/
    apps/                  上位机启动入口
    read_arms/             四条机械臂读取脚本
    teleop/                单关节/多关节遥操作脚本
    visualization/         3D 可视化和网页映射工具
  assets/
    pink_slave_urdf/       粉色从臂 URDF/STL
  web_viewer/
    pink_slave_3d/         Three.js + URDF Loader 网页查看器
  third_party/             第三方 SDK 安装说明
  docs/                    使用说明
```

## 版本范围

V0 保留的是已经实机调试过的基础能力：

- 主臂读取
- 从臂读取
- 标定与限位
- 主从遥操作
- URDF 可视化

V0 暂不包含：

- LeRobot 数据集采集
- VLA 推理
- Paxini 触觉传感器
- Orbbec 深度/IR/IMU SDK
- 双臂协同任务规划
- Pinocchio 正逆解和完整重力补偿

## License

MIT License. See [LICENSE](LICENSE).
