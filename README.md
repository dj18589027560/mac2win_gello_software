# mac2win_gello_software — macOS UR 遥操作

基于 [GELLO](https://github.com/wuphilipp/gello_software) 的跨平台 UR 遥操作精简版。

**场景：** macOS 笔记本连接 Dynamixel 主手，通过局域网远程操控 Windows 电脑上的 UR 机械臂。

## 架构

```
┌─ macOS ─────────────────┐    ZMQ (TCP 6000)    ┌─ Windows ────────────────┐    RTDE    ┌─ UR ────┐
│                          │  ───────────────────► │                           │  ────────► │         │
│  Dynamixel 主手 (USB)    │   joint angles       │  UR 控制 (ur-rtde)       │            │  UR5 臂  │
│  └─ ZMQServer            │                      │  ├─ ZMQClient ← 读角度    │            │         │
│     (0.0.0.0:6000)       │                      │  └─ URRobot  → 发指令    │            │         │
└──────────────────────────┘                      └───────────────────────────┘            └─────────┘
```

- 主手：Dynamixel 电机 → USB 串口 → ZMQ Server
- 通信：局域网 TCP（ZMQ REQ/REP）
- 从手：ZMQ Client → ur-rtde → UR 控制箱
- 完全不需要 Linux、Docker、ROS2

## 文件结构

```
gello_software/
├── leader_mac.py              ← macOS: 读 Dynamixel + ZMQ 发布
├── follower_ur.py             ← Windows: ZMQ 接收 + UR 控制
├── requirements.txt
├── setup.py
├── scripts/
│   └── gello_get_offset.py    ← 关节标定工具
├── gello/
│   ├── dynamixel/driver.py    ← Dynamixel 电机通信
│   ├── robots/
│   │   ├── robot.py           ← Robot 协议基类
│   │   ├── dynamixel.py       ← DynamixelRobot (主手封装)
│   │   ├── ur.py              ← URRobot (从手控制)
│   │   └── robotiq_gripper.py ← 夹爪驱动
│   └── zmq_core/
│       └── robot_node.py      ← ZMQServerRobot / ZMQClientRobot
```

## 快速开始

### macOS 端（主手）

```bash
# 1. 安装依赖（推荐 arm64 Python 环境）
conda create -n gello python=3.9 -y
conda activate gello
pip install -r requirements.txt
pip install -e .

# 2. 确认串口
ls /dev/cu.usbserial-*

# 3. 关节标定（把 GELLO 摆好参考姿势）
python scripts/gello_get_offset.py \
  --port /dev/cu.usbserial-XXXXX \
  --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
  --joint-signs 1 1 -1 1 1 1

# 4. 将标定得到的 offsets 填入 leader_mac.py，然后启动
python leader_mac.py
```

### Windows 端（从手）

```cmd
# 1. 安装依赖
pip install ur-rtde pyzmq numpy

# 2. 修改 follower_ur.py 中的 IP 地址
#    LEADER_IP = macOS 的局域网 IP
#    UR_IP     = UR 控制箱的 IP

# 3. 启动
python follower_ur.py
```

## 标定说明

标定脚本读取 Dynamixel 当前角度，与 `--start-joints` 指定的参考姿势对比，计算出关节偏移量（offset）。每次 Dynamixel 断电重启后内部编码器复位，建议重新标定。

**UR 参考命令：**

```bash
python scripts/gello_get_offset.py \
  --port /dev/cu.usbserial-XXXXX \
  --start-joints 0 -1.57 1.57 -1.57 -1.57 0 \
  --joint-signs 1 1 -1 1 1 1 \
  --gripper
```

## 依赖

| 依赖 | macOS | Windows |
|---|---|---|
| dynamixel_sdk | ✅ pip 安装 | — |
| pyzmq | ✅ pip 安装 | ✅ pip 安装 |
| ur-rtde | — | ✅ pip 安装（需 Boost C++ 库） |
| numpy | ✅ pip 安装 | ✅ pip 安装 |
| tyro | ✅ pip 安装 | — |

## 与原始项目的区别

从原始 [wuphilipp/gello_software](https://github.com/wuphilipp/gello_software) 精简而来，去掉了与本场景无关的部分：

- **移除** ROS2 控制器（Franka FR3）
- **移除** Docker / NVIDIA / CUDA 相关
- **移除** MuJoCo 仿真
- **移除** YAM / Panda / xArm 机器人后端
- **移除** RealSense 相机、SpaceMouse、Quest VR
- **移除** 数据采集工具、重力补偿等高级功能
- **保留** Dynamixel 通信、UR 控制、ZMQ 跨机通信

## License

MIT License（继承原始项目）

原始项目：[wuphilipp/gello_software](https://github.com/wuphilipp/gello_software)
