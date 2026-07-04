# mac2win_gello_software — macOS UR 遥操作 + RealSense 视频回传

基于 [GELLO](https://github.com/wuphilipp/gello_software) 的跨平台 UR 遥操作精简版。

**场景：** macOS 笔记本连接 Dynamixel 主手，通过局域网远程操控 Windows 电脑上的 UR 机械臂，同时实时查看 UR 上的 RealSense 相机画面。

## 架构

### 控制流

```
┌─ macOS ─────────────────┐    ZMQ REQ/REP     ┌─ Windows ────────────────┐    RTDE    ┌─ UR ────┐
│                          │   TCP port 6000    │                           │  ────────► │         │
│  Dynamixel 主手 (USB)    │  ─────────────────► │  UR 控制 (ur-rtde)       │            │  UR5 臂  │
│  └─ ZMQServer            │    joint angles    │  ├─ ZMQClient ← 读角度    │            │         │
│     (0.0.0.0:6000)       │                    │  └─ URRobot  → 发指令    │            │         │
└──────────────────────────┘                    └───────────────────────────┘            └─────────┘
```

### 视频流

```
┌─ Windows ─────────────────────────────────┐   ZMQ PUB/SUB    ┌─ Mac ───────────────────┐
│                                            │  TCP port 6001  │                         │
│  RealSense 相机 ← USB                      │                 │  没有相机硬件            │
│       ↓                                    │                 │       ↓                  │
│  pyrealsense2 读取帧                       │                 │  ZMQ SUB 接收 JPEG      │
│       ↓                                    │                 │       ↓                  │
│  cv2.imencode('.jpg') 压缩                 │   ──────────►   │  cv2.imdecode 解码      │
│       ↓                                    │                 │       ↓                  │
│  ZMQ PUB 发送 (0.0.0.0:6001)              │                 │  cv2.imshow 窗口显示    │
└────────────────────────────────────────────┘                 └──────────────────────────┘
```

### 技术要点

- **控制通道 (port 6000):** ZMQ REQ/REP 模式。Windows 作为客户端请求主手关节角度，Mac 作为服务端响应。数据用 pickle 序列化。
- **视频通道 (port 6001):** ZMQ PUB/SUB 模式。Windows 作为发布者持续发送压缩帧，Mac 作为订阅者接收并显示。一对多，不阻塞。
- **JPEG 压缩:** 640×480×3 的原始帧约 900KB，JPEG 压缩后约 30-50KB，局域网传输无压力。
- **独立线程:** 视频收发在独立线程运行，不干扰 UR 控制循环（500Hz）和主手读取。

## 文件结构

```
gello_software/
├── leader_mac.py              ← macOS: 读 Dynamixel + ZMQ 发布 + 视频接收
├── follower_ur.py             ← Windows: ZMQ 接收 + UR 控制 + 视频发送
├── test_video_receive.py      ← macOS: 单独测试视频接收（不依赖主手）
├── test_video_only.py         ← Windows: 单独测试视频发送（不依赖 UR）
├── requirements.txt
├── setup.py
├── scripts/
│   └── gello_get_offset.py    ← 关节标定工具
└── gello/
    ├── dynamixel/driver.py    ← Dynamixel 电机通信
    ├── robots/
    │   ├── robot.py           ← Robot 协议基类
    │   ├── dynamixel.py       ← DynamixelRobot (主手封装)
    │   ├── ur.py              ← URRobot (从手控制)
    │   └── robotiq_gripper.py ← 夹爪驱动
    └── zmq_core/
        └── robot_node.py      ← ZMQServerRobot / ZMQClientRobot
```

## 快速开始

### macOS 端（主手 + 视频接收）

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

### Windows 端（从手 + 视频发送）

```cmd
# 1. 安装依赖
pip install -r requirements.txt

# 2. 修改 follower_ur.py 中的 IP 地址
#    LEADER_IP = macOS 的局域网 IP
#    UR_IP     = UR 控制箱的 IP

# 3. 启动
python follower_ur.py
```

## 视频流单独测试

不启动 UR 和主手，只验证 RealSense 视频通路：

**Windows 端：**
```cmd
python test_video_only.py
```

**Mac 端：**
```bash
python test_video_receive.py
```

Mac 上会弹出 "RealSense Camera (test)" 窗口。按 `Q` 关闭。

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

## IP 配置参考

两台机器需要在同一个局域网。从 Mac 查看本机 IP：

```bash
ifconfig | grep "inet " | grep -v 127.0.0.1
```

从 Windows 查看本机 IP：

```cmd
ipconfig
```

找到 WLAN 或以太网的 IPv4 地址，分别填入 `leader_mac.py`（`FOLLOWER_IP`）和 `follower_ur.py`（`LEADER_IP`）。

## 依赖

| 依赖 | macOS | Windows | 用途 |
|---|---|---|---|
| dynamixel_sdk | ✅ pip | — | Dynamixel 电机通信 |
| pyzmq | ✅ pip | ✅ pip | 跨机通信 |
| opencv-python | ✅ pip | ✅ pip | 视频编解码与显示 |
| numpy | ✅ pip | ✅ pip | 数值计算 |
| ur-rtde | — | ✅ pip | UR 机器人 RTDE 控制 |
| pyrealsense2 | — | ✅ pip | Intel RealSense 相机驱动 |
| tyro | ✅ pip | — | 命令行参数解析 |

## 与原始项目的区别

从原始 [wuphilipp/gello_software](https://github.com/wuphilipp/gello_software) 精简而来，去掉了与本场景无关的部分：

- **移除** ROS2 控制器（Franka FR3）
- **移除** Docker / NVIDIA / CUDA 相关
- **移除** MuJoCo 仿真
- **移除** YAM / Panda / xArm 机器人后端
- **移除** SpaceMouse、Quest VR
- **移除** 数据采集工具、重力补偿等高级功能
- **保留** Dynamixel 通信、UR 控制、ZMQ 跨机通信
- **新增** RealSense 视频跨机回传

## License

MIT License（继承原始项目）

原始项目：[wuphilipp/gello_software](https://github.com/wuphilipp/gello_software)
