# mac2win_gello_software — macOS UR 遥操作 + RealSense 视频回传

基于 [GELLO](https://github.com/wuphilipp/gello_software) 的跨平台 UR 遥操作精简版。

**场景：** macOS 笔记本连接 Dynamixel 主手，通过局域网（或 Tailscale 虚拟组网）远程操控 Windows 电脑上的 UR 机械臂，同时实时查看 UR 上的 RealSense 相机画面。

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
#    LEADER_IP = Mac 的 Tailscale 虚拟 IP（或同局域网 IP）
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

---

## 跨网络通信：Tailscale 虚拟组网（推荐）

当 Mac 和 Windows 不在同一个 WiFi 网络时（例如移动小车换到不同的 WiFi 环境），两者无法通过局域网 IP 直接通信。Tailscale 可以解决这个问题。

### 原理

Tailscale 在每台设备上安装轻量级客户端，通过 NAT 穿透建立 P2P 加密隧道。每台设备获得一个固定的虚拟 IP（`100.x.x.x`），无论底层是哪个 WiFi 还是 4G 网络，虚拟 IP 始终不变。

```
┌─ Mac ──────────┐        WiFi A          ┌─ Windows ──────┐
│                 │  ───────────────►      │                 │
│  Tailscale      │    P2P 加密隧道        │  Tailscale      │
│  100.x.x.x     │  ◄───────────────      │  100.x.x.x     │
│                 │                        │                 │
└─────────────────┘                        └─────────────────┘
```

- 非传统 VPN（不经过中心服务器转发）
- 底层协议：WireGuard（加密、低延迟）
- 控制服务器仅在初次握手时参与，通信建立后数据直连

### 适用场景

| Mac 网络 | Windows 网络 | 能否通信 |
|---|---|---|
| 同一 WiFi | 同一 WiFi | ✅ 直连 |
| 不同 WiFi（同一区域） | 不同 WiFi | ✅ Tailscale 穿透 |
| 家里 WiFi | 公司 WiFi | ✅ |
| 家里 WiFi | 手机 4G 热点 | ✅ |
| 有互联网的环境 | 有互联网的环境 | ✅ （必须都能上网） |

### 安装与配置

**1. 注册账号**
打开 https://tailscale.com，点击 **Get Started Free**，用 Google / GitHub / Microsoft 账号登录。

**2. Windows 端安装**
```cmd
winget install Tailscale.Tailscale
```
或从 https://tailscale.com/download 下载安装包。安装后任务栏找到 Tailscale 图标 → Sign in → 用同一账号登录。

查虚拟 IP：
```cmd
tailscale ip -4
```

**3. Mac 端安装**
```bash
brew install --cask tailscale
```
或从 https://tailscale.com/download 下载安装。菜单栏 Tailscale 图标 → Sign in → 登录同一账号。

查虚拟 IP：
```bash
tailscale ip -4
```

**4. 验证连通**
Mac 上 ping Windows 的虚拟 IP：
```bash
ping 100.x.x.x
```

**5. 修改代码**
将 `leader_mac.py` 和 `follower_ur.py` 中的 IP 替换为网 Tailscale 虚拟 IP（`100.x.x.x` 格式），之后无需再因网络切换而修改。

### 延迟参考

| 场景 | 典型延迟 |
|---|---|
| 同 WiFi 直连 | <1ms |
| Tailscale 同 WiFi | <2ms |
| Tailscale 跨网络 | 5-30ms |
| 人体可感知阈值 | >50ms |
