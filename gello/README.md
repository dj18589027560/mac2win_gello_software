# gello/ — 核心代码说明

## 目录结构

```
gello/
├── __init__.py               # 包标识
├── agents/                   # 操控代理层（Agent 接口 + 主手读取封装）
│   ├── agent.py              # Agent 协议基类
│   └── gello_agent.py        # GELLO 主手 Agent（含端口配置文件映射）
├── dynamixel/                # Dynamixel 电机硬件驱动
│   └── driver.py             # 串口通信、批量读写、角度/电流控制
├── robots/                   # 机器人接口实现
│   ├── robot.py              # Robot 协议基类（定义 get/command 接口）
│   ├── dynamixel.py          # DynamixelRobot — 主手封装（偏移/符号映射、滤波）
│   ├── ur.py                 # URRobot — UR 从手控制（RTDE 协议）
│   └── robotiq_gripper.py    # Robotiq 夹爪控制（TCP socket 通信）
└── zmq_core/                 # ZMQ 跨机通信层
    └── robot_node.py         # ZMQServerRobot / ZMQClientRobot
```

---

## 分层调用流程

整个遥操作系统的数据流可以抽象为四层：

```
 ┌──────────────────────────────────────────────────────────┐
 │                   入口脚本 (leader_mac.py / follower_ur.py)│
 └──────────┬───────────────────────────────────┬───────────┘
            │                                   │
     ┌──────▼──────┐                    ┌───────▼────────┐
     │  Agent 层    │                    │   Robot 层     │
     │  (主手侧)    │                    │   (从手侧)     │
     └──────┬──────┘                    └───────┬────────┘
            │                                   │
     ┌──────▼──────┐                    ┌───────▼────────┐
     │  Robot 层   │   ZMQ 跨机通信     │   Robot 层     │
     │ Dynamixel  │ ◄──────────────────► │   UR           │
     └──────┬──────┘   robot_node.py    └───────┬────────┘
            │                                   │
     ┌──────▼──────┐                    ┌───────▼────────┐
     │  Driver 层  │                    │   ur-rtde      │
     │ Dynamixel  │                    │   (外部库)      │
     │  串口通信   │                    │   RTDE TCP     │
     └─────────────┘                    └─────────────────┘
```

---

## 各模块详解

### 1. `agents/` — 操控代理层

#### `agent.py` — Agent 协议

定义了一个操控代理的标准接口，所有 Agent 统一实现 `act(obs) -> action`。

```
Agent (Protocol)
├── act(obs) -> np.ndarray     ← 给定观测，返回关节角度指令
├── DummyAgent                 ← 测试用，返回全零角度
└── BimanualAgent              ← 双臂代理，左右各一个子 Agent
```

Agent 协议是依赖倒置的体现：主手（读角度）和从手（发角度）都实现同一个 `act` 接口，上层可以直接互换。

#### `gello_agent.py` — GELLO 主手 Agent

将 Dynamixel 主手封装成 Agent。核心逻辑：

1. **DynamixelRobotConfig** — 数据类，存储一个机械臂配置（ID、偏移量、符号、夹爪参数）
2. **PORT_CONFIG_MAP** — 串口路径到配置的映射表，每个不同的 GELLO 硬件对应一个配置
3. **GelloAgent** — 根据串口路径读取配置 → 创建 DynamixelRobot → `act()` 返回关节角度

```
GelloAgent.__init__(port)
  → 查找 PORT_CONFIG_MAP[port]
  → config.make_robot(port) → DynamixelRobot
  → 初始化完成

GelloAgent.act(obs)
  → self._robot.get_joint_state()
  → 返回 numpy 数组，每个元素是一个关节的角度(rad)
```

---

### 2. `dynamixel/` — 电机驱动层

#### `driver.py` — DynamixelDriver

**功能：** 通过 USB 串口与 ROBOTIS Dynamixel 电机通信，使用 `dynamixel_sdk` 库。

**技术原理：**

- `PortHandler` — 打开 USB 串口（如 `/dev/cu.usbserial-*`）
- `PacketHandler` 2.0 — 构造/解析 Dynamixel 2.0 协议包
- `GroupSyncRead` — 批量读取多个电机的当前位置和速度（单次总线事务）
- `GroupSyncWrite` — 批量写入多个电机的目标位置或电流

**核心方法：**

| 方法 | 功能 | 底层 |
|---|---|---|
| `get_joints()` | 读取所有关节角度 | 从 `GroupSyncRead` 缓存读取，单位：脉冲 → 弧度 |
| `set_joints(angles)` | 设定目标角度 | `GroupSyncWrite` 批量写入位置控制表 |
| `set_torque_mode(bool)` | 使能/关闭力矩 | 逐电机写入 `ADDR_TORQUE_ENABLE` (64) |
| `set_current(currents)` | 设定目标电流（mA） | `GroupSyncWrite` 写入 `ADDR_GOAL_CURRENT` (102) |

**角度转换：**

```
弧度 = 脉冲值 / 2048 × π
```

Dynamixel 电机的内部位置寄存器范围 0-4095（12 位编码器），映射到 0-2π 弧度。`2048 = 4096 / 2` 做半圈修正。

**容错机制：**

- 初始化重试 3 次
- 端口被占用时自动释放（`lsof` / `fuser`）
- 彻底失败可回退到 `FakeDynamixelDriver`（纯内存模拟，方便调试）

**线程模型：**

`_start_reading_thread()` 启动后台线程，以 ~1ms 周期持续轮询电机位置和速度，减少主线程阻塞。

---

### 3. `robots/` — 机器人接口

#### `robot.py` — Robot 协议

定义所有机器人必须实现的接口，使用 Python `Protocol` 实现结构类型系统（structural typing）。

```
Robot (Protocol)
├── num_dofs() -> int                       ← 自由度数量
├── get_joint_state() -> np.ndarray          ← 读关节角度
├── command_joint_state(joint_state)         ← 写关节角度
└── get_observations() -> Dict[str, ...]    ← 读完整观测

辅助类：
├── PrintRobot     ← 调试用，只打印不操控
└── BimanualRobot  ← 双臂组合，左右各一个 Robot
```

#### `dynamixel.py` — DynamixelRobot（主手封装）

将底层的 `DynamixelDriver`（裸电机通信）封装成符合 `Robot` 协议的高层接口。

**初始化流程：**

```
DynamixelRobot.__init__(joint_ids, offsets, signs, real, port, gripper_config)
  → 创建 DynamixelDriver(ids, port=port)        ← 连接硬件
  → 保存 offsets / signs 映射                    ← 坐标系对齐
  → 如果提供 start_joints，自动修正 offset       ← multi-turn 缠绕补偿
```

**关节映射关系：**

```
输出角度 = (原始读数 - offset) × sign
```

- `offset` — 机械零点偏移，通常为 π/2 的整数倍（Dynamixel 编码器零位不一定是机械零位）
- `sign` — 方向符号，1 或 -1，用于修正电机安装方向导致的旋转方向反转

**指数平滑滤波：**

```python
pos = last_pos × (1 - α) + current_pos × α    # α = 0.99
```

对读取的关节角度做低通滤波，减少串口噪声导致的抖动。

**Multi-turn 缠绕补偿：**

Dynamixel 电机断电后内部计数器重置到 [0, 2π)，丢失了多圈信息。`start_joints` 参数通过比较当前读数和期望值，自动修正 ±2π 的整数倍偏移。

**夹爪映射：**

夹爪关节角度（连续旋转）映射到 `[0, 1]` 的开合百分比空间：

```
gripper_pos = (raw - open_angle) / (close_angle - open_angle)
gripper_pos = clamp(gripper_pos, 0, 1)
```

#### `ur.py` — URRobot（从手控制）

通过 `ur-rtde` 库控制 UR 机器人，采用 RTDE（Real-Time Data Exchange）协议。

**核心机制：**

| 方法 | 功能 | 实现 |
|---|---|---|
| `get_joint_state()` | 读取 UR 当前关节角度 | `rtde_receive.getActualQ()` |
| `command_joint_state(joints)` | 下发关节角度指令 | `rtde_control.servoJ()` — 实时伺服控制 |
| `set_freedrive_mode(bool)` | 启用/禁用拖动示教 | `freedriveMode()` / `endFreedriveMode()` |

**伺服控制参数：**

```python
servoJ(target_q, velocity=0.5, acceleration=0.5, dt=0.002, lookahead_time=0.2, gain=100)
```

- `dt=0.002` — 控制周期 2ms，对应 500Hz
- `lookahead_time=0.2` — 200ms 前馈预测，平滑轨迹
- `gain=100` — 位置增益

#### `robotiq_gripper.py` — 夹爪驱动

通过 TCP socket 连接 Robotiq 夹爪，使用 MODBUS RTU 风格的字符串指令集。支持 UR 控制箱转发（UR 的 63352 端口作为夹爪的网关）。

指令包括：激活、设置位置/速度/力、读取状态/故障码。

---

### 4. `zmq_core/` — 跨机通信层

#### `robot_node.py` — ZMQ 机器人节点

**功能：** 通过 ZMQ（ZeroMQ）实现跨机器的 Robot 协议调用。

**两种角色：**

```
ZMQServerRobot (服务端 — Mac)
  绑定 tcp://0.0.0.0:6000
  接收 pickle 序列化的请求包 → 调用本地 Robot 方法 → 返回 pickle 响应

ZMQClientRobot (客户端 — Windows)
  连接 tcp://MacIP:6000
  发送 pickle 序列化的请求包 → 等待响应 → 反序列化返回
```

**通信协议：**

```
请求格式:  {"method": "get_joint_state", "args": {}}
响应格式:  numpy.ndarray (pickle 序列化)

请求格式:  {"method": "command_joint_state", "args": {"joint_state": ...}}
响应格式:  None
```

ZMQ 的 REQ/REP 模式是严格的一问一答，天然同步。客户端发送请求后阻塞等待，服务端处理完一个请求再处理下一个。

**支持的 RPC 方法：**

| Method | 对应 Robot 接口 |
|---|---|
| `num_dofs` | `robot.num_dofs()` |
| `get_joint_state` | `robot.get_joint_state()` |
| `command_joint_state` | `robot.command_joint_state(**args)` |
| `get_observations` | `robot.get_observations()` |

---

## 完整数据流追踪

### 正向路径：主手 → 从手

```
Mac 端:
  1. DynamixelDriver 后台线程读取电机脉冲
  2. DynamixelRobot 转换为弧度，应用 offset/sign 和滤波
  3. ZMQServerRobot 等待请求

Windows 端:
  4. ZMQClientRobot.get_joint_state() 发送 RPC 请求
  5. Mac 端 ZMQServerRobot 收到请求，调用 get_joint_state()
  6. 角度值 pickle 序列化，返回
  7. Windows 端反序列化，得到 numpy 数组
  8. URRobot.command_joint_state() 通过 RTDE servoJ 发送到 UR
  9. UR 控制箱实时驱动关节
```

### 端到端延迟构成

| 环节 | 典型耗时 | 说明 |
|---|---|---|
| Dynamixel 串口轮询 | ~1ms | 后台线程，不阻塞 |
| 角度计算 + 滤波 | <0.1ms | numpy 向量化 |
| ZMQ 序列化 + 网络 | 1-5ms | 局域网 ping <1ms |
| UR servoJ 控制 | 2ms | 500Hz 控制周期 |
| **总延迟** | **~5-10ms** | 远低于人感知阈值 |

## 关键设计决策

| 决策 | 理由 |
|---|---|
| ZMQ 而非 ROS2 | 去中心化，无 master，跨平台（Windows/macOS 原生） |
| Pickle 序列化 | Python 原生，numpy 数组零拷贝 |
| REQ/REP 而非 PUB/SUB | 控制通道需要可靠应答，视频通道用 PUB/SUB 更合适（已在入口脚本中实现） |
| 后台线程读 Dynamixel | 串口通信有阻塞 IO，独立线程保证主循环不卡 |
| 角度映射而非直接透传 | Dynamixel 原始脉冲无物理意义，需要 offset/sign 映射到机器人坐标系 |
