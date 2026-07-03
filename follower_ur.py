#!/usr/bin/env python3
"""UR follower — read leader joints via ZMQ, smoothly control UR via RTDE."""

import time

import numpy as np
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.robots.ur import URRobot

# ---- 修改为你的实际参数 ----
LEADER_IP = "192.168.x.x"        # ← MacBook 的局域网 IP
ZMQ_PORT = 6000
UR_IP = "192.168.1.10"           # ← UR 控制箱 IP
USE_GRIPPER = True
# 如果你发现某个关节方向反了，在这里反转对应 joint_sign
# 例如 Joint 1 反了: -1 改成 1
JOINT_MAP_SIGN = np.array([1, 1, 1, 1, 1, 1])   # ← 可逐个调整
# --------------------------------

leader = ZMQClientRobot(port=ZMQ_PORT, host=LEADER_IP)
follower = URRobot(robot_ip=UR_IP, no_gripper=not USE_GRIPPER)

# 1) 读取当前 UR 位置
ur_joints = follower.get_joint_state()[:6]
print(f"UR 初始关节: {[f'{x:.3f}' for x in ur_joints]}")

# 2) 读取主手位置
leader_joints = leader.get_joint_state()
leader_joints_6 = leader_joints[:6] * JOINT_MAP_SIGN
print(f"主手关节:     {[f'{x:.3f}' for x in leader_joints_6]}")

# 3) 从当前位置缓慢过渡到主手位置（100 步，~0.5 秒）
print("平滑过渡中...")
for t in np.linspace(0, 1, 100):
    blend = (1 - t) * ur_joints + t * leader_joints_6
    follower.command_joint_state(blend)
    time.sleep(0.005)

# 4) 进入主控制循环
print("遥操作启动. Ctrl+C 停止.")
try:
    while True:
        t_start = time.perf_counter()

        joints = leader.get_joint_state()
        cmd = joints[:6] * JOINT_MAP_SIGN
        follower.command_joint_state(cmd)

        elapsed = time.perf_counter() - t_start
        if elapsed < 0.002:
            time.sleep(0.002 - elapsed)
except KeyboardInterrupt:
    print("\n已停止.")
