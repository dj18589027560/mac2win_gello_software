#!/usr/bin/env python3
"""Windows follower — read leader joints via ZMQ + send RealSense video to Mac."""

import threading
import time

import cv2
import numpy as np
import zmq
import pyrealsense2 as rs
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.robots.ur import URRobot

# ---- 通信参数 ----
LEADER_IP = "192.168.x.x"        # ← MacBook 的局域网 IP
ZMQ_PORT = 6000
UR_IP = "192.168.1.10"           # ← UR 控制箱 IP
USE_GRIPPER = True
JOINT_MAP_SIGN = np.array([1, 1, 1, 1, 1, 1])

# ---- 视频参数 ----
VIDEO_PORT = 6001
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30
JPEG_QUALITY = 80                # 1-100, 调低减少带宽
# --------------------------------


def video_sender_thread():
    """读取 RealSense 帧，JPEG 压缩后通过 ZMQ PUB 发送到 Mac."""
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://0.0.0.0:{VIDEO_PORT}")
    print(f"Video sender started on port {VIDEO_PORT}")

    # 初始化 RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
    pipeline.start(config)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue

            img = np.asanyarray(color.get_data())
            _, jpeg = cv2.imencode(".jpg", img, encode_param)
            pub.send(jpeg.tobytes())
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        pipeline.stop()
        pub.close()
        ctx.term()


# --- 主控制逻辑 ---
leader = ZMQClientRobot(port=ZMQ_PORT, host=LEADER_IP)
follower = URRobot(robot_ip=UR_IP, no_gripper=not USE_GRIPPER)

# 读取初始位置，平滑过渡
ur_joints = follower.get_joint_state()[:6]
print(f"UR init: {[f'{x:.3f}' for x in ur_joints]}")

leader_joints = leader.get_joint_state()[:6] * JOINT_MAP_SIGN
print(f"Leader:   {[f'{x:.3f}' for x in leader_joints]}")

print("Smooth transition...")
for t in np.linspace(0, 1, 100):
    blend = (1 - t) * ur_joints + t * leader_joints
    follower.command_joint_state(blend)
    time.sleep(0.005)

# 启动视频发送线程
vt = threading.Thread(target=video_sender_thread, daemon=True)
vt.start()
time.sleep(1)  # 等 RealSense 初始化

# 主控制循环
print("Teleoperation + video streaming started. Ctrl+C to stop.")
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
    print("\nStopped.")
