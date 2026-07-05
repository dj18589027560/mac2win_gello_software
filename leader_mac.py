#!/usr/bin/env python3
"""macOS leader — read Dynamixel, publish joint angles via ZMQ.
   Also receives RealSense video from Windows follower."""

import threading
import time

import cv2
import numpy as np
import zmq
from gello.robots.dynamixel import DynamixelRobot
from gello.zmq_core.robot_node import ZMQServerRobot

# ---- GELLO 主手参数 ----
SERIAL_PORT = "/dev/cu.usbserial-FTBTJFYI"
JOINT_IDS = (1, 2, 3, 4, 5, 6)
JOINT_OFFSETS = [
    1 * np.pi / 2,
    3 * np.pi / 2,
    2 * np.pi / 2,
    2 * np.pi / 2,
    2 * np.pi / 2,
    2 * np.pi / 2,
]
JOINT_SIGNS = (1, 1, -1, 1, 1, 1)
GRIPPER_CONFIG = (7, 202, 160)
ZMQ_PORT = 6000

# ---- 视频接收参数 ----
FOLLOWER_IP = "100.67.171.63"   # ← Windows Tailscale IP
VIDEO_PORT = 6001
# --------------------------------

# 启动 ZMQ 服务（后台线程）
robot = DynamixelRobot(
    joint_ids=JOINT_IDS,
    joint_offsets=JOINT_OFFSETS,
    joint_signs=JOINT_SIGNS,
    real=True,
    port=SERIAL_PORT,
    gripper_config=GRIPPER_CONFIG,
)

server = ZMQServerRobot(robot, port=ZMQ_PORT, host="0.0.0.0")
st = threading.Thread(target=server.serve, daemon=True)
st.start()

# 视频接收（主线程，OpenCV GUI 必须在主线程）
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(f"tcp://{FOLLOWER_IP}:{VIDEO_PORT}")
sock.subscribe(b"")
sock.setsockopt(zmq.RCVTIMEO, 1000)

cv2.namedWindow("RealSense Camera", cv2.WINDOW_NORMAL)
print(f"Leader started on port {ZMQ_PORT}, video from {FOLLOWER_IP}:{VIDEO_PORT}")
print("Press Q on video window to quit.")

try:
    while True:
        try:
            jpeg_buf = sock.recv()
            frame = cv2.imdecode(
                np.frombuffer(jpeg_buf, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is not None:
                cv2.imshow("RealSense Camera", frame)
        except zmq.Again:
            pass
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    sock.close()
    ctx.term()
    robot._driver.close()
