#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows follower - UR teleop + RealSense video stream to Mac."""

import threading
import time

import cv2
import numpy as np
import zmq
import pyrealsense2 as rs
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.robots.ur import URRobot

# ---- CONFIG ----
LEADER_IP = "100.86.175.41"
ZMQ_PORT = 6000
UR_IP = "192.168.12.111"
USE_GRIPPER = True
JOINT_MAP_SIGN = np.array([1, 1, 1, 1, 1, 1])

VIDEO_PORT = 6001
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30
JPEG_QUALITY = 80
# ----------------


def video_sender_thread():
    """Capture RealSense frames and send to Mac via ZMQ PUB."""
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://0.0.0.0:{VIDEO_PORT}")
    print(f"Video sender started on port {VIDEO_PORT}")

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


# --- Main ---
leader = ZMQClientRobot(port=ZMQ_PORT, host=LEADER_IP)
follower = URRobot(robot_ip=UR_IP, no_gripper=not USE_GRIPPER)

# Smooth transition from current UR position to leader position
ur_all = follower.get_joint_state()
ur_joints = np.array(ur_all)
print(f"UR init ({len(ur_joints)}d): {[f'{x:.3f}' for x in ur_joints]}")

leader_all = leader.get_joint_state()
leader_arm = np.array(leader_all[:6]) * JOINT_MAP_SIGN
leader_gripper = np.array([leader_all[6]]) if len(leader_all) > 6 else np.array([])
leader_joints = np.concatenate([leader_arm, leader_gripper])
print(f"Leader   ({len(leader_joints)}d): {[f'{x:.3f}' for x in leader_joints]}")

print("Smooth transition...")
for t in np.linspace(0, 1, 100):
    blend = (1 - t) * ur_joints + t * leader_joints
    follower.command_joint_state(blend)
    time.sleep(0.005)

# Start video thread
vt = threading.Thread(target=video_sender_thread, daemon=True)
vt.start()
time.sleep(1)

print("Teleoperation + video streaming started. Ctrl+C to stop.")
try:
    while True:
        try:
            t_start = time.perf_counter()

            joints = leader.get_joint_state()
            arm = np.array(joints[:6]) * JOINT_MAP_SIGN
            gripper = np.array([joints[6]]) if len(joints) > 6 else np.array([])
            cmd = np.concatenate([arm, gripper])
            follower.command_joint_state(cmd)

            elapsed = time.perf_counter() - t_start
            if elapsed < 0.002:
                time.sleep(0.002 - elapsed)
        except (zmq.error.ZmqError, EOFError, RuntimeError) as e:
            print(f"Connection lost ({e}), reconnecting in 5s...")
            time.sleep(5)
            leader = ZMQClientRobot(port=ZMQ_PORT, host=LEADER_IP)
except KeyboardInterrupt:
