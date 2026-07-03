#!/usr/bin/env python3
"""macOS leader — read Dynamixel, publish joint angles via ZMQ."""

import numpy as np
from gello.robots.dynamixel import DynamixelRobot
from gello.zmq_core.robot_node import ZMQServerRobot

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
# --------------------------------

robot = DynamixelRobot(
    joint_ids=JOINT_IDS,
    joint_offsets=JOINT_OFFSETS,
    joint_signs=JOINT_SIGNS,
    real=True,
    port=SERIAL_PORT,
    gripper_config=GRIPPER_CONFIG,
)

server = ZMQServerRobot(robot, port=ZMQ_PORT, host="0.0.0.0")
print(f"Leader started on port {ZMQ_PORT}, waiting for follower...")
try:
    server.serve()
except KeyboardInterrupt:
    print("\nShutting down.")
    robot._driver.close()
