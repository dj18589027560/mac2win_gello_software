#!/usr/bin/env python3
"""仅测试接收并显示 Windows 发来的视频流（不涉及 Dynamixel/UR）。"""

import cv2
import numpy as np
import zmq

WINDOWS_IP = "192.168.33.119"
VIDEO_PORT = 6001

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(f"tcp://{WINDOWS_IP}:{VIDEO_PORT}")
sock.subscribe(b"")
sock.setsockopt(zmq.RCVTIMEO, 1000)

cv2.namedWindow("RealSense Camera (test)", cv2.WINDOW_NORMAL)
print(f"Waiting for video from {WINDOWS_IP}:{VIDEO_PORT}...")

try:
    while True:
        try:
            jpeg_buf = sock.recv()
            frame = cv2.imdecode(np.frombuffer(jpeg_buf, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("RealSense Camera (test)", frame)
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
