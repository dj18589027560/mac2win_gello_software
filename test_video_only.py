#!/usr/bin/env python3
"""仅测试 RealSense 视频流发送（不涉及 UR 控制）。"""

import time
import cv2
import numpy as np
import zmq
import pyrealsense2 as rs

VIDEO_PORT = 6001
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30
JPEG_QUALITY = 80

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind(f"tcp://0.0.0.0:{VIDEO_PORT}")
print(f"Video sender started on port {VIDEO_PORT}")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
pipeline.start(config)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

print("Streaming RealSense video... Press Ctrl+C to stop.")
try:
    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            continue
        img = np.asanyarray(color.get_data())
        _, jpeg = cv2.imencode(".jpg", img, encode_param)
        pub.send(jpeg.tobytes())
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    pipeline.stop()
    pub.close()
    ctx.term()
