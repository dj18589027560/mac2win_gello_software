#!/usr/bin/env python3
"""
方案一：Mac 本地 MuJoCo UR5 仿真

直接用 Dynamixel 主手驱动 MuJoCo 中的 UR5 模型。
不需要 ZMQ、不需要 Windows、不需要 UR 真实机器人。

数据流：
  Dynamixel 主手 (USB)
    → DynamixelDriver 读取关节角度
    → 映射到 UR5 关节空间 (offset + sign)
    → data.ctrl[3:9] = mapped_angles
    → mj_step() 物理推进
    → GLFW 窗口渲染

使用方式：
  1. 克隆模型文件到上级目录:
      cd ..
      git clone https://github.com/dj18589027560/mujoco-ur5-articulated.git
  2. 运行:
      conda activate gello
      python scripts/sim_ur5_local.py
"""

import math
import os
import sys
import threading
import time

# ============================================================
# 第 1 部分：导入依赖
# ============================================================
import mujoco
import glfw
import numpy as np
from gello.dynamixel.driver import DynamixelDriver

# ============================================================
# 第 2 部分：Dynamixel 主手配置（与 leader_mac.py 保持一致）
# ============================================================
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
GRIPPER_ID = 7  # 夹爪的电机 ID

# ============================================================
# 第 3 部分：MuJoCo 模型路径
# ============================================================
# 默认假设 mujoco-ur5-articulated 克隆在 gello_software 的上级目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)  # gello_software/
DEFAULT_MODEL_DIR = os.path.join(PROJECT_DIR, "..", "mujoco-ur5-articulated")

import argparse
parser = argparse.ArgumentParser(description="MuJoCo UR5 simulation driven by GELLO leader")
parser.add_argument("--model-dir", default=DEFAULT_MODEL_DIR,
                    help="Path to mujoco-ur5-articulated directory")
parser.add_argument("--port", default=SERIAL_PORT,
                    help="Dynamixel serial port")
args = parser.parse_args()

MODEL_DIR = os.path.abspath(args.model_dir)
MODEL_XML = os.path.join(MODEL_DIR, "ur5_articulated.xml")

if not os.path.exists(MODEL_XML):
    print(f"错误：找不到模型文件 {MODEL_XML}")
    print("请先克隆模型仓库:")
    print(f"  cd {os.path.dirname(MODEL_DIR)}")
    print("  git clone https://github.com/dj18589027560/mujoco-ur5-articulated.git")
    sys.exit(1)

# ============================================================
# 第 4 部分：连接到 Dynamixel 主手
# ============================================================
print(f"连接主手到 {args.port}...")
driver = DynamixelDriver(JOINT_IDS, port=args.port, baudrate=57600)
# 不要使能力矩（主手是 passively 读取，不需要 torque）
# driver.set_torque_mode(False)  初始化时 driver 默认 torque off

print(f"主手连接成功, 关节 ID: {JOINT_IDS}")

# ============================================================
# 第 5 部分：初始化 MuJoCo 模型
# ============================================================
os.chdir(MODEL_DIR)  # MuJoCo 需要从模型目录加载 STL 文件

model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

print(f"MuJoCo {mujoco.__version__}  自由度 nq={model.nq}  执行器 nu={model.nu}")

# actuator 索引:
#   [0, 1, 2] = 移动底座 (base_x, base_y, base_theta) — 速度控制
#   [3, 4, 5, 6, 7, 8] = UR5 关节 (J1..J6) — 位置控制
#   [9, 10] = 夹爪 (gripper_left, gripper_right)
J1, J2, J3, J4, J5, J6 = 3, 4, 5, 6, 7, 8
GL, GR = 9, 10

# 设置 UR5 初始姿态（和 run_ur5_articulated.py 一致）
u_init = [-math.pi/2, -3.054, -0.0873, -math.pi, -math.pi/2, -math.pi]
data.qpos[3:9] = u_init
data.ctrl[J1:J6+1] = u_init
data.ctrl[GL] = data.ctrl[GR] = 0.0
mujoco.mj_forward(model, data)

# ============================================================
# 第 6 部分：初始化 GLFW 渲染窗口
# ============================================================
if not glfw.init():
    print("GLFW 初始化失败")
    sys.exit(1)

window = glfw.create_window(1200, 800, "GELLO → MuJoCo UR5", None, None)
if not window:
    glfw.terminate()
    sys.exit(1)
glfw.make_context_current(window)

# 相机设置
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.azimuth = 135
cam.elevation = -20
cam.distance = 3.0
cam.lookat[:] = [0.3, 0.0, 0.3]

# 渲染资源
scene = mujoco.MjvScene(model, maxgeom=1000)
opt = mujoco.MjvOption()
mujoco.mjv_defaultOption(opt)
pert = mujoco.MjvPerturb()
ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# 键盘回调：相机控制 + 退出
def key_callback(w, key, scancode, action, mods):
    if action == glfw.PRESS or action == glfw.REPEAT:
        step = 5.0
        if key == glfw.KEY_LEFT:
            cam.azimuth -= step
        elif key == glfw.KEY_RIGHT:
            cam.azimuth += step
        elif key == glfw.KEY_UP:
            cam.elevation = min(90, cam.elevation + step)
        elif key == glfw.KEY_DOWN:
            cam.elevation = max(-90, cam.elevation - step)
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

def scroll_callback(w, xoff, yoff):
    cam.distance *= (1.0 - yoff * 0.08)
    cam.distance = max(0.5, min(20, cam.distance))

glfw.set_key_callback(window, key_callback)
glfw.set_scroll_callback(window, scroll_callback)

# ============================================================
# 第 7 部分：后台物理线程
# ============================================================
# MuJoCo 物理推进在后台线程以 ~500Hz 运行，
# 这样渲染线程不会被物理计算阻塞
physics_running = True

def physics_loop():
    while physics_running:
        mujoco.mj_step(model, data)
        time.sleep(0.002)  # ~500Hz

t = threading.Thread(target=physics_loop, daemon=True)
t.start()

# ============================================================
# 第 8 部分：主循环 — 读主手 → 映射 → 写入 MuJoCo
# ============================================================

# 读取主手初始位置，让仿真从当前位置起步
joint_offsets_arr = np.array(JOINT_OFFSETS)
joint_signs_arr = np.array(JOINT_SIGNS)

# 先预读几帧，让 Dynamixel 后台线程稳定
for _ in range(10):
    driver.get_joints()
    time.sleep(0.01)

# 读取主手初始角度
raw_init = driver.get_joints()
leader_init = (raw_init[:6] - joint_offsets_arr) * joint_signs_arr
print(f"主手初始角度 (rad): {[f'{x:.3f}' for x in leader_init]}")

# 将 UR5 设为主手初始角度（而不是默认的 u_init）
data.qpos[3:9] = leader_init
data.ctrl[J1:J6+1] = leader_init
mujoco.mj_forward(model, data)

print("\n开始遥操作仿真！掰动主手控制 UR5.")
print("方向键旋转相机, 滚轮缩放, ESC 退出.")

try:
    while not glfw.window_should_close(window):
        # --- 第 8a 步：读主手 ---
        raw = driver.get_joints()  # 返回 numpy 数组, 7 个元素

        # --- 第 8b 步：映射到 UR5 关节空间 ---
        #   leader_joint = (raw_motor - offset) * sign
        #   这是 DynamixelRobot 内部同样的公式
        ur5_targets = (raw[:6] - joint_offsets_arr) * joint_signs_arr

        # --- 第 8c 步：写入 MuJoCo 控制 ---
        #   位置控制模式下, data.ctrl 是目标位置,
        #   物理引擎会自动计算力矩驱动机器人到该位置
        data.ctrl[J1:J6+1] = ur5_targets

        # --- 第 8d 步：夹爪 ---
        if len(raw) > 6:
            # 夹爪原始角度 → [0, 1] 开合比
            gripper_raw = raw[6]
            gripper_open = 202.1  # 度, 同标定值
            gripper_close = 160.1
            gripper_norm = (gripper_raw - math.radians(gripper_close)) / (
                math.radians(gripper_open) - math.radians(gripper_close)
            )
            gripper_norm = max(0.0, min(1.0, gripper_norm))
            # 映射到夹爪开度 (0~0.04 rad)
            data.ctrl[GL] = gripper_norm * 0.04
            data.ctrl[GR] = (1.0 - gripper_norm) * 0.04

        # --- 第 8e 步：渲染 ---
        glfw.poll_events()
        viewport = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
        mujoco.mjv_updateScene(model, data, opt, pert, cam,
                               mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, ctx)
        glfw.swap_buffers(window)

        # 控制循环频率 ~100Hz (10ms)
        # 物理线程仍然以 500Hz 运行，不受此限制
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n退出中...")
finally:
    physics_running = False
    glfw.terminate()
    driver.close()
