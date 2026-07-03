#!/usr/bin/env python3
"""Calibrate GELLO joint offsets.

Usage:
    python scripts/gello_get_offset.py --port /dev/cu.usbserial-XXXXX
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tyro

from gello.dynamixel.driver import DynamixelDriver


@dataclass
class Args:
    port: str = "/dev/ttyUSB0"
    start_joints: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    joint_signs: Tuple[float, ...] = (1, 1, -1, 1, 1, 1)
    gripper: bool = True

    def __post_init__(self):
        assert len(self.joint_signs) == len(self.start_joints)
        for j in self.joint_signs:
            assert j in (-1, 1)

    @property
    def num_robot_joints(self) -> int:
        return len(self.start_joints)

    @property
    def num_joints(self) -> int:
        return self.num_robot_joints + (1 if self.gripper else 0)


def get_config(args: Args) -> None:
    joint_ids = list(range(1, args.num_joints + 1))
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    def get_error(offset: float, index: int, js: np.ndarray) -> float:
        return float(np.abs(args.joint_signs[index] * (js[index] - offset) - args.start_joints[index]))

    for _ in range(10):
        driver.get_joints()

    best_offsets = []
    curr = driver.get_joints()
    for i in range(args.num_robot_joints):
        best = min(
            ((o, get_error(o, i, curr)) for o in np.linspace(-8 * np.pi, 8 * np.pi, 33)),
            key=lambda x: x[1],
        )
        best_offsets.append(best[0])

    print("best offsets               :", [f"{x:.3f}" for x in best_offsets])
    print(
        "best offsets (pi/2 steps): ["
        + ", ".join(f"{int(np.round(x / (np.pi / 2)))} * np.pi / 2" for x in best_offsets)
        + " ]"
    )
    if args.gripper:
        pos = driver.get_joints()[-1]
        print(f"gripper open  (deg): {np.rad2deg(pos) - 0.2:.1f}")
        print(f"gripper close (deg): {np.rad2deg(pos) - 42:.1f}")
    driver.close()


if __name__ == "__main__":
    get_config(tyro.cli(Args))
