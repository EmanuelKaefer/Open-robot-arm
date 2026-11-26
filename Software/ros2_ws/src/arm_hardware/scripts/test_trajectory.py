#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
# See LICENSE-SOFTWARE in the project root for details.

"""End-to-end test: JointTrajectoryController -> ArmSystem -> STM (or stm_sim.py).

Sends FollowJointTrajectory goals to arm_controller (all joints to +target, then back to 0)
and checks after each goal that /joint_states really arrived there.

Start first:  python3 Software/test_codes/stm_sim/stm_sim.py -v
              ros2 launch arm_hardware hw_test.launch.py
Then:         ros2 run arm_hardware test_trajectory.py [--target 0.3] [--duration 3.0]

Exit code 0 = PASS, 1 = FAIL.
"""

import argparse
import sys
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['shoulder_joint1', 'shoulder_joint2', 'elbow_joint',
          'wrist_turn_joint', 'wrist_tilt_joint', 'gripper_turn_joint']
ACTION = '/arm_controller/follow_joint_trajectory'


class TrajectoryTest(Node):

    def __init__(self):
        super().__init__('test_trajectory')
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

    def _on_js(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move(self, target, duration):
        """Move all joints to target [rad] in duration [s]. Return the result error_code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [target] * len(JOINTS)
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected by arm_controller')
            return None
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=duration + 10.0)
        if not res.done():
            self.get_logger().error('no result within duration + 10 s')
            return None
        return res.result().result.error_code

    def check(self, target, tol):
        """Compare the newest /joint_states with target. True = all joints within tol."""
        self.spin_for(0.5)  # STATE lags the command by 3-4 periods
        if self.joint_state is None:
            self.get_logger().error('no /joint_states received')
            return False
        ok = True
        for j in JOINTS:
            pos = self.joint_state.get(j)
            if pos is None or abs(pos - target) > tol:
                self.get_logger().error(f'{j}: {pos} rad, expected {target} +- {tol}')
                ok = False
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--target', type=float, default=0.3, help='rad (default 0.3)')
    ap.add_argument('--duration', type=float, default=3.0, help='s per move (default 3)')
    ap.add_argument('--tol', type=float, default=1e-3, help='rad (default 1e-3)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = TrajectoryTest()
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(f'{ACTION} not available - is hw_test.launch.py running?')
            return 1

        passed = True
        for target in (args.target, 0.0):
            node.get_logger().info(f'all joints -> {target} rad in {args.duration} s')
            code = node.move(target, args.duration)
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                node.get_logger().error(f'trajectory failed, error_code {code}')
                passed = False
                break
            if not node.check(target, args.tol):
                passed = False
                break
            node.get_logger().info(f'reached {target} rad')

        print('PASS' if passed else 'FAIL')
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
