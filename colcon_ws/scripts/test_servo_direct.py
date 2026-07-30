#!/usr/bin/env python3
"""Direct Unity-side test: publish joint_command to a test robot and log states.

Sequence: hold +0.5 rad (5 s) -> step to -0.5 rad (5 s) -> step to +0.5 (5 s).
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NS = sys.argv[1] if len(sys.argv) > 1 else 'ServoTest'
OUT = sys.argv[2] if len(sys.argv) > 2 else f'/home/unity/colcon_ws/{NS}_direct_log.csv'


class DirectTest(Node):
    def __init__(self):
        super().__init__('servo_direct_test')
        self.pub = self.create_publisher(JointState, f'/{NS}/joint_command', 10)
        self.create_subscription(JointState, f'/{NS}/joint_states', self.cb, 50)
        self.f = open(OUT, 'w')
        self.f.write('t,cmd,ideal_pos,cheap_pos,ideal_vel,cheap_vel\n')
        self.t0 = time.monotonic()
        self.cmd = 0.5
        self.create_timer(0.02, self.send)

    def target(self):
        t = time.monotonic() - self.t0
        return 0.5 if (t // 5) % 2 == 0 else -0.5

    def send(self):
        self.cmd = self.target()
        msg = JointState()
        msg.name = ['ideal_joint', 'cheap_joint']
        msg.position = [self.cmd, self.cmd]
        self.pub.publish(msg)

    def cb(self, msg):
        try:
            i = list(msg.name).index('ideal_joint')
            c = list(msg.name).index('cheap_joint')
        except ValueError:
            return
        t = time.monotonic() - self.t0
        self.f.write(f'{t:.4f},{self.cmd:.3f},{msg.position[i]:.6f},{msg.position[c]:.6f},'
                     f'{msg.velocity[i]:.6f},{msg.velocity[c]:.6f}\n')


def main():
    rclpy.init()
    node = DirectTest()
    end = time.monotonic() + 16.0
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.f.close()
    print(f'wrote {OUT}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
