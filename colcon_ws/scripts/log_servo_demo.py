#!/usr/bin/env python3
"""Debug logger: record joint_states + JTC desired positions to CSV."""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

OUT = sys.argv[1] if len(sys.argv) > 1 else '/home/unity/colcon_ws/servo_demo_log.csv'
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 35.0


class Logger(Node):
    def __init__(self):
        super().__init__('servo_demo_logger')
        self.f = open(OUT, 'w')
        self.f.write('t,src,ideal_pos,cheap_pos,ideal_vel,cheap_vel\n')
        self.t0 = time.monotonic()
        self.create_subscription(JointState, '/ServoDemo/joint_states', self.cb_js, 50)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/ServoDemo/joint_trajectory_controller/state',
                                 self.cb_state, 50)

    def row(self, src, names, pos, vel):
        try:
            i = names.index('ideal_joint')
            c = names.index('cheap_joint')
        except ValueError:
            return
        t = time.monotonic() - self.t0
        vi = vel[i] if len(vel) > i else float('nan')
        vc = vel[c] if len(vel) > c else float('nan')
        self.f.write(f'{t:.4f},{src},{pos[i]:.6f},{pos[c]:.6f},{vi:.6f},{vc:.6f}\n')

    def cb_js(self, msg):
        self.row('state', list(msg.name), msg.position, msg.velocity)

    def cb_state(self, msg):
        self.row('desired', list(msg.joint_names),
                 msg.desired.positions, msg.desired.velocities)


def main():
    rclpy.init()
    node = Logger()
    end = time.monotonic() + DURATION
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.f.close()
    print(f'wrote {OUT}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
