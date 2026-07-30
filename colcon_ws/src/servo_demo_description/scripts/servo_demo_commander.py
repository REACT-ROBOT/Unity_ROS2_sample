#!/usr/bin/env python3
"""Send the same command pattern to the ideal servo and the cheap servo.

Pattern (looped):
  1. fast step to +1.0 rad   -> cheap servo overshoots and rings
  2. fast step to -1.0 rad   -> backlash reversal rattle + ringing
  3. slow sweep to +1.0 rad  -> stick-slip judder (Stribeck friction)
  4. slow sweep to -1.0 rad
  5. return to 0 rad
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['ideal_joint', 'cheap_joint']

# (target [rad], move duration [s], hold time after the move [s])
SEQUENCE = [
    (1.0, 0.25, 2.5),
    (-1.0, 0.25, 2.5),
    (1.0, 18.0, 1.0),
    (-1.0, 18.0, 1.0),
    (0.0, 0.25, 2.0),
]


class ServoDemoCommander(Node):

    def __init__(self):
        super().__init__('servo_demo_commander')
        self.pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.index = 0
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        if self.pub.get_subscription_count() == 0:
            # コントローラ起動待ち (1秒周期で再試行)
            return

        target, duration, hold = SEQUENCE[self.index]

        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [target] * len(JOINTS)
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration - int(duration)) * 1e9))
        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info(
            f'target={target:+.2f} rad, duration={duration:.2f} s')

        self.index = (self.index + 1) % len(SEQUENCE)
        self.timer.cancel()
        self.timer = self.create_timer(duration + hold, self.tick)


def main():
    rclpy.init()
    node = ServoDemoCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
