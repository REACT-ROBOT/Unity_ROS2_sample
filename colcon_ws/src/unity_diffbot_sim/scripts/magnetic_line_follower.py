#!/usr/bin/env python3
"""磁気ラインセンサ (simulation_extra_interfaces/MagneticGuide) でテープを追従する。

シミュレータ v1.4.0 の <sensor type="magnetic_guide"> が publish する
「テープの横位置 [m] (+左)」をそのまま比例制御に入れるだけの最小構成。
実機の AGV でもこの段は同じなので、ここに前後の処理 (マーカでの減速、
分岐の選択、脱線時の復帰) を足していくための土台として使える。

  ros2 launch sim_props_description spawn_prop.launch.py prop:=magnetic_course
  ros2 run unity_diffbot_sim magnetic_line_follower

速度指令は /cmd_vel (geometry_msgs/Twist) に出す。teleop_twist_keyboard と
同じ口なので、velocity_pub が diff_drive_controller へ流してくれる。
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from simulation_extra_interfaces.msg import MagneticGuide


class MagneticLineFollower(Node):

    def __init__(self):
        super().__init__('magnetic_line_follower')

        self.declare_parameter('guide_topic', '/diffbot/magnetic_guide_link/magnetic_guide')
        self.declare_parameter('linear_speed', 0.3)
        # 横ずれ [m] -> 角速度 [rad/s]。センサバーは車軸より 0.12 m 前に
        # 出ているので、これは前方注視点の横ずれを見ていることになる。
        # 同じ配置の pure pursuit 相当は 2*v/L^2 = 40 程度なので、10 は
        # 振動しない側に寄せた値。20 mm ずれたまま曲がる程度の追従になる。
        self.declare_parameter('gain', 10.0)
        self.declare_parameter('max_angular', 1.2)
        # ずれが大きいときに落とす速度の下限比。曲率のきついコーナーで
        # 前に出すぎてテープを外さないため。
        self.declare_parameter('min_speed_ratio', 0.4)
        # これだけテープを見失ったら止める [s]
        self.declare_parameter('lost_timeout', 0.5)

        self._speed = self.get_parameter('linear_speed').value
        self._gain = self.get_parameter('gain').value
        self._max_angular = self.get_parameter('max_angular').value
        self._min_speed_ratio = self.get_parameter('min_speed_ratio').value
        self._lost_timeout = self.get_parameter('lost_timeout').value

        self._last_seen = None
        self._stopped = False
        self._markers = (False, False)

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        topic = self.get_parameter('guide_topic').value
        self._sub = self.create_subscription(MagneticGuide, topic, self._on_guide, 10)
        self.get_logger().info(f"following the tape reported on '{topic}'")

    def _on_guide(self, msg):
        now = self.get_clock().now()

        # マーカは立ち上がりだけ出す。実機だと分岐や停止位置の合図に使う。
        markers = (msg.left_marker, msg.right_marker)
        if markers != self._markers:
            if msg.left_marker and not self._markers[0]:
                self.get_logger().info('marker: left')
            if msg.right_marker and not self._markers[1]:
                self.get_logger().info('marker: right')
            self._markers = markers

        if not msg.track_detected:
            # 一瞬の欠けでは止めない。テープの継ぎ目や素子 1 個の抜けは
            # センサ側で埋めてくれるが、コースを外れたときはここで止まる。
            if self._last_seen is None:
                self._last_seen = now
            lost = (now - self._last_seen).nanoseconds * 1e-9
            if lost >= self._lost_timeout:
                if not self._stopped:
                    self.get_logger().warn(f'track lost for {lost:.1f} s, stopping')
                    self._stopped = True
                self._publish(0.0, 0.0)
            return

        self._last_seen = now
        if self._stopped:
            self.get_logger().info('track found again')
            self._stopped = False

        # position は +左。左にテープがあるなら左に曲がる = +z 回り。
        angular = self._gain * msg.position
        angular = max(-self._max_angular, min(self._max_angular, angular))

        ratio = max(self._min_speed_ratio, 1.0 - abs(angular) / self._max_angular)
        self._publish(self._speed * ratio, angular)

    def stop(self):
        self._publish(0.0, 0.0)

    def _publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MagneticLineFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 抜けるときに速度を残さない。Ctrl-C や SIGTERM では rclpy が先に
        # コンテキストを落としていて publish できないので、そのときは触らない
        # (diff_drive_controller の cmd_vel_timeout が受け持つ)。
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
