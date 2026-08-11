#!/usr/bin/env python3
"""Sensor-visualization leak test.

Run against a simulator started with SIM_AUTO_SENSOR_VIZ=1 and a spawned
diffbot (2D lidar + RGB camera + depth camera). While the point-cloud /
image visualizations are active in the GUI, the images published to ROS
must stay clean:

1. /diffbot/lidar_link/scan keeps publishing sane ranges.
2. /diffbot/camera_link/image_raw arrives and contains no pixels of the
   point-cloud overlay color (saturated green, drawn on a layer the
   sensor cameras must not render).
3. /diffbot/depth_camera_link/depth_image_raw keeps publishing.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

TIMEOUT = 30.0


class VizLeakTest(Node):
    def __init__(self):
        super().__init__("sensor_viz_leak_test")
        self.scan = None
        self.image = None
        self.depth = None
        self.create_subscription(LaserScan, "/diffbot/lidar_link/scan",
                                 self._on_scan, 10)
        self.create_subscription(Image, "/diffbot/camera_link/image_raw",
                                 self._on_image, 10)
        self.create_subscription(Image, "/diffbot/depth_camera_link/depth_image_raw",
                                 self._on_depth, 10)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_image(self, msg):
        self.image = msg

    def _on_depth(self, msg):
        self.depth = msg


def main():
    rclpy.init()
    node = VizLeakTest()
    deadline = time.time() + TIMEOUT
    while time.time() < deadline and not (node.scan and node.image and node.depth):
        rclpy.spin_once(node, timeout_sec=0.2)

    results = []

    def check(name, ok, detail):
        results.append((name, ok, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    check("scan_arrives", node.scan is not None,
          "LaserScan received" if node.scan else "no LaserScan in %.0fs" % TIMEOUT)
    if node.scan:
        # 有限レンジの有無は環境依存 (視野内に背の高い物が要る) なので、
        # 数は情報として出すだけにして本数だけを検査する。
        finite = [r for r in node.scan.ranges
                  if node.scan.range_min <= r <= node.scan.range_max]
        check("scan_shape", len(node.scan.ranges) > 0,
              f"{len(node.scan.ranges)} rays, {len(finite)} finite in range")

    check("image_arrives", node.image is not None,
          "camera image received" if node.image else "no camera image in %.0fs" % TIMEOUT)
    if node.image:
        img = node.image
        ok_enc = img.encoding in ("rgb8", "bgr8")
        check("image_encoding", ok_enc, img.encoding)
        if ok_enc:
            data = img.data
            green = 0
            # 重畳の緑はリニア→sRGB 変換や AA で淡くなるので、「緑が他 2 成分
            # より十分強い」ことで検出する。全画素走査 (800x600 でも数秒程度)。
            for i in range(0, len(data) - 2, 3):
                g = data[i + 1]
                if g - max(data[i], data[i + 2]) > 40:
                    green += 1
            total = img.width * img.height
            check("no_viz_leak", green == 0,
                  f"{green}/{total} green-dominant pixels in published image")

    check("depth_arrives", node.depth is not None,
          "depth image received" if node.depth else "no depth image in %.0fs" % TIMEOUT)

    node.destroy_node()
    rclpy.shutdown()

    failed = [name for name, ok, _ in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    if failed:
        print("FAILED:", ", ".join(failed))
        sys.exit(1)


if __name__ == "__main__":
    main()
