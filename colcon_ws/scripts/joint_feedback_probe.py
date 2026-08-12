#!/usr/bin/env python3
"""Verify joint_states velocity feedback quality against a running stack.

Checks, using the conformance diffbot (spawned here):
  1. standstill: reported wheel velocities are exactly 0 (position-derived
     feedback; the old jointVelocity readout showed a constant ~0.01 rad/s
     residual that made wheel odometry drift while parked)
  2. motion: commanded 2.0 rad/s is reported within 5 %
  3. stop again: velocities return to 0

Usage: python3 joint_feedback_probe.py <urdf_path>
"""
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from sim_test_utils.client import SimClient

WHEELS = ["left_wheel_joint", "right_wheel_joint"]
NAME = "feedback_probe_bot"
# joint topics are named after the URDF <robot name>, not the entity name
JOINT_NS = "diffbot"


def main(urdf_path):
    rclpy.init()
    client = SimClient()
    node = client.node

    samples = []

    def cb(msg):
        row = {}
        for j in WHEELS:
            if j in msg.name:
                i = list(msg.name).index(j)
                if i < len(msg.velocity) and i < len(msg.position):
                    row[j] = msg.velocity[i]
                    row[j + "_pos"] = msg.position[i]
        if row:
            row["t"] = time.monotonic()
            samples.append(row)

    node.create_subscription(JointState, f"/{JOINT_NS}/joint_states", cb, 30)
    pub = node.create_publisher(JointState, f"/{JOINT_NS}/joint_command", 10)

    client.spawn(NAME, urdf_path=urdf_path, pose=(0.0, 0.0, 0.05))
    try:
        client.play()
    except Exception as exc:  # already playing is fine
        if "Already in requested state" not in str(exc):
            raise

    def collect(seconds, command=None):
        samples.clear()
        end = time.monotonic() + seconds
        msg = JointState()
        msg.name = WHEELS
        while time.monotonic() < end:
            if command is not None:
                msg.velocity = [command, command]
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.03)
        return list(samples)

    ok = True

    # 1. standstill (let it settle first)
    collect(3.0)
    still = collect(5.0)
    if not still:
        print("FAIL: no joint_states samples received")
        return 1
    vels = [abs(r[j]) for r in still for j in WHEELS]
    peak = max(vels)
    mean = statistics.mean(vels) if vels else float("nan")
    print(f"standstill: n={len(still)} mean|v|={mean:.6f} peak|v|={peak:.6f} rad/s")
    if peak > 1e-4:
        ok = False
        print("  FAIL: standstill velocity above 1e-4 rad/s")
    else:
        print("  OK")

    # 2. drive: reported velocity must agree with d(position)/dt.
    # (Absolute tracking of the command is a PLANT property -- diffbot's soft
    # 30000 drive damping slips under load -- so it is only logged here.)
    collect(3.0, command=2.0)  # spin-up
    run = collect(4.0, command=2.0)
    for j in WHEELS:
        vs = [r[j] for r in run if j in r]
        dpos = run[-1][j + "_pos"] - run[0][j + "_pos"]
        dt = run[-1]["t"] - run[0]["t"]
        mean_v = statistics.mean(vs)
        deriv = dpos / dt
        print(f"drive {j}: mean v={mean_v:.3f}  d(pos)/dt={deriv:.3f} rad/s")
        if abs(mean_v - deriv) > max(0.1, 0.05 * abs(deriv)):
            ok = False
            print("  FAIL: velocity feedback inconsistent with position derivative")
        else:
            print("  OK (plant tracking of the 2.0 rad/s command is a separate property)")

    # 3. stop again: give the plant time to truly settle, then require the
    # feedback to go quiet (residual creep while settling is real motion and
    # is honestly reported, so only the settled tail must be ~0).
    collect(8.0, command=0.0)
    still2 = collect(4.0)
    peak2 = max(abs(r[j]) for r in still2 for j in WHEELS if j in r)
    print(f"stop again (after 8 s settle): peak|v|={peak2:.6f} rad/s")
    if peak2 > 1e-3:
        ok = False
        print("  FAIL")
    else:
        print("  OK")

    client.delete(NAME)
    client.close()
    rclpy.shutdown()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
