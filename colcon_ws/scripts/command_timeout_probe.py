#!/usr/bin/env python3
"""Verify the joint_command comm-timeout watchdog against a running stack.

Runs two diffbots sequentially (one at a time; commanding two bots from a
single spin_once loop starves the subscriptions and gives flaky readings):
  1. guarded bot (command_timeout=0.5 injected into the URDF's existing
     <hardware> block): drives at 3 rad/s, must coast to 0 after 2.5 s of
     command silence, and must respond again when commands resume
  2. stock bot: must KEEP the last command through the same silence
     (the pre-existing behaviour, i.e. the watchdog is opt-in)

Note: diffbot.urdf hardcodes its topic names in ros2_control params, so the
per-bot rename must replace every occurrence of "diffbot".

Usage: python3 command_timeout_probe.py <diffbot_urdf_path>
"""
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

from sim_test_utils.client import SimClient

WHEELS = ["left_wheel_joint", "right_wheel_joint"]


def run_bot(client, name, urdf, phases):
    node = client.node
    client.spawn(name, urdf=urdf, pose=(0.0, -5.0, 0.05))
    try:
        client.play()
    except Exception as exc:
        if "Already in requested state" not in str(exc):
            raise
    vel = {"v": None}

    def cb(msg):
        if WHEELS[0] in msg.name:
            i = list(msg.name).index(WHEELS[0])
            if i < len(msg.velocity):
                vel["v"] = msg.velocity[i]

    sub = node.create_subscription(JointState, f"/{name}/joint_states", cb, 10)
    pub = node.create_publisher(JointState, f"/{name}/joint_command", 10)
    msg = JointState()
    msg.name = WHEELS
    results = {}
    for label, seconds, command in phases:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                msg.velocity = [command, command]
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.03)
        results[label] = vel["v"]
    client.delete(name)
    node.destroy_subscription(sub)
    node.destroy_publisher(pub)
    return results


def main(urdf_path):
    with open(urdf_path) as f:
        base = f.read()
    guarded = base.replace("diffbot", "ct_guarded").replace(
        "<hardware>",
        '<hardware>\n      <param name="command_timeout">0.5</param>', 1)
    stock = base.replace("diffbot", "ct_stock")

    rclpy.init()
    client = SimClient()
    ok = True

    g = run_bot(client, "ct_guarded", guarded,
                [("drive", 2.5, 3.0), ("silence", 2.5, None), ("resume", 2.5, 3.0)])
    print(f"guarded: drive={g['drive']:.2f} silence={g['silence']:.2f} resume={g['resume']:.2f} rad/s")
    if g["drive"] < 1.0:
        ok = False
        print("  FAIL: guarded bot did not spin up")
    if abs(g["silence"]) > 0.2:
        ok = False
        print("  FAIL: watchdog did not stop the guarded bot")
    if g["resume"] < 1.0:
        ok = False
        print("  FAIL: guarded bot did not recover after commands resumed")

    s = run_bot(client, "ct_stock", stock,
                [("drive", 2.5, 3.0), ("silence", 2.5, None)])
    print(f"stock:   drive={s['drive']:.2f} silence={s['silence']:.2f} rad/s")
    if abs(s["silence"]) < 2.0:
        ok = False
        print("  FAIL: stock bot no longer holds the last command (behaviour change!)")

    client.close()
    rclpy.shutdown()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
