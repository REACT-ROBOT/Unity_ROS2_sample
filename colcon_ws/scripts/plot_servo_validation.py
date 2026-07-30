#!/usr/bin/env python3
"""Generate the servo-model validation figure from demo measurement CSVs.

Inputs (recorded with scripts/log_servo_demo.py):
  - a low-friction sweep run  -> backlash hysteresis panel
  - a high-friction sweep run -> stick-slip panel
Defaults point at the bundled datasets in servo_demo_description/doc/data/.

Usage:
  python3 plot_servo_validation.py [low_friction.csv] [stickslip.csv] [out.png]
"""
import csv
import statistics
import sys
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = os.path.join(HERE, '..', 'src', 'servo_demo_description', 'doc')

LOW = sys.argv[1] if len(sys.argv) > 1 else os.path.join(DOC, 'data', 'sweep_low_friction.csv')
STICK = sys.argv[2] if len(sys.argv) > 2 else os.path.join(DOC, 'data', 'sweep_stickslip.csv')
OUT = sys.argv[3] if len(sys.argv) > 3 else os.path.join(DOC, 'servo_model_validation.png')


def load(path):
    rows = {'state': [], 'desired': []}
    with open(path) as f:
        for r in csv.DictReader(f):
            rows[r['src']].append({k: float(v) for k, v in r.items() if k != 'src'})
    return rows


def rollmed(v, k=5):
    h = k // 2
    return [statistics.median(v[max(0, i - h):i + h + 1]) for i in range(len(v))]


def movavg(v, k=5):
    h = k // 2
    return [sum(v[max(0, i - h):i + h + 1]) / len(v[max(0, i - h):i + h + 1])
            for i in range(len(v))]


fig, (axA, axB) = plt.subplots(1, 2, figsize=(14, 6))

# --- (a) backlash hysteresis: flank offset vs joint angle ---
st = load(LOW)['state']
seg = [d for d in st if 21.5 <= d['t'] <= 58.5]
xi = rollmed([d['ideal_pos'] for d in seg], 7)
dy = rollmed([(d['cheap_pos'] - d['ideal_pos']) * 1000 for d in seg], 7)
up = [(x, y) for d, x, y in zip(seg, xi, dy) if d['t'] < 39.7]
dn = [(x, y) for d, x, y in zip(seg, xi, dy) if d['t'] >= 39.7]
axA.plot([x for x, _ in up], [y for _, y in up], 'r.', ms=3, label='sweep up (-1 -> +1)')
axA.plot([x for x, _ in dn], [y for _, y in dn], color='darkorange', marker='.',
         ls='none', ms=3, label='sweep down (+1 -> -1)')
axA.axhline(45, color='gray', ls=':', lw=0.9)
axA.axhline(-45, color='gray', ls=':', lw=0.9)
axA.text(0.32, 42, '+45 mrad = upper gear flank', fontsize=8, color='gray', va='center')
axA.text(0.32, -42, '-45 mrad = lower gear flank', fontsize=8, color='gray', va='center')
axA.annotate('gap traversal at q=0:\narm falls 90 mrad across the\nbacklash to the other flank',
             xy=(0.02, 0), xytext=(0.28, 15),
             arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)
axA.set_ylim(-48, 48)
axA.set_xlabel('joint angle (ideal reference) [rad]')
axA.set_ylabel('cheap_joint - ideal_joint [mrad]')
axA.set_title('Backlash: gravity-loaded flank offset\n'
              '(0.11 rad/s sweep; backlash 0.09 rad = ±45 mrad)')
axA.grid(alpha=0.3)
axA.legend(loc='lower left', fontsize=8)

# --- (b) stick-slip during slow motion ---
st2 = load(STICK)['state']
seg2 = [d for d in st2 if 30.0 <= d['t'] <= 36.0]
ts = [d['t'] for d in seg2]
diff = [(d['cheap_pos'] - d['ideal_pos']) * 1000 for d in seg2]
sm = movavg(diff, 5)  # ~0.3 s window at the ~16 Hz state rate
axB.plot(ts, diff, color='m', lw=0.7, alpha=0.25, label='raw (16 Hz samples)')
axB.plot(ts, sm, color='m', lw=1.8, label='moving average (0.3 s)')
axB.annotate('stick (flat, ~0.7 s)', xy=(32.9, 5), xytext=(30.3, -18),
             arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)
axB.annotate('breakaway jump', xy=(33.25, -2), xytext=(34.2, 8),
             arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)
axB.set_ylim(min(diff) - 4, max(diff) + 4)
axB.set_xlabel('time [s]')
axB.set_ylabel('cheap_joint - ideal_joint [mrad]')
axB.set_title('Stick-slip during slow motion (0.11 rad/s)\n'
              '(static 0.06 / dynamic 0.015 N·m; common ripple removed)')
axB.grid(alpha=0.3)
axB.legend(loc='lower left', fontsize=8)

plt.tight_layout()
plt.savefig(OUT, dpi=120)
print(f'wrote {OUT}')
