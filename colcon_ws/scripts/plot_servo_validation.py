#!/usr/bin/env python3
"""Generate the servo-model runtime validation figure from a demo capture.

Input: a CSV recorded with scripts/log_servo_demo.py while the demo commander
runs its sequence (two fast steps, then 18 s sweeps at ~0.1 rad/s). One capture
covers both panels; the sweep segments are found in the data rather than
hard-coded, so a re-recording of any length works.

  (a) backlash  — cheap_joint minus ideal_joint plotted against the joint angle.
      The deviation sits on whichever gear flank gravity loads and traverses the
      whole gap when the arm passes its lowest point.
  (b) stick-slip — the same deviation during one slow sweep, at full rate. Below
      the Stribeck knee the rotor sticks, the compliance winds up, and it breaks
      away; the judder amplitude is bounded by the compliance the rotor winds up
      against.

Usage:
  python3 plot_servo_validation.py [capture.csv] [out.png]
"""
import csv
import os
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DOC = os.path.join(HERE, '..', 'src', 'servo_demo_description', 'doc')

CAP = sys.argv[1] if len(sys.argv) > 1 else os.path.join(DOC, 'data', 'runtime_capture.csv')
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(DOC, 'servo_model_validation.png')

# servo_model parameters of servo_demo.simulation.xacro
P_GAIN, K_TRANS = 4.0, 2.0
TAU_S, TAU_C = 0.06, 0.015
GAP = 0.09                       # rad, total backlash


def load_state(path):
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items() if k != 'src'}
                for r in csv.DictReader(f) if r['src'] == 'state']
    if not rows:
        sys.exit(f'{path}: no state rows')
    return rows


def movavg_time(t, v, win_s):
    """Moving average over a fixed TIME window (the sample rate is ~100 Hz)."""
    dt = statistics.median([t[i + 1] - t[i] for i in range(len(t) - 1)]) or 0.01
    h = max(1, int(win_s / dt / 2))
    return [sum(v[max(0, i - h):i + h + 1]) / len(v[max(0, i - h):i + h + 1])
            for i in range(len(v))]


def find_sweeps(rows, min_len_s=12.0, rate_lo=0.05, rate_hi=0.2, grid_s=0.5):
    """Return (start, end, direction) for each long constant-rate sweep of the
    ideal joint. The commander alternates 18 s sweeps with fast steps, so the
    sweeps stand out as long stretches of steady, moderate rate.

    The rate is taken across a coarse time grid rather than sample to sample:
    ideal_pos carries its own ripple, and differentiating it at 100 Hz gives a
    rate several times the real sweep speed."""
    t = [r['t'] for r in rows]
    q = [r['ideal_pos'] for r in rows]
    grid_t, grid_q, i = [], [], 0
    while i < len(t):
        j = i
        while j < len(t) and t[j] < t[i] + grid_s:
            j += 1
        grid_t.append((t[i] + t[j - 1]) / 2)
        grid_q.append(statistics.median(q[i:j]))
        i = j
    rate = [(grid_q[i + 1] - grid_q[i]) / max(grid_t[i + 1] - grid_t[i], 1e-6)
            for i in range(len(grid_q) - 1)]

    out, i = [], 0
    while i < len(rate):
        if rate_lo < abs(rate[i]) < rate_hi:
            j, sign = i, (1 if rate[i] > 0 else -1)
            while j < len(rate) and rate_lo < abs(rate[j]) < rate_hi \
                    and (rate[j] > 0) == (sign > 0):
                j += 1
            if grid_t[j - 1] - grid_t[i] >= min_len_s:
                out.append((grid_t[i], grid_t[j - 1], sign))
            i = j
        else:
            i += 1
    return out


def window(rows, a, b):
    return [r for r in rows if a <= r['t'] <= b]


def main():
    rows = load_state(CAP)
    sweeps = find_sweeps(rows)
    if not sweeps:
        sys.exit('no slow sweep found in the capture')
    up = next((s for s in sweeps if s[2] > 0), None)
    down = next((s for s in sweeps if s[2] < 0), None)
    print('sweeps found:', [(round(a, 1), round(b, 1), d) for a, b, d in sweeps])

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(14, 6))

    # --- (a) backlash: flank offset vs joint angle -------------------------
    for seg, color, label in ((up, 'r', 'sweep up'), (down, 'darkorange', 'sweep down')):
        if seg is None:
            continue
        # trim a second off each end: the commander holds there and the
        # arm is settling rather than sweeping
        s = window(rows, seg[0] + 1.0, seg[1] - 1.0)
        t = [r['t'] for r in s]
        x = movavg_time(t, [r['ideal_pos'] for r in s], 0.07)
        y = movavg_time(t, [(r['cheap_pos'] - r['ideal_pos']) * 1000 for r in s], 0.07)
        axA.plot(x, y, color=color, marker='.', ls='none', ms=2.5,
                 label=f'{label} ({seg[0]:.0f}-{seg[1]:.0f} s)')
    half = GAP / 2 * 1000
    axA.axhline(half, color='gray', ls=':', lw=0.9)
    axA.axhline(-half, color='gray', ls=':', lw=0.9)
    axA.text(-0.98, half + 8, f'+{half:.0f} mrad = upper gear flank',
             fontsize=8, color='gray', va='center')
    axA.text(-0.98, -half - 8, f'-{half:.0f} mrad = lower gear flank',
             fontsize=8, color='gray', va='center')
    axA.annotate('gap traversal at q=0:\nthe arm crosses the whole\nbacklash to the other flank',
                 xy=(0.02, 0), xytext=(0.28, 16),
                 arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)
    axA.set_ylim(-half - 12, half + 12)
    axA.set_xlabel('joint angle (ideal reference) [rad]')
    axA.set_ylabel('cheap_joint - ideal_joint [mrad]')
    axA.set_title('Backlash: gravity-loaded flank offset\n'
                  f'(~0.1 rad/s sweep; backlash {GAP} rad = ±{half:.0f} mrad)')
    axA.grid(alpha=0.3)
    axA.legend(loc='center right', fontsize=8)

    # --- (b) stick-slip judder during a slow sweep -------------------------
    seg = up or down
    mid = (seg[0] + seg[1]) / 2
    s = window(rows, mid - 1.5, mid + 1.5)
    t = [r['t'] for r in s]
    d = [(r['cheap_pos'] - r['ideal_pos']) * 1000 for r in s]
    trend = movavg_time(t, d, 0.5)
    judder = [d[i] - trend[i] for i in range(len(d))]
    pp = max(judder) - min(judder)

    # The rotor winds up against p_gain and the transmission; those two give the
    # bounds the judder has to sit between.
    lo = (TAU_S - TAU_C) / (P_GAIN + K_TRANS) * 1000
    hi = (TAU_S - TAU_C) / (1 / (1 / P_GAIN + 1 / K_TRANS)) * 1000

    axB.plot(t, d, color='m', lw=0.8, alpha=0.35, label='raw (~100 Hz)')
    axB.plot(t, trend, color='m', lw=2.0, label='trend (0.5 s average)')
    axB.fill_between(t, [trend[i] - pp / 2 for i in range(len(t))],
                     [trend[i] + pp / 2 for i in range(len(t))],
                     color='m', alpha=0.10,
                     label=f'judder band, {pp:.0f} mrad p-p')
    axB.set_xlabel('time [s]')
    axB.set_ylabel('cheap_joint - ideal_joint [mrad]')
    axB.set_title('Stick-slip judder during slow motion\n'
                  f'(static {TAU_S} / dynamic {TAU_C} N·m; '
                  f'compliance bounds {lo:.0f}-{hi:.0f} mrad)')
    axB.grid(alpha=0.3)
    axB.legend(loc='lower left', fontsize=8)

    extrema = sum(1 for i in range(1, len(d) - 1)
                  if (d[i] - d[i - 1]) * (d[i + 1] - d[i]) < 0)
    axB.text(0.98, 0.04,
             f'~{extrema / 2 / (t[-1] - t[0]):.0f} cycles/s — at the 100 Hz\n'
             'state-publication limit, so individual\nstick/slip events are not resolved',
             fontsize=7.5, color='0.35', ha='right', transform=axB.transAxes)

    print(f'judder {pp:.1f} mrad p-p, bounds {lo:.1f}..{hi:.1f} mrad, '
          f'~{extrema / 2 / (t[-1] - t[0]):.0f} cycles/s')

    plt.tight_layout()
    plt.savefig(OUT, dpi=120)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
