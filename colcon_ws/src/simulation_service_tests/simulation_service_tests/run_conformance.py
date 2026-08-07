"""サービス適合性テストの実行と結果出力。

    ros2 run simulation_service_tests service_conformance --profile diffbot

シミュレータと ROS-TCP-Endpoint が既に上がっている前提で動く。
両方の起動込みで回したいときは scripts/service_conformance_test.sh を使う。
"""

import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy

from . import scenarios
from .profile import load_profile
from .scenarios import ERROR, FAIL, KNOWN_GAP, PASS, SKIP
from .sim_harness import ServiceTimeout, SimHarness

COLORS = {
    PASS: '\033[32m',
    FAIL: '\033[31m',
    KNOWN_GAP: '\033[33m',
    SKIP: '\033[90m',
    ERROR: '\033[35m',
}
RESET = '\033[0m'
BOLD = '\033[1m'


def colorize(text, status, enabled):
    if not enabled:
        return text
    return f'{COLORS.get(status, "")}{text}{RESET}'


def wrap(text, width, indent):
    """detail を端末幅に収める簡易折り返し。"""
    out, line = [], ''
    for word in str(text).replace('\n', ' ').split(' '):
        if len(line) + len(word) + 1 > width and line:
            out.append(line)
            line = word
        else:
            line = f'{line} {word}'.strip()
    if line:
        out.append(line)
    return f'\n{indent}'.join(out)


def print_summary(results, color, width=100):
    counts = {PASS: 0, FAIL: 0, KNOWN_GAP: 0, SKIP: 0, ERROR: 0}
    group = None
    print()
    print(f'{BOLD if color else ""}==== シミュレータ サービス適合性テスト 結果 ===={RESET if color else ""}')
    for r in results:
        counts[r.status] = counts.get(r.status, 0) + 1
        if r.scenario.group != group:
            group = r.scenario.group
            print(f'\n{BOLD if color else ""}{group}{RESET if color else ""}')
        badge = colorize(f'{r.status:<9}', r.status, color)
        print(f'  {badge} {r.scenario.key:<4} {r.scenario.title}  ({r.elapsed:.1f}s)')
        if r.detail:
            print(f'           {wrap(r.detail, width - 11, " " * 11)}')
        if r.scenario.known_gap and r.status == KNOWN_GAP and r.scenario.why:
            print(f'           理由: {wrap(r.scenario.why, width - 17, " " * 17)}')

    print()
    parts = [
        colorize(f'PASS {counts[PASS]}', PASS, color),
        colorize(f'FAIL {counts[FAIL]}', FAIL, color),
        colorize(f'KNOWN_GAP {counts[KNOWN_GAP]}', KNOWN_GAP, color),
        colorize(f'SKIP {counts[SKIP]}', SKIP, color),
        colorize(f'ERROR {counts[ERROR]}', ERROR, color),
    ]
    print('  '.join(parts))
    return counts


def write_junit(results, path, profile_name):
    suite = ET.Element('testsuite', {
        'name': f'simulation_service_conformance.{profile_name}',
        'tests': str(len(results)),
        'failures': str(sum(1 for r in results if r.status == FAIL)),
        'errors': str(sum(1 for r in results if r.status == ERROR)),
        'skipped': str(sum(1 for r in results if r.status in (SKIP, KNOWN_GAP))),
        'time': f'{sum(r.elapsed for r in results):.3f}',
    })
    for r in results:
        case = ET.SubElement(suite, 'testcase', {
            'classname': f'{profile_name}.{r.scenario.group}',
            'name': f'{r.scenario.key} {r.scenario.title}',
            'time': f'{r.elapsed:.3f}',
        })
        if r.status == FAIL:
            ET.SubElement(case, 'failure', {'message': r.detail[:400]}).text = r.detail
        elif r.status == ERROR:
            ET.SubElement(case, 'error', {'message': r.detail[:400]}).text = r.detail
        elif r.status == SKIP:
            ET.SubElement(case, 'skipped', {'message': r.detail[:400]})
        elif r.status == KNOWN_GAP:
            ET.SubElement(case, 'skipped', {
                'message': f'KNOWN_GAP: {r.scenario.why} / {r.detail}'[:400]})
        else:
            ET.SubElement(case, 'system-out').text = r.detail

    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    ET.ElementTree(suite).write(path, encoding='utf-8', xml_declaration=True)


def write_json(results, path, profile_name):
    payload = {
        'profile': profile_name,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'results': [
            {
                'key': r.scenario.key,
                'group': r.scenario.group,
                'title': r.scenario.title,
                'status': r.status,
                'detail': r.detail,
                'known_gap': r.scenario.known_gap,
                'why': r.scenario.why,
                'elapsed_sec': round(r.elapsed, 3),
            }
            for r in results
        ],
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def build_parser():
    p = argparse.ArgumentParser(
        prog='service_conformance',
        description='Unity_ROS2_Robot_Simulator の ROS2 サービス適合性テスト')
    p.add_argument('--profile', default='diffbot',
                   help='テスト用ロボットのプロファイル名か YAML パス (既定: diffbot)')
    p.add_argument('--only', nargs='*', metavar='KEY',
                   help='実行するシナリオ ID (例: D5 D10)')
    p.add_argument('--skip', nargs='*', metavar='KEY', default=[],
                   help='除外するシナリオ ID')
    p.add_argument('--list', action='store_true', help='シナリオ一覧を出して終了')
    p.add_argument('--junit', metavar='PATH', help='JUnit XML の出力先')
    p.add_argument('--json', metavar='PATH', dest='json_path',
                   help='JSON レポートの出力先')
    p.add_argument('--urdf-out-dir', metavar='DIR',
                   help='xacro を展開した URDF の置き場所 (既定: 一時ディレクトリ)')
    p.add_argument('--strict', action='store_true',
                   help='KNOWN_GAP も失敗として exit code に数える')
    p.add_argument('--no-color', action='store_true', help='色を付けない')
    p.add_argument('--startup-timeout', type=float, default=60.0,
                   help='サービスが現れるまでの待ち時間 [s] (既定: 60)')
    return p


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    # ros2 run 経由だと --ros-args 以降が付いてくるので落とす
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = build_parser().parse_args(argv)
    color = not args.no_color and sys.stdout.isatty()

    if args.list:
        group = None
        for sc in scenarios.SCENARIOS:
            if sc.group != group:
                group = sc.group
                print(f'\n{group}')
            tag = ' [known gap]' if sc.known_gap else ''
            print(f'  {sc.key:<4} {sc.title}{tag}')
        return 0

    profile = load_profile(args.profile)
    try:
        urdf_path = profile.resolve_urdf(args.urdf_out_dir)
    except Exception as exc:  # noqa: BLE001
        print(f'URDF を用意できない: {exc}', file=sys.stderr)
        return 2
    print(f'プロファイル: {profile.name} / ロボット: {profile.robot_name}')
    print(f'URDF: {urdf_path}')

    rclpy.init()
    harness = SimHarness(profile, service_timeout=profile.service_timeout)
    results = []
    try:
        missing = harness.wait_for_services(args.startup_timeout)
        if missing:
            print(f'\nシミュレータのサービスが見つからない: {", ".join(missing)}',
                  file=sys.stderr)
            print('ROS-TCP-Endpoint とシミュレータ本体が起動しているか確認すること。',
                  file=sys.stderr)
            return 2

        # discovery できても応答するとは限らない。ROS-TCP-Endpoint はサービスを
        # 登録したまま Unity 側が死ぬ/切断すると、名前だけ残って全呼び出しが
        # タイムアウトする。テストを始める前に 1 回叩いて疎通を確かめる。
        for attempt in range(3):
            try:
                harness.get_state(timeout=10.0)
                break
            except ServiceTimeout:
                if attempt == 2:
                    print('\nサービスは見えているが応答しない。'
                          'ROS-TCP-Endpoint は生きていて Unity 側が切断している状態と思われる。',
                          file=sys.stderr)
                    return 2
                time.sleep(2.0)

        def log(sc, status, detail):
            badge = colorize(f'{status:<9}', status, color)
            print(f'  {badge} {sc.key:<4} {sc.title}', flush=True)
            if detail:
                print(f'           {wrap(detail, 89, " " * 11)}', flush=True)

        ctx = scenarios.Context(harness, profile, urdf_path, log)
        print('\n--- 実行 ---')
        results = scenarios.run(ctx, only=args.only, skip_keys=args.skip)
    except ServiceTimeout as exc:
        print(f'\nシミュレータが応答しない: {exc}', file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('\n中断された', file=sys.stderr)
    finally:
        # 後片付け: 走らせっぱなしのロボットを止める
        try:
            harness.stop()
        except Exception:  # noqa: BLE001
            pass
        harness.shutdown()
        rclpy.try_shutdown()

    if not results:
        return 2

    counts = print_summary(results, color)
    if args.junit:
        write_junit(results, args.junit, profile.name)
        print(f'JUnit XML: {args.junit}')
    if args.json_path:
        write_json(results, args.json_path, profile.name)
        print(f'JSON: {args.json_path}')

    bad = counts[FAIL] + counts[ERROR]
    if args.strict:
        bad += counts[KNOWN_GAP]
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
