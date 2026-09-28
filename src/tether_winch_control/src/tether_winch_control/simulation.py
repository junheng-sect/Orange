"""Deterministic no-ROS/no-network manual-carry rehearsal."""
import argparse
import csv
from dataclasses import asdict, replace
import math

from .controller import Config, Controller
from .transport import make_transport


def run_scenario(config=None):
    # EXAMPLE ONLY: these numbers have no hardware calibration or force meaning.
    config = config or Config(low_duty=0.2, high_duty=0.4)
    controller = Controller(config)
    transport = make_transport('mock')
    rows = []
    step = 0
    radius, angle = 2.0, 0.0
    # phase, seconds, radial speed, angular speed, localization available
    phases = [
        ('warmup', 0.5, 0.0, 0.0, True),
        ('depart', 1.2, 0.3, 0.0, True),
        ('stop_after_depart', 0.6, 0.0, 0.0, True),
        ('approach', 1.2, -0.3, 0.0, True),
        ('stop_after_approach', 0.6, 0.0, 0.0, True),
        ('constant_radius', 1.2, 0.0, 0.25, True),
        ('reverse_in', 0.8, -0.25, 0.0, True),
        ('reverse_out', 0.8, 0.25, 0.0, True),
        ('approach_before_dropout', 0.8, -0.25, 0.0, True),
        ('dropout', 0.8, 0.0, 0.0, False),
        ('recover_without_enable', 0.6, 0.0, 0.0, True),
        ('explicit_reenable', 0.6, 0.0, 0.0, True),
    ]
    for phase, duration, radial_speed, angular_speed, available in phases:
        for index in range(round(duration / 0.02)):
            step += 1
            now = step * 0.02
            stamp = 1000.0 + now
            radius += radial_speed * 0.02
            angle += angular_speed * 0.02
            if available:
                controller.observe(stamp, (radius * math.cos(angle), radius * math.sin(angle), 0),
                                   (0, 0, 0, 1), 'map', 'base_link', now, stamp)
            controller.tick(now, stamp)
            event = ''
            if ((phase == 'warmup' and index == round(duration / 0.02) - 1)
                    or (phase == 'explicit_reenable' and index == 0)):
                success, reason = controller.enable(now, stamp)
                event = 'enable: %s (%s)' % (success, reason)
            status = controller.status()
            transport.update(status, now)
            rows.append(dict(time_sec=round(now, 3), phase=phase, event=event,
                             **asdict(status)))
    return rows


def main():
    parser = argparse.ArgumentParser(description='Offline MOCK ONLY winch simulation; no ROS or ESP32.')
    parser.add_argument('--low-duty', type=float, default=0.2, help='simulation example, not calibrated')
    parser.add_argument('--high-duty', type=float, default=0.4, help='simulation example, not calibrated')
    parser.add_argument('--window-sec', type=float, default=0.2)
    parser.add_argument('--enter-speed-mps', type=float, default=-0.05)
    parser.add_argument('--exit-speed-mps', type=float, default=-0.02)
    parser.add_argument('--enter-hold-sec', type=float, default=0.15)
    parser.add_argument('--csv', help='optional detailed trace output path')
    args = parser.parse_args()
    try:
        config = replace(Config(), low_duty=args.low_duty, high_duty=args.high_duty,
                         window_sec=args.window_sec, enter_speed_mps=args.enter_speed_mps,
                         exit_speed_mps=args.exit_speed_mps, enter_hold_sec=args.enter_hold_sec)
    except ValueError as error:
        parser.error(str(error))
    rows = run_scenario(config)
    print('MOCK ONLY: example duties are not calibrated. No network or motor output.')
    previous = None
    for row in rows:
        key = row['phase'], row['mode']
        if key != previous or row['event']:
            speed = row['radial_velocity_mps']
            print('%5.2fs %-25s %-8s vr=%7s duty=%s %s' % (
                row['time_sec'], row['phase'], row['mode'],
                '---' if speed is None else '%.3f' % speed,
                row['target_duty'], row['event'] or row['reason']))
        previous = key
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print('Trace: %s (%d samples)' % (args.csv, len(rows)))


if __name__ == '__main__':
    main()
