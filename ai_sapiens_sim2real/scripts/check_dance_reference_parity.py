#!/usr/bin/env python3
"""Compare the C++ probe with a recorded humanoid_motion reference sequence."""

import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('expected')
    parser.add_argument('actual')
    parser.add_argument('--report-only', action='store_true',
                        help='Measure CSV-derived contact differences without claiming parity')
    args = parser.parse_args()
    expected = np.loadtxt(args.expected, delimiter=',', ndmin=2)
    actual = np.loadtxt(args.actual, delimiter=',', ndmin=2)
    if expected.shape != actual.shape or expected.shape[1] != 153 or not len(expected):
        raise ValueError(f'Expected matching nonempty N x 153 arrays: '
                         f'{expected.shape}, {actual.shape}')
    if not np.isfinite(expected).all() or not np.isfinite(actual).all():
        raise ValueError('Non-finite reference output')
    error = np.abs(expected-actual)
    valid = np.ones_like(expected[:, :23], dtype=bool)
    for side in range(2):
        valid[:, side*6:side*6+6] = expected[:, 148+side, None] > .5
    # Float32 IK can oscillate at joint limits when the foot target is infeasible.
    # Report those errors separately; do not describe the whole actor as identical.
    # Successful IK must agree within 1 mrad; infeasible closest poses within 0.05 rad.
    checks = [
        ('valid joint position (rad)', error[:, :23][valid], 1e-3),
        ('invalid IK joint position (rad)', error[:, :23][~valid], .05),
        ('joint velocity (rad/s)', error[:, 23:46], 1e-3),
        ('anchor orientation', error[:, 46:52], 1e-5),
        ('sensor/XY/command observations', error[:, 52:131], 5e-6),
        ('root shift / foot pose', error[:, 131:148], 1e-5),
        ('IK validity', error[:, 148:150], 0),
        ('air fraction', error[:, 150:152], 1e-5),
        ('added step count', error[:, 152], 0),
    ]
    failed = False
    for name, errors, tolerance in checks:
        maximum = float(errors.max()) if errors.size else 0.
        passed = maximum <= tolerance
        failed |= not passed
        status = 'PASS' if passed else 'DIFF' if args.report_only else 'FAIL'
        mean = float(errors.mean()) if errors.size else 0.
        print(f'{status}: {name}: max={maximum:.8g}, mean={mean:.8g}, '
              f'tolerance={tolerance:g}')
    print(f'Compared {len(expected)} frames.')
    if args.report_only:
        print('Comparison only: CSV-derived contacts are not the original training labels.')
    if failed and not args.report_only:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
