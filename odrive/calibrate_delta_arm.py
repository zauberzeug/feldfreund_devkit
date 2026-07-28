#!/usr/bin/env python3
"""Encoder offset calibration for a delta arm whose motors cannot rotate freely.

Unlike ``calibrate_two_motors_{l,r}.py`` (which carry the *track* parameters and run a
full calibration sequence), this script assumes the board already holds a valid motor
calibration and hall polarity, and re-runs only the one step that needs rotation:
``AXIS_STATE_ENCODER_OFFSET_CALIBRATION``.

Why that is normally the only missing piece: a firmware flash wipes the config, and
``odrivetool``'s restore brings every calibration *result* back (phase_resistance,
phase_inductance, hall phase_offset, direction, hall_polarity) — but the firmware
refuses to accept ``encoder.config.pre_calibrated = True`` while the encoder is not
ready, so the board boots with ``UNKNOWN_PHASE_ESTIMATE``. One successful scan makes
the flag settable again.

The scan is open loop: it ignores the endstops and drives with
``motor.config.calibration_current``. Two guards compensate for that:

* ``--window`` bounds the travel at the output. ``shadow_count`` is polled at ~20 ms and
  the axis is dropped to IDLE the moment it leaves the window. At the scan speed
  (7.2 deg/s at the output with the defaults) that stops within a fraction of a degree.
* ``--calibration-current`` lowers the torque for the scan only and is restored after.

Nothing is persisted unless ``--commit`` is given.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import odrive
from odrive.enums import AXIS_STATE_ENCODER_OFFSET_CALIBRATION, AXIS_STATE_IDLE

# What the last good calibration of this rig produced; a fresh scan should reproduce it.
EXPECTED_PHASE_OFFSET = {'axis0': 48, 'axis1': 24}
EXPECTED_DIRECTION = {'axis0': 1, 'axis1': 1}

POLL_INTERVAL = 0.02
SETTLE = 0.3


def parse_window(text: str) -> tuple[float, float]:
    """Parse ``--window`` (``low,high`` in output degrees, relative to the start position)."""
    try:
        low, high = (float(part) for part in text.replace(' ', '').split(','))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f'expected "low,high" in output degrees, got {text!r}') from error
    if low > 0 or high < 0:
        raise argparse.ArgumentTypeError(f'the window must contain 0 (the start position), got {low}..{high}')
    return low, high


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--axis', choices=['0', '1', 'both'], default='both',
                        help='which axis to calibrate (default: both, one after the other)')
    parser.add_argument('--scan-distance', type=float, default=4 * math.pi,
                        help='calib_scan_distance in electrical rad; must be a multiple of 2*pi '
                             '(default: %(default).3f = 2 electrical revolutions)')
    parser.add_argument('--window', type=parse_window, default='-10,+10',
                        help='allowed travel at the output in degrees as "low,high", relative to the '
                             'start position; leaving it aborts the scan. The scan runs its full '
                             'distance in one direction, so at least one side must exceed that travel. '
                             'Pass as --window=-2,+10 (the leading minus needs the "="). '
                             '(default: %(default)s)')
    parser.add_argument('--calibration-current', type=float, default=None,
                        help='motor.config.calibration_current for the scan only, restored afterwards '
                             '(default: leave the board value untouched)')
    parser.add_argument('--gear-ratio', type=float, default=12.5,
                        help='motor revolutions per output revolution (default: %(default)s)')
    parser.add_argument('--timeout', type=float, default=40.0,
                        help='seconds to wait for a scan to finish (default: %(default)s)')
    parser.add_argument('--dry-run', action='store_true',
                        help='report the plan and the board state; move nothing, write nothing')
    parser.add_argument('--commit', action='store_true',
                        help='after a successful scan set encoder.config.pre_calibrated and save '
                             '(the board reboots and, with startup_closed_loop_control, arms the arm)')
    return parser.parse_args()


def clear_errors(odrv, axis) -> None:
    try:
        odrv.clear_errors()
        return
    except AttributeError:
        pass
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0


def describe(axis, name: str, deg_per_count: float) -> None:
    print(f'  {name}: state={axis.current_state} axis_err={axis.error:#x} '
          f'motor_err={axis.motor.error:#x} enc_err={axis.encoder.error:#x}')
    print(f'        motor.is_calibrated={axis.motor.is_calibrated} encoder.is_ready={axis.encoder.is_ready} '
          f'pre_calibrated: motor={axis.motor.config.pre_calibrated} encoder={axis.encoder.config.pre_calibrated}')
    print(f'        phase_offset={axis.encoder.config.phase_offset} '
          f'(expected {EXPECTED_PHASE_OFFSET.get(name, "?")}) '
          f'direction={axis.encoder.config.direction} hall_polarity={axis.encoder.config.hall_polarity} '
          f'shadow_count={axis.encoder.shadow_count} ({deg_per_count:.3f} deg/count at the output)')


def run_scan(axis, name: str, args: argparse.Namespace, deg_per_count: float) -> dict:
    """Run one encoder offset calibration under a travel guard.

    :param axis: The ``axis0``/``axis1`` object to calibrate.
    :param name: Its name, for reporting.
    :param args: Parsed command line arguments.
    :param deg_per_count: Output degrees per encoder count, for the guard and the report.
    :return: A result dict with the travel envelope and the resulting calibration values.
    """
    low_deg, high_deg = args.window
    start = axis.encoder.shadow_count
    saved_scan = axis.encoder.config.calib_scan_distance
    saved_current = axis.motor.config.calibration_current

    axis.encoder.config.calib_scan_distance = args.scan_distance
    if args.calibration_current is not None:
        axis.motor.config.calibration_current = args.calibration_current
    print(f'  scan_distance={args.scan_distance:.4f} el rad '
          f'(~{args.scan_distance / axis.motor.config.pole_pairs / (2 * math.pi) * 360 / args.gear_ratio:.2f} deg '
          f'travel at the output), calibration_current={axis.motor.config.calibration_current:.1f} A, '
          f'window {low_deg:+.1f}..{high_deg:+.1f} deg')

    # The scan rebases shadow_count onto count_in_cpr when it starts (a jump by a multiple of cpr),
    # so an absolute comparison against the start position is meaningless. Real motion is bounded by
    # the scan speed, so accumulate only plausible per-poll deltas and treat the rest as bookkeeping.
    counts_per_second = (axis.encoder.config.calib_scan_omega / axis.motor.config.pole_pairs
                         / (2 * math.pi) * axis.encoder.config.cpr)
    track: list[tuple[float, float]] = []
    aborted_at: float | None = None
    travel_deg = 0.0
    previous = start
    previous_time = began = time.monotonic()
    rebases = 0
    try:
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(0.05)
        while True:
            now = time.monotonic()
            elapsed = now - began
            count = axis.encoder.shadow_count
            delta = count - previous
            # 3x the theoretical maximum plus 2 counts of slack absorbs USB and scheduling jitter.
            plausible = counts_per_second * (now - previous_time) * 3 + 2
            if abs(delta) > plausible:
                rebases += 1
                print(f'  .. encoder count rebased by {delta:+d} at t={elapsed:.2f}s '
                      f'(> {plausible:.1f} plausible) — not counted as travel')
            else:
                travel_deg += delta * deg_per_count
            previous, previous_time = count, now
            track.append((elapsed, travel_deg))
            # Completion first: the scan ends at its full one-way travel, so checking the guard
            # first would report a finished calibration as an abort.
            if elapsed > 0.2 and axis.current_state == AXIS_STATE_IDLE:
                break
            if travel_deg < low_deg or travel_deg > high_deg:
                axis.requested_state = AXIS_STATE_IDLE
                aborted_at = travel_deg
                print(f'  !! travel guard tripped at {travel_deg:+.2f} deg — dropped to IDLE')
                break
            if elapsed > args.timeout:
                axis.requested_state = AXIS_STATE_IDLE
                print(f'  !! timeout after {elapsed:.1f} s — dropped to IDLE')
                break
            time.sleep(POLL_INTERVAL)
    finally:
        time.sleep(SETTLE)
        axis.requested_state = AXIS_STATE_IDLE
        axis.encoder.config.calib_scan_distance = saved_scan
        axis.motor.config.calibration_current = saved_current

    excursions = [degrees for _, degrees in track]
    result = {
        'aborted_at': aborted_at,
        'seconds': time.monotonic() - began,
        'min_deg': min(excursions, default=0.0),
        'max_deg': max(excursions, default=0.0),
        'net_deg': excursions[-1] if excursions else 0.0,
        'samples': len(track),
        'rebases': rebases,
        'axis_error': axis.error,
        'motor_error': axis.motor.error,
        'encoder_error': axis.encoder.error,
        'is_ready': axis.encoder.is_ready,
        'phase_offset': axis.encoder.config.phase_offset,
        'phase_offset_float': axis.encoder.config.phase_offset_float,
        'direction': axis.encoder.config.direction,
    }
    print(f'  travel: min {result["min_deg"]:+.2f} deg, max {result["max_deg"]:+.2f} deg, '
          f'net {result["net_deg"]:+.2f} deg over {result["seconds"]:.1f} s ({result["samples"]} samples)')
    print(f'  result: is_ready={result["is_ready"]} phase_offset={result["phase_offset"]} '
          f'direction={result["direction"]} '
          f'axis_err={result["axis_error"]:#x} motor_err={result["motor_error"]:#x} '
          f'enc_err={result["encoder_error"]:#x}')

    expected = EXPECTED_PHASE_OFFSET.get(name)
    if expected is not None:
        # phase_offset only means anything modulo one *electrical* revolution: cpr/pole_pairs counts
        # (6 here). Comparing modulo cpr would flag electrically identical results as a mismatch.
        period = axis.encoder.config.cpr // axis.motor.config.pole_pairs
        if result['phase_offset'] % period == expected % period:
            print(f'  ✓ phase_offset {result["phase_offset"]} is electrically identical to the stored '
                  f'{expected} (both {expected % period} mod {period} counts per electrical rev)')
        else:
            print(f'  ⚠ phase_offset {result["phase_offset"]} differs from the stored {expected} by '
                  f'{(result["phase_offset"] - expected) % period} of {period} counts per electrical rev '
                  f'— the stored value was NOT reproduced, investigate before trusting it')
    if result['direction'] != EXPECTED_DIRECTION.get(name, result['direction']):
        print(f'  ⚠ direction flipped to {result["direction"]} — the bench `reversed` flags '
              f'for this axis are now inverted, check them before moving the arm')
    return result


def main() -> int:
    args = parse_args()
    names = ['axis0', 'axis1'] if args.axis == 'both' else [f'axis{args.axis}']

    print('connecting...')
    odrv = odrive.find_any(timeout=15)
    print(f'connected: serial {odrv.serial_number:x}, hw {odrv.hw_version_major}.{odrv.hw_version_minor}, '
          f'vbus {odrv.vbus_voltage:.2f} V')

    cpr = odrv.axis0.encoder.config.cpr
    deg_per_count = 360.0 / (cpr * args.gear_ratio)

    print('\nboard state before:')
    for name in ('axis0', 'axis1'):
        describe(getattr(odrv, name), name, deg_per_count)

    if args.scan_distance % (2 * math.pi) > 1e-3:
        print(f'\nrefusing: --scan-distance {args.scan_distance} is not a multiple of 2*pi '
              f'({2 * math.pi:.4f}); the offset needs whole electrical revolutions')
        return 1

    # The scan travels its full distance in ONE direction (measured on ts2), so the window must have
    # room for all of it on at least one side, plus the initial lock-in snap and a count of slack.
    pole_pairs = odrv.axis0.motor.config.pole_pairs
    expected_deg = args.scan_distance / pole_pairs / (2 * math.pi) * 360 / args.gear_ratio
    needed_deg = expected_deg + 1.5
    low_deg, high_deg = args.window
    print(f'\nexpected travel: {expected_deg:.2f} deg at the output, one direction '
          f'(sign not guaranteed) — the window needs {needed_deg:.2f} deg of room')
    if max(-low_deg, high_deg) < needed_deg:
        print(f'refusing: window {low_deg:+.1f}..{high_deg:+.1f} deg is smaller than the scan travel; '
              f'the guard would abort a healthy calibration. Widen it to at least '
              f'{needed_deg:.1f} deg on the side the arm is free to move, or reduce --scan-distance.')
        return 1
    if min(-low_deg, high_deg) < needed_deg:
        print(f'note: only the {"negative" if -low_deg >= needed_deg else "positive"} side has room; '
              f'if the scan runs the other way the guard stops it early and no harm is done.')

    for name in ('axis0', 'axis1'):
        axis = getattr(odrv, name)
        if not axis.motor.config.pre_calibrated or not axis.motor.is_calibrated:
            print(f'\nrefusing: {name} has no valid motor calibration — this script only re-runs the '
                  f'encoder offset scan. Run a motor calibration (AXIS_STATE_MOTOR_CALIBRATION) first.')
            return 1

    # Bail out before any write, so --dry-run is read-only by construction rather than by luck.
    if args.dry_run:
        busy = [name for name in names if getattr(odrv, name).current_state != AXIS_STATE_IDLE]
        print('\n--dry-run: nothing moved, nothing written.')
        if busy:
            print(f'note: {busy} not in IDLE — a real run would first request IDLE and refuse if it '
                  f'does not stick (something commanding over CAN).')
        print(f'plan: scan {names} at scan_distance {args.scan_distance:.4f} el rad '
              f'(~{args.scan_distance / 8 / (2 * math.pi) * 360 / args.gear_ratio:.2f} deg at the output), '
              f'window {args.window[0]:+.1f}..{args.window[1]:+.1f} deg, '
              f'calibration_current {args.calibration_current or "unchanged"}')
        return 0

    # A live Lizard/CAN master would fight the calibration; make sure both axes stay put.
    for name in names:
        axis = getattr(odrv, name)
        if axis.current_state != AXIS_STATE_IDLE:
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(SETTLE)
    time.sleep(0.5)
    for name in names:
        axis = getattr(odrv, name)
        if axis.current_state != AXIS_STATE_IDLE:
            print(f'\nrefusing: {name} will not stay in IDLE (state {axis.current_state}) — something is '
                  f'commanding it over CAN. Stop the bench app or the ESP first.')
            return 1

    results = {}
    for name in names:
        print(f'\n=== {name}: encoder offset calibration ===')
        clear_errors(odrv, getattr(odrv, name))
        results[name] = run_scan(getattr(odrv, name), name, args, deg_per_count)

    good = [name for name, r in results.items()
            if r['is_ready'] and not r['axis_error'] and not r['motor_error'] and not r['encoder_error']
            and r['aborted_at'] is None]
    print(f'\nsucceeded: {good or "none"}')

    if not args.commit:
        print('not committing (no --commit): encoder.config.pre_calibrated is unchanged and the '
              'calibration is lost on the next reboot.')
        return 0 if len(good) == len(names) else 2
    if len(good) != len(names):
        print('refusing to commit: not every requested axis calibrated cleanly.')
        return 2

    for name in names:
        axis = getattr(odrv, name)
        axis.encoder.config.pre_calibrated = True
        held = axis.encoder.config.pre_calibrated
        print(f'{name}: encoder.config.pre_calibrated -> {held}')
        if not held:
            print(f'refusing to save: {name} rejected the flag, the encoder is not ready after all.')
            return 2

    print('saving configuration (the board reboots; with startup_closed_loop_control the arm goes live)...')
    try:
        odrv.save_configuration()
    except Exception as error:  # the reboot drops the USB connection
        print(f'  (expected disconnect during reboot: {type(error).__name__})')
    time.sleep(3.0)
    odrv = odrive.find_any(timeout=20)
    print('reconnected after reboot:')
    ok = True
    for name in ('axis0', 'axis1'):
        describe(getattr(odrv, name), name, deg_per_count)
        axis = getattr(odrv, name)
        if name in names:
            ok &= bool(axis.encoder.config.pre_calibrated and axis.encoder.is_ready)
    print('\nRESULT: calibration persisted, encoder ready' if ok else
          '\nRESULT: the flag did not survive the reboot')
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
