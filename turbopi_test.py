from __future__ import annotations

import argparse
import sys
import time

sys.path.append('/home/pi/TurboPi/')

try:
    from HiwonderSDK.Sonar import Sonar
    from HiwonderSDK.mecanum import MecanumChassis
except Exception as exc:
    raise RuntimeError("Run this file on the TurboPi with the SDK installed.") from exc

DEFAULT_CELL_MM = 540
DEFAULT_CELL_TIME_SEC = 0.9
DEFAULT_SPEED_MM_S = DEFAULT_CELL_MM / DEFAULT_CELL_TIME_SEC
DEFAULT_TURN_RATE = 1.2
DEFAULT_TURN_90_TIME_SEC = 0.20


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive TurboPi test tool with sonar readouts.",
    )
    parser.add_argument(
        "--speed-mm-s",
        type=float,
        default=DEFAULT_SPEED_MM_S,
        help="Estimated forward speed in mm/s (default: %(default).1f)",
    )
    parser.add_argument(
        "--turn-rate",
        type=float,
        default=DEFAULT_TURN_RATE,
        help="Chassis turn rate (default: %(default).2f)",
    )
    parser.add_argument(
        "--turn-90-time-sec",
        type=float,
        default=DEFAULT_TURN_90_TIME_SEC,
        help="Seconds for a 90-degree turn (default: %(default).2f)",
    )
    return parser.parse_args()


def move_distance_mm(chassis: MecanumChassis, distance_mm: float, speed_mm_s: float) -> None:
    if speed_mm_s <= 0:
        raise ValueError("speed_mm_s must be positive")
    if distance_mm == 0:
        return

    direction_deg = 90 if distance_mm > 0 else 270
    duration = abs(distance_mm) / speed_mm_s
    chassis.set_velocity(speed_mm_s, direction_deg, 0)
    time.sleep(duration)
    chassis.reset_motors()


def turn_left_90(chassis: MecanumChassis, turn_rate: float, duration: float) -> None:
    chassis.set_velocity(0, 0, -turn_rate)
    time.sleep(duration)
    chassis.reset_motors()


def turn_right_90(chassis: MecanumChassis, turn_rate: float, duration: float) -> None:
    chassis.set_velocity(0, 0, turn_rate)
    time.sleep(duration)
    chassis.reset_motors()


def print_help() -> None:
    print(
        "Commands:\n"
        "  move <mm>        Move forward/backward by distance in mm.\n"
        "  left             Turn left 90 degrees.\n"
        "  right            Turn right 90 degrees.\n"
        "  stop             Stop motors.\n"
        "  quit             Exit the program.\n"
        "  help             Show this help.\n"
    )


def main() -> None:
    args = parse_args()
    sonar = Sonar()
    chassis = MecanumChassis()

    print("TurboPi test console. Type 'help' for commands.")
    print_help()

    while True:
        dist_mm = sonar.getDistance()
        print(f"Sonar: {dist_mm} mm")
        try:
            raw = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in {"quit", "exit"}:
            break
        if cmd == "help":
            print_help()
            continue
        if cmd == "stop":
            chassis.reset_motors()
            continue
        if cmd == "left":
            turn_left_90(chassis, args.turn_rate, args.turn_90_time_sec)
            continue
        if cmd == "right":
            turn_right_90(chassis, args.turn_rate, args.turn_90_time_sec)
            continue
        if cmd == "move":
            if len(parts) != 2:
                print("Usage: move <mm>")
                continue
            try:
                distance_mm = float(parts[1])
            except ValueError:
                print("Distance must be a number in millimeters.")
                continue
            move_distance_mm(chassis, distance_mm, args.speed_mm_s)
            continue

        print("Unknown command. Type 'help' for commands.")

    chassis.reset_motors()


if __name__ == "__main__":
    main()
