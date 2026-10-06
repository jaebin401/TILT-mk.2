#!/usr/bin/env python3

"""Safe host-side setup utility for Feetech STS3215 servos."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any


DEFAULT_BAUD_RATE = 1_000_000
DEFAULT_SERVO_ID = 1
CENTER_POSITION = 2047
CENTER_SPEED = 200
CENTER_ACCELERATION = 20
CENTER_TOLERANCE = 10
CENTER_TIMEOUT_SECONDS = 20.0

TORQUE_ENABLE_ADDRESS = 40
SERVO_ID_ADDRESS = 5
MIN_SERVO_ID = 0
MAX_SERVO_ID = 253
POSITION_STEPS_PER_REVOLUTION = 4096


class ServoToolError(RuntimeError):
    """Raised when the servo setup operation cannot continue safely."""


def servo_id(value: str) -> int:
    parsed = int(value)
    if not MIN_SERVO_ID <= parsed <= MAX_SERVO_ID:
        raise argparse.ArgumentTypeError(
            f"servo ID must be between {MIN_SERVO_ID} and {MAX_SERVO_ID}"
        )
    return parsed


def offset_degrees(position: int) -> float:
    return (position - CENTER_POSITION) * 360.0 / POSITION_STEPS_PER_REVOLUTION


def load_sdk() -> tuple[Any, Any, int]:
    try:
        from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
    except ModuleNotFoundError as error:
        raise ServoToolError(
            "Missing servo SDK. Activate the virtual environment and run "
            "`python -m pip install -r requirements.txt`."
        ) from error

    return PortHandler, sms_sts, COMM_SUCCESS


@dataclass
class ServoBus:
    port_name: str
    baud_rate: int = DEFAULT_BAUD_RATE

    def __post_init__(self) -> None:
        port_handler, servo_handler, comm_success = load_sdk()
        self._comm_success = comm_success
        self.port = port_handler(self.port_name)
        self.servo = servo_handler(self.port)
        self._opened = False

    def __enter__(self) -> "ServoBus":
        if not self.port.openPort():
            raise ServoToolError(f"Failed to open serial port: {self.port_name}")

        self._opened = True
        if not self.port.setBaudRate(self.baud_rate):
            self.close()
            raise ServoToolError(
                f"Failed to configure {self.port_name} at {self.baud_rate} baud"
            )

        print(f"Connected to {self.port_name} at {self.baud_rate} baud")
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        self.close()

    def close(self) -> None:
        if self._opened:
            self.port.closePort()
            self._opened = False

    def _check(self, operation: str, result: int, error: int) -> None:
        if result != self._comm_success:
            detail = self.servo.getTxRxResult(result)
            raise ServoToolError(f"{operation} failed: {detail}")

        if error != 0:
            detail = self.servo.getRxPacketError(error)
            raise ServoToolError(f"{operation} failed: {detail}")

    def ping(self, target_id: int) -> int:
        model_number, result, error = self.servo.ping(target_id)
        self._check(f"Ping servo {target_id}", result, error)
        return model_number

    def try_ping(self, target_id: int) -> int | None:
        """Return the model number when an ID responds, otherwise return None."""
        model_number, result, error = self.servo.ping(target_id)
        if result != self._comm_success or error != 0:
            return None
        return model_number

    def read_position(self, target_id: int) -> int:
        position, result, error = self.servo.ReadPos(target_id)
        self._check(f"Read position from servo {target_id}", result, error)
        return position

    def set_torque(self, target_id: int, enabled: bool) -> None:
        result, error = self.servo.write1ByteTxRx(
            target_id,
            TORQUE_ENABLE_ADDRESS,
            1 if enabled else 0,
        )
        state = "enable" if enabled else "disable"
        self._check(f"{state.capitalize()} torque on servo {target_id}", result, error)

    def unlock_eeprom(self, target_id: int) -> None:
        result, error = self.servo.unLockEprom(target_id)
        self._check(f"Unlock EEPROM on servo {target_id}", result, error)

    def lock_eeprom(self, target_id: int) -> None:
        result, error = self.servo.LockEprom(target_id)
        self._check(f"Lock EEPROM on servo {target_id}", result, error)

    def write_id(self, current_id: int, new_id: int) -> None:
        result, error = self.servo.write1ByteTxRx(
            current_id,
            SERVO_ID_ADDRESS,
            new_id,
        )
        if result != self._comm_success:
            time.sleep(0.1)
            if self.try_ping(new_id) is not None:
                return
        self._check(f"Change servo ID {current_id} to {new_id}", result, error)

    def try_lock_eeprom(self, target_id: int) -> bool:
        result, error = self.servo.LockEprom(target_id)
        return result == self._comm_success and error == 0

    def write_center(self, target_id: int) -> None:
        result, error = self.servo.WritePosEx(
            target_id,
            CENTER_POSITION,
            CENTER_SPEED,
            CENTER_ACCELERATION,
        )
        self._check(f"Move servo {target_id} to center", result, error)

    def release_after_failure(self, target_id: int) -> None:
        try:
            self.set_torque(target_id, False)
            print("Torque released after an interrupted or failed move.")
        except ServoToolError as error:
            print(f"WARNING: Could not release torque automatically: {error}", file=sys.stderr)


def list_serial_ports() -> int:
    try:
        from serial.tools import list_ports
    except ModuleNotFoundError as error:
        raise ServoToolError(
            "Missing pyserial. Activate the virtual environment and run "
            "`python -m pip install -r requirements.txt`."
        ) from error

    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        print(f"{port.device}\n  {port.description}\n  {port.hwid}")

    return 0


def check_id(bus: ServoBus) -> list[tuple[int, int]]:
    """Ping every valid servo ID and return (ID, model number) pairs."""
    found: list[tuple[int, int]] = []

    print(
        f"Scanning servo IDs {MIN_SERVO_ID} through {MAX_SERVO_ID} "
        f"at {bus.baud_rate} baud."
    )
    print("This may take about 15 seconds when no servo responds.")

    for target_id in range(MIN_SERVO_ID, MAX_SERVO_ID + 1):
        if target_id % 32 == 0:
            print(f"Checking ID {target_id}...", flush=True)

        model_number = bus.try_ping(target_id)
        if model_number is not None:
            found.append((target_id, model_number))
            print(f"Found servo ID {target_id}; model number: {model_number}")

    return found


def confirm_id_change(current_id: int, new_id: int, model_number: int) -> bool:
    print()
    print(f"Current servo ID: {current_id}")
    print(f"New servo ID:     {new_id}")
    print(f"Model number:     {model_number}")
    print()
    print("Keep exactly one servo connected while changing its ID.")
    expected = f"SET {current_id} {new_id}"
    response = input(f"Type {expected} to continue: ").strip()
    return response == expected


def change_servo_id(bus: ServoBus, current_id: int, new_id: int) -> int:
    if current_id == new_id:
        raise ServoToolError("Current ID and new ID must be different.")

    model_number = bus.ping(current_id)
    if bus.try_ping(new_id) is not None:
        raise ServoToolError(
            f"Servo ID {new_id} already responds. Choose an unused ID and keep "
            "only one servo connected."
        )

    if not confirm_id_change(current_id, new_id, model_number):
        print("ID change cancelled; no EEPROM setting was changed.")
        return 2

    eeprom_unlocked = False
    try:
        bus.set_torque(current_id, False)
        bus.unlock_eeprom(current_id)
        eeprom_unlocked = True
        bus.write_id(current_id, new_id)
        time.sleep(0.1)
        bus.lock_eeprom(new_id)
        eeprom_unlocked = False
    except (ServoToolError, KeyboardInterrupt):
        if eeprom_unlocked:
            locked = bus.try_lock_eeprom(new_id) or bus.try_lock_eeprom(current_id)
            if not locked:
                print(
                    "WARNING: Could not confirm that EEPROM was re-locked. "
                    "Turn the servo power off before retrying.",
                    file=sys.stderr,
                )
        raise

    verified_model = bus.ping(new_id)
    if bus.try_ping(current_id) is not None:
        raise ServoToolError(
            f"Both ID {current_id} and ID {new_id} responded after the change. "
            "Disconnect power and verify that only one servo is connected."
        )

    print(
        f"Servo ID changed successfully: {current_id} -> {new_id} "
        f"(model number: {verified_model})."
    )
    print(f"Label this servo and its cable with ID {new_id} before disconnecting it.")
    return 0


def confirm_center_move(target_id: int, current_position: int) -> bool:
    delta = CENTER_POSITION - current_position
    delta_degrees = delta * 360.0 / POSITION_STEPS_PER_REVOLUTION

    print()
    print(f"Servo ID:        {target_id}")
    print(f"Current position: {current_position} ({offset_degrees(current_position):+.1f}°)")
    print(f"Target position:  {CENTER_POSITION} (+0.0°)")
    print(f"Expected move:    {delta:+d} steps ({delta_degrees:+.1f}°)")
    print()
    print("Remove the horn and linkage before continuing.")
    print("Keep access to the external servo power switch.")

    response = input("Type CENTER to move the servo: ").strip()
    return response == "CENTER"


def wait_for_center(
    bus: ServoBus,
    target_id: int,
    timeout_seconds: float,
    tolerance: int,
) -> int:
    deadline = time.monotonic() + timeout_seconds
    last_position: int | None = None

    while time.monotonic() < deadline:
        last_position = bus.read_position(target_id)
        if abs(last_position - CENTER_POSITION) <= tolerance:
            return last_position
        time.sleep(0.1)

    last_text = "unknown" if last_position is None else str(last_position)
    raise ServoToolError(
        f"Servo did not reach {CENTER_POSITION} ± {tolerance} within "
        f"{timeout_seconds:.1f} seconds; last position: {last_text}"
    )


def add_port_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port",
        required=True,
        help="serial port such as /dev/cu.wchusbserialXXXX",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"servo baud rate (default: {DEFAULT_BAUD_RATE})",
    )


def add_connection_arguments(parser: argparse.ArgumentParser) -> None:
    add_port_arguments(parser)
    parser.add_argument(
        "--id",
        type=servo_id,
        default=DEFAULT_SERVO_ID,
        help=f"servo ID (default: {DEFAULT_SERVO_ID})",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and mechanically center an STS3215 servo safely."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list-ports", help="list serial ports visible to macOS")

    ping_parser = subparsers.add_parser("ping", help="verify communication with one servo")
    add_connection_arguments(ping_parser)

    check_id_parser = subparsers.add_parser(
        "check-id",
        help="scan every servo ID from 0 through 253",
    )
    add_port_arguments(check_id_parser)

    set_id_parser = subparsers.add_parser(
        "set-id",
        help="change the ID of exactly one connected servo",
    )
    add_port_arguments(set_id_parser)
    set_id_parser.add_argument(
        "--current-id",
        type=servo_id,
        required=True,
        help="current servo ID",
    )
    set_id_parser.add_argument(
        "--new-id",
        type=servo_id,
        required=True,
        help="new unused servo ID",
    )

    read_parser = subparsers.add_parser("read", help="read the current servo position")
    add_connection_arguments(read_parser)

    center_parser = subparsers.add_parser("center", help="move one unloaded servo to position 2047")
    add_connection_arguments(center_parser)
    center_parser.add_argument(
        "--timeout",
        type=float,
        default=CENTER_TIMEOUT_SECONDS,
        help=f"move timeout in seconds (default: {CENTER_TIMEOUT_SECONDS:g})",
    )
    center_parser.add_argument(
        "--tolerance",
        type=int,
        default=CENTER_TOLERANCE,
        help=f"accepted center error in steps (default: {CENTER_TOLERANCE})",
    )

    release_parser = subparsers.add_parser("release", help="disable servo torque")
    add_connection_arguments(release_parser)

    return parser


def run_bus_command(args: argparse.Namespace) -> int:
    with ServoBus(args.port, args.baud) as bus:
        if args.command == "check-id":
            found = check_id(bus)
            if not found:
                print("Scan complete; no servo IDs responded.")
                return 0

            id_list = ", ".join(str(target_id) for target_id, _ in found)
            print(f"Scan complete; found {len(found)} servo(s): {id_list}")
            return 0

        if args.command == "set-id":
            return change_servo_id(bus, args.current_id, args.new_id)

        if args.command == "ping":
            model_number = bus.ping(args.id)
            print(f"Servo {args.id} responded; model number: {model_number}")
            return 0

        if args.command == "read":
            model_number = bus.ping(args.id)
            position = bus.read_position(args.id)
            print(f"Servo {args.id} model: {model_number}")
            print(f"Position: {position} ({offset_degrees(position):+.1f}° from center)")
            return 0

        if args.command == "release":
            bus.ping(args.id)
            bus.set_torque(args.id, False)
            print(f"Torque released on servo {args.id}.")
            return 0

        if args.command == "center":
            model_number = bus.ping(args.id)
            current_position = bus.read_position(args.id)
            print(f"Servo {args.id} model: {model_number}")

            if not confirm_center_move(args.id, current_position):
                print("Center move cancelled; no motion command was sent.")
                return 2

            try:
                bus.set_torque(args.id, True)
                bus.write_center(args.id)
                final_position = wait_for_center(
                    bus,
                    args.id,
                    args.timeout,
                    args.tolerance,
                )
            except (ServoToolError, KeyboardInterrupt):
                bus.release_after_failure(args.id)
                raise

            print(f"Servo reached center at position {final_position}.")
            print("Turn off the external servo power before attaching the horn.")
            return 0

    raise ServoToolError(f"Unsupported command: {args.command}")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.command == "list-ports":
            return list_serial_ports()
        return run_bus_command(args)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130
    except (ServoToolError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
