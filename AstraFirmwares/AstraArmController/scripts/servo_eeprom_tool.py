#!/usr/bin/env python3
"""Read and write SMS/STS servo EEPROM through the Astra controller.
# 对比12、13、14号内部参数
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right dump 12 13 14

# 读取12号P参数，地址21，1字节
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right read 12 21

# 写入12号P参数并读回验证
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right write 12 21 28

# 读取当前ID
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right get-id 12

# 将ID 1改为12
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right set-id 1 12

# 直接将12号舵机设为PWM模式(2)，并由控制器回读验证
python3 scripts/servo_eeprom_tool.py /dev/tty_puppet_right set-mode 12 2"""

import argparse
import serial
import struct
import time


BAUDRATE = 921600
HEADER = 0x5A
COMM_TYPE_TORQUE = 4
COMM_TYPE_DEBUG = 9

SUBCMD_DEBUG_MODE = 0
SUBCMD_EEPROM_READ = 5
SUBCMD_EEPROM_WRITE = 6

SMS_STS_MODE = 33

STATUS_TEXT = {
    0: "ok",
    1: "debug mode is required",
    2: "invalid servo ID",
    3: "invalid EEPROM address or width",
    4: "servo-bus read/readback failed",
    5: "value does not fit register width",
    6: "invalid servo ID-register write",
    7: "write verification mismatch",
}

# ST-series EEPROM fields used for diagnosis. Values are deliberately kept
# raw: model-specific interpretation can be added after the model is known.
REGISTERS = (
    ("model", 3, 2),
    ("id", 5, 1),
    ("baud_rate", 6, 1),
    ("response_level", 8, 1),
    ("min_angle_limit", 9, 2),
    ("max_angle_limit", 11, 2),
    ("max_temperature", 13, 1),
    ("max_voltage", 14, 1),
    ("min_voltage", 15, 1),
    ("max_torque", 16, 2),
    ("phase", 18, 1),
    ("protection_switch", 19, 1),
    ("led_alarm", 20, 1),
    ("position_p", 21, 1),
    ("position_d", 22, 1),
    ("position_i", 23, 1),
    ("minimum_startup_force", 24, 2),
    ("cw_dead_zone", 26, 1),
    ("ccw_dead_zone", 27, 1),
    ("protection_current", 28, 2),
    ("angular_resolution", 30, 1),
    ("position_offset", 31, 2),
    ("mode", SMS_STS_MODE, 1),
    ("protection_torque", 34, 1),
    ("protection_time", 35, 1),
    ("overload_torque", 36, 1),
)


class ProtocolError(RuntimeError):
    pass


class ServoEepromTool:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        # Opening the controller port may reset the ESP32.
        time.sleep(2.0)

    def close(self):
        self.ser.close()

    def _send(self, packet_type, payload, payload_size):
        payload = payload.ljust(payload_size, b"\x00")[:payload_size]
        checksum = (packet_type + sum(payload)) & 0xFF
        self.ser.write(bytes((HEADER, packet_type)) + payload + bytes((checksum,)))

    def _recv_debug(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        sizes = {0: 16, 1: 16, 2: 16, 3: 16, 4: 16,
                 5: 16, 6: 16, 7: 16, 8: 32, 9: 32}
        while time.monotonic() < deadline:
            first = self.ser.read(1)
            if not first or first[0] != HEADER:
                continue
            type_bytes = self.ser.read(1)
            if not type_bytes:
                continue
            packet_type = type_bytes[0]
            size = sizes.get(packet_type)
            if size is None:
                continue
            payload = self.ser.read(size)
            checksum = self.ser.read(1)
            if len(payload) != size or len(checksum) != 1:
                continue
            if checksum[0] != ((packet_type + sum(payload)) & 0xFF):
                continue
            if packet_type == COMM_TYPE_DEBUG:
                return struct.unpack(">IIIIIIII", payload)
        raise ProtocolError("timed out waiting for controller response")

    def enter_safe_debug_mode(self):
        # Stop all outputs before suspending the periodic bus-control loop.
        self._send(COMM_TYPE_TORQUE, b"\x00", 16)
        time.sleep(0.2)
        payload = struct.pack(">IIIIIIII", SUBCMD_DEBUG_MODE, 1, 0, 0, 0, 0, 0, 0)
        self._send(COMM_TYPE_DEBUG, payload, 32)
        time.sleep(0.1)

    def leave_debug_mode(self):
        payload = struct.pack(">IIIIIIII", SUBCMD_DEBUG_MODE, 0, 0, 0, 0, 0, 0, 0)
        self._send(COMM_TYPE_DEBUG, payload, 32)

    def read(self, servo_id, address, width):
        payload = struct.pack(">IIIIIIII", SUBCMD_EEPROM_READ, servo_id,
                              address, width, 0, 0, 0, 0)
        self._send(COMM_TYPE_DEBUG, payload, 32)
        response = self._recv_debug()
        if response[:4] != (SUBCMD_EEPROM_READ, servo_id, address, width):
            raise ProtocolError(f"unexpected read response: {response}")
        status = response[5]
        if status:
            raise ProtocolError(STATUS_TEXT.get(status, f"firmware status {status}"))
        return response[4]

    def write(self, servo_id, address, width, value):
        payload = struct.pack(">IIIIIIII", SUBCMD_EEPROM_WRITE, servo_id,
                              address, width, value, 0, 0, 0)
        self._send(COMM_TYPE_DEBUG, payload, 32)
        response = self._recv_debug()
        if response[:4] != (SUBCMD_EEPROM_WRITE, servo_id, address, width):
            raise ProtocolError(f"unexpected write response: {response}")
        status = response[5]
        if status:
            raise ProtocolError(STATUS_TEXT.get(status, f"firmware status {status}"))
        return response[4], response[6]


def parse_int(value):
    return int(value, 0)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Read/write ST-series servo EEPROM via AstraArmController")
    parser.add_argument("port", help="controller serial port, e.g. /dev/tty_puppet_right")
    sub = parser.add_subparsers(dest="command", required=True)

    dump = sub.add_parser("dump", help="read diagnostic EEPROM fields")
    dump.add_argument("ids", type=parse_int, nargs="+", help="one or more servo IDs")

    read = sub.add_parser("read", help="read an arbitrary EEPROM address")
    read.add_argument("id", type=parse_int)
    read.add_argument("address", type=parse_int)
    read.add_argument("--width", type=int, choices=(1, 2), default=1)

    write = sub.add_parser("write", help="write and verify an arbitrary EEPROM address")
    write.add_argument("id", type=parse_int)
    write.add_argument("address", type=parse_int)
    write.add_argument("value", type=parse_int)
    write.add_argument("--width", type=int, choices=(1, 2), default=1)

    get_id = sub.add_parser("get-id", help="read a connected servo's configured ID")
    get_id.add_argument("id", type=parse_int, help="ID currently used to address the servo")

    set_id = sub.add_parser("set-id", help="change a connected servo's ID")
    set_id.add_argument("current_id", type=parse_int)
    set_id.add_argument("new_id", type=parse_int)

    set_mode = sub.add_parser(
        "set-mode",
        help="write a servo mode and verify the EEPROM readback")
    set_mode.add_argument("id", type=parse_int)
    set_mode.add_argument(
        "mode", type=parse_int,
        help="raw SMS/STS mode value (for this firmware: 0=position, 2=PWM)")
    return parser


def validate_args(args, parser):
    ids = []
    if args.command == "dump":
        ids = args.ids
    elif args.command in ("read", "write", "get-id", "set-mode"):
        ids = [args.id]
    elif args.command == "set-id":
        ids = [args.current_id, args.new_id]
    if any(not 0 <= servo_id <= 253 for servo_id in ids):
        parser.error("servo IDs must be in the range 0..253")
    if args.command in ("read", "write"):
        if args.address < 0 or args.address + args.width > 40:
            parser.error("EEPROM address range must stay within 0..39")
    if args.command == "write":
        maximum = 0xFF if args.width == 1 else 0xFFFF
        if not 0 <= args.value <= maximum:
            parser.error(f"value must fit in {args.width} byte(s): 0..{maximum}")
        if args.address <= 5 < args.address + args.width and not (
                args.address == 5 and args.width == 1 and args.value <= 253):
            parser.error("writes overlapping the ID register must use address 5, width 1, value 0..253")
    if args.command == "set-mode" and not 0 <= args.mode <= 0xFF:
        parser.error("mode must fit in one byte: 0..255")


def main():
    parser = build_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    tool = ServoEepromTool(args.port)
    try:
        tool.enter_safe_debug_mode()
        if args.command == "dump":
            print(f"{'parameter':<24}", end="")
            for servo_id in args.ids:
                print(f" ID{servo_id:<7}", end="")
            print()
            for name, address, width in REGISTERS:
                print(f"{name:<24}", end="")
                for servo_id in args.ids:
                    try:
                        value = tool.read(servo_id, address, width)
                        print(f" {value:<10}", end="")
                    except ProtocolError as exc:
                        print(f" {'ERR':<10}", end="")
                        print(f"\n  ID{servo_id} {name}: {exc}", end="")
                print()
        elif args.command == "read":
            value = tool.read(args.id, args.address, args.width)
            print(f"ID {args.id} EEPROM[{args.address}] width={args.width}: "
                  f"{value} (0x{value:0{args.width * 2}X})")
        elif args.command == "write":
            readback, resulting_id = tool.write(
                args.id, args.address, args.width, args.value)
            print(f"verified ID {resulting_id} EEPROM[{args.address}] "
                  f"width={args.width}: {readback} (0x{readback:0{args.width * 2}X})")
        elif args.command == "get-id":
            value = tool.read(args.id, 5, 1)
            print(f"servo addressed as ID {args.id} reports configured ID {value}")
        elif args.command == "set-id":
            readback, resulting_id = tool.write(args.current_id, 5, 1, args.new_id)
            print(f"servo ID changed: {args.current_id} -> {resulting_id}; "
                  f"readback={readback}")
        elif args.command == "set-mode":
            readback, resulting_id = tool.write(
                args.id, SMS_STS_MODE, 1, args.mode)
            print(f"verified ID {resulting_id} mode: {readback}")
    except (ProtocolError, serial.SerialException) as exc:
        parser.exit(1, f"error: {exc}\n")
    finally:
        try:
            tool.leave_debug_mode()
        except (ProtocolError, serial.SerialException):
            pass
        tool.close()


if __name__ == "__main__":
    main()
