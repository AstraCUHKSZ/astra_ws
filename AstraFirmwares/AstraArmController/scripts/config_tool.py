import serial
import struct
import time
import argparse

# Communication Constants
BAUDRATE = 921600
HEADER = 0x5A

# Command Types
COMM_TYPE_PING = 0
COMM_TYPE_PONG = 1
COMM_TYPE_CTRL = 2
COMM_TYPE_FEEDBACK = 3
COMM_TYPE_TORQUE = 4
COMM_TYPE_CONFIG_WRITE = 5
COMM_TYPE_CONFIG_READ = 6
COMM_TYPE_CONFIG_FEEDBACK = 7
COMM_TYPE_PIDTUNE = 8

PAYLOAD_SIZES = {
    COMM_TYPE_PING: 16,
    COMM_TYPE_PONG: 16,
    COMM_TYPE_CTRL: 16,
    COMM_TYPE_FEEDBACK: 16,
    COMM_TYPE_TORQUE: 16,
    COMM_TYPE_CONFIG_WRITE: 16,
    COMM_TYPE_CONFIG_READ: 16,
    COMM_TYPE_CONFIG_FEEDBACK: 16,
    COMM_TYPE_PIDTUNE: 32,
}

# Parameter IDs
PARAMS = {
    "EIShaper_enabled": (0x01, "int"),
    "EIShaper_freq": (0x02, "float"),
    "EIShaper_V": (0x03, "float"),
    "EIShaper_ctrl_freq": (0x04, "float"),
    "joint_vel_max": (0x05, "float"),
    "joint_acc": (0x06, "float"),
    "joint_backlash_compensate_feedforward": (0x07, "int"),
    "non_joint_vel_max": (0x08, "int"),
    "non_joint_acc": (0x09, "int"),
    "max_load": (0x0A, "int"),
    "max_temper": (0x0B, "int"),
    "max_current": (0x0C, "int"),
}

class AstraConfigTool:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for ESP32 to reboot if DTR/RTS is toggled

    def _send_packet(self, type, payload):
        size = PAYLOAD_SIZES[type]
        if len(payload) < size:
            payload += b'\x00' * (size - len(payload))
        elif len(payload) > size:
            payload = payload[:size]

        packet = bytearray()
        packet.append(HEADER)
        packet.append(type)
        packet.extend(payload)
        
        checksum = (type + sum(payload)) & 0xFF
        packet.append(checksum)
        
        self.ser.write(packet)

    def _recv_packet(self, expected_type=None):
        start_time = time.time()
        while time.time() - start_time < 1.0:
            header_byte = self.ser.read(1)
            if not header_byte:
                continue
            if header_byte == bytes([HEADER]):
                type_byte = self.ser.read(1)
                if not type_byte:
                    continue
                type = type_byte[0]
                size = PAYLOAD_SIZES.get(type, 0)
                if size == 0:
                    continue
                
                payload = self.ser.read(size)
                if len(payload) != size:
                    continue
                
                checksum_byte = self.ser.read(1)
                if not checksum_byte:
                    continue
                checksum = checksum_byte[0]
                calculated_checksum = (type + sum(payload)) & 0xFF
                
                if checksum == calculated_checksum:
                    if expected_type is None or type == expected_type:
                        return type, payload
        return None, None

    def read_param(self, param_name):
        if param_name not in PARAMS:
            print(f"Unknown parameter: {param_name}")
            return None
        
        param_id, param_type = PARAMS[param_name]
        # cmd[0] = param_id, cmd[1] = 0
        payload = struct.pack(">II", param_id, 0)
        self._send_packet(COMM_TYPE_CONFIG_READ, payload)
        
        rtype, rpayload = self._recv_packet(COMM_TYPE_CONFIG_FEEDBACK)
        if rtype is not None:
            # cmd is first 8 bytes
            cmd = struct.unpack(">II", rpayload[:8])
            if cmd[0] == param_id:
                if param_type == "float":
                    # Convert uint32 bits to float
                    return struct.unpack(">f", struct.pack(">I", cmd[1]))[0]
                else:
                    return cmd[1]
        return None

    def write_param(self, param_name, value):
        if param_name not in PARAMS:
            print(f"Unknown parameter: {param_name}")
            return False
        
        param_id, param_type = PARAMS[param_name]
        
        if param_type == "float":
            # Convert float to uint32 bits
            bits = struct.unpack(">I", struct.pack(">f", float(value)))[0]
        else:
            bits = int(value)
            
        payload = struct.pack(">II", param_id, bits)
        self._send_packet(COMM_TYPE_CONFIG_WRITE, payload)
        
        rtype, rpayload = self._recv_packet(COMM_TYPE_CONFIG_FEEDBACK)
        if rtype is not None:
            cmd = struct.unpack(">II", rpayload[:8])
            if cmd[0] == param_id:
                return True
        return False

    def close(self):
        self.ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Astra Arm Controller Config Tool")
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--read", help="Parameter name to read")
    parser.add_argument("--write", nargs=2, metavar=("PARAM", "VALUE"), help="Parameter name and value to write")
    parser.add_argument("--list", action="store_true", help="List available parameters")

    args = parser.parse_args()

    if args.list:
        print("Available parameters:")
        for name, (pid, ptype) in PARAMS.items():
            print(f"  - {name} ({ptype})")
        exit()

    if not args.read and not args.write:
        parser.print_help()
        exit()

    tool = AstraConfigTool(args.port)
    try:
        if args.read:
            val = tool.read_param(args.read)
            if val is not None:
                print(f"{args.read} = {val}")
            else:
                print(f"Failed to read {args.read}")
        
        if args.write:
            name, val = args.write
            if tool.write_param(name, val):
                print(f"Successfully wrote {name} = {val}")
                # Verify
                new_val = tool.read_param(name)
                print(f"Current {name} = {new_val}")
            else:
                print(f"Failed to write {name}")
    finally:
        tool.close()
