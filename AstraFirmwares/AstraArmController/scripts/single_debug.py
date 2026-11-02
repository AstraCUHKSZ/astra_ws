import serial
import struct
import time
import sys
import argparse

# Config
BAUDRATE = 921600
HEADER = 0x5A
IDS = [12, 13, 14, 15]  # Target Motor IDs for testing (Non-joint)
# IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Target Motor IDs for testing (Joint)

# Comm Types
COMM_TYPE_DEBUG = 9

def send_packet(ser, type, payload):
    if len(payload) < 32:
        payload += b"\x00" * (32 - len(payload))
    
    packet = bytearray()
    packet.append(HEADER)
    packet.append(type)
    packet.extend(payload)
    
    checksum = (type + sum(payload)) & 0xFF
    packet.append(checksum)
    
    ser.write(packet)

def recv_packet(ser, expected_type=None, timeout=1.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            header = ser.read(1)
            if header == bytes([HEADER]):
                type_byte = ser.read(1)
                if not type_byte: continue
                type = type_byte[0]
                
                # Check known types if needed, for Debug assume 32 bytes
                size = 32 if type == COMM_TYPE_DEBUG else 16 
                
                payload = ser.read(size)
                if len(payload) != size: continue
                
                checksum_byte = ser.read(1)
                if not checksum_byte: continue
                checksum = checksum_byte[0]
                
                calc_checksum = (type + sum(payload)) & 0xFF
                if checksum == calc_checksum:
                    if expected_type is None or type == expected_type:
                        return payload
    return None

def set_debug_mode(ser, enable):
    # Subcmd 0
    cmd = struct.pack(">IIIIIIII", 0, 1 if enable else 0, 0, 0, 0, 0, 0, 0)
    send_packet(ser, COMM_TYPE_DEBUG, cmd)
    time.sleep(0.1)

def set_servo_pwm(ser, id, pwm):
    # Subcmd 1: PWM Control
    cmd = struct.pack(">IiiIIIII", 1, id, pwm, 0, 0, 0, 0, 0)
    send_packet(ser, COMM_TYPE_DEBUG, cmd)

def set_servo_position(ser, id, position):
    # Subcmd 4: Position Control
    # cmd[0]=4, cmd[1]=id, cmd[2]=position
    cmd = struct.pack(">IiiIIIII", 4, id, position, 0, 0, 0, 0, 0)
    send_packet(ser, COMM_TYPE_DEBUG, cmd)

def set_torque_enable(ser, id, enable):
    # Subcmd 3
    cmd = struct.pack(">IIIIIIII", 3, id, 1 if enable else 0, 0, 0, 0, 0, 0)
    send_packet(ser, COMM_TYPE_DEBUG, cmd)

def get_servo_status(ser, id):
    # Try multiple times to handle buffer noise
    for attempt in range(3):
        try:
            # Flush input buffer to remove accumulated FEEDBACK packets
            ser.reset_input_buffer()
            
            # Subcmd 2
            cmd = struct.pack(">IIIIIIII", 2, id, 0, 0, 0, 0, 0, 0)
            send_packet(ser, COMM_TYPE_DEBUG, cmd)
            
            # Timeout 0.2s per attempt is enough if synced
            resp = recv_packet(ser, COMM_TYPE_DEBUG, timeout=0.2)
            if resp:
                # Unpack response
                vals = struct.unpack(">IIIIIIII", resp)
                if vals[0] == 2 and vals[1] == id:
                    def to_int16(v):
                        if v > 32767: return v - 65536
                        return v

                    return {
                        "id": id,
                        "pos": to_int16(vals[2]),
                        "speed": to_int16(vals[3]),
                        "load": to_int16(vals[4]),
                        "voltage": vals[5],
                        "temper": vals[6]
                    }
        except:
             pass
    return None

def main():
    parser = argparse.ArgumentParser(description="Single Motor Debug Tool (Non-Joint)")
    parser.add_argument("port", help="Serial port")
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUDRATE, timeout=0.1)
    time.sleep(2)

    print("Entering Debug Mode (Stops PID loop)...")
    set_debug_mode(ser, True)
    
    # Disable all torque first (Safety) to allow manual moving if needed
    print("Disabling torque on target motors...")
    for id in IDS:
        set_torque_enable(ser, id, False)
    
    try:
        while True:
            print("\n--- Menu ---")
            print("1. Move Motor by Position Step (Recommended for Non-Joints)")
            print("2. Drive Motor by PWM (Only for Joint Motors or PWM Mode)")
            print("3. Read Status Only")
            print("4. Toggle Torque Enable")
            print("q. Quit")
            
            choice = input("Select: ")
            
            if choice == "q":
                break
            
            if choice == "1":
                try:
                    target_id = int(input("Enter Motor ID (12-15): "))
                except ValueError: continue
                
                # Check status first
                stat = get_servo_status(ser, target_id)
                if stat:
                    print(f"Current Status: {stat}")
                else:
                    print("Failed to read status.")
                    continue

                try:
                    delta = int(input("Enter Position Delta (e.g. 500 or -500): "))
                except ValueError: continue
                
                target_pos = stat["pos"] + delta
                print(f"Moving ID {target_id} from {stat['pos']} to {target_pos}...")
                
                set_torque_enable(ser, target_id, True)
                set_servo_position(ser, target_id, target_pos)
                
                # Monitor for 2 seconds
                start = time.time()
                while time.time() - start < 2.0:
                    stat = get_servo_status(ser, target_id)
                    if stat:
                        sys.stdout.write(f"\rPos: {stat['pos']} Load: {stat['load']} Speed: {stat['speed']}   ")
                        sys.stdout.flush()
                    time.sleep(0.1)
                print("\nDone.")

            elif choice == "2":
                try:
                    target_id = int(input("Enter Motor ID (12-15): "))
                    pwm_val = int(input("Enter PWM (-1000 to 1000): "))
                except ValueError: continue
                
                print(f"Driving Motor {target_id} with PWM {pwm_val}...")
                set_torque_enable(ser, target_id, True)
                
                start = time.time()
                while time.time() - start < 1.5:
                    set_servo_pwm(ser, target_id, pwm_val)
                    stat = get_servo_status(ser, target_id)
                    if stat:
                        sys.stdout.write(f"\rPos: {stat['pos']} Load: {stat['load']}   ")
                        sys.stdout.flush()
                    time.sleep(0.1)
                
                set_servo_pwm(ser, target_id, 0)
                print("\nStopped.")

            elif choice == "3":
                for id in IDS:
                    s = get_servo_status(ser, id)
                    print(f"ID {id}: {s}")
                    
            elif choice == "4":
                try:
                    target_id = int(input("Enter Motor ID: "))
                    state = input("Enable? (y/n): ").lower() == "y"
                    set_torque_enable(ser, target_id, state)
                    print(f"Torque {'Enabled' if state else 'Disabled'} for ID {target_id}")
                except ValueError: continue

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Stopping all motors...")
        for id in IDS:
            set_servo_pwm(ser, id, 0)
            set_torque_enable(ser, id, False)
        print("Done.")
        ser.close()

main()