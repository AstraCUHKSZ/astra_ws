import serial
import struct
import time
import sys
import argparse

# Config
BAUDRATE = 921600
HEADER = 0x5A
# IDS = [4, 5, 6, 7]  # Target Motor IDs for testing
# IDS = [8, 9, 10, 11]  # Alternative Motor IDs for testing
IDS = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Extended Motor IDs for testing
# IDS = [12, 13, 14, 15]  # Alternative Motor IDs for testing

# Comm Types
COMM_TYPE_DEBUG = 9

def send_packet(ser, type, payload):
    if len(payload) < 32:
        payload += b'\x00' * (32 - len(payload))
    
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
                size = 32 if type == COMM_TYPE_DEBUG else 16 # Default others
                
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
    # Subcmd 1
    # Note: pwm is int, supports negative
    # Use signed int (i) for id and pwm to handle negative values properly
    cmd = struct.pack(">IiiIIIII", 1, id, pwm, 0, 0, 0, 0, 0)
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
                # cmd[0]=2, cmd[1]=id, cmd[2]=pos, cmd[3]=speed, cmd[4]=load, cmd[5]=volt, cmd[6]=temp
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
    parser = argparse.ArgumentParser(description="Group Debug Tool for Astra Arm")
    parser.add_argument("port", help="Serial port")
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUDRATE, timeout=0.1)
    time.sleep(2)

    print("Entering Debug Mode (Stops PID loop)...")
    set_debug_mode(ser, True)
    
    # Disable all torque first (Safety)
    print("Disabling torque on target motors...")
    for id in IDS:
        set_servo_pwm(ser, id, 0)
        set_torque_enable(ser, id, False)
    
    try:
        while True:
            print("\n--- Menu ---")
            print("1. Test Individual Motor Direction")
            print("2. Test Permutation 1 (4+, 5-, 6+, 7-)")
            print("3. Test Permutation H (All +)")
            print("4. Read Status Only")
            print("q. Quit")
            
            choice = input("Select: ")
            
            if choice == 'q':
                break
            
            if choice == '1':
                target_id = int(input("Enter Motor ID to test (4-7): "))
                pwm_val = int(input("Enter PWM (e.g. 100): "))
                
                # Check status first
                stat = get_servo_status(ser, target_id)
                if stat:
                    print(f"Status before: {stat}")
                    if abs(stat['load']) > 200 or stat['temper'] > 60:
                        print("WARNING: High Load/Temp. Aborting.")
                        continue
                else:
                    print("Failed to read status. Aborting.")
                    continue
                
                print(f"Driving Motor {target_id} with {pwm_val} for 1 second...")
                
                # Enable Torque Only for Target
                for id in IDS:
                    if id == target_id and id != 11:
                        set_torque_enable(ser, id, True)
                    else:
                        set_torque_enable(ser, id, False) # Ensure others are free
                
                set_servo_pwm(ser, target_id, pwm_val)
                
                start = time.time()
                while time.time() - start < 1.0:
                    stat = get_servo_status(ser, target_id)
                    if stat:
                        sys.stdout.write(f"\rPos: {stat['pos']} Load: {stat['load']} Speed: {stat['speed']}")
                        # sys.stdout.flush()
                    else:
                        sys.stdout.write(f"\rNo Feedback")
                    time.sleep(0.1)
                    # Resend PWM command to prevent watchdog timeout
                    set_servo_pwm(ser, target_id, pwm_val)

                print("\nStopping...")
                set_servo_pwm(ser, target_id, 0)
                set_torque_enable(ser, target_id, False)

            elif choice == '2': 
                # print(f"Testing Standard Pattern: {IDS[0]}+, {IDS[1]}-, {IDS[2]}+, {IDS[3]}-")
                pwm_base = int(input("Enter Base PWM (e.g. 50): "))
                
                # Check status
                safe = True
                for id in IDS:
                    s = get_servo_status(ser, id)
                    if s and (abs(s['load']) > 100 or s['temper'] > 60):
                         print(f"Motor {id} unsafe status: {s}")
                         safe = False
                if not safe: continue

                # Enable Torque for all
                for id in IDS:
                    set_torque_enable(ser, id, True)

                cmd_map = {IDS[0]: pwm_base, IDS[1]: -pwm_base, IDS[2]: pwm_base, IDS[3]: -pwm_base}
                # cmd_map = {IDS[0]: pwm_base, IDS[1]: -pwm_base}
                
                print("Driving...")
                for id, pwm in cmd_map.items():
                    set_servo_pwm(ser, id, pwm)
                    
                start = time.time()
                while time.time() - start < 1.0:
                     for id in IDS:
                         s = get_servo_status(ser, id)
                         if s:
                             print(f"ID {id}: Load {s['load']}")
                     time.sleep(0.2)
                     # Resend
                     for id, pwm in cmd_map.items():
                        set_servo_pwm(ser, id, pwm)
                
                print("Stopping...")
                for id in IDS:
                    set_servo_pwm(ser, id, 0)
                    set_torque_enable(ser, id, False)

            elif choice == '3':
                 print("Scanning all 16 polarity permutations...")
                 pwm_val = int(input("Enter Test PWM (rec: 30-50): "))
                 duration = 0.5
                 
                 results = []
                 
                 # Enable Torque for all
                 for id in IDS:
                    set_torque_enable(ser, id, True)

                 # 16 combinations (0 to 15)
                 # 0 -> -1 -1 -1 -1
                 # 15 -> +1 +1 +1 +1
                 for i in range(16):
                     signs = []
                     for bit in range(4):
                         if (i >> bit) & 1:
                             signs.append(1)
                         else:
                             signs.append(-1)
                     
                     # Map to IDs [4, 5, 6, 7]
                     current_map = {}
                     sign_str = ""
                     for idx, s_id in enumerate(IDS):
                         current_map[s_id] = signs[idx] * pwm_val
                         sign_str += "+" if signs[idx] > 0 else "-"
                     
                     print(f"Testing [{sign_str}] ... ", end="", flush=True)
                     
                     # Safety Check pre-run
                     safe = True
                     start_pos = {}
                     for id in IDS:
                         s = get_servo_status(ser, id)
                         if not s:
                             print("Comm Error")
                             safe = False
                             break
                         if abs(s['load']) > 150 or s['temper'] > 60:
                             print(f"Unsafe start: ID{id} L{s['load']} T{s['temper']}")
                             safe = False
                             break
                         start_pos[id] = s['pos']
                     
                     if not safe:
                         print("Skipped (Safety)")
                         continue
                         
                     # Run
                     for id, pwm in current_map.items():
                         set_servo_pwm(ser, id, pwm)
                     
                     time.sleep(duration)
                     
                     # Read during/end
                     avg_load = 0
                     count = 0
                     movements = []
                     
                     for id in IDS:
                         s = get_servo_status(ser, id)
                         if s:
                             avg_load += abs(s['load'])
                             count += 1
                             delta = s['pos'] - start_pos[id]
                             movements.append(delta)
                     
                     # Stop
                     for id in IDS:
                         set_servo_pwm(ser, id, 0)
                     time.sleep(0.5) # Wait for stop
                     
                     if count == 0:
                         print("No Data")
                         continue
                     
                     avg_load /= count
                     
                     # Calculate total movement magnitude (vector sum? or sum of abs?)
                     # If they are fighting, the joint won't move, so individual encoders won't move much (unless elastic)
                     # If they are cooperating, all encoders should move significantly.
                     # However, some might be physically reversed mounted, so their "Positive" rotation helps the joint's "Negative" rotation.
                     # We just care if *encodes changed*.
                     avg_movement = sum([abs(m) for m in movements]) / len(movements)
                     
                     # Score: We want High efficiency. Movement / Load.
                     # Higher is better.
                     score = avg_movement / (avg_load + 1)
                     
                     print(f"Mov: {avg_movement:.1f}, Load: {avg_load:.1f}, Score: {score:.3f}")
                     
                     results.append({
                         "pattern": sign_str,
                         "movement": avg_movement,
                         "load": avg_load,
                         "score": score
                     })
                 
                 # Disable Torque all
                 for id in IDS:
                    set_torque_enable(ser, id, False)

                 print("\n--- Results (Ranked by Score) ---")
                 results.sort(key=lambda x: x['score'], reverse=True)
                 print(f"{'Pattern':<10} | {'Move':<8} | {'Load':<8} | {'Score':<8}")
                 for r in results:
                     print(f"{r['pattern']:<10} | {r['movement']:<8.1f} | {r['load']:<8.1f} | {r['score']:<8.3f}")

            elif choice == '4':
                for id in IDS:
                    s = get_servo_status(ser, id)
                    print(f"ID {id}: {s}")
                    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except serial.SerialException as e:
        print(f"\nSerial Error (Connection lost): {e}")
    except OSError as e:
        print(f"\nOS Error (Cable disconnected?): {e}")
    finally:
        print("Stopping all motors...")
        try:
            for id in IDS:
                set_servo_pwm(ser, id, 0)
        except Exception:
            print("Could not send stop command (Connection already lost).")
            
        # Optional: Disable Debug Mode?
        # set_debug_mode(ser, False) 
        # Note: If we disable debug mode, PID might kick in efficiently if torque isn't disabled in underlying logic.
        # But our debug set_pwm does raw writes.
        # Restoring normal operation would require reset or code handling.
        print("Done.")
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()
