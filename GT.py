#!/usr/bin/env python3
import serial
import struct
import threading
import time
import csv
from datetime import datetime

class DynamixelController:
    def __init__(self, port='COM3', baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.servo_center = 512
        self.resolution = 0.29
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_PRESENT_POSITION = 36
        self.MIN_POSITION = 200
        self.MAX_POSITION = 824
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(1)
            print(f"✓ Connected to Dynamixel on {self.port}")
            self.enable_torque(1)
            self.enable_torque(2)
            return True
        except Exception as e:
            print(f"✗ Failed: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected from Dynamixel")
    
    def _checksum(self, pkt):
        return (~sum(pkt[2:])) & 0xFF
    
    def _build_packet(self, servo_id, instruction, address, data=None, data_len=2):
        packet = bytearray([0xFF, 0xFF, servo_id])
        if instruction == 3:
            if data_len == 1:
                packet.extend([4, instruction, address, data & 0xFF])
            else:
                packet.extend([5, instruction, address])
                packet.extend(struct.pack('<H', data))
        elif instruction == 2:
            packet.extend([4, instruction, address, 2])
        packet.append(self._checksum(packet))
        return bytes(packet)
    
    def enable_torque(self, servo_id):
        packet = self._build_packet(servo_id, 3, self.ADDR_TORQUE_ENABLE, 1, data_len=1)
        try:
            self.ser.write(packet)
            self.ser.flush()
            time.sleep(0.2)
            self.ser.read(20)
            return True
        except:
            return False
    
    def write_position(self, servo_id, angle_deg, read_response=True):
        units = int(self.servo_center + (angle_deg / self.resolution))
        units = max(self.MIN_POSITION, min(self.MAX_POSITION, units))
        packet = self._build_packet(servo_id, 3, self.ADDR_GOAL_POSITION, units, data_len=2)
        try:
            self.ser.write(packet)
            self.ser.flush()
            if read_response:
                time.sleep(0.05)
                self.ser.read(20)
            return True
        except:
            return False
    
    def move_to(self, pitch_deg, yaw_deg):
        self.write_position(2, pitch_deg, read_response=False)
        self.write_position(1, yaw_deg, read_response=False)
        time.sleep(0.1)
        self.ser.read(40)
    
    def read_position(self, servo_id):
        packet = self._build_packet(servo_id, 2, self.ADDR_PRESENT_POSITION)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            self.ser.flush()
            time.sleep(0.2)
            response = self.ser.read(20)
            if len(response) >= 7:
                units = struct.unpack('<H', response[5:7])[0]
                angle_deg = (units - self.servo_center) * self.resolution
                return angle_deg
        except:
            pass
        return None
    
    def set_speed(self, servo_id, speed):
        speed = max(0, min(1023, speed))
        packet = self._build_packet(servo_id, 3, 32, speed, data_len=2)
        try:
            self.ser.write(packet)
            self.ser.flush()
            time.sleep(0.2)
            self.ser.read(20)
            return True
        except:
            return False
    
    def get_angles(self):
        pitch = self.read_position(2)
        yaw = self.read_position(1)
        return pitch, yaw

class WitMotionIMU:
    def __init__(self, port='COM4', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.latest_euler = None
        self.lock = threading.Lock()
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(1)
            self.running = True
            threading.Thread(target=self._read_loop, daemon=True).start()
            print(f"✓ Connected to IMU on {self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected from IMU")
    
    def _read_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    while len(buffer) >= 11:
                        idx = -1
                        for i in range(len(buffer) - 1):
                            if buffer[i] == 0x55 and i + 1 < len(buffer) and buffer[i+1] in [0x50, 0x51, 0x52, 0x53, 0x54, 0x59]:
                                idx = i
                                break
                        if idx == -1:
                            buffer = buffer[1:] if len(buffer) > 0 else bytearray()
                            continue
                        if idx > 0:
                            buffer = buffer[idx:]
                        if len(buffer) < 11:
                            break
                        frame = buffer[:11]
                        checksum = sum(frame[:10]) & 0xFF
                        if checksum != frame[10]:
                            buffer = buffer[1:]
                            continue
                        if frame[1] == 0x53:
                            self._parse_euler(frame)
                        buffer = buffer[11:]
                time.sleep(0.01)
            except:
                pass
    
    def _parse_euler(self, frame):
        try:
            roll_raw = struct.unpack('<h', frame[2:4])[0]
            pitch_raw = struct.unpack('<h', frame[4:6])[0]
            yaw_raw = struct.unpack('<h', frame[6:8])[0]
            roll = (roll_raw / 32768.0) * 180.0
            pitch = -(pitch_raw / 32768.0) * 180.0
            yaw = (yaw_raw / 32768.0) * 180.0
            with self.lock:
                self.latest_euler = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
        except:
            pass
    
    def get_euler(self, timeout=2):
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                if self.latest_euler is not None:
                    return self.latest_euler.copy()
            time.sleep(0.05)
        return None
    
    def zero_calibration(self):
        packet = bytearray([0xFF, 0xAA, 0x01, 0x04, 0xE0])
        try:
            self.ser.write(packet)
            self.ser.flush()
            time.sleep(0.5)
            print("✓ IMU angle reference reset to zero")
            return True
        except:
            print("✗ Failed to zero IMU")
            return False
    
    def wait_for_data(self, timeout=10):
        start = time.time()
        while time.time() - start < timeout:
            with self.lock:
                if self.latest_euler is not None:
                    return True
            time.sleep(0.1)
        return False

class RobotHeadTool:
    def __init__(self, dyn_port='COM3', imu_port='COM4'):
        self.dyn = DynamixelController(port=dyn_port)
        self.imu = WitMotionIMU(port=imu_port)
    
    def connect(self):
        print("\n" + "="*70)
        print("CONNECTING TO ROBOT HEAD")
        print("="*70 + "\n")
        if not self.dyn.connect():
            return False
        if not self.imu.connect():
            self.dyn.disconnect()
            return False
        print("\nWaiting for IMU data...")
        if not self.imu.wait_for_data(timeout=5):
            print("✗ No IMU data received")
            self.disconnect()
            return False
        print("✓ IMU data OK\n")
        return True
    
    def disconnect(self):
        self.dyn.disconnect()
        self.imu.disconnect()
    
    def run_calibration(self, speed):
        print("\n" + "="*70)
        print("EYE-GT CALIBRATION")
        print("="*70 + "\n")
        print(f"Setting motor speed to {speed}...")
        self.dyn.set_speed(1, speed)
        self.dyn.set_speed(2, speed)
        time.sleep(0.5)
        print("Moving to center...")
        self.dyn.move_to(0, 0)
        time.sleep(2)
        print("="*70)
        print("INFINITY SIGN (∞) - POSITIVE →")
        print("="*70 + "\n")
        infinity_pos = [(40, -30, "Top-Left"), (-40, 30, "Bottom-Right"), (40, 30, "Top-Right"), (-40, -30, "Bottom-Left")]
        for pitch, yaw, desc in infinity_pos:
            print(desc)
            self.dyn.move_to(pitch, yaw)
            time.sleep(1.0)
        print("✓ Infinity sign (positive) complete")
        self.dyn.move_to(0, 0)
        time.sleep(2)
        print("="*70)
        print("INFINITY SIGN (∞) - NEGATIVE ←")
        print("="*70 + "\n")
        infinity_neg = [(-40, 30, "Top-Right"), (40, -30, "Bottom-Left"), (-40, -30, "Top-Left"), (40, 30, "Bottom-Right")]
        for pitch, yaw, desc in infinity_neg:
            print(desc)
            self.dyn.move_to(pitch, yaw)
            time.sleep(1.0)
        print("✓ Infinity sign (negative) complete")
        self.dyn.move_to(0, 0)
        time.sleep(2)
        print("="*70)
        print("EYE-GT CALIBRATION COMPLETE!")
        print("="*70 + "\n")
    
    def run_scenario(self, speed):
        """Run through user-defined scenario points"""
        print("\n" + "="*70)
        print("SCENARIO MODE")
        print("="*70 + "\n")
        
        print(f"Setting motor speed to {speed}...")
        self.dyn.set_speed(1, speed)
        self.dyn.set_speed(2, speed)
        time.sleep(0.5)
        
        print("Enter scenario points (format: yaw pitch, one per line)")
        print("Example: 10 20")
        print("         30 20")
        print("         60 0")
        print("Enter 'done' when finished\n")
        
        points = []
        while True:
            try:
                line = input("Enter point (yaw pitch) or 'done': ").strip()
                if line.lower() == 'done':
                    break
                
                parts = line.split()
                if len(parts) != 2:
                    print("✗ Invalid format, use: yaw pitch")
                    continue
                
                yaw = float(parts[0])
                pitch = float(parts[1])
                
                if not (-90 <= yaw <= 90):
                    print("✗ Yaw must be between -90° and +90°")
                    continue
                
                if not (-40 <= pitch <= 40):
                    print("✗ Pitch must be between -40° and +40°")
                    continue
                
                points.append((yaw, pitch))
                print(f"  ✓ Added: Yaw={yaw:+.1f}° Pitch={pitch:+.1f}°")
            
            except ValueError:
                print("✗ Invalid input, use numbers only")
        
        if not points:
            print("✗ No points entered")
            return
        
        print(f"\n✓ Scenario has {len(points)} points\n")
        
        # Run scenario
        print("="*70)
        print("RUNNING SCENARIO")
        print("="*70 + "\n")
        
        for i, (yaw, pitch) in enumerate(points, 1):
            print(f"Point {i}/{len(points)}: Yaw={yaw:+.1f}° Pitch={pitch:+.1f}°")
            self.dyn.move_to(pitch, yaw)
            print(f"Waiting 5 seconds for stabilization...")
            time.sleep(5)
            print()
        
        print("✓ Scenario complete, returning to center...")
        self.dyn.move_to(0, 0)
        time.sleep(2)
        
        print("="*70)
        print("SCENARIO FINISHED!")
        print("="*70 + "\n")
    
    def run_measurements(self, speed, save_data=True):
        print("\n" + "="*70)
        print("STARTING MEASUREMENT SESSION")
        print("="*70 + "\n")
        print(f"Setting motor speed to {speed}...")
        self.dyn.set_speed(1, speed)
        self.dyn.set_speed(2, speed)
        time.sleep(0.5)
        
        filename = None
        if save_data:
            while True:
                filename_input = input("Enter filename for measurements (without .csv): ").strip()
                if filename_input:
                    filename = f"{filename_input}.csv"
                    break
                print("✗ Filename cannot be empty")
        
        measurements = []
        if filename:
            print(f"✓ Measurements will be saved to: {filename}\n")
        else:
            print("✓ Measurements will NOT be saved\n")
        print("To stop: press 'M' then Enter\n")
        
        measurement_id = 0
        try:
            while True:
                measurement_id += 1
                print("="*70)
                print(f"MEASUREMENT #{measurement_id}")
                print("="*70 + "\n")
                
                try:
                    yaw_input = input("Enter yaw angle (degrees): ").strip()
                    if yaw_input.upper() == 'M':
                        break
                    yaw = float(yaw_input)
                except ValueError:
                    print("✗ Invalid input, try again")
                    measurement_id -= 1
                    continue
                
                try:
                    pitch_input = input("Enter pitch angle (degrees): ").strip()
                    if pitch_input.upper() == 'M':
                        break
                    pitch = float(pitch_input)
                except ValueError:
                    print("✗ Invalid input, try again")
                    measurement_id -= 1
                    continue
                
                if not (-40 <= pitch <= 40):
                    print("✗ Pitch must be between -40° and +40°")
                    measurement_id -= 1
                    continue
                
                if not (-90 <= yaw <= 90):
                    print("✗ Yaw must be between -90° and +90°")
                    measurement_id -= 1
                    continue
                
                print(f"\nMoving to Pitch={pitch:+.1f}° Yaw={yaw:+.1f}°...")
                pitch_before, yaw_before = self.dyn.get_angles()
                if pitch_before is None or yaw_before is None:
                    print("✗ Failed to read motor position")
                    measurement_id -= 1
                    continue
                
                self.dyn.move_to(pitch, yaw)
                time.sleep(2)
                
                pitch_after, yaw_after = self.dyn.get_angles()
                if pitch_after is None or yaw_after is None:
                    print("✗ Failed to read motor position")
                    measurement_id -= 1
                    continue
                
                euler = self.imu.get_euler(timeout=2)
                if euler is None:
                    print("✗ Failed to read IMU data")
                    measurement_id -= 1
                    continue
                
                measurement = {
                    "ID": measurement_id,
                    "Timestamp": datetime.now().isoformat(),
                    "Target_Pitch": round(pitch, 2),
                    "Target_Yaw": round(yaw, 2),
                    "Real_Pitch": round(pitch_after, 2),
                    "Real_Yaw": round(yaw_after, 2),
                    "IMU_Roll": round(euler['roll'], 2),
                    "IMU_Pitch": round(euler['pitch'], 2),
                    "IMU_Yaw": round(euler['yaw'], 2)
                }
                measurements.append(measurement)
                
                print(f"\n✓ Measurement saved\n")
                print(f"  Target:   Pitch={pitch:+.1f}° Yaw={yaw:+.1f}°")
                print(f"  Real:     Pitch={pitch_after:+.2f}° Yaw={yaw_after:+.2f}°")
                print(f"  IMU:      Roll={euler['roll']:+.2f}° Pitch={euler['pitch']:+.2f}° Yaw={euler['yaw']:+.2f}°\n")
                
                cont = input("Continue? (any key for next, M+Enter to stop): ").strip()
                if cont.upper() == 'M':
                    break
                print()
        
        except KeyboardInterrupt:
            print("\n✗ Interrupted by user")
        
        if save_data and filename and measurements:
            try:
                with open(filename, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=measurements[0].keys())
                    writer.writeheader()
                    writer.writerows(measurements)
                print("\n" + "="*70)
                print(f"✓ Saved to {filename}")
                print(f"✓ Total: {len(measurements)} measurements")
                print("="*70 + "\n")
            except Exception as e:
                print(f"✗ Failed to save: {e}")
        else:
            print("\n" + "="*70)
            print("✓ Session complete (data not saved)")
            print("="*70 + "\n")

def main():
    print("\n" + "="*70)
    print("EYE-GT TESTING TOOL")
    print("="*70 + "\n")
    
    tool = RobotHeadTool(dyn_port='COM3', imu_port='COM4')
    
    try:
        if not tool.connect():
            print("✗ Connection failed")
            return
        
        print("\nMoving motors to home position (0°, 0°)...")
        tool.dyn.move_to(0, 0)
        time.sleep(2)
        print("Resetting IMU angle reference to zero...")
        tool.imu.zero_calibration()
        time.sleep(1)
        
        save_choice = input("Do you want to save measurement data? (y/n): ").strip().lower()
        save_data = save_choice == 'y'
        
        calibration_choice = input("Do you want to run eye-GT calibration? (y/n): ").strip().lower()
        
        if calibration_choice == 'y':
            print("\n" + "="*70)
            print("CALIBRATION SPEED SETTINGS")
            print("="*70)
            print("Speed range: 10-500 (10=slowest, 500=fastest)")
            print("Recommended: 250")
            print("="*70 + "\n")
            while True:
                try:
                    cal_speed = int(input("Enter motor speed for calibration (10-500, recommended 250): ").strip())
                    if 10 <= cal_speed <= 500:
                        break
                    print("✗ Speed must be between 10 and 500")
                except ValueError:
                    print("✗ Invalid input")
            tool.run_calibration(cal_speed)
        
        print("\n" + "="*70)
        print("MEASUREMENT SPEED SETTINGS")
        print("="*70)
        print("Speed range: 10-500 (10=slowest, 500=fastest)")
        print("Recommended: 100")
        print("="*70 + "\n")
        while True:
            try:
                meas_speed = int(input("Enter motor speed (10-500, recommended 100): ").strip())
                if 10 <= meas_speed <= 500:
                    break
                print("✗ Speed must be between 10 and 500")
            except ValueError:
                print("✗ Invalid input")
        
        mode_choice = input("\nChoose mode:\n  (m)easurements - Record angle data\n  (s)cenario - Move through predefined points\nEnter choice (m/s): ").strip().lower()
        
        if mode_choice == 's':
            tool.run_scenario(meas_speed)
        else:
            tool.run_measurements(meas_speed, save_data)
    
    except KeyboardInterrupt:
        print("\n✗ Interrupted by user")
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tool.disconnect()

if __name__ == "__main__":
    main()
