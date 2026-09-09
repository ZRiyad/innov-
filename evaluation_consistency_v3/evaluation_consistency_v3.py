#!/usr/bin/env python3
import socket
import struct
import time
import csv
import threading
import os
from dynamixel_sdk import *

DEVICENAME = 'COM3'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
SERVO_ID = 1
SERVO_ID_ROLL = 2
SERVO_ID_YAW = 3
SERVO_CENTER = 516
UDP_HOST = '127.0.0.1'
UDP_PORT = 8782
WITMOTION_SAMPLES = 3
WITMOTION_SAMPLE_DELAY = 3.0
WITMOTION_READ_TIMEOUT = 10

class UDPAngleReader:
    def __init__(self, host=UDP_HOST, port=UDP_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.latest_angles = {'roll': None, 'pitch': None, 'yaw': None}
        self.packet_count = 0
        self.running = False
        self.x_offset = 0.0
        self.y_offset = 0.0
    
    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self.thread.start()
            print("UDP: " + self.host + ":" + str(self.port))
            time.sleep(0.5)
            return True
        except Exception as e:
            print("UDP error: " + str(e))
            return False
    
    def _receiver_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if len(data) == 11 and data[0] == 0x55 and data[1] == 0x53:
                    self._decode_packet(data)
            except socket.timeout:
                pass
            except:
                pass
    
    def _decode_packet(self, data):
        try:
            if data[0] != 0x55 or data[1] != 0x53:
                return
            roll_raw = struct.unpack('<h', data[2:4])[0]
            pitch_raw = struct.unpack('<h', data[4:6])[0]
            yaw_raw = struct.unpack('<h', data[6:8])[0]
            roll = (roll_raw / 32768.0) * 180.0
            pitch = -((pitch_raw / 32768.0) * 180.0)
            yaw = (yaw_raw / 32768.0) * 180.0
            checksum_calc = (0x55 + 0x53 + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[9]) & 0xFF
            if checksum_calc == data[10]:
                self.latest_angles = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                self.packet_count += 1
        except:
            pass
    
    def calibrate_offsets(self, num_samples=3):
        x_samples = []
        y_samples = []
        start_time = time.time()
        last_x = None
        last_y = None
        max_wait = 20
        
        while self.latest_angles['roll'] is None:
            elapsed = time.time() - start_time
            if elapsed > max_wait:
                return False
            time.sleep(0.1)
        
        sample_start = time.time()
        while len(x_samples) < num_samples:
            if time.time() - sample_start > 15:
                break
            
            if self.latest_angles['roll'] is not None and self.latest_angles['pitch'] is not None:
                x_raw = self.latest_angles['roll']
                y_raw = -self.latest_angles['pitch']
                
                if x_raw != last_x or y_raw != last_y:
                    x_samples.append(x_raw)
                    y_samples.append(y_raw)
                    last_x = x_raw
                    last_y = y_raw
            
            time.sleep(0.05)
        
        if x_samples and y_samples:
            self.x_offset = sum(x_samples) / len(x_samples)
            self.y_offset = sum(y_samples) / len(y_samples)
            return True
        return False
    
    def read_pitch(self, num_samples=WITMOTION_SAMPLES, timeout=WITMOTION_READ_TIMEOUT):
        samples = []
        start_time = time.time()
        last_value = None
        
        time.sleep(3)
        
        while len(samples) < num_samples:
            if time.time() - start_time > timeout:
                if len(samples) == 0:
                    return None, None, None, None, None
                break
            if self.latest_angles['pitch'] is not None:
                current = -self.latest_angles['pitch'] - self.y_offset
                if current != last_value:
                    samples.append(current)
                    last_value = current
                    
                    if len(samples) < num_samples:
                        time.sleep(WITMOTION_SAMPLE_DELAY)
            time.sleep(0.05)
        
        if samples:
            mean, std_dev, min_val, max_val = calculate_stats(samples)
            return mean, std_dev, min_val, max_val, samples
        return None, None, None, None, None
    
    def read_roll(self, num_samples=WITMOTION_SAMPLES, timeout=WITMOTION_READ_TIMEOUT):
        samples = []
        start_time = time.time()
        last_value = None
        
        time.sleep(3)
        
        while len(samples) < num_samples:
            if time.time() - start_time > timeout:
                if len(samples) == 0:
                    return None, None, None, None, None
                break
            if self.latest_angles['roll'] is not None:
                current = self.latest_angles['roll'] - self.x_offset
                if current != last_value:
                    samples.append(current)
                    last_value = current
                    
                    if len(samples) < num_samples:
                        time.sleep(WITMOTION_SAMPLE_DELAY)
            time.sleep(0.05)
        if samples:
            mean, std_dev, min_val, max_val = calculate_stats(samples)
            return mean, std_dev, min_val, max_val, samples
        return None, None, None, None, None
    
    def read_yaw(self, num_samples=WITMOTION_SAMPLES, timeout=WITMOTION_READ_TIMEOUT):
        samples = []
        start_time = time.time()
        last_value = None
        
        time.sleep(3)
        
        while len(samples) < num_samples:
            if time.time() - start_time > timeout:
                if len(samples) == 0:
                    return None, None, None, None, None
                break
            if self.latest_angles['yaw'] is not None:
                current = self.latest_angles['yaw']
                if current != last_value:
                    samples.append(current)
                    last_value = current
                    
                    if len(samples) < num_samples:
                        time.sleep(WITMOTION_SAMPLE_DELAY)
            time.sleep(0.05)
        if samples:
            mean, std_dev, min_val, max_val = calculate_stats(samples)
            return mean, std_dev, min_val, max_val, samples
        return None, None, None, None, None
    
    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

def init_dynamixel():
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    if not port_handler.openPort():
        return None, None
    if not port_handler.setBaudRate(BAUDRATE):
        return None, None
    print("Dynamixel: " + DEVICENAME)
    return port_handler, packet_handler

def move_servo(port_handler, packet_handler, position_units):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, SERVO_ID, 30, position_units)
    return dxl_comm_result == COMM_SUCCESS

def read_servo_position(port_handler, packet_handler):
    position_units, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, SERVO_ID, 36)
    if dxl_comm_result != COMM_SUCCESS:
        return None
    return (position_units - SERVO_CENTER) * 0.29

def move_roll_servo(port_handler, packet_handler, position_units):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, SERVO_ID_ROLL, 30, position_units)
    return dxl_comm_result == COMM_SUCCESS

def read_roll_servo_position(port_handler, packet_handler):
    position_units, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, SERVO_ID_ROLL, 36)
    if dxl_comm_result != COMM_SUCCESS:
        return None
    return (position_units - SERVO_CENTER) * 0.29

def move_yaw_servo(port_handler, packet_handler, position_units):
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, SERVO_ID_YAW, 30, position_units)
    return dxl_comm_result == COMM_SUCCESS

def read_yaw_servo_position(port_handler, packet_handler):
    position_units, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, SERVO_ID_YAW, 36)
    if dxl_comm_result != COMM_SUCCESS:
        return None
    return (position_units - SERVO_CENTER) * 0.29

def get_next_filename(base_name):
    counter = 1
    while True:
        filename = base_name + "_" + str(counter) + ".csv"
        if not os.path.exists(filename):
            return filename
        counter += 1

def circular_difference(angle1, angle2):
    diff = angle1 - angle2
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def calculate_stats(samples):
    if not samples:
        return None, None, None, None
    
    mean = sum(samples) / len(samples)
    variance = sum((x - mean) ** 2 for x in samples) / len(samples)
    std_dev = variance ** 0.5
    
    min_val = min(samples)
    max_val = max(samples)
    
    return mean, std_dev, min_val, max_val

def measure_coupling_matrix(port_handler, packet_handler, wit_reader):
    """
    Measure coupling matrix coefficients:
    R = a*uR + b*uP
    P = c*uR + d*uP
    
    Returns: (a, b, c, d)
    """
    print("\n" + "="*60)
    print("MEASURING COUPLING MATRIX")
    print("="*60 + "\n")
    
    step = int(0.5 / 0.29)  # ~0.5 degrees in units
    
    # Baseline at center
    move_servo(port_handler, packet_handler, SERVO_CENTER)
    move_roll_servo(port_handler, packet_handler, SERVO_CENTER)
    time.sleep(2)
    
    if not wit_reader.calibrate_offsets(num_samples=3):
        print("ERROR: Could not calibrate offsets")
        return None
    
    # Read baseline (should be ~0)
    roll_base, _, _, _, _ = wit_reader.read_roll()
    pitch_base, _, _, _, _ = wit_reader.read_pitch()
    
    print("Baseline: Roll=" + "%.3f" % roll_base + "°, Pitch=" + "%.3f" % pitch_base + "°\n")
    
    # Move Roll only, measure effect on both
    print("Moving Roll servo only...")
    move_roll_servo(port_handler, packet_handler, SERVO_CENTER + step)
    time.sleep(2)
    
    roll_resp_r, _, _, _, _ = wit_reader.read_roll()
    pitch_resp_r, _, _, _, _ = wit_reader.read_pitch()
    
    # a = dRoll/dRoll_servo, c = dPitch/dRoll_servo
    a = (roll_resp_r - roll_base) / 0.5
    c = (pitch_resp_r - pitch_base) / 0.5
    
    print("  Roll response: " + "%.3f" % roll_resp_r + "° (a=" + "%.3f" % a + ")")
    print("  Pitch response: " + "%.3f" % pitch_resp_r + "° (c=" + "%.3f" % c + ")\n")
    
    # Move Pitch only, measure effect on both
    move_roll_servo(port_handler, packet_handler, SERVO_CENTER)
    time.sleep(1)
    print("Moving Pitch servo only...")
    move_servo(port_handler, packet_handler, SERVO_CENTER + step)
    time.sleep(2)
    
    roll_resp_p, _, _, _, _ = wit_reader.read_roll()
    pitch_resp_p, _, _, _, _ = wit_reader.read_pitch()
    
    # b = dRoll/dPitch_servo, d = dPitch/dPitch_servo
    b = (roll_resp_p - roll_base) / 0.5
    d = (pitch_resp_p - pitch_base) / 0.5
    
    print("  Roll response: " + "%.3f" % roll_resp_p + "° (b=" + "%.3f" % b + ")")
    print("  Pitch response: " + "%.3f" % pitch_resp_p + "° (d=" + "%.3f" % d + ")\n")
    
    # Return to center
    move_servo(port_handler, packet_handler, SERVO_CENTER)
    move_roll_servo(port_handler, packet_handler, SERVO_CENTER)
    time.sleep(2)
    
    print("Coupling matrix:")
    print("  R = " + "%.3f" % a + "*uR + " + "%.3f" % b + "*uP")
    print("  P = " + "%.3f" % c + "*uR + " + "%.3f" % d + "*uP\n")
    
    return (a, b, c, d)

def solve_2x2_system(R_error, P_error, a, b, c, d):
    """
    Solve 2x2 linear system:
    a*uR + b*uP = -R_error
    c*uR + d*uP = -P_error
    
    Returns: (delta_uR, delta_uP)
    """
    det = a*d - b*c
    
    if abs(det) < 0.0001:
        print("    WARNING: Matrix near singular, det=" + "%.6f" % det)
        return (0, 0)
    
    delta_uR = (-R_error * d - (-P_error) * b) / det
    delta_uP = (a * (-P_error) - c * (-R_error)) / det
    
    return (delta_uR, delta_uP)

def micro_align_dual_pure(port_handler, packet_handler, wit_reader, coupling_matrix, tolerance=0.15):
    """Micro-align using 2x2 system solution (mathematically pure)"""
    print("    Micro-aligning (pure 2x2 method)...")
    
    a, b, c, d = coupling_matrix
    
    roll_units = SERVO_CENTER
    pitch_units = SERVO_CENTER
    iteration = 0
    
    while True:
        roll_wit = wit_reader.latest_angles['roll']
        pitch_wit = wit_reader.latest_angles['pitch']
        
        if roll_wit is None or pitch_wit is None:
            time.sleep(0.1)
            continue
        
        if abs(roll_wit) < tolerance and abs(pitch_wit) < tolerance:
            print("    ✓ Roll " + "%.2f" % roll_wit + "° + Pitch " + "%.2f" % pitch_wit + "° aligned (iteration " + str(iteration) + ")\n")
            return True
        
        # Solve 2x2 system
        delta_uR, delta_uP = solve_2x2_system(roll_wit, pitch_wit, a, b, c, d)
        
        # Apply solution
        roll_units = int(roll_units + delta_uR)
        pitch_units = int(pitch_units + delta_uP)
        
        move_roll_servo(port_handler, packet_handler, roll_units)
        move_servo(port_handler, packet_handler, pitch_units)
        
        time.sleep(1)
        iteration += 1

def auto_align(port_handler, packet_handler, wit_reader):
    print("\n" + "="*60)
    print("AUTO ALIGN TO X≈0, Y≈0")
    print("="*60 + "\n")
    
    pitch_units = SERVO_CENTER
    roll_units = SERVO_CENTER
    step = int(0.5 / 0.29)
    max_iterations = 50
    pitch_tol = 0.1
    roll_tol = 0.1
    
    for iteration in range(max_iterations):
        roll_wit = wit_reader.latest_angles['roll']
        pitch_wit = wit_reader.latest_angles['pitch']
        
        if roll_wit is None or pitch_wit is None:
            time.sleep(0.1)
            continue
        
        print("[{}/{}] X: {:+.2f}° | Y: {:+.2f}°".format(iteration+1, max_iterations, roll_wit, pitch_wit))
        
        if abs(pitch_wit) < 0.1 and abs(roll_wit) < 0.1:
            print("\n✓ X and Y < 0.1°! Waiting 5 seconds to verify stability...\n")
            time.sleep(5)
            
            roll_check = wit_reader.latest_angles['roll']
            pitch_check = wit_reader.latest_angles['pitch']
            print("  Stability check: X: " + "%.2f" % roll_check + "° | Y: " + "%.2f" % pitch_check + "°\n")
            
            if abs(pitch_check) < 0.1 and abs(roll_check) < 0.1:
                print("✓ ALIGNED AND STABLE!\n")
                return True
            else:
                print("⚠ Not stable yet, continuing alignment...\n")
        
        if pitch_wit > pitch_tol:
            pitch_units -= step
            move_servo(port_handler, packet_handler, pitch_units)
        elif pitch_wit < -pitch_tol:
            pitch_units += step
            move_servo(port_handler, packet_handler, pitch_units)
        
        if roll_wit > roll_tol:
            roll_units -= step
            move_roll_servo(port_handler, packet_handler, roll_units)
        elif roll_wit < -roll_tol:
            roll_units += step
            move_roll_servo(port_handler, packet_handler, roll_units)
        
        time.sleep(2)
    
    print("⚠ Max iterations reached\n")
    return True

def main():
    print("\n" + "="*60)
    print("GIMBAL CONSISTENCY TEST - 5 RUNS (PURE 2x2 METHOD)")
    print("="*60)
    
    wit_reader = UDPAngleReader(UDP_HOST, UDP_PORT)
    if not wit_reader.start():
        return
    time.sleep(1)
    
    port_handler, packet_handler = init_dynamixel()
    if not port_handler:
        wit_reader.stop()
        return
    
    # MEASURE COUPLING MATRIX ONCE AT START
    coupling_matrix = measure_coupling_matrix(port_handler, packet_handler, wit_reader)
    if coupling_matrix is None:
        print("ERROR: Could not measure coupling matrix")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    test_angles = [-30, -20, -10, 0, 10, 20, 30]
    warm_offset = int(103)
    
    # ===== PITCH - 5 RUNS WITH SAME REFERENCE =====
    print("\n" + "="*60)
    print("PITCH CALIBRATION - 5 RUNS (SAME REFERENCE)")
    print("="*60)
    
    print("\nMoving all to 516...")
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    print("Warming up Pitch servo (4 cycles)...\n")
    for cycle in range(4):
        move_servo(port_handler, packet_handler, 516 - warm_offset)
        time.sleep(1)
        move_servo(port_handler, packet_handler, 516 + warm_offset)
        time.sleep(1)
        print("  Warm-up cycle " + str(cycle+1) + "/4 done")
    
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    if not auto_align(port_handler, packet_handler, wit_reader):
        print("ERROR: Auto alignment failed")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    time.sleep(1)
    
    if not wit_reader.calibrate_offsets(num_samples=3):
        print("ERROR: Could not calibrate offsets")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    servo_zero = read_servo_position(port_handler, packet_handler)
    wit_zero, wit_std_zero, wit_min_zero, wit_max_zero, _ = wit_reader.read_pitch()
    
    if servo_zero is None or wit_zero is None:
        print("ERROR: Could not read sensors at zero position")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    print("    Servo at zero: " + "%.2f" % servo_zero + " degrees")
    print("    WitMotion at zero: " + "%.2f" % wit_zero + " degrees")
    print("      Std Dev: " + "%.3f" % wit_std_zero + ", Range: " + "%.2f" % wit_min_zero + " to " + "%.2f" % wit_max_zero)
    
    ref_servo = servo_zero
    ref_wit = wit_zero
    print("    (Reference point set - SAME FOR ALL 5 RUNS)")
    print("")
    
    # RUN 5 TIMES WITH SAME REFERENCE
    for pitch_run in range(1, 6):
        print("\nPITCH CALIBRATION TEST - RUN " + str(pitch_run) + "/5:")
        print("="*60 + "\n")
        
        results_pitch = []
        
        for i, gimbal_angle in enumerate(test_angles, 1):
            print("[" + str(i) + "/7] Pitch angle: " + str(gimbal_angle))
            servo_units = int(SERVO_CENTER + (gimbal_angle / 0.29))
            if not move_servo(port_handler, packet_handler, servo_units):
                continue
            
            time.sleep(2)
            
            # MICRO-ALIGN using pure 2x2 method
            micro_align_dual_pure(port_handler, packet_handler, wit_reader, coupling_matrix)
            
            time.sleep(2)
            
            servo_angle = read_servo_position(port_handler, packet_handler)
            if servo_angle is None:
                continue
            print("    Servo: " + "%.2f" % servo_angle)
            
            wit_angle, wit_std, wit_min, wit_max, _ = wit_reader.read_pitch()
            if wit_angle is None:
                continue
            print("    WitMotion: " + "%.2f" % wit_angle)
            print("      Std Dev: " + "%.3f" % wit_std + ", Range: " + "%.2f" % wit_min + " to " + "%.2f" % wit_max)
            
            servo_change = servo_angle - ref_servo
            wit_change = wit_angle - ref_wit
            error = abs(abs(servo_change) - abs(wit_change))
            
            print("    Error: " + "%.2f" % error)
            print("")
            
            results_pitch.append([gimbal_angle, servo_angle, wit_angle, servo_change, wit_change, error, wit_std, wit_min, wit_max])
        
        csv_file = get_next_filename("gimbal_calibration_pitch_run")
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Angle", "Servo", "WitMotion", "Servo_Change", "WitMotion_Change", "Error", "Std_Dev", "Min", "Max"])
            writer.writerows(results_pitch)
        
        print("Saved: " + csv_file)
    
    # ===== ROLL - 5 RUNS WITH SAME REFERENCE =====
    print("\n" + "="*60)
    print("ROLL CALIBRATION - 5 RUNS (SAME REFERENCE)")
    print("="*60)
    
    print("\nMoving all to 516...")
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    print("Warming up Roll servo (4 cycles)...\n")
    for cycle in range(4):
        move_roll_servo(port_handler, packet_handler, 516 - warm_offset)
        time.sleep(1)
        move_roll_servo(port_handler, packet_handler, 516 + warm_offset)
        time.sleep(1)
        print("  Warm-up cycle " + str(cycle+1) + "/4 done")
    
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    if not auto_align(port_handler, packet_handler, wit_reader):
        print("ERROR: Auto alignment failed")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    time.sleep(1)
    
    if not wit_reader.calibrate_offsets(num_samples=3):
        print("ERROR: Could not calibrate offsets")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    roll_zero = read_roll_servo_position(port_handler, packet_handler)
    roll_wit_zero, roll_std_zero, roll_min_zero, roll_max_zero, _ = wit_reader.read_roll()
    
    if roll_zero is None or roll_wit_zero is None:
        print("ERROR: Could not read roll at zero")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    ref_roll_servo = roll_zero
    ref_roll_wit = roll_wit_zero
    print("    Roll servo at zero: " + "%.2f" % roll_zero)
    print("    Roll WitMotion at zero: " + "%.2f" % roll_wit_zero)
    print("      Std Dev: " + "%.3f" % roll_std_zero + ", Range: " + "%.2f" % roll_min_zero + " to " + "%.2f" % roll_max_zero)
    print("    (Reference point set - SAME FOR ALL 5 RUNS)")
    print("")
    
    # RUN 5 TIMES WITH SAME REFERENCE
    for roll_run in range(1, 6):
        print("\nROLL CALIBRATION TEST - RUN " + str(roll_run) + "/5:")
        print("="*60 + "\n")
        
        results_roll = []
        
        for i, gimbal_angle in enumerate(test_angles, 1):
            print("[" + str(i) + "/7] Roll angle: " + str(gimbal_angle))
            servo_units = int(SERVO_CENTER + (gimbal_angle / 0.29))
            if not move_roll_servo(port_handler, packet_handler, servo_units):
                continue
            
            time.sleep(2)
            
            # MICRO-ALIGN using pure 2x2 method
            micro_align_dual_pure(port_handler, packet_handler, wit_reader, coupling_matrix)
            
            time.sleep(2)
            
            roll_servo_angle = read_roll_servo_position(port_handler, packet_handler)
            if roll_servo_angle is None:
                continue
            print("    Servo: " + "%.2f" % roll_servo_angle)
            
            roll_angle, roll_std, roll_min, roll_max, _ = wit_reader.read_roll()
            if roll_angle is None:
                continue
            print("    WitMotion: " + "%.2f" % roll_angle)
            print("      Std Dev: " + "%.3f" % roll_std + ", Range: " + "%.2f" % roll_min + " to " + "%.2f" % roll_max)
            
            servo_change = roll_servo_angle - ref_roll_servo
            wit_change = roll_angle - ref_roll_wit
            error = abs(abs(servo_change) - abs(wit_change))
            
            print("    Error: " + "%.2f" % error)
            print("")
            
            results_roll.append([gimbal_angle, roll_servo_angle, roll_angle, servo_change, wit_change, error, roll_std, roll_min, roll_max])
        
        csv_file = get_next_filename("gimbal_calibration_roll_run")
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Angle", "Servo", "WitMotion", "Servo_Change", "WitMotion_Change", "Error", "Std_Dev", "Min", "Max"])
            writer.writerows(results_roll)
        
        print("Saved: " + csv_file)
    
    # ===== YAW - 5 RUNS WITH SAME REFERENCE =====
    print("\n" + "="*60)
    print("YAW CALIBRATION - 5 RUNS (SAME REFERENCE)")
    print("="*60)
    
    print("\nMoving all to 516...")
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    print("Warming up Yaw servo (4 cycles)...\n")
    for cycle in range(4):
        move_yaw_servo(port_handler, packet_handler, 516 - warm_offset)
        time.sleep(1)
        move_yaw_servo(port_handler, packet_handler, 516 + warm_offset)
        time.sleep(1)
        print("  Warm-up cycle " + str(cycle+1) + "/4 done")
    
    move_servo(port_handler, packet_handler, 516)
    move_roll_servo(port_handler, packet_handler, 516)
    move_yaw_servo(port_handler, packet_handler, 516)
    time.sleep(2)
    
    if not auto_align(port_handler, packet_handler, wit_reader):
        print("ERROR: Auto alignment failed")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    time.sleep(1)
    
    if not wit_reader.calibrate_offsets(num_samples=3):
        print("ERROR: Could not calibrate offsets")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    yaw_zero = read_yaw_servo_position(port_handler, packet_handler)
    yaw_wit_zero, yaw_std_zero, yaw_min_zero, yaw_max_zero, _ = wit_reader.read_yaw()
    
    if yaw_zero is None or yaw_wit_zero is None:
        print("ERROR: Could not read yaw at zero")
        port_handler.closePort()
        wit_reader.stop()
        return
    
    ref_yaw_servo = yaw_zero
    ref_yaw_wit = yaw_wit_zero
    print("    Yaw servo at zero: " + "%.2f" % yaw_zero)
    print("    Yaw WitMotion at zero: " + "%.2f" % yaw_wit_zero)
    print("      Std Dev: " + "%.3f" % yaw_std_zero + ", Range: " + "%.2f" % yaw_min_zero + " to " + "%.2f" % yaw_max_zero)
    print("    (Reference point set - SAME FOR ALL 5 RUNS)")
    print("")
    
    # RUN 5 TIMES WITH SAME REFERENCE
    for yaw_run in range(1, 6):
        print("\nYAW CALIBRATION TEST - RUN " + str(yaw_run) + "/5:")
        print("="*60 + "\n")
        
        results_yaw = []
        
        for i, gimbal_angle in enumerate(test_angles, 1):
            print("[" + str(i) + "/7] Yaw angle: " + str(gimbal_angle))
            servo_units = int(SERVO_CENTER + (gimbal_angle / 0.29))
            if not move_yaw_servo(port_handler, packet_handler, servo_units):
                continue
            
            time.sleep(2)
            
            # MICRO-ALIGN using pure 2x2 method
            micro_align_dual_pure(port_handler, packet_handler, wit_reader, coupling_matrix)
            
            time.sleep(2)
            
            yaw_servo_angle = read_yaw_servo_position(port_handler, packet_handler)
            if yaw_servo_angle is None:
                continue
            print("    Servo: " + "%.2f" % yaw_servo_angle)
            
            yaw_angle, yaw_std, yaw_min, yaw_max, _ = wit_reader.read_yaw()
            if yaw_angle is None:
                continue
            print("    WitMotion: " + "%.2f" % yaw_angle)
            print("      Std Dev: " + "%.3f" % yaw_std + ", Range: " + "%.2f" % yaw_min + " to " + "%.2f" % yaw_max)
            
            servo_change = yaw_servo_angle - ref_yaw_servo
            wit_change = yaw_angle - ref_yaw_wit
            
            servo_change_circular = circular_difference(yaw_servo_angle, ref_yaw_servo)
            wit_change_circular = circular_difference(yaw_angle, ref_yaw_wit)
            error = abs(abs(servo_change_circular) - abs(wit_change_circular))
            
            print("    Error: " + "%.2f" % error)
            print("")
            
            results_yaw.append([gimbal_angle, yaw_servo_angle, yaw_angle, servo_change_circular, wit_change_circular, error, yaw_std, yaw_min, yaw_max])
        
        csv_file = get_next_filename("gimbal_calibration_yaw_run")
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Angle", "Servo", "WitMotion", "Servo_Change", "WitMotion_Change", "Error", "Std_Dev", "Min", "Max"])
            writer.writerows(results_yaw)
        
        print("Saved: " + csv_file)
    
    print("\n" + "="*60)
    print("CONSISTENCY TEST COMPLETE!")
    print("="*60)
    print("")
    
    port_handler.closePort()
    wit_reader.stop()

if __name__ == "__main__":
    main()
