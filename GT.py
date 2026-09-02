#!/usr/bin/env python3
"""
Robot Head Controller - High-level interface
Integrates Dynamixel motors and WitMotion IMU
"""

import time
import json
from datetime import datetime
from dynamixel_controller import DynamixelController
from witmotion_imu import WitMotionIMU

class RobotHead:
    """Simple interface to control robot head and read sensors"""
    
    def __init__(self, dynamixel_port='/dev/ttyUSB0', imu_port='/dev/ttyUSB1'):
        """
        Initialize robot head
        
        Args:
            dynamixel_port: Dynamixel USB adapter port
            imu_port: IMU serial port
        """
        self.dynamixel = DynamixelController(port=dynamixel_port)
        self.imu = WitMotionIMU(port=imu_port)
        self.connected = False
    
    def connect(self):
        """Connect to both Dynamixel and IMU"""
        print("\n" + "="*70)
        print("CONNECTING TO ROBOT HEAD")
        print("="*70 + "\n")
        
        dyn_ok = self.dynamixel.connect()
        imu_ok = self.imu.connect()
        
        if not dyn_ok or not imu_ok:
            self.disconnect()
            return False
        
        # Wait for IMU to send data
        print("\nWaiting for IMU data...")
        if not self.imu.wait_for_data(timeout=5):
            print("✗ No IMU data received")
            self.disconnect()
            return False
        
        print("✓ IMU data OK\n")
        self.connected = True
        return True
    
    def disconnect(self):
        """Disconnect from both devices"""
        self.dynamixel.disconnect()
        self.imu.disconnect()
        self.connected = False
    
    def move_to(self, pitch_deg, yaw_deg):
        """Move head to target angles"""
        if not self.connected:
            print("✗ Robot not connected")
            return False
        
        print(f"Moving to Pitch={pitch_deg:+.1f}° Yaw={yaw_deg:+.1f}°")
        self.dynamixel.move_to(pitch_deg, yaw_deg)
        return True
    
    def read_motor_angles(self):
        """Read current motor angles from Dynamixel feedback"""
        if not self.connected:
            print("✗ Robot not connected")
            return None, None
        
        pitch, yaw = self.dynamixel.get_angles()
        return pitch, yaw
    
    def read_imu(self):
        """Read current IMU data"""
        if not self.connected:
            print("✗ Robot not connected")
            return None, None
        
        euler, quaternion = self.imu.get_both(timeout=2)
        return euler, quaternion
    
    def measure(self, pitch_deg, yaw_deg, settle_time=2):
        """
        Move to target angles and measure both motor and IMU positions
        
        Args:
            pitch_deg: Target pitch angle
            yaw_deg: Target yaw angle
            settle_time: Seconds to wait for motion to stabilize
        
        Returns:
            dict with all measurements
        """
        if not self.connected:
            print("✗ Robot not connected")
            return None
        
        print(f"\nMoving to Pitch={pitch_deg:+.1f}° Yaw={yaw_deg:+.1f}°...")
        self.move_to(pitch_deg, yaw_deg)
        
        print(f"Waiting {settle_time} seconds for stabilization...")
        time.sleep(settle_time)
        
        # Read measurements
        motor_pitch, motor_yaw = self.read_motor_angles()
        euler, quaternion = self.read_imu()
        
        if motor_pitch is None or motor_yaw is None:
            print("✗ Failed to read motor positions")
            return None
        
        if euler is None or quaternion is None:
            print("✗ Failed to read IMU data")
            return None
        
        # Build result
        result = {
            "commanded": {
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg
            },
            "dynamixel_feedback": {
                "pitch_deg": round(motor_pitch, 2),
                "yaw_deg": round(motor_yaw, 2)
            },
            "imu": {
                "euler_deg": {
                    "roll": round(euler['roll'], 2),
                    "pitch": round(euler['pitch'], 2),
                    "yaw": round(euler['yaw'], 2)
                },
                "quaternion": {
                    "w": round(quaternion['w'], 4),
                    "x": round(quaternion['x'], 4),
                    "y": round(quaternion['y'], 4),
                    "z": round(quaternion['z'], 4)
                }
            },
            "timestamp": datetime.now().isoformat()
        }
        
        return result
    
    def print_result(self, result):
        """Pretty-print measurement result"""
        if result is None:
            return
        
        print("\n" + "="*70)
        print("MEASUREMENT RESULT")
        print("="*70 + "\n")
        
        cmd = result['commanded']
        print(f"COMMANDED:")
        print(f"  Pitch: {cmd['pitch_deg']:+.1f}°")
        print(f"  Yaw:   {cmd['yaw_deg']:+.1f}°\n")
        
        dyn = result['dynamixel_feedback']
        print(f"DYNAMIXEL FEEDBACK:")
        print(f"  Pitch: {dyn['pitch_deg']:+.2f}°")
        print(f"  Yaw:   {dyn['yaw_deg']:+.2f}°\n")
        
        imu = result['imu']['euler_deg']
        print(f"IMU EULER ANGLES:")
        print(f"  Roll:  {imu['roll']:+.2f}°")
        print(f"  Pitch: {imu['pitch']:+.2f}°")
        print(f"  Yaw:   {imu['yaw']:+.2f}°\n")
        
        quat = result['imu']['quaternion']
        print(f"IMU QUATERNION:")
        print(f"  w: {quat['w']:+.4f}")
        print(f"  x: {quat['x']:+.4f}")
        print(f"  y: {quat['y']:+.4f}")
        print(f"  z: {quat['z']:+.4f}\n")
        
        print(f"Timestamp: {result['timestamp']}\n")
    
    def save_result(self, result, filename='measurement.json'):
        """Save measurement result to JSON file"""
        if result is None:
            return False
        
        try:
            with open(filename, 'w') as f:
                json.dump(result, f, indent=2)
            print(f"✓ Saved to {filename}")
            return True
        except Exception as e:
            print(f"✗ Failed to save: {e}")
            return False
