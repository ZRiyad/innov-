#!/usr/bin/env python3
"""
Simple Automated Warm-up (Echauffement)
- Infinity sign (∞) = 4 corners: top-left → bottom-right → top-right → bottom-left
- Circle = 4 quadrants
"""

import time
from dynamixel_sdk import *

DEVICENAME = 'COM3'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

SERVO_ID_PITCH = 2
SERVO_ID_YAW = 1
SERVO_CENTER = 512
RESOLUTION = 0.29

class Servo:
    def __init__(self):
        self.port = PortHandler(DEVICENAME)
        self.pkt = PacketHandler(PROTOCOL_VERSION)
        
        if not self.port.openPort():
            raise Exception(f"Failed to open {DEVICENAME}")
        if not self.port.setBaudRate(BAUDRATE):
            raise Exception("Failed to set baud")
        
        print(f"✓ Servo on {DEVICENAME}\n")
    
    def move_both(self, pitch_deg, yaw_deg, wait_sec=1.5):
        """Move to angle in degrees"""
        pitch_units = int(SERVO_CENTER + (pitch_deg / RESOLUTION))
        yaw_units = int(SERVO_CENTER + (yaw_deg / RESOLUTION))
        
        self.pkt.write2ByteTxRx(self.port, SERVO_ID_PITCH, 30, pitch_units)
        self.pkt.write2ByteTxRx(self.port, SERVO_ID_YAW, 30, yaw_units)
        
        print(f"  → Pitch: {pitch_deg:+6.1f}° | Yaw: {yaw_deg:+6.1f}°")
        time.sleep(wait_sec)
    
    def close(self):
        self.port.closePort()

def warmup():
    """Automated warm-up with big jumps"""
    
    print("\n" + "="*70)
    print("WARM-UP ROUTINE - BIG JUMPS")
    print("="*70 + "\n")
    
    servo = Servo()
    
    try:
        # CENTER
        print("Moving to center...\n")
        servo.move_both(0, 0, wait_sec=2)
        
        # ===== INFINITY SIGN (POSITIVE) =====
        print("\n" + "="*70)
        print("INFINITY SIGN (∞) - POSITIVE →")
        print("="*70 + "\n")
        
        infinity_pos = [
            (40, -30, "Top-Left"),
            (-40, 30, "Bottom-Right"),
            (40, 30, "Top-Right"),
            (-40, -30, "Bottom-Left"),
        ]
        
        for pitch, yaw, desc in infinity_pos:
            print(f"{desc}")
            servo.move_both(pitch, yaw, wait_sec=1.5)
        
        print("\n✓ Infinity sign (positive) complete")
        servo.move_both(0, 0, wait_sec=2)
        
        # ===== CIRCLE CLOCKWISE =====
        print("\n" + "="*70)
        print("CIRCLE - CLOCKWISE ↻")
        print("="*70 + "\n")
        
        circle_cw = [
            (30, 0, "Top"),
            (0, 30, "Right"),
            (-30, 0, "Bottom"),
            (0, -30, "Left"),
        ]
        
        for pitch, yaw, desc in circle_cw:
            print(f"{desc}")
            servo.move_both(pitch, yaw, wait_sec=1.5)
        
        print("\n✓ Circle (clockwise) complete")
        servo.move_both(0, 0, wait_sec=2)
        
        # ===== INFINITY SIGN (NEGATIVE) =====
        print("\n" + "="*70)
        print("INFINITY SIGN (∞) - NEGATIVE ←")
        print("="*70 + "\n")
        
        infinity_neg = [
            (-40, 30, "Top-Right"),
            (40, -30, "Bottom-Left"),
            (-40, -30, "Top-Left"),
            (40, 30, "Bottom-Right"),
        ]
        
        for pitch, yaw, desc in infinity_neg:
            print(f"{desc}")
            servo.move_both(pitch, yaw, wait_sec=1.5)
        
        print("\n✓ Infinity sign (negative) complete")
        servo.move_both(0, 0, wait_sec=2)
        
        # ===== CIRCLE COUNTER-CLOCKWISE =====
        print("\n" + "="*70)
        print("CIRCLE - COUNTER-CLOCKWISE ↺")
        print("="*70 + "\n")
        
        circle_ccw = [
            (30, 0, "Top"),
            (0, -30, "Left"),
            (-30, 0, "Bottom"),
            (0, 30, "Right"),
        ]
        
        for pitch, yaw, desc in circle_ccw:
            print(f"{desc}")
            servo.move_both(pitch, yaw, wait_sec=1.5)
        
        print("\n✓ Circle (counter-clockwise) complete")
        servo.move_both(0, 0, wait_sec=2)
        
        print("\n" + "="*70)
        print("WARM-UP COMPLETE!")
        print("="*70 + "\n")
        
    except Exception as e:
        print(f"✗ Error: {e}")
    finally:
        servo.close()
        print("✓ Servo closed\n")

if __name__ == "__main__":
    try:
        warmup()
    except Exception as e:
        print(f"✗ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()