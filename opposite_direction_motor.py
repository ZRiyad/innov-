import time
from dynamixel_sdk import *

DEVICENAME = "COM3"      # change this
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

ID_1 = 1
ID_2 = 2
ID_3 = 3  # New motor for yaw

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36  # reads current motor position

CENTER = 512
UNITS_PER_DEGREE = 1023 / 300
DEGREES_PER_UNIT = 0.29297  # conversion factor

# Zero offsets for each motor
OFFSET_1 = 90   # Motor 1 zero is at +90 degrees
OFFSET_2 = -90  # Motor 2 zero is at -90 degrees
OFFSET_3 = 0    # Motor 3 zero is normal

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Enable torque + set speed
for dxl_id in [ID_1, ID_2, ID_3]:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MOVING_SPEED, 100)

def get_present_position(dxl_id, offset):
    """Read the current position of a motor in degrees"""
    present_pos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, dxl_id, ADDR_PRESENT_POSITION
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to read position from Motor {dxl_id}")
        return None
    
    # Convert to degrees relative to center, then subtract the offset
    degrees = (present_pos - CENTER) * DEGREES_PER_UNIT - offset
    return degrees

def move_table(pitch_deg, yaw_deg):
    # Motor 1: zero is at +90, so add 90 to command
    motor1_pos = CENTER + int((pitch_deg + OFFSET_1) * UNITS_PER_DEGREE)
    
    # Motor 2: zero is at -90, so subtract 90 from command (add negative offset)
    motor2_pos = CENTER - int((pitch_deg - OFFSET_2) * UNITS_PER_DEGREE)
    
    # Motor 3: Yaw control
    motor3_pos = CENTER + int((yaw_deg + OFFSET_3) * UNITS_PER_DEGREE)

    motor1_pos = max(0, min(1023, motor1_pos))
    motor2_pos = max(0, min(1023, motor2_pos))
    motor3_pos = max(0, min(1023, motor3_pos))

    packetHandler.write2ByteTxRx(portHandler, ID_1, ADDR_GOAL_POSITION, motor1_pos)
    packetHandler.write2ByteTxRx(portHandler, ID_2, ADDR_GOAL_POSITION, motor2_pos)
    packetHandler.write2ByteTxRx(portHandler, ID_3, ADDR_GOAL_POSITION, motor3_pos)

    time.sleep(3)

    actual_deg_1 = get_present_position(ID_1, OFFSET_1)
    actual_deg_2 = get_present_position(ID_2, OFFSET_2)
    actual_deg_3 = get_present_position(ID_3, OFFSET_3)

    print(f"\nPitch = {pitch_deg}° | Yaw = {yaw_deg}°")
    print(f"Motor {ID_1} (Pitch) → Target: {pitch_deg:6.2f}° | Actual: {actual_deg_1:6.2f}°")
    print(f"Motor {ID_2} (Pitch) → Target: {pitch_deg:6.2f}° | Actual: {actual_deg_2:6.2f}°")
    print(f"Motor {ID_3} (Yaw)   → Target: {yaw_deg:6.2f}° | Actual: {actual_deg_3:6.2f}°")

try:
    while True:
        pitch_str = input("\nEnter pitch angle in degrees: ")
        if pitch_str.lower() == "q":
            break
        
        yaw_str = input("Enter yaw angle in degrees: ")
        if yaw_str.lower() == "q":
            break

        pitch = float(pitch_str)
        yaw = float(yaw_str)
        move_table(pitch, yaw)

finally:
    # return to center
    move_table(0, 0)

    # disable torque
    for dxl_id in [ID_1, ID_2, ID_3]:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

    portHandler.closePort()