## 2. Automated Warm-up Routine

### Description

`GT_eye_calibration_movements` performs an automated warm-up routine for the robotic eye/head mechanism using two Dynamixel motors for pitch and yaw.

The routine successively commands large angular movements following:

* An infinity (`∞`) pattern in the positive direction.
* A clockwise circular pattern.
* An infinity (`∞`) pattern in the negative direction.
* A counter-clockwise circular pattern.

The mechanism returns to the center position between each movement sequence.

### Requirements

#### Hardware

* Two Dynamixel motors.
* Dynamixel USB interface connected to the computer.

#### Software

* Python 3.
* `dynamixel_sdk` Python library.

### Communication Parameters

The communication parameters are defined directly in the code:

```python
DEVICENAME = 'COM3'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
```

If the Dynamixel USB interface is assigned to another COM port, `DEVICENAME` must be modified accordingly.

### Motor Configuration

```python
SERVO_ID_PITCH = 2
SERVO_ID_YAW = 1
```

The code uses:

* **Motor ID 2** for pitch.
* **Motor ID 1** for yaw.

The center position is defined as:

```python
SERVO_CENTER = 512
```

The position resolution used for the conversion from degrees to Dynamixel position units is:

```python
RESOLUTION = 0.29
```

The target position is calculated from the commanded angle using:

```python
position = SERVO_CENTER + (angle / RESOLUTION)
```

### Warm-up Movements

The routine consists of four movement sequences.

#### 1. Infinity — Positive Direction

The mechanism moves through four positions:

```text
(40°, -30°) → (-40°, 30°) → (40°, 30°) → (-40°, -30°)
```

A 1.5-second waiting time is applied at each position.

#### 2. Circle — Clockwise

The mechanism moves through:

```text
(30°, 0°) → (0°, 30°) → (-30°, 0°) → (0°, -30°)
```

#### 3. Infinity — Negative Direction

The mechanism moves through:

```text
(-40°, 30°) → (40°, -30°) → (-40°, -30°) → (40°, 30°)
```

#### 4. Circle — Counter-clockwise

The mechanism moves through:

```text
(30°, 0°) → (0°, -30°) → (-30°, 0°) → (0°, 30°)
```

The mechanism returns to `(0°, 0°)` after each sequence.

### Timing

The default waiting time between positions is:

```python
wait_sec = 1.5
```

The center position uses a 2-second waiting time.

This value can be modified in the `servo.move_both()` calls if a different stabilization time is required.

### Launch Command

Open a terminal in the directory containing the script and run:

```bash
python GT_eye_calibration_movements
```

or, depending on the Python installation:

```bash
python3 GT_eye_calibration_movements
```

The script can be executed directly because it contains a main execution block:

```python
if __name__ == "__main__":
    warmup()
```

### Expected Behavior

When launched, the script:

1. Opens the Dynamixel communication port.
2. Moves both motors to the center position.
3. Executes the positive infinity movement.
4. Returns to center.
5. Executes the clockwise circle.
6. Returns to center.
7. Executes the negative infinity movement.
8. Returns to center.
9. Executes the counter-clockwise circle.
10. Returns to center.
11. Closes the Dynamixel communication port.

The terminal displays the commanded pitch and yaw positions during execution.
