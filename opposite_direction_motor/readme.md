# Manual Gimbal Control

## Description

This Python script provides a simple manual interface for controlling a three-motor robotic gimbal using Dynamixel motors.

The user enters a desired **Pitch** and **Yaw** angle through the terminal. The script converts these angles into Dynamixel position units, commands the three motors, waits for the movement to finish, and then reads back the actual motor positions.

The system uses:

* **Motor 1** for Pitch
* **Motor 2** for Pitch
* **Motor 3** for Yaw

Both Pitch motors are commanded simultaneously to produce the desired Pitch orientation.

---

## Requirements

### Hardware

* 3 Dynamixel motors
* Dynamixel communication interface
* Computer with a USB connection to the Dynamixel interface

### Software

* Python 3
* `dynamixel_sdk`

The script also uses the standard Python `time` library, which is included with Python.

Install the Dynamixel SDK if required:

```bash
pip install dynamixel-sdk
```

---

## Communication Parameters

The following parameters are defined in the script:

| Parameter          |     Value | Description                               |
| ------------------ | --------: | ----------------------------------------- |
| `DEVICENAME`       |    `COM3` | Dynamixel communication port              |
| `BAUDRATE`         | `1000000` | Communication baud rate                   |
| `PROTOCOL_VERSION` |     `1.0` | Dynamixel protocol version                |
| `CENTER`           |     `512` | Dynamixel center position                 |
| `DEGREES_PER_UNIT` | `0.29297` | Conversion from position units to degrees |
| `UNITS_PER_DEGREE` |    `3.41` | Conversion from degrees to position units |

If the Dynamixel interface is connected to another COM port, modify:

```python
DEVICENAME = "COM3"
```

---

## Motor Configuration

The three motors are identified by their Dynamixel IDs:

| Motor   |  ID | Function | Zero Offset |
| ------- | --: | -------- | ----------: |
| Motor 1 | `1` | Pitch    |      `+90°` |
| Motor 2 | `2` | Pitch    |      `-90°` |
| Motor 3 | `3` | Yaw      |        `0°` |

The offsets compensate for the mechanical zero positions of the motors.

```python
OFFSET_1 = 90
OFFSET_2 = -90
OFFSET_3 = 0
```

---

## Motor Parameters

Torque is enabled for all three motors when the program starts.

The moving speed is set to:

```python
100
```

The relevant Dynamixel control addresses are:

| Address | Function         |
| ------: | ---------------- |
|    `24` | Torque Enable    |
|    `30` | Goal Position    |
|    `32` | Moving Speed     |
|    `36` | Present Position |

---

## Angle Conversion

The script uses the Dynamixel position center:

```text
CENTER = 512
```

and the conversion factor:

```text
0.29297° / unit
```

The motor position is therefore calculated from the requested angle while taking the mechanical zero offset into account.

### Motor 1

```text
Position = 512 + (Pitch + 90) / 0.29297
```

### Motor 2

```text
Position = 512 - (Pitch - (-90)) / 0.29297
```

### Motor 3

```text
Position = 512 + (Yaw + 0) / 0.29297
```

The resulting positions are limited to the Dynamixel range:

```text
0 to 1023
```

---

## Launch

Run the Python script from a terminal:

```bash
python manual_gimbal_control.py
```

or:

```bash
python3 manual_gimbal_control.py
```

Make sure the Dynamixel interface is connected and that `DEVICENAME` corresponds to the correct COM port.

---

## User Input

Once the program starts, it asks for the desired Pitch angle:

```text
Enter pitch angle in degrees:
```

Then it asks for the desired Yaw angle:

```text
Enter yaw angle in degrees:
```

Enter the angles in degrees, for example:

```text
Enter pitch angle in degrees: 20
Enter yaw angle in degrees: -15
```

The program then moves the three motors to the corresponding positions.

---

## Quit Command

To stop the program, enter:

```text
q
```

at either the Pitch or Yaw input.

Before terminating, the script automatically:

1. Returns the gimbal to `(Pitch = 0°, Yaw = 0°)`.
2. Disables torque on all three motors.
3. Closes the Dynamixel communication port.

---

## Movement and Feedback

After sending a command, the script waits approximately **3 seconds** before reading the present motor positions.

The terminal then displays:

```text
Pitch = XX° | Yaw = XX°

Motor 1 (Pitch) → Target: XX.XX° | Actual: XX.XX°
Motor 2 (Pitch) → Target: XX.XX° | Actual: XX.XX°
Motor 3 (Yaw)   → Target: XX.XX° | Actual: XX.XX°
```

The `Target` value corresponds to the user-commanded angle, while `Actual` is calculated from the motor's present position.

---

## Example

For:

```text
Pitch = 20°
Yaw = -15°
```

the script commands:

* Motor 1 → Pitch = `20°`
* Motor 2 → Pitch = `20°`
* Motor 3 → Yaw = `-15°`

After the 3-second waiting period, the actual motor positions are read and displayed.

---

## Important Notes

* The three motors must have the IDs `1`, `2`, and `3`.
* The Dynamixel interface must use the configured baud rate of `1,000,000`.
* The current script assumes a Dynamixel center position of `512`.
* Motor 1 and Motor 2 use opposite mechanical zero offsets for Pitch.
* The script limits commanded positions to the range `0–1023`.
* Make sure the mechanical system can safely move through the requested Pitch and Yaw angles before running the test.
