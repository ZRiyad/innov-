# Gimbal Consistency Calibration

## Description

This Python script validates the consistency of the robotic head gimbal by comparing the angular positions measured by the Dynamixel motors with the angular measurements provided by a WitMotion IMU.

The test is performed independently for the **Pitch, Roll, and Yaw axes**. For each axis, the script performs **5 independent runs** using the same reference position and the following commanded angles:

```text
-30°, -20°, -10°, 0°, +10°, +20°, +30°
```

The script also compensates for mechanical coupling between the Roll and Pitch axes using a measured **2×2 coupling matrix**.

At the end of each run, the measurements are automatically saved to a CSV file.

---

## Requirements

### Hardware

* Robotic head / gimbal with Dynamixel motors
* Three Dynamixel motors:

  * Servo ID 1
  * Servo ID 2
  * Servo ID 3
* WitMotion IMU
* USB connection to the Dynamixel interface
* Computer connected to the WitMotion system through UDP

### Software

* Python 3
* `dynamixel_sdk`

The script also uses the following standard Python libraries:

```text
socket
struct
time
csv
threading
os
```

These libraries are included with Python and do not require separate installation.

---

## Communication Parameters

The following parameters are defined at the beginning of the script:

| Parameter          |       Value | Description                  |
| ------------------ | ----------: | ---------------------------- |
| `DEVICENAME`       |      `COM3` | Dynamixel communication port |
| `BAUDRATE`         |   `1000000` | Dynamixel baud rate          |
| `PROTOCOL_VERSION` |       `1.0` | Dynamixel protocol           |
| `SERVO_ID`         |         `1` | Main servo ID                |
| `SERVO_ID_ROLL`    |         `2` | Roll servo ID                |
| `SERVO_ID_YAW`     |         `3` | Yaw servo ID                 |
| `SERVO_CENTER`     |       `516` | Dynamixel center position    |
| `UDP_HOST`         | `127.0.0.1` | Local UDP host               |
| `UDP_PORT`         |      `8782` | UDP communication port       |

The Dynamixel communication parameters must match the configuration of the connected motors and interface.

---

## WitMotion Configuration

The IMU data are received through UDP.

The script expects packets with:

* Header: `0x55 0x53`
* Packet length: 11 bytes
* Roll, Pitch and Yaw values encoded in the packet
* Checksum verification

The received raw values are converted into degrees.

The script applies the following convention:

* Roll → directly converted to degrees
* Pitch → sign inverted
* Yaw → directly converted to degrees

---

## Calibration and Coupling Compensation

Before performing the consistency tests, the script measures the mechanical coupling between the Roll and Pitch axes.

A small movement of approximately **0.5°** is commanded independently on:

1. Roll
2. Pitch

The corresponding WitMotion responses are measured and used to calculate the coupling matrix:

```text
R = a·uR + b·uP
P = c·uR + d·uP
```

where:

* `R` = Roll response
* `P` = Pitch response
* `uR` = Roll correction
* `uP` = Pitch correction

The resulting coefficients `(a, b, c, d)` are then used during the automatic micro-alignment procedure.

The coupling matrix is measured **once at the beginning of the test**.

---

## Automatic Alignment

Before each axis calibration, the script automatically aligns the system so that the WitMotion Roll and Pitch measurements are close to:

```text
Roll  = 0°
Pitch = 0°
```

with an accepted tolerance of:

```text
±0.1°
```

The alignment can perform up to **50 iterations**.

After reaching the tolerance, the script waits **5 seconds** and performs a stability check.

The alignment is considered stable if both Roll and Pitch remain within ±0.1°.

---

## Test Procedure

The complete test is performed separately for:

```text
Pitch
Roll
Yaw
```

For each axis:

1. All three motors are returned to the center position (`516`).
2. The tested motor is warmed up using 4 cycles.
3. The system is automatically aligned.
4. WitMotion offsets are calibrated.
5. A zero/reference position is recorded.
6. The seven test angles are commanded.
7. The system waits for the movement to settle.
8. The Roll/Pitch micro-alignment is performed when required.
9. Dynamixel and WitMotion angles are recorded.
10. The angular error is calculated.
11. The measurements are saved to a CSV file.

Each axis is tested **5 times using the same reference position**.

---

## Test Angles

The following angles are used for each axis:

```text
-30°
-20°
-10°
  0°
+10°
+20°
+30°
```

These values are defined by:

```python
test_angles = [-30, -20, -10, 0, 10, 20, 30]
```

---

## Servo Position Conversion

The Dynamixel position is converted into an angular value using:

```text
Angle = (Position - 516) × 0.29
```

Conversely, a desired test angle is converted into Dynamixel position units using:

```text
Position = 516 + Angle / 0.29
```

---

## Error Calculation

For Pitch and Roll, the error is calculated from the absolute difference between the magnitude of the Dynamixel movement and the magnitude of the WitMotion movement:

```text
Error = | |Servo Change| - |WitMotion Change| |
```

For Yaw, a circular-angle difference is first calculated to handle the ±180° angular boundary correctly.

---

## Output Files

For every completed run, the script automatically creates a new CSV file.

### Pitch

```text
gimbal_calibration_pitch_run_1.csv
gimbal_calibration_pitch_run_2.csv
...
gimbal_calibration_pitch_run_5.csv
```

### Roll

```text
gimbal_calibration_roll_run_1.csv
gimbal_calibration_roll_run_2.csv
...
gimbal_calibration_roll_run_5.csv
```

### Yaw

```text
gimbal_calibration_yaw_run_1.csv
gimbal_calibration_yaw_run_2.csv
...
gimbal_calibration_yaw_run_5.csv
```

If a file with the same name already exists, the script automatically increments the number to avoid overwriting previous results.

---

## CSV Data

Each CSV file contains the following columns:

```text
Angle
Servo
WitMotion
Servo_Change
WitMotion_Change
Error
Std_Dev
Min
Max
```

Where:

* `Angle` → commanded test angle
* `Servo` → measured Dynamixel angle
* `WitMotion` → measured IMU angle
* `Servo_Change` → change from the reference Dynamixel position
* `WitMotion_Change` → change from the reference IMU position
* `Error` → difference between the two measurements
* `Std_Dev` → standard deviation of the IMU samples
* `Min` → minimum IMU measurement
* `Max` → maximum IMU measurement

---

## Launch Command

Place the script in the desired directory and run:

```bash
python gimbal_consistency.py
```

or:

```bash
python3 gimbal_consistency.py
```

The script contains:

```python
if __name__ == "__main__":
    main()
```

so it can be launched directly from the terminal.

---

## Expected Behavior

When launched, the program:

1. Starts the WitMotion UDP receiver.
2. Connects to the Dynamixel motors.
3. Measures the Roll/Pitch coupling matrix.
4. Performs Pitch calibration.
5. Performs Roll calibration.
6. Performs Yaw calibration.
7. Repeats each axis test 5 times.
8. Saves the results as CSV files.
9. Closes the Dynamixel connection and stops the UDP receiver.

The terminal displays the measured servo angle, WitMotion angle, standard deviation, measurement range, and calculated error during the test.

---

## Important Notes

* The Dynamixel interface must be available on `COM3`, or `DEVICENAME` must be modified.
* The WitMotion UDP stream must be available on `127.0.0.1:8782`.
* The Dynamixel baud rate must be `1,000,000` for the current configuration.
* The script assumes the motors use the position conversion factor of `0.29°` per position unit.
* The test requires the mechanical system to be safely able to move through ±30°.
* The test can take a significant amount of time because of the warm-up cycles, stabilization delays, repeated measurements, and five runs per axis.
