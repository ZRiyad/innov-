
# Automatic Eyelid Blinking

## Description

This Arduino program controls four eyelid servos through an **Adafruit PWM Servo Driver (PCA9685)**. It automatically performs a blink every 2 seconds by moving the four eyelid servos from their open positions to their closed positions and back.

The four servo channels correspond to:

* Left lower eyelid
* Left upper eyelid
* Right lower eyelid
* Right upper eyelid

The program starts with the eyes open and then continuously repeats the blinking sequence.

---

## Requirements

### Hardware

* Arduino board compatible with the `Wire` library
* Adafruit PWM Servo Driver (PCA9685)
* Four servo motors for the eyelids
* I²C connection between the Arduino and the PWM driver
* Servo power supply

### Software

* Arduino IDE
* `Wire` library
* `Adafruit_PWMServoDriver` library

---

## PWM Driver Configuration

The PCA9685 is initialized with the I²C address:

```cpp
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
```

The PWM frequency is configured to:

```cpp
pwm.setPWMFreq(50);
```

Therefore, the servo control frequency is **50 Hz**.

---

## Servo Channel Assignments

| Eyelid      | PCA9685 Channel |
| ----------- | --------------: |
| Left lower  |               1 |
| Left upper  |               0 |
| Right lower |               3 |
| Right upper |               4 |

These channel assignments are defined at the beginning of the program and should be modified if the physical servo connections are changed.

---

## Servo Positions

The open and closed positions are defined individually for each eyelid.

| Eyelid      | Closed Position | Open Position |
| ----------- | --------------: | ------------: |
| Left lower  |             360 |           425 |
| Left upper  |             425 |           330 |
| Right lower |             360 |           310 |
| Right upper |             350 |           425 |

The values correspond to the PWM counts sent to the PCA9685.

Because each eyelid mechanism has a different orientation, the open and closed values are not identical between servos.

---

## Timing Parameters

Two timing parameters control the blinking behavior:

```cpp
#define BLINK_INTERVAL_MS 2000
#define BLINK_HOLD_MS      150
```

### Blink interval

`BLINK_INTERVAL_MS = 2000 ms`

The program waits **2 seconds between blinks**.

### Blink hold time

`BLINK_HOLD_MS = 150 ms`

After closing the eyes, the program waits **150 ms** before opening them again.

Therefore, one blink consists of:

1. Close all four eyelids.
2. Hold the closed position for 150 ms.
3. Open all four eyelids.
4. Wait until the next 2-second interval.

---

## Program Operation

### Startup

During `setup()`:

1. Serial communication is initialized at **115200 baud**.
2. The PCA9685 driver is initialized.
3. The PWM frequency is set to 50 Hz.
4. The program waits 100 ms.
5. The eyes are moved to the open position.
6. The blink timer is initialized.
7. A message is printed to the Serial Monitor.

Expected message:

```text
Automatic blinking every 2 seconds.
```

---

## Automatic Blinking

The `loop()` function continuously checks the elapsed time using `millis()`.

When 2 seconds have elapsed since the previous blink:

```text
Eyes open
    ↓
Close all eyelids
    ↓
Wait 150 ms
    ↓
Open all eyelids
    ↓
Print "Blink."
    ↓
Restart 2-second timer
```

Each completed blink generates:

```text
Blink.
```

in the Serial Monitor.

---

## Functions

### `openEyes()`

Moves all four eyelids to their predefined open positions.

### `closeEyes()`

Moves all four eyelids to their predefined closed positions.

Both functions use:

```cpp
pwm.setPWM(channel, 0, position);
```

to command the corresponding servo through the PCA9685.

---

## Launch Procedure

1. Open the Arduino sketch in the Arduino IDE.
2. Connect the Arduino to the PCA9685.
3. Connect the four eyelid servos to the specified PCA9685 channels.
4. Upload the program to the Arduino.
5. Open the Serial Monitor.
6. Set the Serial Monitor baud rate to **115200 baud**.

The eyes should immediately move to the open position. The system will then automatically blink every 2 seconds.

---

## Expected Behavior

After startup:

```text
Automatic blinking every 2 seconds.
Blink.
Blink.
Blink.
...
```

The eyelids should:

* Start open.
* Close simultaneously when a blink is triggered.
* Remain closed for approximately 150 ms.
* Return to the open position.
* Repeat every 2 seconds.

---

## Important Notes

* The PWM position values are specific to the mechanical configuration of the eyelid system.
* If an eyelid moves in the wrong direction, check its open/closed position values and its mechanical orientation.
* The PCA9685 channels must correspond to the physical servo connections listed above.
* The program does not provide manual control or interactive angle input; blinking is fully automatic.
* The blink interval and closed-position duration can be modified using `BLINK_INTERVAL_MS` and `BLINK_HOLD_MS`.
