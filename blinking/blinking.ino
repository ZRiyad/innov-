#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Channel assignments
#define CH_LEFT_LOWER   1
#define CH_LEFT_UPPER   0
#define CH_RIGHT_LOWER  3
#define CH_RIGHT_UPPER  4

// Positions
#define LEFT_LOWER_CLOSE   360
#define LEFT_LOWER_OPEN    425

#define LEFT_UPPER_CLOSE   425
#define LEFT_UPPER_OPEN    330

#define RIGHT_LOWER_CLOSE  360
#define RIGHT_LOWER_OPEN   310

#define RIGHT_UPPER_CLOSE  350
#define RIGHT_UPPER_OPEN   425

// Timing
#define BLINK_INTERVAL_MS 2000
#define BLINK_HOLD_MS      150

unsigned long lastBlinkTime = 0;

void setup() {

  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(100);

  // Start with eyes open
  openEyes();

  lastBlinkTime = millis();

  Serial.println("Automatic blinking every 2 seconds.");
}

void loop() {

  unsigned long currentTime = millis();

  if (currentTime - lastBlinkTime >= BLINK_INTERVAL_MS) {

    closeEyes();

    delay(BLINK_HOLD_MS);

    openEyes();

    Serial.println("Blink.");

    lastBlinkTime = millis();
  }
}

void closeEyes() {

  pwm.setPWM(CH_LEFT_LOWER, 0, LEFT_LOWER_CLOSE);
  pwm.setPWM(CH_LEFT_UPPER, 0, LEFT_UPPER_CLOSE);

  pwm.setPWM(CH_RIGHT_LOWER, 0, RIGHT_LOWER_CLOSE);
  pwm.setPWM(CH_RIGHT_UPPER, 0, RIGHT_UPPER_CLOSE);
}

void openEyes() {

  pwm.setPWM(CH_LEFT_LOWER, 0, LEFT_LOWER_OPEN);
  pwm.setPWM(CH_LEFT_UPPER, 0, LEFT_UPPER_OPEN);

  pwm.setPWM(CH_RIGHT_LOWER, 0, RIGHT_LOWER_OPEN);
  pwm.setPWM(CH_RIGHT_UPPER, 0, RIGHT_UPPER_OPEN);
}