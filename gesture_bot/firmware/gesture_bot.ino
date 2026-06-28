/*
 * gesture_bot firmware -- differential-drive base driven by two
 * continuous-rotation servos (the "wheels").
 *
 * Serial protocol (115200 baud), one command per line:
 *     "<left_us>,<right_us>\n"      e.g.  "1650,1350"
 * where *_us is a servo pulse width in microseconds:
 *     1500 = stop, 2000 = full forward, 1000 = full reverse.
 * The host (gesture_bot/actuators.py : SerialServo) already converts the
 * (linear, angular) Twist command into these two wheel microseconds.
 *
 * Failsafe: if no valid command arrives for WATCHDOG_MS, both wheels stop.
 * This protects the robot if the USB cable is pulled or the host crashes.
 *
 * Wiring:  left servo signal -> D9,  right servo signal -> D10.
 *          Servo V+ to a 5V supply (use an external supply for real motors,
 *          not the Arduino's 5V pin), all grounds common with the Arduino.
 *          Onboard LED (D13) lights while commands are being received.
 */
#include <Servo.h>

const uint8_t PIN_LEFT  = 9;
const uint8_t PIN_RIGHT = 10;
const unsigned long WATCHDOG_MS = 500;
const int STOP_US = 1500;
const int MIN_US  = 1000;
const int MAX_US  = 2000;

Servo wheelLeft;
Servo wheelRight;
unsigned long lastCommandMs = 0;

int clampUs(int v) {
  if (v < MIN_US) return MIN_US;
  if (v > MAX_US) return MAX_US;
  return v;
}

void stopWheels() {
  wheelLeft.writeMicroseconds(STOP_US);
  wheelRight.writeMicroseconds(STOP_US);
}

void setup() {
  Serial.begin(115200);
  wheelLeft.attach(PIN_LEFT);
  wheelRight.attach(PIN_RIGHT);
  pinMode(LED_BUILTIN, OUTPUT);
  stopWheels();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma > 0) {
      int leftUs  = clampUs(line.substring(0, comma).toInt());
      int rightUs = clampUs(line.substring(comma + 1).toInt());
      wheelLeft.writeMicroseconds(leftUs);
      wheelRight.writeMicroseconds(rightUs);
      lastCommandMs = millis();
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  // watchdog failsafe: no command recently -> stop
  if (millis() - lastCommandMs > WATCHDOG_MS) {
    stopWheels();
    digitalWrite(LED_BUILTIN, LOW);
  }
}
