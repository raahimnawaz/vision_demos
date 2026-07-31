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
 * Telemetry (Phase 4), OFF by default:
 *     "T1\n"  enable,  "T0\n"  disable
 * While enabled, every applied command is acknowledged with
 *     "A,<seq>,<left_us>,<right_us>,<millis>\n"
 * and each watchdog stop emits "W,<millis>\n" once, on the transition.
 *
 * seq counts commands the DEVICE accepted, so the host can detect drops; the
 * *_us values are post-clamp, which is how host-side clamping gets verified
 * against what the board actually applied. millis() is the device clock, so a
 * host round-trip measurement never has to trust its own scheduling.
 *
 * Telemetry is opt-in because a host that never reads would otherwise let the
 * ack stream fill the USB buffer and stall this loop. Silence stays the
 * default; nothing changes for run_local.py or the Wokwi walkthrough.
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
bool telemetry = false;        // Phase 4 acks; see the protocol note above
bool stopped = true;           // watchdog state, so "W" fires on the edge only
unsigned long rxCount = 0;     // commands the device accepted

int clampUs(int v) {
  if (v < MIN_US) return MIN_US;
  if (v > MAX_US) return MAX_US;
  return v;
}

void stopWheels() {
  wheelLeft.writeMicroseconds(STOP_US);
  wheelRight.writeMicroseconds(STOP_US);
}

/*
 * Parse one pulse-width field, rejecting anything non-numeric.
 *
 * String::toInt() returns 0 for garbage, and 0 clamps to MIN_US -- so a line
 * corrupted in transit ("ab,cd", or a truncated frame) used to be applied as
 * FULL REVERSE on both wheels rather than being rejected. Pulse widths are
 * always positive here, so digits-only is the whole grammar.
 */
bool parseUs(const String &text, int &out) {
  if (text.length() == 0) return false;
  for (unsigned int i = 0; i < text.length(); i++) {
    if (!isDigit(text[i])) return false;
  }
  out = clampUs(text.toInt());
  return true;
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
    line.trim();

    // Not an early return: falling through keeps the watchdog below running on
    // every iteration, whatever the host sent.
    if (line.length() >= 2 && line.charAt(0) == 'T') {   // telemetry on/off
      telemetry = (line.charAt(1) == '1');
    } else {
      int comma = line.indexOf(',');
      int leftUs, rightUs;
      if (comma > 0 &&
          parseUs(line.substring(0, comma), leftUs) &&
          parseUs(line.substring(comma + 1), rightUs)) {
        wheelLeft.writeMicroseconds(leftUs);
        wheelRight.writeMicroseconds(rightUs);
        lastCommandMs = millis();
        stopped = false;
        digitalWrite(LED_BUILTIN, HIGH);
        rxCount++;

        if (telemetry) {
          // Post-clamp values, so the host can see what was actually applied
          // rather than what it believes it asked for.
          Serial.print("A,");
          Serial.print(rxCount);
          Serial.print(',');
          Serial.print(leftUs);
          Serial.print(',');
          Serial.print(rightUs);
          Serial.print(',');
          Serial.println(lastCommandMs);
        }
      }
      // A malformed line is ignored entirely: no motion, no ack, and no
      // refreshed watchdog, so a host sending only garbage still gets stopped.
    }
  }

  // watchdog failsafe: no command recently -> stop
  if (millis() - lastCommandMs > WATCHDOG_MS) {
    if (!stopped) {                             // report the edge, not the state
      stopWheels();
      digitalWrite(LED_BUILTIN, LOW);
      stopped = true;
      if (telemetry) {
        Serial.print("W,");
        Serial.println(millis());
      }
    }
  }
}
