# gesture_bot firmware (Phase 2)

Arduino firmware for the differential-drive base. The host
(`gesture_bot/actuators.py : SerialServo`) converts each `(linear, angular)`
command into two wheel-servo microseconds and streams them over USB serial; this
sketch drives two continuous-rotation servos accordingly, with a watchdog failsafe.

## Serial protocol
`115200` baud, one line per command:
```
<left_us>,<right_us>\n        e.g.  1650,1350
```
`*_us` = servo pulse width in microseconds: **1500 = stop, 2000 = full forward,
1000 = full reverse**. Values are clamped to `[1000, 2000]` on the device.

**Failsafe:** if no valid command arrives for 500 ms, both wheels stop (handles a
yanked cable or a crashed host). The onboard LED (D13) is lit while commands flow.

## Wiring
| signal | Arduino pin |
|---|---|
| left wheel servo (PWM) | D9 |
| right wheel servo (PWM) | D10 |
| servo V+ | 5V supply* |
| servo GND | GND (common) |

\* For real motors use a **separate 5V supply** for the servos (not the Arduino's
5V pin) with grounds tied together — servos draw more than USB can safely give.

## Test it in Wokwi (no parts needed)
1. Go to <https://wokwi.com> → **New Project** → Arduino Uno.
2. Paste `gesture_bot.ino` into the code tab and the contents of
   `wokwi/diagram.json` into the `diagram.json` tab.
3. Click **▶**. Open the **Serial Monitor**, type `2000,2000` ⏎ — both servos
   spin "forward"; `1500,1500` stops them; `2000,1000` spins them opposite
   (turning in place). Stop typing for >0.5 s and the watchdog stops them.

## Flash a real Arduino
With [`arduino-cli`](https://arduino.github.io/arduino-cli/):
```bash
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:uno gesture_bot.ino
arduino-cli upload  -b arduino:avr:uno -p /dev/cu.usbmodemXXXX gesture_bot.ino
```
Then run the live loop from the host:
```bash
cd ~/vision_demos/gesture_bot
python run_local.py --actuator serial          # auto-detects the port
# or: python run_local.py --actuator serial --serial-port /dev/cu.usbmodemXXXX
```
