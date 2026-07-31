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

## Telemetry (Phase 4)

Off by default. `SerialServo(expect_ack=True)` turns it on by sending `T1`;
`T0` turns it back off.

| line | direction | meaning |
|---|---|---|
| `T1` / `T0` | host → board | enable / disable acknowledgements |
| `A,<seq>,<left_us>,<right_us>,<millis>` | board → host | a command was applied |
| `W,<millis>` | board → host | the watchdog stopped the wheels |

`seq` counts commands the **board** accepted, so a jump tells the host that
commands went unacknowledged. The `*_us` values are **post-clamp** — that is how
host-side clamping gets checked against what the board actually did, rather than
against what the host believes it asked for. `W` fires once on the transition,
not repeatedly while stopped.

It is opt-in because a host that never reads would let the ack stream fill the
USB buffer and stall `loop()`. Silence stays the default, so `run_local.py` and
the Wokwi walkthrough below are unaffected.

## Getting the board into WSL

The Arduino enumerates on **Windows** as a COM port; WSL cannot see it without
[usbipd-win](https://github.com/dorssel/usbipd-win). From an **admin** PowerShell:

```powershell
winget install usbipd
usbipd list                          # find the Arduino's BUSID
usbipd bind   --busid <BUSID>        # once per board, persists across reboots
usbipd attach --wsl --busid <BUSID>  # once per WSL session
```

It then appears in WSL as `/dev/ttyACM0` (or `/dev/ttyUSB0` for CH340 clones),
and `SerialServo.find_arduino_port()` picks it up. You may need
`sudo usermod -aG dialout $USER` plus a new login for permissions.

Alternatively, skip WSL entirely and run the host side from Windows against
`COM<n>` — the serial path has no Linux dependency.

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
