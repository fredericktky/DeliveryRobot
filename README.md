# DeliveryRobot

ESP32-S3 firmware for a dual-mode delivery robot: the basic task of AIROST IoT Project 1 (Dual-Mode Autonomous Delivery Robot).

The robot follows a black line with PID control and reads a colour card with a TCS34725 sensor (**red → ball 1, blue → ball 2, green → ball 3**). It counts the ball branches, turns into the right one, stops at the ball, and then switches itself to manual so you can grip the ball and drive it back.

**Testing, flashing and tuning on the track:** see [TESTING.md](TESTING.md) for the IR sensor layout, the reliable flash procedure, live telemetry, the test ladder and the debugging history. For a timed lab session use [LAB_PLAN.md](LAB_PLAN.md) and `tools/lab_test.py`.

## Hardware

| Part | GPIO |
|---|---|
| ESP32-S3 DevKitC-1 (octal PSRAM) | — |
| L298N motor driver | ENA 4, IN1 5, IN2 6, IN3 7, IN4 15, ENB 16 |
| 5-channel IR line sensor (left → right) | 17, 18, 8, 9, 10 |
| SSD1306 128×64 OLED (I²C `Wire`, 0x3C) | SDA 11, SCL 12 |
| TCS34725 colour sensor (I²C `Wire1`, 0x29) | SDA 1, SCL 2, LED 14 (optional), VIN 3V3 |
| Gripper servo | 13 |

Free GPIOs: 21, 38–42 (47/48 with caveats). Avoid 0, 3, 45, 46 (strapping), 19/20 (USB), and 26–37 (flash/PSRAM).

Mount the colour sensor on the **right** side, facing down, **2 cm** above the track, so it passes over the colour card. The calibration in `include/ColorSensor.h` was measured at that height.

## Build and flash

Open this folder in VS Code with the PlatformIO extension, or:

```
pio run -t upload
pio device monitor
```

The PlatformIO platform is pinned to `espressif32@6.8.1` (Arduino-ESP32 2.x), because the code uses the 2.x LEDC API.

## Phone controls (Adafruit Bluefruit Connect → Controller → Control Pad)

Connect to **ESP32-S3-Robot**:

| Button | Action |
|---|---|
| Arrows | Drive (manual, hold) |
| 1 | Manual (aborts an auto run) |
| 2 | Start auto run |
| 3 | Gripper open / close |
| 4 | Force target ball: colour → B1 → B2 → B3 |

## Live tuning and telemetry over Bluetooth

A PC can be connected at the same time as the phone. Install the BLE library once:

```
pip install bleak
```

```
python tools/robot_link.py get                        # list tuning values
python tools/robot_link.py "set kp 45" "set kd 50"    # change them live
python tools/robot_link.py --tel 20 -t 25 A           # start a run, log 25 s of telemetry
python tools/robot_link.py --tel 10 -t 0              # stream until Ctrl+C
```

Logs are saved to `logs/` (ignored by git). Values set live are lost on reset, so copy good ones into `src/main.cpp`.

**Commands** (from the PC tool or the app's UART screen): `F B L R S` drive · `A` auto · `M` manual · `G` gripper · `stop` · `force <0-3>` · `set <name> <value>` · `get` · `tel on|off|<hz>` · `ping`

**Tunable values:** `speed kp ki kd brake lostspin grace jreads jgap fwdspd fwdms turnspd turnmin turnto creads manual`

**Telemetry line:**

```
T,ms,phase,ir,err,pid,L,R,branches,target,see,locked,C
```

| Field | Meaning |
|---|---|
| `phase` | 0 manual/idle · 1 to branch · 2 turn (forward) · 3 turn (spin) · 4 to ball |
| `ir` | 5 sensor bits, left → right (1 = black) |
| `err`, `pid` | Line error and PID output |
| `L`, `R` | Motor commands (−255 to 255) |
| `branches`, `target` | Branches counted so far, and the target ball |
| `see`, `locked` | Colour seen now, and colour locked for this run (N R G B) |
| `C` | Colour sensor clear (brightness) count |

Event lines start with `E,`, and parameter lines with `P,`.

## Colour calibration

`include/ColorSensor.h` holds the reference r/g/b percentages for each surface. To re-calibrate, use the RGBSensorTest project (from the original AutonomousRobot2.0 repo). It shows live readings on the OLED. Measure each card, the floor and the line at the mounting height, and copy the values into `COLOR_REFS`. In manual mode, this firmware's OLED also shows a live `Sees:` line, for checking the mounted sensor.
