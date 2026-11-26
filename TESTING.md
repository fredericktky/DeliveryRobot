# DeliveryRobot – testing & debugging guide

How we test and tune the robot on the track with live Bluetooth telemetry, what we found, and where to carry on.
Written so another person or coding agent can pick up the work without the original chat.

**Status (2026-11-26):** basic task on the **advanced map** reaches the right ball for `force 1`, `force 2` and `force 3` (all confirmed on the track).
The gripper holds a ball at 0° / 90° and closes on arrival by default (`gauto 1`; the default in `main.cpp` is set but
**not flashed yet**, it was only set live). Next: a real ball and colour card, then full basic runs. See [Next steps](#10-next-steps).

---

## 1. Hardware facts that matter for testing

| Item | Value / note |
|---|---|
| MCU | ESP32-S3 DevKitC-1, native USB, shows up as **COM3** (VID:PID 303A:1001) |
| Motors | ENA 4, IN1 5, IN2 6 (left) · IN3 7, IN4 15, ENB 16 (right), 20 kHz PWM, 8-bit |
| IR array | IR1..IR5 = GPIO 17, 18, 8, 9, 10, left → right, **1 = black** |
| Servo (gripper) | GPIO 13, 50 Hz |
| Colour sensor | TCS34725 on the right side, 2 cm above the track |
| Battery | IR modules and motors need the **battery on**; USB alone powers only the MCU |

### IR sensor layout ("4 + 1")
The board has 5 sensors, 23 mm apart. The line is 19 mm wide, so a line can never cover two neighbouring sensors.
The board is **shifted to the robot's left** so the line sits in the gap between IR3 and IR4:

```
 IR1        IR2        IR3   |▓▓▓|   IR4        IR5
 far left   edge L     inner L  line  inner R    edge R
 branch-    steer      steer          steer      steer
 only
```

* Centred on the line reads `00000` (both inner sensors white). This is normal, not "lost".
* IR1 only ever sees a **left branch** (or the box/crossbar). It is not used for steering.
* As built, the gap is 5–10 mm right of the robot's centre, so the robot rides that far left of the line (fine, the map allows ±2 cm).

Steering error (in sensor pitches): IR2 −1.5, IR3 −0.5, IR4 +0.5, IR5 +1.5, averaged over the black sensors.

### Known hardware quirks
* **Pivot speed below ~170 stalls** (125 did not turn at all). `lostspin` and `turnspd` are 170.
* **Motors can twitch during boot/reset/download mode**: the motor pins float until `setup()` runs.
  Hardware fix (not done yet): 10 kΩ pull-down from ENA and ENB to GND.
* The paper map is thin and wrinkled; the robot has got stuck on it once. A flatter map is coming.

---

## 2. The map (advanced map, Figure 2 of the problem statement)

Route for the **basic task** (all turns on the way to the ball):

```
START box ─ 20 cm ─ dashed 40 cm (1 cm gaps) ─ 20 cm ─┐ Corner 1 (right)
   (3 colour squares below the line, robot's right)    │
                              station 1 ── left branch ┤ (~10 cm after corner 1)
                              station 2 ── left branch ┤ (20 cm apart)
                              station 3 ── left branch ┤
 UNLOAD/base 2 ─── ball 3 ─── ball 2 ─── ball 1 ───────┘ Corner 2 (right)
                   (ball branches turn LEFT, 20 cm apart, 20 cm long, end in a 15 cm T crossbar)
```

What the sensors read at each feature (IR1..IR5):

| Feature | Pattern | Firmware treats it as |
|---|---|---|
| Centred on line | `00000` | go straight |
| Drift (one sensor) | `01000`, `00100`, `00010`, `00001` | steer back |
| Right corner strip | `00011` (IR4 **and** IR5) | corner |
| Left branch / station | `11000`, `11100`, `11110`, `01100` | cross straight / count |
| Start box edge, ball crossbar | `11111` | grace period / stop |

The basic map (Figure 1) is the same route without the stations: `set map 0`.

---

## 3. Build and flash

PlatformIO is not on PATH. From `DeliveryRobot/`:

```sh
~/.platformio/penv/Scripts/pio.exe run            # build only
```

**Flashing that works reliably on this board:**

1. **Battery OFF**, USB-C in. (With the battery on, the floating motor pins make flashing fail.)
2. Hold **BOOT**, tap **RESET**, release BOOT (download mode).
3. Flash with the ROM loader (no stub, 115200):

```sh
B=.pio/build/esp32-s3-devkitc-1
~/.platformio/penv/Scripts/python.exe ~/.platformio/packages/tool-esptoolpy/esptool.py \
  --chip esp32s3 --port COM3 --baud 115200 --before default_reset --after hard_reset --no-stub \
  write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0000 $B/bootloader.bin 0x8000 $B/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 $B/firmware.bin
```

4. Tap **RESET**, battery on.
5. Check the new firmware is running: `python tools/robot_link.py get` and look for a parameter you just added.

`pio run -t upload` works sometimes but often fails with *"No serial data received"*.

---

## 4. Talking to the robot: `tools/robot_link.py`

Connects over BLE (Nordic UART) to `ESP32-S3-Robot`. No Windows pairing needed, just Bluetooth on.
Needs `bleak` (installed in the user site-packages – run Python **without `-I`**).

```sh
python DeliveryRobot/tools/robot_link.py get                          # list tuning values
python DeliveryRobot/tools/robot_link.py "set kp 45"                  # change a value live
python DeliveryRobot/tools/robot_link.py --tel 20 -t 20 --quiet A     # start auto, log 20 s
python DeliveryRobot/tools/robot_link.py --tel 20 -t 20 --quiet "force 2" A
python DeliveryRobot/tools/robot_link.py --no-log M                   # ALWAYS after a run
```

> **Always send `M` after a run.** The user's phone is usually connected too. The robot only aborts
> auto when the *last* BLE link drops, so if the phone stays connected the robot keeps driving
> after the PC script exits.

Logs are written to `DeliveryRobot/logs/<date-time>.log` (git-ignored), one line per message: `pc_seconds,<message>`.

### Message types
* `T,ms,phase,ir,err,pid,L,R,branches,target,see,locked,C` – telemetry
  * `phase`: 0 idle/manual · 1 to branch · 2 turn forward · 3 turn spin · 4 to ball
  * `ir`: 5 bits IR1..IR5 · `L`,`R`: motor commands (−255..255) · `branches`: ball branches counted
* `E,ms,text` – events: `Corner 1`, `Branch 1 of 2`, `On branch`, `ARRIVED ball 2`, `Line lost`, ...
* `P,name=value,...` – parameters (sent on `get` and at every auto start)

### Reading a run quickly
Collapse repeated telemetry lines so each change of state shows once:

```sh
f=$(ls -t DeliveryRobot/logs/*.log | head -1)
grep -E ",(E|P)," $f
grep ",T," $f | cut -d, -f3-11 | \
  awk -F, '{printf "%s ph%s %s e=%s pid=%s L=%s R=%s br=%s\n",$1,$2,$3,$4,$5,$6,$7,$8}' | uniq -f2 -c
```

Telemetry is 20 Hz but the control loop runs every ~5 ms, so short patterns can fall between samples.
`err`/`pid` keep their last value while the robot pivots or drives straight; trust `L`/`R` there.

---

## 5. Commands and live parameters

Commands (BLE text): `A` auto · `M` manual/stop auto · `F B L R S` drive · `G` toggle gripper ·
`force 0..3` target ball (0 = use colour card) · `grip open|close|<deg>` · `set <name> <v>` · `get` ·
`tel on|off|<hz>` · `ping` · `stop`.

Live values are lost on reset. Copy good values into the tuning block at the top of `src/main.cpp`.

| Name | Default | What it does |
|---|---|---|
| `speed` | 155 | base driving PWM (~21 cm/s on paper) |
| `kp` / `ki` / `kd` | 40 / 0 / 40 | PID on the sensor error |
| `brake` | 35 | speed drop when an edge sensor sees the line |
| `lostspin` | 170 | pivot PWM at corners / drift-out (≤125 stalls) |
| `outerms` | 300 | all-white this soon after an edge sensor alone = drifted out → pivot back |
| `loststop` | 2000 | no black on any sensor this long = off the track → stop (`Line lost`) |
| `cfwdms` | 150 | drive forward this long after a corner strip before pivoting |
| `cornreads` | 3 | IR4+IR5 must both be black this many loops (noise filter) |
| `grace` | 700 | ms after start where the box outline is ignored (no pivots) |
| `jreads` / `jgap` | 2 / 350 | ball-branch confirm loops / min ms between branches |
| `map` | 1 | 0 basic map (count all left branches) · 1 advanced (count after `corners` corners) |
| `corners` / `cgap` | 2 / 1000 | right corners before ball branches / min ms between corners |
| `fwdspd` / `fwdms` | 130 / 180 | turn-in: forward speed / time before pivoting into the branch |
| `turnspd` / `turnmin` / `turnto` | 170 / 150 / 2500 | turn-in pivot speed / ignore sensors first ms / give up |
| `creads` | 3 | colour must match this many reads to lock |
| `manual` | 200 | manual driving PWM |
| `gopen` / `gclose` | 0 / 90 | gripper servo angles (servo moves when set) |
| `gauto` | 1 | close the gripper automatically on `ARRIVED` (0 = leave it open) |

---

## 6. How the line logic works (`followLine` in `src/main.cpp`)

1. **Steering:** PID on IR2..IR5. All white (`00000`) = centred → straight.
2. **Left branch:** IR2 together with IR1 or IR3 (two neighbours = not drift) → drive straight until IR1/IR2 clear.
   In `autoStep`, IR1 + IR2 black and IR5 white counts as a ball branch (only after 2 corners on `map 1`).
3. **Right corner:** IR4 **and** IR5 black for `cornreads` loops → hold the heading across the strip →
   when both clear: drive forward `cfwdms`, then pivot right until an **inner** sensor (IR3/IR4) finds the line.
   Each one logs `Corner n`. The edge sensors are ignored during the pivot (they sweep over the corner line early).
4. **Drift out:** an edge sensor alone, then all white within `outerms` → pivot back in place.
5. **Safety:** no black for `loststop` ms → stop. All five black (`11111`) on the way to the branches → stop.
6. **Start:** for `grace` ms, steer on inner sensors only or go straight; no pivots (the box edge looks like a corner).
7. **Turn-in:** at the target branch drive `fwdms` at `fwdspd`, pivot left at `turnspd` until IR3 finds the branch
   (only after IR1/IR2 saw it sweep in from the left, so the main line can't end the turn), follow it,
   stop when ≥4 sensors are black (crossbar) → `ARRIVED ball n` → manual mode.

---

## 7. Test ladder (do in order; each needs the robot placed by a person)

| # | Test | How | Pass when | Status |
|---|---|---|---|---|
| 1 | Link | `robot_link.py ping get` | `pong` and params | ✅ |
| 2 | Sensors, motors off | robot on line, `--tel 10 -t 3 ping` | centred = `00000` | ✅ |
| 3 | Sensor order | record 30 s while sliding the robot sideways across the line | `00001…10000` in order | ✅ |
| 4 | Route to ball 1 branch | START box, no card, `--tel 20 -t 20 A` | `Corner 1`, `Corner 2`, stops `No colour read!` | ✅ |
| 5 | Turn-in ball 1 | `"force 1" A` | `ARRIVED ball 1` | ✅ |
| 6 | Turn-in ball 2 | `"force 2" A` | `Branch 1 of 2`, `Branch 2 of 2`, `ARRIVED ball 2` | ✅ |
| 7 | Turn-in ball 3 | `"force 3" A` | `ARRIVED ball 3` **and the robot is on the ball 3 branch** | ✅ after the turn-in fix (confirmed on the track; an earlier run "arrived" at base 2) |
| 8 | Gripper | `grip <deg>` to find angles, then `set gopen/gclose`, try `G` | ball held firmly | ✅ 0° / 90° hold the ball (hand test) |
| 9 | Colour card | `force 0`, card on colour square 1 | `Colour RED -> B1` etc. in the log | ⬜ |
| 10 | Full basic run | `force 0`, each colour | correct ball each time, 3 runs in a row | ⬜ |
| 11 | Save values | copy live values into `src/main.cpp`, commit | | ⬜ |

Repeat 4–6 after any change to `followLine`.

### Reference timings (speed 155, paper map, from AUTO)
| Event | Time |
|---|---|
| Corner 1 | 4.0–4.5 s |
| Stations 1 / 2 / 3 | ~+0.7 / +1.6 / +2.6 s after corner 1 |
| Corner 2 | 7.0–7.6 s |
| Ball branch 1 / 2 | ~+1.0 / +2.1 s after corner 2 |
| Arrived (ball 1 / 2 / 3) | 9.6 s / 10.5 s / 13.7 s (the ball 3 run was slower overall: corner 1 at 6.2 s) |

Timings drift with battery level and the paper surface, so any timing windows must scale from the measured box-to-corner-1 time.

---

## 8. Debugging history (symptom → cause → fix)

| Symptom | Cause | Fix |
|---|---|---|
| IR always `00000` in manual | IR only read during auto | `readIR()` in manual too |
| IR `00000` after flashing | battery off → IR unpowered | battery on |
| Robot blind more than half the time | 19 mm line between 23 mm-pitch sensors | moved to the 4+1 layout (line between IR3/IR4) |
| Spins in place on straights | old lost-line pivot after an inner sensor | pivot only after an edge sensor |
| Stalled at corner, no turn | pivot PWM 125 too weak | `lostspin`/`turnspd` 170 |
| Auto kept running after the PC script ended | phone still connected | always send `M` after a run |
| Turned into station 3 | edge sensor alone read as drift → left pivot | branch guard, later the two-neighbour rule |
| Drove past corner 1, turned at the station T-bar | IR4-alone moment cancelled the corner memory; immediate in-place pivot | corner = IR4+IR5 together, latched; forward-then-pivot, exit on inner sensor |
| Slow drift before station 3 read as a branch, left the line | time-based branch rule | two-neighbour rule (drift can never light two neighbours) |
| Drove off the map for 13 s | all-white = "centred" | `loststop` 2000 ms |
| Run ended stuck at corner 2 | robot caught on the paper map | none (map issue) |
| `force 3`: pivoted a little, drove on to base 2, reported `ARRIVED ball 3` | turn-in ended when IR3 saw the **main** line again; base 2's box edge looks like the crossbar | turn-in ends only on IR3 **after** IR1/IR2 saw the branch during the pivot. Always check the robot is really on the branch, not just the `ARRIVED` event |

---

## 9. Ideas not built yet

* **Timing windows / confidence:** the route order is fixed and the timings are repeatable (table above).
  Use the time from leaving the box to corner 1 to estimate speed, then accept a feature only inside its expected
  window, accept weaker evidence inside it, and stop with e.g. `Missed corner 1` if a window passes with nothing.
* **Advanced task:** read all 3 colour cards (ball / station / base), pick up automatically, back out of the branch,
  left corners on the way back, place at the station, finish in base 1 or 2.

## 10. Next steps

1. The gripper now closes on `ARRIVED` by default (`gauto 1`). Not yet tested with a real ball on the ring:
   check the ball ends up inside the gripper; if not, add a short reverse/forward move at the stop.
3. Colour card runs (tests 9–10).
4. Copy tuned values into `src/main.cpp`, commit (no AI co-author trailers), and mirror with
   `git subtree push --prefix=DeliveryRobot delivery main` from the repo root.
