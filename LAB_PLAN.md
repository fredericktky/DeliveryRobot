# 1-hour lab plan

Everything is recorded by `tools/lab_test.py` into `logs/lab/<date-time>/` (raw logs, readable timelines,
`SUMMARY.md` with pass/fail and your notes). Bring those folders back for debugging.
Details of every command and setting: [TESTING.md](TESTING.md).

## Before you go (or first 5 minutes at the lab)

- [ ] Laptop: Bluetooth on, `python -c "import bleak"` works (run Python **without `-I`**).
- [ ] Robot charged, ping-pong ball, the 3 colour cards (red, green, blue), the map laid flat.
- [ ] **Flash the lab firmware** (it has `mode`, `auto <leg>`, `craw`/`cref`/`cdist`):
  battery OFF, USB in, hold BOOT + tap RESET, then from `DeliveryRobot/`:

  ```sh
  ~/.platformio/penv/Scripts/pio.exe run
  B=.pio/build/esp32-s3-devkitc-1
  ~/.platformio/penv/Scripts/python.exe ~/.platformio/packages/tool-esptoolpy/esptool.py \
    --chip esp32s3 --port COM3 --baud 115200 --before default_reset --after hard_reset --no-stub \
    write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB \
    0x0000 $B/bootloader.bin 0x8000 $B/partitions.bin \
    0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
    0x10000 $B/firmware.bin
  ```
  Tap RESET, battery ON.

All commands below run from the repo root.

## Schedule

| Time | What | Command |
|---|---|---|
| 0–5 | Flash (above), then link, IR sensors, mode button | `python DeliveryRobot/tools/lab_test.py --only link sensors mode_button` |
| 5–15 | **Colour calibration** (robot on the top line, sensor over a colour square) | `python DeliveryRobot/tools/calibrate_color.py --push` |
| 15–18 | Check the cards with the new values | `python DeliveryRobot/tools/lab_test.py --only colour_static` |
| 18–25 | Basic task: regression + one colour card run | `python DeliveryRobot/tools/lab_test.py --only basic_force1 basic_colour` |
| 25–40 | Advanced legs on their own (tune here) | `python DeliveryRobot/tools/lab_test.py --only adv_leg_ball adv_leg_station3 adv_leg_station1` |
| 40–55 | Advanced full runs (+ cards if calibration was good) | `python DeliveryRobot/tools/lab_test.py --only adv_full_131 adv_full_312 adv_cards` |
| 55–60 | Save: calibration into the source, write down tuned values | see "End of session" |

To keep all tests in one session folder, add `--session DeliveryRobot/logs/lab/<the folder from the first run>`
to the later commands. If time is short, skip `adv_leg_station1` and `adv_cards` first.

## Colour calibration (5–15 min)

1. Robot in manual, battery on, on the top line with the colour sensor over colour square 1.
2. `calibrate_color.py --push` asks for WHITE (paper next to the squares), RED, GREEN, BLUE, BLACK (skip with `s`).
   Press Enter, then **slide the card slowly** under the sensor for the 4 s.
3. Read the report:
   - **Replay** should be ~100% for every card. Below ~95% for GREEN/BLUE: shield the sensor from room light
     (black shroud / tape), keep the height steady, record again.
   - **Brightness** must say cards and white are separated (gives `CARD_MAX_C`). The old calibration had white
     at ~4,000, but the last runs read ~9,000 on white, so the old values are likely wrong here.
4. `--push` makes the values live (lost on reset). Check with `lab_test.py --only colour_static`.
5. Happy? `python DeliveryRobot/tools/calibrate_color.py --analyse DeliveryRobot/logs/calib/<folder>/samples.json --apply`
   writes them into the source (flash later to keep them). Everything is saved under `logs/calib/<folder>/`.

## While testing

At each test prompt: **Enter** run · **s** skip · **q** quit · **t name value** change a setting first
(e.g. `t tfwdms 260`) · **c command** send anything (e.g. `c grip open`). After a run: **r** run again,
**t …** tweak and run again, **Enter** next test. Type a short note on what you saw every time: it goes into the summary.

Catch the robot if it leaves the track; the script always stops it at the end (also on Ctrl+C).

## What to tune when it goes wrong

| Symptom | Setting (start value) | Direction |
|---|---|---|
| Snakes while reversing (`Reverse: realign` back and forth) | `rkp` (25) | lower, 0 = straight back + realign only |
| Drifts off while reversing, `Reverse lost line` / `Back timeout` | `backspd` (135), `rkp` | slower; `rkp` a little higher |
| Reversing stops on the end bar itself | `bback` / `sback` (250) | higher |
| After the T, the 90° turn ends off the line or on the old line | `tfwdms` (220) | wheels must be over the line: more if it turns early, less if late |
| Turns into a station branch too early / late | `fwdms` (180) | higher = turns later |
| Corner missed or turned too early | `cfwdms` (150), `cornreads` (3) | more `cfwdms` = turns later |
| Pivot stalls | `lostspin`, `turnspd` (170) | higher |
| Card not read / wrong colour | recalibrate; `cardmax`, `cdist`, `creads` (3) | see the `K` lines in the log |
| Counts a station twice / misses one | `jgap` (350), `jreads` (2) | higher `jgap` = fewer double counts |

## End of session (55–60 min)

- [ ] `calibrate_color.py --analyse … --apply` if not done yet.
- [ ] Write the tweaks that worked into `SUMMARY.md` (or tell Claude/Gemini "use the values from session X"):
      every `tweak:` line is already in the logs.
- [ ] Copy the `DeliveryRobot/logs/lab/<session>/` and `logs/calib/<folder>/` folders somewhere safe
      (`logs/` is not committed to git).
