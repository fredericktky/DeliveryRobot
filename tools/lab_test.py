"""
lab_test.py - guided track tests with full recording, for a timed lab session.

  python tools/lab_test.py                 # run the whole plan in order
  python tools/lab_test.py --list          # show the tests
  python tools/lab_test.py --only adv_full_131 basic_force1
  python tools/lab_test.py --from adv_leg_ball
  python tools/lab_test.py --session logs/lab/20261127-1400   # continue a session (appends)

For every test it prints where to put the robot, waits for Enter, sends the
setup + start commands, records telemetry (20 Hz), events and raw colour
readings, stops the robot when the run ends (always sends M, also on Ctrl+C),
checks the expected events in order, asks for a one-line note, and writes:

  logs/lab/<session>/<NN>_<test>[_tryK].log   raw log (same format as robot_link.py)
  logs/lab/<session>/<NN>_<test>[_tryK].txt   readable timeline (log_summary.py)
  logs/lab/<session>/SUMMARY.md               one section per test: result, events, note

At the prompt:  Enter = run   r = run again (after a run)   s = skip   q = quit
                t <name> <value> = "set" a value before the run (e.g. t cfwdms 180)
                c <command> = send any command (e.g. c get, c grip open)
Run Python without -I so bleak imports.
"""
import argparse
import asyncio
import datetime
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from rlink import RobotLink  # noqa: E402
import log_summary  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent

# Events that end a run (the robot is back in manual mode after them)
END_EVENTS = ("DONE", "ARRIVED", "Stopped", "Line lost", "lost line", "timeout", "No colour", "Cards:",
              "Manual (aborted)", "BLE lost", "mode 2 only", "set adv", "auto ball|")

# ------------------------------------------------------------------ the plan
# place   : where to put the robot (shown before the run)
# setup   : commands sent before the start command
# start   : command(s) that start the run ([] = just record, e.g. a manual check)
# seconds : max recording time
# expect  : event texts that must appear, in this order
# stop_after: (event text, seconds) - stop the robot this long after that event (leg tests)
# craw    : also record raw colour readings
TESTS = [
    dict(id="link", title="Link + firmware check",
         place="Robot anywhere, battery on, phone can stay connected.",
         setup=["ping", "get", "cref"], start=[], seconds=3, expect=["pong"],
         check="firmware"),
    dict(id="sensors", title="IR sensors, motors off",
         place="Robot on a straight with the line centred between the two middle sensors.",
         setup=[], start=[], seconds=4, expect=[], check="centred"),
    dict(id="mode_button", title="Run-mode button + OLED",
         place="Robot anywhere. During the 15 s recording press phone button 4 three times "
               "and watch the OLED (top right): BASIC/basic map -> BASIC/adv map -> ADVANCED.",
         setup=["set mode 1"], start=[], seconds=15, expect=["Mode 2", "Mode 0", "Mode 1"]),
    dict(id="colour_static", title="Colour cards under the sensor (after calibration)",
         place="Robot on the top line with the colour sensor over a colour square. During the 15 s "
               "recording show RED, then GREEN, then BLUE (about 4 s each), white in between.",
         setup=["set mode 1"], start=[], seconds=15, expect=[], craw=True, check="colours"),
    dict(id="basic_force1", title="Regression: basic task, force 1 (advanced map)",
         place="Robot in the START box, no cards needed.",
         setup=["set mode 1", "adv 0 0 0", "force 1", "grip open"], start=["A"], seconds=25,
         expect=["Corner 1", "Corner 2", "Branch 1 of 1", "On branch", "ARRIVED ball 1"]),
    dict(id="basic_colour", title="Basic task with a colour card",
         place="Robot in the START box. ONE card on colour square 1 (try RED, BLUE, GREEN on repeats).",
         setup=["set mode 1", "adv 0 0 0", "force 0", "grip open"], start=["A"], seconds=25,
         expect=["Colour", "Corner 1", "Corner 2", "On branch", "ARRIVED ball"], craw=True),
    dict(id="adv_leg_ball", title="Advanced leg: grab, reverse to the T, turn east",
         place="Ball on ball 1. Robot on the ball 1 branch, about 10 cm before the end bar, facing it.",
         setup=["set mode 2", "adv 1 3 1"], start=["auto ball"], seconds=20,
         expect=["AUTO from ball", "Got ball 1", "T: turn left", "To station"],
         stop_after=("To station", 1.5)),
    dict(id="adv_leg_station3", title="Advanced leg: count stations, turn in, place, reverse",
         place="Ball in the (closed) gripper. Robot on the vertical line heading north (up), about "
               "15 cm below station 3.",
         setup=["set mode 2", "adv 1 3 1"], start=["auto station"], seconds=25,
         expect=["AUTO from station", "Station pass 1 of 1", "On station branch", "Placed at station 3",
                 "T: turn left", "To base"], stop_after=("To base", 1.5)),
    dict(id="adv_leg_station1", title="Advanced leg: station 1 (passes 3 and 2 first)",
         place="As before: ball held, robot heading north about 15 cm below station 3.",
         setup=["set mode 2", "adv 1 1 2"], start=["auto station"], seconds=30,
         expect=["Station pass 1 of 3", "Station pass 2 of 3", "Station pass 3 of 3", "On station branch",
                 "Placed at station 1", "T: turn right", "To base"], stop_after=("To base", 1.5)),
    dict(id="adv_full_131", title="Advanced full run: ball 1 -> station 3 -> base 1",
         place="Robot in the START box, ball on ball 1, station 3 empty.",
         setup=["set mode 2", "adv 1 3 1"], start=["A"], seconds=75,
         expect=["Corner 1 R", "Corner 2 R", "Branch 1 of 1", "Got ball 1", "T: turn left", "To station",
                 "Corner 1 L", "Station pass 1 of 1", "Placed at station 3", "T: turn left", "To base",
                 "Corner 1 L", "DONE base 1"]),
    dict(id="adv_full_312", title="Advanced full run: ball 3 -> station 1 -> base 2 (longest)",
         place="Robot in the START box, ball on ball 3, station 1 empty.",
         setup=["set mode 2", "adv 3 1 2"], start=["A"], seconds=90,
         expect=["Branch 3 of 3", "Got ball 3", "T: turn left", "Corner 1 L", "Station pass 3 of 3",
                 "Placed at station 1", "T: turn right", "Corner 1 R", "DONE base 2"]),
    dict(id="adv_cards", title="Advanced full run with the 3 colour cards",
         place="Robot in the START box. Cards on squares 1-2-3 (ball / station / base: base card RED or BLUE).",
         setup=["set mode 2", "adv 0 0 0"], start=["A"], seconds=90,
         expect=["Card 1", "Card 2", "Card 3", "Got ball", "Placed at station", "DONE base"], craw=True),
]


# ------------------------------------------------------------------ helpers
async def ask(prompt):
    try:
        return (await asyncio.to_thread(input, prompt)).strip()
    except EOFError:
        return "q"


def check_expected(events, expect):
    """Expected texts must appear in order. Returns (passed, missing)."""
    i = 0
    missing = []
    msgs = [m for _, m in events]
    pos = 0
    for want in expect:
        found = next((k for k in range(pos, len(msgs)) if want in msgs[k]), None)
        if found is None:
            missing.append(want)
        else:
            pos = found + 1
            i += 1
    return not missing, missing


def extra_check(kind, lines):
    if kind == "firmware":
        params = ",".join(l for _, l in lines if l.startswith("P,"))
        need = ["mode=", "tfwdms=", "cdist=", "cardmax="]
        miss = [n for n in need if n not in params]
        return (not miss, "firmware has the lab build" if not miss else f"OLD FIRMWARE? missing {miss} - flash first")
    if kind == "centred":
        irs = [l.split(",")[3] for _, l in lines if l.startswith("T,") and l.count(",") >= 12]
        if not irs:
            return False, "no telemetry"
        share = irs.count("00000") / len(irs)
        common = max(set(irs), key=irs.count)
        return share > 0.8, f"{100 * share:.0f}% read 00000 (most common {common}); IR modules powered?"
    if kind == "colours":
        seen = []
        for _, l in lines:
            if l.startswith("K,"):
                lab = l.split(",")[6]
                if not seen or seen[-1][0] != lab:
                    seen.append([lab, 1])
                else:
                    seen[-1][1] += 1
        runs = " -> ".join(f"{a}x{n}" for a, n in seen if n >= 3)
        got = [a for a, n in seen if n >= 3]
        ok = all(c in got for c in ("RED", "GREEN", "BLUE"))
        return ok, f"sequence: {runs or 'no colour readings'}"
    return True, ""


class Session:
    def __init__(self, folder: pathlib.Path):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        self.summary = folder / "SUMMARY.md"
        if not self.summary.exists():
            self.summary.write_text(f"# Lab session {folder.name}\n\n", encoding="utf-8")

    def path_for(self, index, test_id):
        base = f"{index:02d}_{test_id}"
        k = 1
        while (self.folder / (base + ("" if k == 1 else f"_try{k}") + ".log")).exists():
            k += 1
        return self.folder / (base + ("" if k == 1 else f"_try{k}") + ".log")

    def add(self, text):
        with open(self.summary, "a", encoding="utf-8") as f:
            f.write(text + "\n")


async def run_test(robot, session, index, test, tweaks):
    log_path = session.path_for(index, test["id"])
    robot.set_log(log_path)
    robot.note(f"test {test['id']}: {test['title']}")
    for t in tweaks:
        robot.note(f"tweak: {t}")
    t_start = robot.now()
    stopped_by = "time limit"
    try:
        await robot.send("tel 20")
        if test.get("craw"):
            await robot.send("craw on")
        for cmd in test["setup"]:
            await robot.send(cmd)
        for cmd in tweaks:
            await robot.send(cmd)
        t_go = robot.now()
        for cmd in test["start"]:
            await robot.send(cmd)
        deadline = t_go + test["seconds"]
        stop_after = test.get("stop_after")
        stop_at = None
        while robot.now() < deadline:
            await asyncio.sleep(0.1)
            evs = robot.events_since(t_go)
            if stop_after and stop_at is None and any(stop_after[0] in m for _, m in evs):
                stop_at = robot.now() + stop_after[1]
            if stop_at and robot.now() >= stop_at:
                stopped_by = f"leg done ({stop_after[0]})"
                break
            if test["start"] and any(e in m for _, m in evs for e in END_EVENTS):
                await asyncio.sleep(1.0)       # a little more telemetry after the end
                stopped_by = "robot ended the run"
                break
            if not robot.connected:
                stopped_by = "BLE disconnected"
                break
    except (KeyboardInterrupt, asyncio.CancelledError):
        stopped_by = "Ctrl+C"
    finally:
        await robot.stop_robot()
        if robot.connected:
            for cmd in ("craw off", "tel off"):
                try:
                    await robot.send(cmd)
                except Exception:
                    pass
    lines = [(t, l) for t, l in robot.lines if t >= t_start]
    events = robot.events_since(t_start)
    return log_path, lines, events, stopped_by


def write_reports(session, index, test, log_path, lines, events, stopped_by, tweaks, note):
    ok, missing = check_expected(events, test.get("expect", []))
    xok, xmsg = extra_check(test.get("check"), lines) if test.get("check") else (True, "")
    passed = ok and xok
    rows = log_summary.parse(log_path)
    with open(log_path.with_suffix(".txt"), "w", encoding="utf-8") as f:
        if rows:
            log_summary.summarise(rows, out=f)
    t0 = next((t for t, m in events if "AUTO" in m), events[0][0] if events else 0)
    md = [f"## {index:02d} {test['title']} - {'PASS' if passed else 'FAIL'}",
          f"- when: {datetime.datetime.now():%H:%M:%S} | ended: {stopped_by} | log: `{log_path.name}`"]
    if tweaks:
        md.append(f"- tweaks: {', '.join(tweaks)}")
    if missing:
        md.append(f"- missing (in order): {', '.join(missing)}")
    if xmsg:
        md.append(f"- check: {xmsg}")
    if note:
        md.append(f"- note: {note}")
    md.append("- events: " + " | ".join(f"{t - t0:.1f}s {m}" for t, m in events
                                         if not m.startswith(("telemetry", "colour raw", "set "))))
    session.add("\n".join(md) + "\n")
    print(f"\n  => {'PASS' if passed else 'FAIL'}  ({stopped_by})")
    if missing:
        print(f"     missing: {', '.join(missing)}")
    if xmsg:
        print(f"     {xmsg}")
    return passed


async def main_async(args):
    tests = TESTS
    if args.only:
        tests = [t for t in TESTS if t["id"] in args.only]
    elif args.start_from:
        ids = [t["id"] for t in TESTS]
        tests = TESTS[ids.index(args.start_from):]
    folder = pathlib.Path(args.session) if args.session else \
        ROOT / "logs" / "lab" / datetime.datetime.now().strftime("%Y%m%d-%H%M")
    session = Session(folder)
    print(f"Session folder: {folder}\nConnecting...")

    def echo(line):
        if line.startswith("E,"):
            print("   ** " + line.split(",", 2)[2])
        elif line.startswith("R,"):
            print("   ref " + line[2:])

    robot = RobotLink(echo=echo)
    await robot.__aenter__()
    print("Connected.\n")
    try:
        for test in tests:
            index = TESTS.index(test) + 1
            tweaks = []
            while True:
                print(f"\n=== {index:02d} {test['id']}: {test['title']} ===")
                print(f"  PLACE: {test['place']}")
                if test["start"]:
                    print(f"  start: {' ; '.join(test['setup'] + test['start'])}   (max {test['seconds']} s)")
                if test.get("expect"):
                    print(f"  expect: {' -> '.join(test['expect'])}")
                a = await ask("  Enter = run, s = skip, q = quit, t <name> <v> = tweak, c <cmd> = send: ")
                if a == "q":
                    return
                if a == "s":
                    break
                if a.startswith("t "):
                    tweaks.append("set " + a[2:].strip())
                    print(f"  will send: {tweaks[-1]}")
                    continue
                if a.startswith("c "):
                    if not robot.connected:
                        await robot.__aenter__()
                    await robot.send(a[2:].strip())
                    await asyncio.sleep(0.5)
                    continue
                if not robot.connected:
                    print("  reconnecting...")
                    await robot.__aenter__()
                log_path, lines, events, stopped_by = await run_test(robot, session, index, test, tweaks)
                note = await ask("  What did you see? (one line, Enter to skip): ")
                write_reports(session, index, test, log_path, lines, events, stopped_by, tweaks, note)
                again = await ask("  r = run again (same tweaks), t... = tweak and run again, Enter = next test: ")
                if again.startswith("t "):
                    tweaks.append("set " + again[2:].strip())
                    continue
                if again == "r":
                    continue
                break
    finally:
        await robot.stop_robot()
        robot.set_log(None)
        await robot.__aexit__(None, None, None)
        print(f"\nSummary: {session.summary}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list the tests and exit")
    ap.add_argument("--only", nargs="+", metavar="ID", help="run only these tests")
    ap.add_argument("--from", dest="start_from", metavar="ID", help="start the plan at this test")
    ap.add_argument("--session", help="session folder to continue (default: new logs/lab/<date-time>)")
    args = ap.parse_args()
    if args.list:
        for i, t in enumerate(TESTS, 1):
            print(f"{i:02d} {t['id']:<18} {t['title']}")
        return 0
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nInterrupted - the robot was sent M (manual).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
