"""
log_summary.py - turn a robot log (robot_link.py / lab_test.py) into a short,
readable timeline for debugging.

  python tools/log_summary.py logs/20261126-131648.log
  python tools/log_summary.py logs/lab/<session>/03_force1.log --all   # every IR change

Shows: commands sent, events (E), notes (#), and the telemetry compressed so
each change of phase / IR pattern / motor command appears once, with times
relative to the AUTO start. Phase names follow enum AutoPhase in src/main.cpp.
"""
import argparse
import pathlib
import sys

PHASES = {
    0: "idle", 1: "to-branch", 2: "turn-fwd", 3: "turn-spin", 4: "to-ball",
    5: "grab", 6: "back-to-T", 7: "to-station", 8: "station-branch", 9: "to-base",
}


def parse(path: pathlib.Path):
    rows = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = raw.split(",", 1)
        if len(parts) != 2:
            continue
        try:
            t = float(parts[0])
        except ValueError:
            continue
        rows.append((t, parts[1]))
    return rows


def summarise(rows, show_all=False, out=sys.stdout):
    # time zero = first "AUTO" event, else the first line
    t_auto = next((t for t, l in rows if l.startswith("E,") and ",AUTO" in l), rows[0][0] if rows else 0)
    last_key = None
    phase_times = []
    for t, line in rows:
        rel = t - t_auto
        if line.startswith(">,"):
            print(f"{rel:7.2f}  >> {line[2:]}", file=out)
        elif line.startswith("#,"):
            print(f"{rel:7.2f}  ## {line[2:]}", file=out)
        elif line.startswith("E,"):
            msg = line.split(",", 2)[2] if line.count(",") >= 2 else line
            print(f"{rel:7.2f}  ** {msg}", file=out)
        elif line.startswith("P,") and show_all:
            print(f"{rel:7.2f}  P  {line[2:]}", file=out)
        elif line.startswith("T,"):
            f = line.split(",")
            if len(f) < 13:
                continue
            phase = int(f[2]) if f[2].isdigit() else -1
            ir, err, pid, L, R, br = f[3], f[4], f[5], f[6], f[7], f[8]
            if not phase_times or phase_times[-1][1] != phase:
                phase_times.append((rel, phase))
            key = (phase, ir, L, R) if show_all else (phase, ir if phase in (2, 3, 6) else None, L, R)
            if key != last_key:
                last_key = key
                name = PHASES.get(phase, str(phase))
                print(f"{rel:7.2f}     {name:<14} ir={ir} L={L:>4} R={R:>4} err={err} br={br} C={f[12]}", file=out)
    if phase_times:
        print("\nPhase timeline:", file=out)
        for i, (t, p) in enumerate(phase_times):
            end = phase_times[i + 1][0] if i + 1 < len(phase_times) else None
            dur = f"{end - t:5.2f}s" if end is not None else "  ...  "
            print(f"  {t:7.2f}  {dur}  {PHASES.get(p, p)}", file=out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=pathlib.Path)
    ap.add_argument("--all", action="store_true", help="show every IR change and parameter dumps")
    args = ap.parse_args()
    rows = parse(args.log)
    if not rows:
        print("empty log")
        return 1
    summarise(rows, args.all)
    return 0


if __name__ == "__main__":
    sys.exit(main())
