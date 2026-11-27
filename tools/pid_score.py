"""
pid_score.py - score "auto pid" / "auto loop" test runs so kp/kd changes can be compared.

  python tools/pid_score.py logs/20261127-101500.log [more logs ...]

Uses the telemetry rows (T,...) while phase is 10 (pid) or 11 (loop). Per run:
  time      seconds in the pid-test phase (shorter = faster lap)
  |err|     mean absolute line error (sensor pitches; 0 = centred)
  off%      share of rows with the line on an OUTER sensor (|err| >= 1.5)
  flips/s   steering sign changes per second (high = wobble/oscillation)
  pivots    pivots (motors spinning opposite ways): pid test should be 0,
            loop test should equal its corners (extra = drift-out recoveries)
  branches  branches counted (loop test: 3 per lap on the square)
  end       how the run ended (last status event)
"""
import pathlib
import sys


def score(path: pathlib.Path):
    rows, end, branches = [], "?", 0
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = raw.split(",")
        if len(parts) < 2:
            continue
        if parts[1] == "E":
            end = ",".join(parts[3:]) if len(parts) > 3 else end
            continue
        if parts[1] != "T" or len(parts) < 10:
            continue
        try:
            ms, phase = int(parts[2]), int(parts[3])
            err, pid, left, right = float(parts[5]), float(parts[6]), int(parts[7]), int(parts[8])
            br = int(parts[9])
        except ValueError:
            continue
        if phase in (10, 11):
            rows.append((ms, err, pid, left, right))
            branches = max(branches, br)
    if len(rows) < 2:
        return None
    dur = (rows[-1][0] - rows[0][0]) / 1000.0
    mean_abs = sum(abs(r[1]) for r in rows) / len(rows)
    outer = 100.0 * sum(1 for r in rows if abs(r[1]) >= 1.5) / len(rows)
    flips, prev, pivots, spinning = 0, 0, 0, False
    for _, _, pid, left, right in rows:
        sign = (pid > 5) - (pid < -5)
        if sign and prev and sign != prev:
            flips += 1
        if sign:
            prev = sign
        spin = left * right < 0
        if spin and not spinning:
            pivots += 1
        spinning = spin
    return dur, mean_abs, outer, flips / dur if dur else 0.0, pivots, branches, end


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    print(f"{'log':28} {'time':>6} {'|err|':>6} {'off%':>5} {'flips/s':>7} {'pivots':>6} {'branch':>6}  end")
    for name in sys.argv[1:]:
        path = pathlib.Path(name)
        s = score(path)
        if s is None:
            print(f"{path.name:28} no pid/loop telemetry (phase 10/11)")
            continue
        dur, mean_abs, outer, fps, pivots, branches, end = s
        print(f"{path.name:28} {dur:6.2f} {mean_abs:6.2f} {outer:5.1f} {fps:7.2f} {pivots:6d} {branches:6d}  {end}")


if __name__ == "__main__":
    main()
