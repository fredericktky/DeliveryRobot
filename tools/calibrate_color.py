"""
calibrate_color.py - guided colour-sensor calibration over BLE.

Put the robot on the track where it reads the cards (top line, sensor over a
colour square), motors off (manual mode), battery on. The script asks you to
show each surface, records the raw readings ("craw on"), and then:

  * computes a reference (mean r/g/b %) and the spread for every surface
  * checks how far apart the colours are (blue vs green is the weak pair)
  * picks CARD_MAX_C (card vs white floor brightness) and the match distance
  * replays every sample through the robot's own classifier with the new
    values and prints how many would be read correctly
  * saves samples, report and a C++ snippet to logs/calib/<date-time>/

  python tools/calibrate_color.py                 # measure + report
  python tools/calibrate_color.py --push          # ...and send the new values live (lost on reset)
  python tools/calibrate_color.py --apply         # ...and write them into the source (flash to keep)
  python tools/calibrate_color.py --analyse logs/calib/<dir>/samples.json [--push/--apply]
  python tools/calibrate_color.py --verify        # show what the robot reads now, card by card

Tips: slide the card slowly under the sensor while recording (covers small
height / angle changes); record WHITE on the same paper next to the squares.
Run Python without -I so bleak imports.
"""
import argparse
import asyncio
import datetime
import json
import math
import pathlib
import re
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from rlink import RobotLink  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
HEADER = ROOT / "include" / "ColorSensor.h"
MAIN = ROOT / "src" / "main.cpp"
MIN_CLEAR = 150                       # COLOR_MIN_CLEAR in ColorSensor.h
SURFACES = [
    ("WHITE", "the WHITE paper next to the colour squares (no card)"),
    ("RED", "the RED card"),
    ("GREEN", "the GREEN card"),
    ("BLUE", "the BLUE card"),
    ("BLACK", "the BLACK line (optional - press s to skip)"),
]
CARDS = ("RED", "GREEN", "BLUE")


# ---------------------------------------------------------------- measuring
def parse_k(line):
    # K,ms,c,r%,g%,b%,label,dist
    f = line.split(",")
    if len(f) < 8 or f[0] != "K":
        return None
    try:
        return {"c": int(f[2]), "r": float(f[3]), "g": float(f[4]), "b": float(f[5]), "fw": f[6]}
    except ValueError:
        return None


async def ask(prompt):
    return (await asyncio.to_thread(input, prompt)).strip().lower()


async def record(robot, seconds):
    t0 = robot.now()
    await asyncio.sleep(seconds)
    return [s for t, l in robot.lines if t >= t0 and (s := parse_k(l))]


async def measure(args, outdir):
    samples = {}
    async with RobotLink(outdir / "calibration.log") as robot:
        print("Connected.")
        await robot.send("M")
        await robot.send("cref")
        await asyncio.sleep(0.5)
        old = {}
        for _, l in robot.lines:                  # R,label,r,g,b = the robot's current references
            f = l.split(",")
            if f[0] == "R" and len(f) == 5:
                old[f[1]] = {"r": float(f[2]), "g": float(f[3]), "b": float(f[4])}
        (outdir / "old_refs.json").write_text(json.dumps(old, indent=2), encoding="utf-8")
        await robot.send("craw on")
        for label, what in SURFACES:
            while True:
                a = await ask(f"\n[{label}] Put the sensor over {what}.\n"
                              f"  Enter = record {args.seconds:g} s (slide it slowly), s = skip, q = quit: ")
                if a == "q":
                    await robot.send("craw off")
                    return samples
                if a == "s":
                    break
                robot.note(f"record {label}")
                got = await record(robot, args.seconds)
                if len(got) < 10:
                    print(f"  only {len(got)} readings - is the colour sensor working? try again")
                    continue
                c = [s["c"] for s in got]
                print(f"  {len(got)} readings, clear {min(c)}..{max(c)}, firmware read them as: "
                      + ", ".join(f"{k} {v}" for k, v in count([s['fw'] for s in got]).items()))
                samples[label] = got
                break
        await robot.send("craw off")
    return samples


def count(items):
    out = {}
    for i in items:
        out[i] = out.get(i, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


# ---------------------------------------------------------------- analysis
def pct(values, q):
    v = sorted(values)
    if not v:
        return 0
    k = (len(v) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def dist(a, b):
    return math.sqrt((a["r"] - b["r"]) ** 2 + (a["g"] - b["g"]) ** 2 + (a["b"] - b["b"]) ** 2)


def classify(s, refs, cdist, cardmax):
    """Same as classifyColor() + cardUnderSensor() on the robot."""
    if s["c"] < MIN_CLEAR:
        return "dark"
    best, bd = None, 1e9
    for label, ref in refs.items():
        d = dist(s, ref)
        if d < bd:
            best, bd = label, d
    if bd > cdist:
        return "none"
    if best in CARDS and cardmax > 0 and s["c"] >= cardmax:
        return "none(bright)"
    return best


def analyse(samples, out, old_refs=None):
    lines = []

    def p(text=""):
        lines.append(text)
        print(text)

    refs, stats = {}, {}
    for label, got in samples.items():
        r = [s["r"] for s in got]
        g = [s["g"] for s in got]
        b = [s["b"] for s in got]
        c = [s["c"] for s in got]
        refs[label] = {"r": statistics.mean(r), "g": statistics.mean(g), "b": statistics.mean(b)}
        own = [dist(s, refs[label]) for s in got]
        stats[label] = {"n": len(got), "c_min": min(c), "c_p01": pct(c, 0.01), "c_mean": statistics.mean(c),
                        "c_p99": pct(c, 0.99), "c_max": max(c), "spread_p95": pct(own, 0.95),
                        "spread_max": max(own)}

    p("Measured references (mean r/g/b % of clear):")
    p(f"  {'surface':<7} {'n':>4} {'r%':>6} {'g%':>6} {'b%':>6}   clear (min/mean/max)   spread p95/max")
    for label in refs:
        R, S = refs[label], stats[label]
        p(f"  {label:<7} {S['n']:>4} {R['r']:6.1f} {R['g']:6.1f} {R['b']:6.1f}   "
          f"{S['c_min']:>5}/{S['c_mean']:7.0f}/{S['c_max']:>5}     {S['spread_p95']:4.1f}/{S['spread_max']:4.1f}")

    p("\nSeparation (distance between references; want > 3x the spread):")
    labels = list(refs)
    worst = None
    for i, a in enumerate(labels):
        for b_ in labels[i + 1:]:
            d = dist(refs[a], refs[b_])
            sp = stats[a]["spread_p95"] + stats[b_]["spread_p95"]
            ratio = d / sp if sp > 0 else 99
            flag = "  <-- WEAK" if ratio < 2 else ("  (ok-ish)" if ratio < 3 else "")
            if a in CARDS or b_ in CARDS:
                if worst is None or ratio < worst[2]:
                    worst = (a, b_, ratio, d)
            p(f"  {a:>5} - {b_:<5} {d:5.1f}   x{ratio:4.1f} of spread{flag}")

    # match distance: cover the cards' own spread, but stay below half the gap to any other reference
    card_spread = max((stats[c]["spread_max"] for c in CARDS if c in stats), default=6.0)
    nearest_other = min((dist(refs[c], refs[o]) for c in CARDS if c in refs for o in refs if o != c), default=24.0)
    cdist = round(min(max(card_spread * 1.3, 4.0), max(nearest_other / 2, 3.0), 15.0), 1)

    # card brightness threshold: brighter than every card, darker than the white floor
    cardmax = 0
    if "WHITE" in stats and any(c in stats for c in CARDS):
        card_hi = max(stats[c]["c_p99"] for c in CARDS if c in stats)
        white_lo = stats["WHITE"]["c_p01"]
        if card_hi < white_lo:
            cardmax = int((card_hi + white_lo) / 2)
            p(f"\nBrightness: cards up to {card_hi:.0f}, white from {white_lo:.0f} -> CARD_MAX_C = {cardmax}")
        else:
            p(f"\nBrightness: cards ({card_hi:.0f}) and white ({white_lo:.0f}) overlap -> CARD_MAX_C = 0 (off)."
              "  Check the LED / shroud: the clear count should drop clearly on a card.")
    p(f"Match distance (cdist) = {cdist}  (cards' max spread {card_spread:.1f}, nearest other reference {nearest_other:.1f})")

    replay_refs = dict(refs)
    for label, R in (old_refs or {}).items():   # surfaces not measured keep the robot's old reference
        replay_refs.setdefault(label, R)
    p("\nReplay of every sample with the new values (what the robot would read):")
    total_ok = total = 0
    for label, got in samples.items():
        res = count([classify(s, replay_refs, cdist, cardmax) for s in got])
        want = label if label in CARDS else None
        ok = sum(v for k, v in res.items() if (k == want) or (want is None and k not in CARDS))
        total_ok += ok
        total += len(got)
        p(f"  {label:<6} {100 * ok / len(got):5.1f}% correct   " + ", ".join(f"{k} {v}" for k, v in res.items()))
    p(f"  overall {100 * total_ok / max(total, 1):.1f}%")
    if worst and worst[2] < 2:
        p(f"\nWARNING: {worst[0]} and {worst[1]} are close ({worst[3]:.1f} apart). A black shroud around the sensor"
          " and a steady height help; check the replay above before trusting colour runs.")

    snippet = ["static ColorRef COLOR_REFS[] = {", "  //  class         r%    g%    b%    label"]
    cls = {"RED": "COLOR_RED", "GREEN": "COLOR_GREEN", "BLUE": "COLOR_BLUE", "WHITE": "COLOR_NONE", "BLACK": "COLOR_NONE"}
    stamp = datetime.date.today().isoformat()
    for label in ("RED", "GREEN", "BLUE", "WHITE", "BLACK"):
        if label in refs:
            R = refs[label]
            snippet.append(f"  {{ {cls[label] + ',':<14} {R['r']:5.1f}, {R['g']:4.1f}, {R['b']:4.1f}, \"{label}\"{' ' * (6 - len(label))}}},"
                           f"  // calibrated {stamp} (C~{stats[label]['c_mean']:.0f})")
    snippet.append("};")
    p("\nC++ for include/ColorSensor.h:")
    for s_ in snippet:
        p("  " + s_)
    p(f"and in src/main.cpp: int CARD_MAX_C = {cardmax};   ColorSensor.h: #define COLOR_MAX_DISTANCE {cdist}f")

    (out / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    result = {"refs": refs, "stats": stats, "cdist": cdist, "cardmax": cardmax}
    (out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


# ---------------------------------------------------------------- push / apply
async def push(result, outdir):
    async with RobotLink(outdir / "push.log", echo=lambda l: print("  " + l) if l[:2] in ("R,", "E,") else None) as robot:
        for label, R in result["refs"].items():
            await robot.send(f"cref {label} {R['r']:.1f} {R['g']:.1f} {R['b']:.1f}")
        await robot.send(f"set cardmax {result['cardmax']}")
        await robot.send(f"set cdist {result['cdist']}")
        await robot.send("cref")
        await asyncio.sleep(1)
    print("Pushed live. These are lost on reset - use --apply and flash to keep them.")


def apply(result):
    h = HEADER.read_text(encoding="utf-8")
    stamp = datetime.date.today().isoformat()
    for label, R in result["refs"].items():
        pat = re.compile(r'^(\s*\{\s*COLOR_\w+,\s*)[\d.]+,\s*[\d.]+,\s*[\d.]+,(\s*"' + label + r'"\s*\},).*$', re.M)
        new, n = pat.subn(lambda m: f"{m.group(1)}{R['r']:.1f}, {R['g']:.1f}, {R['b']:.1f},{m.group(2)}"
                                    f"  // calibrated {stamp} (C~{result['stats'][label]['c_mean']:.0f})", h)
        if n != 1:
            print(f"  could not find the {label} row in ColorSensor.h - edit it by hand")
        h = new
    h, n = re.subn(r"#define COLOR_MAX_DISTANCE\s+[\d.]+f", f"#define COLOR_MAX_DISTANCE  {result['cdist']}f", h)
    HEADER.write_text(h, encoding="utf-8")
    m = MAIN.read_text(encoding="utf-8")
    m, n2 = re.subn(r"int CARD_MAX_C = \d+;", f"int CARD_MAX_C = {result['cardmax']};", m)
    MAIN.write_text(m, encoding="utf-8")
    print(f"Wrote {HEADER.relative_to(ROOT)} and {MAIN.relative_to(ROOT)} (CARD_MAX_C: {'ok' if n2 else 'NOT FOUND'})."
          " Build and flash to keep them.")


# ---------------------------------------------------------------- verify
async def verify(args, outdir):
    async with RobotLink(outdir / "verify.log") as robot:
        await robot.send("M")
        await robot.send("craw on")
        while True:
            a = await ask("\nPut a card (or white) under the sensor. Enter = read 2 s, q = quit: ")
            if a == "q":
                break
            got = await record(robot, 2.0)
            print("  robot reads: " + ", ".join(f"{k} {v}" for k, v in count([s["fw"] for s in got]).items())
                  + f"   clear {min(s['c'] for s in got) if got else 0}..{max(s['c'] for s in got) if got else 0}")
        await robot.send("craw off")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=4.0, help="recording time per surface")
    ap.add_argument("--push", action="store_true", help="send the new values to the robot (live)")
    ap.add_argument("--apply", action="store_true", help="write the new values into the source files")
    ap.add_argument("--analyse", type=pathlib.Path, help="re-analyse a saved samples.json instead of measuring")
    ap.add_argument("--verify", action="store_true", help="only show what the robot reads now")
    args = ap.parse_args()

    outdir = ROOT / "logs" / "calib" / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)

    if args.verify:
        asyncio.run(verify(args, outdir))
        return 0
    if args.analyse:
        samples = json.loads(args.analyse.read_text(encoding="utf-8"))
    else:
        samples = asyncio.run(measure(args, outdir))
        (outdir / "samples.json").write_text(json.dumps(samples), encoding="utf-8")
    if not samples:
        print("No samples recorded.")
        return 1
    missing = [c for c in CARDS + ("WHITE",) if c not in samples]
    if missing:
        print(f"Note: no samples for {', '.join(missing)} - their old references stay in use.")
    src_dir = args.analyse.parent if args.analyse else outdir
    old_path = src_dir / "old_refs.json"
    old_refs = json.loads(old_path.read_text(encoding="utf-8")) if old_path.exists() else None
    result = analyse(samples, outdir, old_refs)
    print(f"\nSaved to {outdir}")
    if args.push:
        asyncio.run(push(result, outdir))
    if args.apply:
        apply(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
