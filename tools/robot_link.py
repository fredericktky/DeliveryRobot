"""
robot_link.py - talk to the DeliveryRobot over Bluetooth LE from a PC.

Connects to "ESP32-S3-Robot" (Nordic UART service), sends commands,
and prints + logs everything the robot sends back (telemetry "T,...",
events "E,...", parameters "P,...").

Your phone can stay connected at the same time for driving.

Examples
  python tools/robot_link.py get                       # show tuning values
  python tools/robot_link.py "set kp 45" "set kd 50"   # change values live
  python tools/robot_link.py --tel 20 -t 30            # log 30 s of telemetry
  python tools/robot_link.py --tel 20 -t 25 A          # start an auto run and log it
  python tools/robot_link.py --tel 10 -t 0             # stream until Ctrl+C

Logs go to logs/<date-time>.log (one line per message, PC time in seconds first).
Requires: pip install bleak
"""
import argparse
import asyncio
import datetime
import pathlib
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32-S3-Robot"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # PC -> robot
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # robot -> PC

REPO = pathlib.Path(__file__).resolve().parent.parent


async def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("commands", nargs="*", help='commands to send, e.g. A  "set kp 45"  get')
    ap.add_argument("-t", "--time", type=float, default=3.0,
                    help="seconds to keep listening after the commands (0 = until Ctrl+C)")
    ap.add_argument("--tel", type=int, metavar="HZ", help="turn telemetry on at HZ for this session")
    ap.add_argument("--log", type=pathlib.Path, help="log file (default logs/<timestamp>.log)")
    ap.add_argument("--no-log", action="store_true", help="don't write a log file")
    ap.add_argument("--address", help="connect to this BLE address instead of scanning by name")
    ap.add_argument("--quiet", action="store_true", help="don't print telemetry lines (still logged)")
    args = ap.parse_args()

    if args.address:
        target = args.address
    else:
        print(f"Scanning for {DEVICE_NAME} ...", flush=True)
        target = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
        if target is None:
            print("Robot not found. Is it powered, and not already connected to 3 devices?")
            return 1

    log = None
    if not args.no_log:
        path = args.log or REPO / "logs" / (datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".log")
        path.parent.mkdir(parents=True, exist_ok=True)
        log = open(path, "w", encoding="utf-8", newline="\n")
        print(f"Logging to {path}")

    t0 = asyncio.get_event_loop().time()
    buf = ""
    counts = {"T": 0, "E": 0, "P": 0}

    def on_line(line: str) -> None:
        stamp = asyncio.get_event_loop().time() - t0
        kind = line[:1]
        counts[kind] = counts.get(kind, 0) + 1
        if log:
            log.write(f"{stamp:.3f},{line}\n")
        if not (args.quiet and kind == "T"):
            print(line, flush=True)

    def on_notify(_handle, data: bytearray) -> None:
        nonlocal buf
        buf += data.decode(errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            if line.strip():
                on_line(line.strip())

    async with BleakClient(target) as client:
        print(f"Connected (MTU {client.mtu_size})", flush=True)
        await client.start_notify(TX_UUID, on_notify)

        async def send(cmd: str) -> None:
            if log:
                log.write(f"{asyncio.get_event_loop().time() - t0:.3f},>,{cmd}\n")
            await client.write_gatt_char(RX_UUID, (cmd + "\n").encode(), response=True)
            await asyncio.sleep(0.25)   # robot handles one text command per loop

        if args.tel:
            await send(f"tel {args.tel}")
        for cmd in args.commands:
            print(f"> {cmd}", flush=True)
            await send(cmd)

        try:
            if args.time > 0:
                await asyncio.sleep(args.time)
            else:
                while True:
                    await asyncio.sleep(1)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            if args.tel and client.is_connected:
                try:
                    await send("tel off")
                except Exception:
                    pass

    if log:
        log.close()
    print(f"Done: {counts.get('T', 0)} telemetry, {counts.get('E', 0)} events, {counts.get('P', 0)} param lines")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        pass
