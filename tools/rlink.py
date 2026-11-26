"""
rlink.py - shared BLE link to the DeliveryRobot, used by calibrate_color.py
and lab_test.py (robot_link.py is the stand-alone command-line version).

    async with RobotLink(log_path) as robot:
        await robot.send("get")
        await robot.wait_event("pong", timeout=3)

Every line from the robot is timestamped and written to the log as
"<pc_seconds>,<line>"; commands are logged as "<pc_seconds>,>,<cmd>",
the same format robot_link.py uses, so log_summary.py reads both.
Requires: pip install bleak  (run Python without -I so bleak imports)
"""
import asyncio
import pathlib
import time
from typing import Callable, List, Optional

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32-S3-Robot"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # PC -> robot
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # robot -> PC


class RobotLink:
    def __init__(self, log_path: Optional[pathlib.Path] = None, address: Optional[str] = None,
                 echo: Optional[Callable[[str], None]] = None):
        self.log_path = log_path
        self.address = address
        self.echo = echo                 # called with each received line (e.g. print events)
        self.lines: List[tuple] = []     # (pc_seconds, line) for everything received
        self._buf = ""
        self._log = None
        self._client: Optional[BleakClient] = None
        self._t0 = time.monotonic()
        self._waiters: List[tuple] = []

    # ---------- connection ----------
    async def __aenter__(self):
        target = self.address
        if not target:
            target = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
            if target is None:
                raise RuntimeError("Robot not found. Is it powered, and not already connected to 3 devices?")
        if self.log_path:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log = open(self.log_path, "w", encoding="utf-8", newline="\n")
        self._client = BleakClient(target)
        await self._client.connect()
        await self._client.start_notify(TX_UUID, self._on_notify)
        return self                      # the clock (_t0) keeps running across reconnects

    async def __aexit__(self, *exc):
        try:
            if self._client and self._client.is_connected:
                await self._client.disconnect()
        finally:
            if self._log:
                self._log.close()
                self._log = None

    def set_log(self, path: Optional[pathlib.Path]) -> None:
        """Close the current log and start writing to path (one file per test)."""
        if self._log:
            self._log.close()
            self._log = None
        self.log_path = path
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._log = open(path, "w", encoding="utf-8", newline="\n")

    @property
    def connected(self) -> bool:
        return bool(self._client and self._client.is_connected)

    def now(self) -> float:
        return time.monotonic() - self._t0

    # ---------- receive ----------
    def _on_notify(self, _handle, data: bytearray) -> None:
        self._buf += data.decode(errors="replace")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line:
                self._on_line(line)

    def _on_line(self, line: str) -> None:
        stamp = self.now()
        self.lines.append((stamp, line))
        if self._log:
            self._log.write(f"{stamp:.3f},{line}\n")
        if self.echo:
            self.echo(line)
        for w in list(self._waiters):
            pred, fut = w
            if not fut.done() and pred(line):
                fut.set_result(line)

    # ---------- send ----------
    async def send(self, cmd: str, pause: float = 0.25) -> None:
        if self._log:
            self._log.write(f"{self.now():.3f},>,{cmd}\n")
        await self._client.write_gatt_char(RX_UUID, (cmd + "\n").encode(), response=True)
        await asyncio.sleep(pause)        # the robot handles one text command per loop

    def note(self, text: str) -> None:
        """Write a free-text note into the log (shown by log_summary.py)."""
        if self._log:
            self._log.write(f"{self.now():.3f},#,{text}\n")

    # ---------- waiting ----------
    async def wait_line(self, pred: Callable[[str], bool], timeout: float) -> Optional[str]:
        fut = asyncio.get_event_loop().create_future()
        entry = (pred, fut)
        self._waiters.append(entry)
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            return None
        finally:
            self._waiters.remove(entry)

    async def wait_event(self, text: str, timeout: float) -> Optional[str]:
        """Wait for an event line E,<ms>,<msg> whose msg contains text."""
        return await self.wait_line(lambda l: l.startswith("E,") and text in l, timeout)

    def events_since(self, t: float) -> List[tuple]:
        return [(s, l.split(",", 2)[2]) for s, l in self.lines if s >= t and l.startswith("E,") and l.count(",") >= 2]

    async def stop_robot(self) -> None:
        """Manual mode = motors off and auto aborted. Always call after a run:
        the phone may still be connected, which keeps an auto run alive."""
        if self.connected:
            try:
                await self.send("M")
            except Exception:
                pass
