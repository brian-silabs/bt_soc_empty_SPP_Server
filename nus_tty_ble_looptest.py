#!/usr/bin/env python3
"""
Loopback tester for a TTY <-> BLE (Nordic UART Service) bridge.

- BLE side uses the 'ble-serial' library (https://pypi.org/project/ble-serial/)
- TTY side uses pyserial (`pip install pyserial`)
- Validates integrity both ways (TTY->BLE and BLE->TTY).

Arguments:
  -t <ttyFilePath>
  -b <bluetooh_device_name_to_connect>   (GAP name OR MAC/UUID/ID)
  -s <data size in bytes>
  -p <period at which packets are sent in ms>
  -i <number of iterations>              (0 = run until failure or Ctrl-C)
  -d <debug>                              (0 = off, >0 = print hex dumps: 'Sent : [...] Received : [...]')
"""

import argparse
import asyncio
import itertools
import os
import re
import sys
import time
import threading
from typing import Optional

# ---- BLE (ble-serial) ----
try:
    from ble_serial.bluetooth.ble_client import BLE_client
except Exception:
    print("ERROR: ble-serial is required (pip install ble-serial).", file=sys.stderr)
    raise

# Name resolution (optional)
try:
    from bleak import BleakScanner
except Exception:
    BleakScanner = None

# ---- TTY (pyserial) ----
try:
    import serial
except Exception:
    print("ERROR: pyserial is required (pip install pyserial).", file=sys.stderr)
    raise

# ---- NUS UUIDs (Nordic standard) ----
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # central writes here
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # central subscribes notify here

# Defaults
DEFAULT_TTY_BAUD = 115200
DEFAULT_TIMEOUT_S = 15.0     # overall step timeout per direction
BLE_CHUNK = 20               # conservative chunking for BLE writes (MTU data size)


def is_mac_or_uuid(s: str) -> bool:
    mac_pat = re.compile(r"^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$")
    uuid_pat = re.compile(r"^[0-9A-Fa-f\-]{32,36}$")
    return bool(mac_pat.match(s) or uuid_pat.match(s))


async def resolve_device_address(name_or_addr: str, scan_timeout: float = 10.0) -> str:
    """Resolve a human-readable device name to its address/ID, else return the input unchanged."""
    if is_mac_or_uuid(name_or_addr):
        return name_or_addr
    if BleakScanner is None:
        raise RuntimeError("bleak is required to resolve device names; install via 'pip install bleak'")
    print(f"[BLE] Scanning for '{name_or_addr}' up to {scan_timeout:.0f}s ...")
    devices = await BleakScanner.discover(timeout=scan_timeout)
    for d in devices:
        if (d.name or "").strip() == name_or_addr.strip():
            print(f"[BLE] Found '{name_or_addr}' at address/ID: {d.address}")
            return d.address
    raise RuntimeError(f"Device with name '{name_or_addr}' not found during scan.")


class BLEEndpoint:
    """Thin wrapper around ble-serial BLE_client for NUS I/O."""

    def __init__(self, adapter_index: int = 0, gap_name: Optional[str] = None):
        self.bt = BLE_client(adapter_index, gap_name)
        self.rx_queue: asyncio.Queue[bytes] = asyncio.Queue()
        self._send_task = None
        self._check_task = None
        self._connected = False

    def _on_ble_rx(self, data: bytes):
        try:
            loop = asyncio.get_running_loop()
            loop.call_soon_threadsafe(self.rx_queue.put_nowait, bytes(data))
        except RuntimeError:
            self.rx_queue.put_nowait(bytes(data))

    async def connect(self, device_addr_or_id: str, timeout: float = 10.0):
        self.bt.set_receiver(self._on_ble_rx)
        await self.bt.connect(
            device_addr_or_id,
            "public",
            NUS_SERVICE_UUID,
            timeout
        )
        await self.bt.setup_chars(
            NUS_RX_CHAR_UUID,
            NUS_TX_CHAR_UUID,
            "rw",
            False
        )
        self._send_task = asyncio.create_task(self.bt.send_loop())
        self._check_task = asyncio.create_task(self.bt.check_loop())
        self._connected = True
        print("[BLE] Connected and NUS set up.")

    async def write_bytes(self, payload: bytes, chunk_size: int = BLE_CHUNK):
        if not self._connected:
            raise RuntimeError("BLE not connected")
        for i in range(0, len(payload), chunk_size):
            self.bt.queue_send(payload[i:i+chunk_size])

    async def read_exact(self, n: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        chunks = []
        remaining = n
        while remaining > 0:
            time_left = deadline - time.monotonic()
            if time_left <= 0:
                break
            try:
                b = await asyncio.wait_for(self.rx_queue.get(), timeout=time_left)
            except asyncio.TimeoutError:
                break
            if not b:
                continue
            if len(b) <= remaining:
                chunks.append(b)
                remaining -= len(b)
            else:
                chunks.append(b[:remaining])
                leftover = b[remaining:]
                self.rx_queue.put_nowait(leftover)
                remaining = 0
        data = b"".join(chunks)
        if len(data) != n:
            raise TimeoutError(f"[BLE] Timed out assembling {n} bytes (got {len(data)})")
        return data

    async def disconnect(self):
        try:
            if self._send_task:
                self._send_task.cancel()
            if self._check_task:
                self._check_task.cancel()
            await self.bt.disconnect()
        finally:
            self._connected = False
            print("[BLE] Disconnected.")


class SerialEndpoint:
    """
    Async-style serial endpoint using a background reader thread that pushes
    incoming bytes into an asyncio.Queue (callback-like behavior).
    """
    def __init__(self, path: str, baud: int = DEFAULT_TTY_BAUD):
        self.ser = serial.Serial(
            port=path,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,          # read timeout for thread loop
            write_timeout=5.0
        )
        self.loop = asyncio.get_running_loop()
        self.rx_queue: asyncio.Queue[bytes] = asyncio.Queue()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._reader.start()
        print(f"[TTY] Opened {path} @{self.ser.baudrate} baud")

    def _reader_loop(self):
        try:
            while not self._stop.is_set():
                n = self.ser.in_waiting
                count = n if n > 0 else 1
                try:
                    data = self.ser.read(count)
                except Exception:
                    data = b""
                if data:
                    try:
                        self.loop.call_soon_threadsafe(self.rx_queue.put_nowait, bytes(data))
                    except Exception:
                        break
        except Exception:
            pass

    def purge(self):
        """Drop any pending input so the next read_exact only sees fresh bytes."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        try:
            while True:
                self.rx_queue.get_nowait()
        except asyncio.QueueEmpty:
            pass

    def write(self, payload: bytes):
        self.ser.write(payload)
        self.ser.flush()

    async def read_exact(self, n: int, timeout: float) -> bytes:
        """Assemble exactly n bytes from the background reader within timeout."""
        deadline = time.monotonic() + timeout
        out = bytearray()
        while len(out) < n:
            time_left = deadline - time.monotonic()
            if time_left <= 0:
                break
            try:
                chunk = await asyncio.wait_for(self.rx_queue.get(), timeout=time_left)
            except asyncio.TimeoutError:
                break
            if chunk:
                need = n - len(out)
                if len(chunk) <= need:
                    out.extend(chunk)
                else:
                    out.extend(chunk[:need])
                    self.rx_queue.put_nowait(chunk[need:])
        if len(out) != n:
            raise TimeoutError(f"[TTY] Timed out assembling {n} bytes (got {len(out)})")
        return bytes(out)

    def close(self):
        self._stop.set()
        try:
            self._reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        print("[TTY] Closed.")


def generate_random(n: int) -> bytes:
    return os.urandom(n)


def to_hex_bracket(b: bytes) -> str:
    # Format: [00 11 22 33]
    return "[" + " ".join(f"{x:02X}" for x in b) + "]"


async def run_test(args):
    # Resolve BLE target
    target = await resolve_device_address(args.b)

    # Bring up BLE
    ble = BLEEndpoint(adapter_index=0, gap_name=None)
    await ble.connect(target, timeout=10.0)

    # Bring up TTY with async-style reader
    ser = SerialEndpoint(args.t, baud=DEFAULT_TTY_BAUD)

    ok = 0
    failures = 0

    period_s = max(0.0, args.p / 1000.0)
    data_n = args.s
    iterations = args.i
    infinite = (iterations == 0)
    debug = args.d > 0

    try:
        iter_src = itertools.count(1) if infinite else range(1, iterations + 1)
        for it in iter_src:
            print(f"\n=== Iteration {it}{'' if not infinite else ' (until failure)'} ===")
            payload = generate_random(data_n)

            # 1) TTY -> BLE: send on TTY, expect same on BLE
            ser.write(payload)
            print(f"[TTY->BLE] Sent {len(payload)} bytes over TTY, waiting on BLE ...")
            try:
                got_on_ble = await ble.read_exact(len(payload), timeout=DEFAULT_TIMEOUT_S)
            except Exception as e:
                print(f"[TTY->BLE] FAIL: {e}")
                failures += 1
                if infinite:
                    break
                if period_s:
                    await asyncio.sleep(period_s)
                continue

            if debug:
                print(f"Sent : {to_hex_bracket(payload)} Received : {to_hex_bracket(got_on_ble)}")

            if got_on_ble != payload:
                print(f"[TTY->BLE] FAIL: payload mismatch (got {len(got_on_ble)} bytes)")
                failures += 1
                if infinite:
                    break
                if period_s:
                    await asyncio.sleep(period_s)
                continue
            print("[TTY->BLE] OK")

            if period_s:
                await asyncio.sleep(period_s)

            # 2) BLE -> TTY: purge stale serial, send over BLE, expect on TTY
            ser.purge()
            print(f"[BLE->TTY] Sending {len(payload)} bytes over BLE ...")
            await ble.write_bytes(payload, chunk_size=BLE_CHUNK)

            try:
                got_on_tty = await ser.read_exact(len(payload), timeout=DEFAULT_TIMEOUT_S)
            except Exception as e:
                print(f"[BLE->TTY] FAIL: {e}")
                failures += 1
                if infinite:
                    break
                if period_s:
                    await asyncio.sleep(period_s)
                continue

            if debug:
                print(f"Sent : {to_hex_bracket(payload)} Received : {to_hex_bracket(got_on_tty)}")

            if got_on_tty != payload:
                print(f"[BLE->TTY] FAIL: payload mismatch (got {len(got_on_tty)} bytes)")
                failures += 1
                if infinite:
                    break
                if period_s:
                    await asyncio.sleep(period_s)
                continue

            print("[BLE->TTY] OK")
            ok += 1

            if period_s:
                await asyncio.sleep(period_s)

    finally:
        print("\n=== Summary ===")
        print(f"  OK:       {ok}")
        print(f"  FAILURES: {failures}")
        pass_rate = 0.0 if (ok + failures) == 0 else (ok * 100.0 / (ok + failures))
        print(f"  Pass rate: {pass_rate:.2f}%")
        ser.close()
        await ble.disconnect()

    if failures:
        sys.exit(1)


def parse_args():
    p = argparse.ArgumentParser(description="TTY <-> BLE (NUS) loopback integrity tester")
    p.add_argument("-t", required=True, metavar="ttyFilePath", help="Path to TTY (e.g. /dev/ttyUSB0 or COM7)")
    p.add_argument("-b", required=True, metavar="ble_device_name_or_address", help="BLE GAP name or address/UUID")
    p.add_argument("-s", required=True, type=int, metavar="bytes", help="Data size per iteration")
    p.add_argument("-p", required=True, type=int, metavar="ms", help="Period between sends in milliseconds")
    p.add_argument("-i", required=True, type=int, metavar="count", help="Number of iterations (0 = until failure or Ctrl-C)")
    p.add_argument("-d", required=False, type=int, default=0, metavar="debug", help="Debug level (0=off, >0=print hex dumps)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.s <= 0 or args.p < 0 or args.i < 0:
        print("Invalid arguments: ensure s>0, p>=0, i>=0", file=sys.stderr)
        sys.exit(2)
    try:
        asyncio.run(run_test(args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(130)


if __name__ == "__main__":
    main()
