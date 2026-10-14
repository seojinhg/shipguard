#!/usr/bin/env python3
"""ShipGuard PC receiver: stands in for the driver's-phone gateway (fog).

In the deployed design the node talks BLE 5 to the driver's phone (in the van)
or LTE-M to a cell tower (warehouse). In the prototype that wireless link is
the USB serial cable, and this script plays the gateway role:

  * parses the node's CRC-8 frames (docs/design.md section 8),
  * prints every frame and logs it to a CSV file for the dashboard,
  * sends commands back to the node (ACK, profile, report interval, ARM, ping).

Usage:
  python3 pc/gateway.py                    # auto-detects /dev/cu.usbmodem*
  python3 pc/gateway.py --port /dev/cu.usbmodem21401 --interval 1

Keyboard commands (type, then Enter):
  a        ACK the current alert (same as the board button)
  p0/p1/p2 set profile GLASS / ELECTRONICS / ROBUST
  i N      report period N seconds (1-60)
  r        ARM: new shipment (recalibrate tilt, reset risk). Keep the box still.
  s        ping: request a STATUS frame
  t NAME   experiment trial start: ARM, then label every following row NAME
  m NAME   label every following row NAME (no ARM); "m" alone clears the label
  q        quit
"""
import argparse
import csv
import datetime as dt
import glob
import os
import select
import struct
import sys
import time

SOF_TO_PC = 0xA5
SOF_TO_NODE = 0x5A
PAYLOAD_MAX = 24

TYPE_NAMES = {0x01: "TELEMETRY", 0x02: "EVENT", 0x03: "ALERT", 0x04: "ACK_LOG", 0x05: "STATUS",
              0x06: "PRESENCE"}
CMD_ACK, CMD_SET_PROFILE, CMD_PING, CMD_SET_INTERVAL, CMD_ARM = 0x81, 0x82, 0x84, 0x85, 0x87
PROFILES = ["GLASS", "ELECTRONICS", "ROBUST"]
STATES = ["MONITORING", "ALERT"]
FLAG_BITS = [(0x01, "SAT"), (0x02, "OPEN"), (0x04, "PIR"), (0x10, "ACK_PENDING"), (0x20, "SENSOR_FAULT")]

CSV_FIELDS = [
    "pc_time", "type", "seq", "t_s", "state", "level", "profile", "flags",
    "sat", "open", "pir", "ack_pending", "sensor_fault",
    "risk", "peak_g", "dv_mms", "vib_rms_mg", "tilt_deg", "temp_c", "hum_pct", "press_hpa",
    "ack_source", "free_ram", "deadline_miss", "label",
]


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07, init 0x00 (matches firmware/shipguard/crc8.cpp)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), cmd, seq & 0xFF]) + payload
    return bytes([SOF_TO_NODE]) + body + bytes([crc8(body)])


# ---------------- serial port (pyserial if present, POSIX termios otherwise) ----------------
class Port:
    def __init__(self, path: str, baud: int = 115200):
        try:
            import serial  # type: ignore
            self._ser = serial.Serial(path, baud, timeout=0)
            self.fd = self._ser.fileno()
        except ImportError:
            import termios
            self._ser = None
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            a = termios.tcgetattr(self.fd)
            a[0] = 0
            a[1] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[3] = 0
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = 0
            a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)

    def read(self) -> bytes:
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> None:
        os.write(self.fd, data)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
        else:
            os.close(self.fd)


# ---------------- frame parser ----------------
class FrameParser:
    """Byte-at-a-time parser. Bytes outside frames are collected as text lines."""

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        frames, lines = [], []
        self.buf.extend(data)
        while self.buf:
            if self.buf[0] != SOF_TO_PC:
                b = self.buf.pop(0)
                if b == 10:
                    lines.append(self.text.decode(errors="replace").strip())
                    self.text.clear()
                elif 32 <= b < 127:
                    self.text.append(b)
                continue
            if len(self.buf) < 2:
                break
            length = self.buf[1]
            if length > PAYLOAD_MAX:
                self.buf.pop(0)  # not a real frame start
                continue
            total = length + 5
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if crc8(frame[1:-1]) != frame[-1]:
                self.crc_errors += 1
                self.buf.pop(0)  # resync on the next 0xA5
                continue
            del self.buf[:total]
            frames.append((frame[2], frame[3], frame[4:-1]))
        return frames, [l for l in lines if l]


def decode(ftype: int, seq: int, payload: bytes) -> dict:
    row = {k: "" for k in CSV_FIELDS}
    row["pc_time"] = dt.datetime.now().isoformat(timespec="milliseconds")
    row["type"] = TYPE_NAMES.get(ftype, hex(ftype))
    row["seq"] = seq
    if ftype in (0x01, 0x02, 0x03, 0x06) and len(payload) == 24:
        (t_s, state, level, profile, flags, risk, peak, dv, vib, tilt, temp, hum, press) = struct.unpack(
            "<IBBBBHHHHhhHH", payload)
        row.update(t_s=t_s, state=STATES[state] if state < 2 else state, level=level,
                   profile=PROFILES[profile] if profile < 3 else profile, flags=flags,
                   risk=risk, peak_g=round(peak / 1000.0, 2), dv_mms=dv, vib_rms_mg=vib,
                   tilt_deg=tilt / 10.0, temp_c=temp / 100.0, hum_pct=hum / 100.0, press_hpa=press / 10.0)
        for bit, name in FLAG_BITS:
            row[name.lower()] = 1 if flags & bit else 0
    elif ftype == 0x04 and len(payload) == 8:
        t_s, source, level, risk = struct.unpack("<IBBH", payload)
        row.update(t_s=t_s, level=level, risk=risk, ack_source="button" if source == 0 else "pc")
    elif ftype == 0x05 and len(payload) == 8:
        t_s, free_ram, miss, fault = struct.unpack("<IHBB", payload)
        row.update(t_s=t_s, free_ram=free_ram, deadline_miss=miss, sensor_fault=fault)
    return row


def describe(row: dict) -> str:
    t = row["type"]
    if t in ("TELEMETRY", "EVENT", "ALERT", "PRESENCE"):
        flags = [n for _, n in FLAG_BITS if row[n.lower()] == 1]
        return (f"{t:9s} #{row['seq']:<3} t={row['t_s']:>5}s L{row['level']} {row['state']:<10} "
                f"risk={row['risk']:<3} peak={row['peak_g']:>5}g dV={row['dv_mms']:>4}mm/s "
                f"tilt={row['tilt_deg']:>5}deg {row['temp_c']}C {row['hum_pct']}% "
                f"{row['press_hpa']}hPa {row['profile']} {' '.join(flags)}")
    if t == "ACK_LOG":
        return f"ACK_LOG   #{row['seq']:<3} t={row['t_s']}s by {row['ack_source']} at L{row['level']} risk={row['risk']}"
    if t == "STATUS":
        return (f"STATUS    #{row['seq']:<3} uptime={row['t_s']}s free_ram={row['free_ram']}B "
                f"deadline_miss={row['deadline_miss']} sensor_fault={row['sensor_fault']}")
    return f"{t} #{row['seq']}"


def find_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No Arduino port found. Plug in the board or pass --port.")
    return ports[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--interval", type=int, default=1, help="report period to request, seconds (default 1)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data"),
                    help="directory for CSV logs")
    args = ap.parse_args()

    port_path = args.port or find_port()
    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, dt.datetime.now().strftime("run_%Y%m%d_%H%M%S.csv"))
    latest = os.path.join(args.out, "latest.csv")

    port = Port(port_path)
    parser = FrameParser()
    tx_seq = 0
    label = ""
    last_seq = None
    seq_gaps = 0

    def send(cmd, payload=b""):
        nonlocal tx_seq
        port.write(build_frame(cmd, tx_seq, payload))
        tx_seq = (tx_seq + 1) & 0xFF

    print(f"Opened {port_path}. Logging to {csv_path}")
    print("Waiting for the board to boot (opening the port resets the UNO)...")
    time.sleep(2.5)
    send(CMD_SET_INTERVAL, bytes([max(1, min(60, args.interval))]))
    send(CMD_PING)
    print("Commands: a=ACK  p0/p1/p2=profile  i N=interval  r=ARM(new shipment)  s=status\n"
          "          t NAME=trial start (ARM + label)  m NAME=label only  q=quit\n")

    with open(csv_path, "w", newline="") as f:
        if os.path.lexists(latest):
            os.remove(latest)
        os.symlink(os.path.basename(csv_path), latest)
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        watch_stdin = True
        try:
            while True:
                r, _, _ = select.select([port.fd] + ([sys.stdin] if watch_stdin else []), [], [], 0.2)
                if port.fd in r:
                    frames, lines = parser.feed(port.read())
                    for line in lines:
                        print(f"[node] {line}")
                        if "boot ok" in line:  # board was reset: it is back at the 60 s default
                            send(CMD_SET_INTERVAL, bytes([max(1, min(60, args.interval))]))
                    for ftype, seq, payload in frames:
                        if last_seq is not None and seq != ((last_seq + 1) & 0xFF):
                            seq_gaps += 1
                        last_seq = seq
                        row = decode(ftype, seq, payload)
                        row["label"] = label
                        writer.writerow(row)
                        f.flush()
                        print(describe(row))
                if watch_stdin and sys.stdin in r:
                    raw = sys.stdin.readline()
                    if raw == "":  # stdin closed (e.g. run in the background): keep logging
                        watch_stdin = False
                        continue
                    cmd = raw.strip().lower()
                    if cmd == "q":
                        break
                    elif cmd == "a":
                        send(CMD_ACK)
                        print("-> ACK sent")
                    elif cmd in ("p0", "p1", "p2"):
                        send(CMD_SET_PROFILE, bytes([int(cmd[1])]))
                        print(f"-> profile {PROFILES[int(cmd[1])]}")
                    elif cmd.startswith("i"):
                        try:
                            n = int(cmd[1:].strip())
                            send(CMD_SET_INTERVAL, bytes([max(1, min(60, n))]))
                            print(f"-> report period {n}s")
                        except ValueError:
                            print("usage: i N")
                    elif cmd == "r":
                        send(CMD_ARM)
                        print("-> ARM (new shipment). Keep the box still.")
                    elif cmd == "s":
                        send(CMD_PING)
                    elif cmd == "m" or cmd.startswith(("m ", "t ")):
                        if cmd.startswith("t "):
                            send(CMD_ARM)  # each trial starts from a clean risk state
                        label = cmd[2:].strip() if len(cmd) > 1 else ""
                        mark = {k: "" for k in CSV_FIELDS}
                        mark.update(pc_time=dt.datetime.now().isoformat(timespec="milliseconds"),
                                    type="MARK", label=label)
                        writer.writerow(mark)
                        f.flush()
                        print(f"-> {'trial (ARM) ' if cmd.startswith('t ') else ''}label = '{label}'")
                    elif cmd:
                        print("?  a | p0 p1 p2 | i N | r | s | t NAME | m NAME | q")
        except KeyboardInterrupt:
            pass
        finally:
            port.close()
            print(f"\nSaved {csv_path}  (CRC errors: {parser.crc_errors}, sequence gaps: {seq_gaps})")


if __name__ == "__main__":
    main()
