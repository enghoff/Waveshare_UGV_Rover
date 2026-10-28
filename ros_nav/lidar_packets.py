"""The D500's packets as the lidar node reads them, for a recording session.

A recording session for Perception (rover_daemon/rover_recording.py) keeps the
lidar as the sensor sends it, packet by packet, rather than the 360 directions
the node publishes: each packet's twelve returns with their intensities, its
start and end angles, its rotation speed and the sensor's own millisecond
clock. Only the lidar node can read the sensor -- it holds the serial port --
so this lives inside it, and the node does no more for it than hand over each
chunk of bytes it read with the moment it read them. Finding the packets in
those bytes, checking each one's CRC and writing it happen on this module's own
thread, so the node's loop, which the rover's odometry and map both wait on, is
not made any slower (R-CTL-13).

The protocol (docs/reference/d500-lidar.md): 47 bytes, header 54 2C, speed in
degrees a second, start angle in hundredths of a degree, twelve returns of a
16-bit distance in millimetres and an 8-bit intensity, end angle, a 16-bit
millisecond timestamp, and a CRC-8 (polynomial 0x4d, init 0, no reflection).
"""
from __future__ import annotations

import math
import queue
import struct
import threading

PACKET_LEN = 47
HEADER = b"\x54\x2c"
POINTS = 12


def _crc_table():
    table = []
    for byte in range(256):
        c = byte
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc8(data: bytes) -> int:
    c = 0
    for byte in data:
        c = CRC_TABLE[c ^ byte]
    return c


def parse(packet: bytes) -> dict:
    """One packet's fields, in the session schema's units."""
    speed, start = struct.unpack_from("<HH", packet, 2)
    returns = struct.unpack_from("<" + "HB" * POINTS, packet, 6)
    end, stamp = struct.unpack_from("<HH", packet, 42)
    distances, intensities = returns[0::2], returns[1::2]
    return {"t_sensor_ms": stamp, "speed_dps": speed,
            "angle_start": math.radians(start / 100.0),
            "angle_end": math.radians(end / 100.0),
            "ranges": [d / 1000.0 if d else None for d in distances],
            "intensities": list(intensities)}


class Splitter:
    """Whole, CRC-checked packets out of a byte stream that may start anywhere."""

    def __init__(self):
        self._buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        out = []
        while True:
            at = self._buf.find(HEADER)
            if at < 0:
                del self._buf[:-1]
                return out
            if at:
                del self._buf[:at]
            if len(self._buf) < PACKET_LEN:
                return out
            packet = bytes(self._buf[:PACKET_LEN])
            if crc8(packet[:-1]) == packet[-1]:
                out.append(packet)
                del self._buf[:PACKET_LEN]
            else:
                # A 54 2C that fell inside a distance, not a packet: look again
                # one byte on.
                self.bad += 1
                del self._buf[:1]


class PacketRecorder:
    """Takes the node's chunks and writes the packets in them to a stream."""

    def __init__(self, stream):
        self.stream = stream
        self._chunks: queue.Queue = queue.Queue(maxsize=4096)
        self.dropped_chunks = 0
        self._splitter = Splitter()
        self._thread = threading.Thread(target=self._work, name="lidar-packets", daemon=True)
        self._thread.start()

    def chunk(self, data: bytes, arrived_ns: int) -> None:
        """Called by the node with every read. Never waits."""
        try:
            self._chunks.put_nowait((data, arrived_ns))
        except queue.Full:
            self.dropped_chunks += 1

    def _work(self) -> None:
        while True:
            data, arrived = self._chunks.get()
            if data is None:
                return
            for packet in self._splitter.feed(data):
                record = parse(packet)
                record["t_arr"] = arrived
                self.stream.put(record)

    def close(self) -> dict:
        self._chunks.put((None, None))
        self._thread.join(10.0)
        counts = self.stream.close()
        return {"written": counts["written"], "dropped": counts["dropped"]}
