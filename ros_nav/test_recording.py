"""A recording session's lidar packets: found, checked and written as the sensor sent them.

The lidar node hands every chunk it reads to `lidar_packets.PacketRecorder`, so
what matters is that packets cut anywhere by the reads come out whole, that a
`54 2C` inside a distance is not taken for a packet, and that the fields reach
the file in the session schema's units.
"""
from __future__ import annotations

import json
import math
import os
import random
import struct
import sys
import tempfile

from test_harness import HERE, check, section

for _candidate in (HERE, os.path.join(HERE, "..", "rover_daemon")):
    if os.path.exists(os.path.join(_candidate, "session_record.py")):
        sys.path.append(os.path.abspath(_candidate))
        break


def _packet(start_cdeg, stamp_ms, distances, speed=3600):
    import lidar_packets

    body = struct.pack("<BBHH", 0x54, 0x2C, speed, start_cdeg)
    for i, d in enumerate(distances):
        body += struct.pack("<HB", d, 200 + i)
    body += struct.pack("<HH", (start_cdeg + 800) % 36000, stamp_ms)
    return body + bytes([lidar_packets.crc8(body)])


def test_packets_come_out_whole_however_the_reads_cut_them():
    import lidar_packets
    import session_record

    section("lidar packets for a recording session")
    distances = [0, 1000, 2000, 0x2C54, 500, 600, 700, 800, 900, 1000, 1100, 1200]
    packets = [_packet(35900, 29990, distances), _packet(100, 29999, distances),
               _packet(900, 3, distances)]
    stream_bytes = b"\x54\x00garbage" + b"".join(packets)
    rng = random.Random(7)
    with tempfile.TemporaryDirectory() as root:
        stream = session_record.Stream(root, "lidar", "lidar/packets.jsonl")
        recorder = lidar_packets.PacketRecorder(stream)
        at = 0
        while at < len(stream_bytes):
            n = rng.randint(1, 30)
            recorder.chunk(stream_bytes[at:at + n], 1000 + at)
            at += n
        counts = recorder.close()
        with open(os.path.join(root, "lidar", "packets.jsonl")) as f:
            lines = [json.loads(line) for line in f][1:]
    check("every packet comes out once, however the reads cut it", len(lines), 3)
    check("...and the counts say so", counts, {"written": 3, "dropped": 0})
    first = lines[0]
    check("the sensor's millisecond clock and speed are kept verbatim",
          (first["t_sensor_ms"], first["speed_dps"]), (29990, 3600))
    check("angles are the fields' hundredths of a degree in radians",
          (round(first["angle_start"], 6), round(first["angle_end"], 6)),
          (round(math.radians(359.0), 6), round(math.radians(7.0), 6)))
    check("ranges in metres, no return as null, a 54 2C inside a distance kept as one",
          first["ranges"][:4], [None, 1.0, 2.0, 0x2C54 / 1000.0])
    check("intensities as sent", first["intensities"][:3], [200, 201, 202])
    check("the clock's wrap is left for the reader", [r["t_sensor_ms"] for r in lines],
          [29990, 29999, 3])


def test_the_fit_follows_the_agreed_rule():
    import recorder_node as r

    section("the map pose's fit")
    status = {"started": True, "map": [389, 416, 0.05]}
    nav = {"map_settled": True, "gyro_fault": "", "map_drift": None}
    now = 1000.0
    check("a searched step a second ago that scored 0.8 is good",
          r.fit_of(status, nav, (now - 1.0, 0.8), now), "good")
    check("...under the localizer's 0.55 it is poor",
          r.fit_of(status, nav, (now - 1.0, 0.5), now), "poor")
    check("...and nothing searched for over 2 s is lost",
          r.fit_of(status, nav, (now - 2.5, 0.8), now), "lost")
    check("an unconfirmed restore is lost",
          r.fit_of(status, dict(nav, map_settled=False), (now, 0.8), now), "lost")
    check("a latched gyro fault is lost",
          r.fit_of(status, dict(nav, gyro_fault="spins without turning"), (now, 0.8), now),
          "lost")
    check("a drift check that disagrees with the map is poor",
          r.fit_of(status, dict(nav, map_drift={"trusted": True, "agrees": False}),
                   (now, 0.8), now), "poor")
    check("no map is lost", r.fit_of({"started": True, "map": None}, nav, (now, 0.8), now),
          "lost")
    check("nothing heard from the localizer is unknown", r.fit_of(None, nav, None, now),
          "unknown")
    base, odom = (2.0, 1.0, 0.5), (0.3, -0.2, 0.2)
    m = r.map_odom(base, odom)
    c, s = math.cos(m[2]), math.sin(m[2])
    check("map -> odom composed with odometry gives back the pose handed over",
          [round(v, 9) for v in (m[0] + c * odom[0] - s * odom[1],
                                 m[1] + s * odom[0] + c * odom[1], m[2] + odom[2])],
          [round(v, 9) for v in base])


TESTS = (test_packets_come_out_whole_however_the_reads_cut_them,
         test_the_fit_follows_the_agreed_rule)
