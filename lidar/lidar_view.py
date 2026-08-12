"""Live top-down view of the D500 (STL-19P / LD19) lidar on the Waveshare UGV Rover.

Reads the lidar's serial stream off the rover driver board's LIDAR Type-C port
and draws the point cloud looking down on the robot. The lidar is mounted 90° off
the chassis, so the view is swung 90° counter-clockwise to cancel that: the
rover's own 0° heading ends up straight **up** the window, marked by the green
line. See VIEW_ROTATION_DEG.

    python lidar_view.py            # auto-detect the port
    python lidar_view.py COM14

Keys: q quit, [ and ] change the range shown, s save a PNG, c toggle colouring
between intensity and distance.

The rover's main power switch gates the lidar's 5V -- the serial port enumerates
over USB whether or not the robot is on, so an empty window with a live COM port
means the switch is off, not that the cable is wrong.
"""

import struct
import sys
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

BAUD = 230400
# WCH CH343 on the current driver board revision; CP2102 on the older one and on
# the standalone D500 adapter board.
KNOWN_VID_PID = {(0x1A86, 0x55D3), (0x1A86, 0x7523), (0x10C4, 0xEA60)}

PACKET_LEN = 47  # header, ver/len, speed, start angle, 12x(dist,intensity), end angle, timestamp, crc
POINTS_PER_PACKET = 12
HEADER = b"\x54\x2c"

BINS = 3600  # 0.1 degree resolution
CANVAS = 820
MARGIN = 40
# One revolution is ~42 packets at 10 Hz. Holding a little over one revolution
# keeps the picture complete without smearing when the robot turns.
STALE_AFTER_PACKETS = 60

RANGE_STEPS_MM = [1000, 2000, 4000, 6000, 8000, 12000]
DEFAULT_RANGE_INDEX = 3

# How far to swing the scene counter-clockwise, in degrees, to undo how the lidar
# sits on the rover: the sensor's zero points 90 deg off the chassis, so 90 here
# lines the picture up with the robot and leaves 0 deg heading pointing up.
# This rotates only what is drawn -- the parsed angles, and so any distance you
# read off a point, are untouched. The HUD text and range labels stay upright,
# which is why it is applied to the point geometry and not the finished canvas.
VIEW_ROTATION_DEG = 90


def crc8_table():
    """LD19 uses a bytewise CRC-8 with polynomial 0x4d, init 0, no reflection."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = crc8_table()


def crc8(data):
    c = 0
    for byte in data:
        c = CRC_TABLE[c ^ byte]
    return c


def find_port():
    for port in list_ports.comports():
        if (port.vid, port.pid) in KNOWN_VID_PID:
            return port.device
    return None


def parse(buffer, points, packet_no):
    """Consume whole packets from buffer, writing into points.

    Returns (bytes eaten, next packet number, rotation speed of the last packet in
    deg/s or None). points is (BINS, 3) float32: distance mm, intensity, packet
    number last written.
    """
    i = 0
    speed = None
    limit = len(buffer) - PACKET_LEN
    while i <= limit:
        if buffer[i] != 0x54 or buffer[i + 1] != 0x2C:
            i += 1
            continue
        packet = buffer[i : i + PACKET_LEN]
        if crc8(packet[:-1]) != packet[-1]:
            i += 1
            continue

        speed = struct.unpack("<H", packet[2:4])[0]
        start, end = struct.unpack("<HH", packet[4:6] + packet[42:44])
        span = (end - start) % 36000
        step = span / (POINTS_PER_PACKET - 1)

        for k in range(POINTS_PER_PACKET):
            distance, intensity = struct.unpack("<HB", packet[6 + 3 * k : 9 + 3 * k])
            if distance == 0:
                continue
            angle = (start + step * k) % 36000
            points[int(angle * BINS / 36000)] = (distance, intensity, packet_no)

        packet_no += 1
        i += PACKET_LEN
    return i, packet_no, speed


def to_screen(bearing_rad, radius_px, centre):
    """Sensor bearing (0 = front, growing clockwise) -> canvas pixel."""
    angle = bearing_rad - np.deg2rad(VIEW_ROTATION_DEG)
    return (centre + radius_px * np.sin(angle), centre - radius_px * np.cos(angle))


def render(points, packet_no, max_range_mm, colour_by_intensity, rotation_hz):
    canvas = np.zeros((CANVAS, CANVAS, 3), np.uint8)
    centre = CANVAS // 2
    scale = (CANVAS / 2 - MARGIN) / max_range_mm

    ring_mm = 1000 if max_range_mm <= 4000 else 2000
    for r in range(ring_mm, max_range_mm + 1, ring_mm):
        cv2.circle(canvas, (centre, centre), int(r * scale), (48, 48, 48), 1)
        cv2.putText(
            canvas, f"{r // 1000}m", (centre + 4, centre - int(r * scale) + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1,
        )
    # Heading marker. The view rotation cancels the lidar's mounting offset, so
    # screen-up is the rover's own 0 deg heading -- which is what this marks,
    # rather than the sensor's zero. Drawn through to_screen so it stays vertical
    # for any VIEW_ROTATION_DEG instead of being a hardcoded straight-up line.
    heading = to_screen(np.deg2rad(VIEW_ROTATION_DEG), centre - MARGIN // 2, centre)
    cv2.line(canvas, (centre, centre), (int(heading[0]), int(heading[1])), (60, 90, 60), 1)

    fresh = points[:, 2] > packet_no - STALE_AFTER_PACKETS
    visible = fresh & (points[:, 0] > 0) & (points[:, 0] <= max_range_mm)
    index = np.nonzero(visible)[0]

    if len(index):
        # Left-handed frame: zero is the front of the sensor, angle grows clockwise.
        theta = index * (2 * np.pi / BINS)
        radius = points[index, 0] * scale
        xs, ys = to_screen(theta, radius, centre)
        xs = xs.astype(np.int32)
        ys = ys.astype(np.int32)

        if colour_by_intensity:
            key = np.clip(points[index, 1], 0, 255).astype(np.uint8)
        else:
            key = (255 - points[index, 0] / max_range_mm * 255).astype(np.uint8)
        colours = cv2.applyColorMap(key.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)

        for x, y, colour in zip(xs, ys, colours):
            cv2.circle(canvas, (x, y), 2, tuple(int(c) for c in colour), -1)

    cv2.circle(canvas, (centre, centre), 4, (200, 200, 200), -1)
    lines = [
        f"{len(index)} points   {max_range_mm // 1000} m range",
        f"{rotation_hz:.2f} Hz   colour: {'intensity' if colour_by_intensity else 'distance'}",
    ]
    for n, text in enumerate(lines):
        cv2.putText(
            canvas, text, (10, 22 + 20 * n),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1,
        )
    return canvas


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if port is None:
        sys.exit("No lidar serial port found. Pass one explicitly, e.g. lidar_view.py COM14")

    points = np.zeros((BINS, 3), np.float32)
    points[:, 2] = -STALE_AFTER_PACKETS
    buffer = bytearray()
    packet_no = 0
    range_index = DEFAULT_RANGE_INDEX
    colour_by_intensity = True
    rotation_hz = 0.0

    with serial.Serial(port, BAUD, timeout=0.1) as link:
        print(f"reading {port} at {BAUD}; q to quit")
        link.reset_input_buffer()
        last_draw = 0.0

        while True:
            chunk = link.read(4096)
            if chunk:
                buffer += chunk
                eaten, packet_no, speed = parse(buffer, points, packet_no)
                if speed:
                    rotation_hz = speed / 360
                del buffer[:eaten]

            now = time.monotonic()
            if now - last_draw < 1 / 30:
                continue
            last_draw = now

            canvas = render(
                points, packet_no, RANGE_STEPS_MM[range_index], colour_by_intensity, rotation_hz
            )
            cv2.imshow("D500 lidar", canvas)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("]"):
                range_index = min(range_index + 1, len(RANGE_STEPS_MM) - 1)
            elif key == ord("["):
                range_index = max(range_index - 1, 0)
            elif key == ord("c"):
                colour_by_intensity = not colour_by_intensity
            elif key == ord("s"):
                name = f"lidar-{time.strftime('%Y%m%dT%H%M%S')}.png"
                cv2.imwrite(name, canvas)
                print(f"wrote {name}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
