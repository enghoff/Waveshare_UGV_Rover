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

The window is resizable and kept square, so the plot is a disc at any size and
the margin, dot size and text scale with it. On Windows the drag itself is
constrained -- see keep_window_square -- and elsewhere the window is squared up
once the drag ends; either way the frame is drawn to the shorter side and padded,
so a window that is momentarily not square shows bars rather than an ellipse.

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
WINDOW = "D500 lidar"
INITIAL_CANVAS = 820  # the window opens square and is held square as it is dragged
# The plot is drawn as a disc inscribed in the shorter side, so a window that is
# not square (a drag in flight, or a window manager that ignored the constraint)
# gets bars rather than a stretched picture.
MARGIN_FRAC = 0.05
UI_SCALE_LIMITS = (0.6, 2.0)  # relative to INITIAL_CANVAS, so text stays legible
MIN_SIDE = 40  # below this the window is being minimised, not resized
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
    cx, cy = centre
    angle = bearing_rad - np.deg2rad(VIEW_ROTATION_DEG)
    return (cx + radius_px * np.sin(angle), cy - radius_px * np.cos(angle))


def render(points, packet_no, max_range_mm, colour_by_intensity, rotation_hz, side):
    """Draw the scene into a square `side` x `side` canvas."""
    canvas = np.zeros((side, side, 3), np.uint8)
    centre = (side // 2, side // 2)
    ui = min(max(side / INITIAL_CANVAS, UI_SCALE_LIMITS[0]), UI_SCALE_LIMITS[1])
    margin = side * MARGIN_FRAC
    radius_px = max(side / 2 - margin, 1.0)
    scale = radius_px / max_range_mm
    dot = max(1, round(2 * ui))

    ring_mm = 1000 if max_range_mm <= 4000 else 2000
    for r in range(ring_mm, max_range_mm + 1, ring_mm):
        cv2.circle(canvas, centre, int(r * scale), (48, 48, 48), 1)
        cv2.putText(
            canvas, f"{r // 1000}m",
            (centre[0] + int(4 * ui), centre[1] - int(r * scale) + int(14 * ui)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4 * ui, (90, 90, 90), 1,
        )
    # Heading marker. The view rotation cancels the lidar's mounting offset, so
    # screen-up is the rover's own 0 deg heading -- which is what this marks,
    # rather than the sensor's zero. Drawn through to_screen so it stays vertical
    # for any VIEW_ROTATION_DEG instead of being a hardcoded straight-up line.
    heading = to_screen(np.deg2rad(VIEW_ROTATION_DEG), radius_px + margin / 2, centre)
    cv2.line(canvas, centre, (int(heading[0]), int(heading[1])), (60, 90, 60), 1)

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
            cv2.circle(canvas, (x, y), dot, tuple(int(c) for c in colour), -1)

    cv2.circle(canvas, centre, max(2, round(4 * ui)), (200, 200, 200), -1)
    lines = [
        f"{len(index)} points   {max_range_mm // 1000} m range",
        f"{rotation_hz:.2f} Hz   colour: {'intensity' if colour_by_intensity else 'distance'}",
    ]
    for n, text in enumerate(lines):
        cv2.putText(
            canvas, text, (int(10 * ui), int((22 + 20 * n) * ui)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5 * ui, (220, 220, 220), max(1, round(ui)),
        )
    return canvas


def keep_window_square(name):
    """Have Windows itself refuse a non-square window mid-drag.

    highgui has no aspect constraint, and a Win32 resize drag runs inside the
    system's own modal loop: waitKey does not return until the mouse is
    released, so nothing here can redraw while the window is being dragged --
    Windows just stretches the last frame, which turns the disc into an
    ellipse. Squaring up afterwards therefore fixes the picture but not the
    drag. So the shape is constrained at the source instead, by intercepting
    WM_SIZING and squaring the rectangle Windows is about to apply.

    The edge being dragged decides the side, and the opposite edge stays put:
    drag the right edge and the window widens and heightens together, exactly
    as SquareWindow does after the fact. `chrome` is the difference between the
    outer window and the image area, measured once, so it is the *image* that
    ends up square whatever the border and title bar cost.

    Returns a callable restoring the original window procedure, or None off
    Win32 -- there the caller falls back to snapping when the drag ends.
    """
    if sys.platform != "win32":
        return None

    import ctypes
    from ctypes import wintypes

    WM_SIZING, GWLP_WNDPROC = 0x0214, -4
    LEFT_EDGES = (1, 4, 7)  # WMSZ_LEFT, WMSZ_TOPLEFT, WMSZ_BOTTOMLEFT
    TOP_EDGES = (3, 4, 5)  # WMSZ_TOP, WMSZ_TOPLEFT, WMSZ_TOPRIGHT
    HORIZONTAL_ONLY, VERTICAL_ONLY = (1, 2), (3, 6)

    user32 = ctypes.WinDLL("user32", use_last_error=True)
    hwnd = user32.FindWindowW(None, name)
    if not hwnd:
        return None

    outer = wintypes.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(outer))
    _, _, image_width, image_height = cv2.getWindowImageRect(name)
    chrome = (outer.right - outer.left - image_width,
              outer.bottom - outer.top - image_height)
    if min(chrome) < 0:  # the window is not laid out the way we assumed
        return None

    WNDPROC = ctypes.WINFUNCTYPE(
        ctypes.c_ssize_t, wintypes.HWND, ctypes.c_uint,
        ctypes.c_size_t, ctypes.c_ssize_t,
    )
    # 32-bit Python has no ...PtrW; there the plain LONG version is the same call.
    set_long = getattr(user32, "SetWindowLongPtrW", None) or user32.SetWindowLongW
    set_long.restype = ctypes.c_ssize_t
    set_long.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_ssize_t]
    user32.CallWindowProcW.restype = ctypes.c_ssize_t
    user32.CallWindowProcW.argtypes = [
        ctypes.c_ssize_t, wintypes.HWND, ctypes.c_uint,
        ctypes.c_size_t, ctypes.c_ssize_t,
    ]

    previous = None

    def window_proc(handle, message, wparam, lparam):
        # Anything raised here would surface inside Windows' own message
        # dispatch, so the constraint is best-effort: on any surprise the
        # message goes through to highgui untouched.
        try:
            if message == WM_SIZING:
                rect = ctypes.cast(lparam, ctypes.POINTER(wintypes.RECT)).contents
                width = rect.right - rect.left - chrome[0]
                height = rect.bottom - rect.top - chrome[1]
                if wparam in HORIZONTAL_ONLY:
                    side = width
                elif wparam in VERTICAL_ONLY:
                    side = height
                else:  # a corner: follow whichever way the pointer pulled harder
                    side = max(width, height)
                side = max(side, MIN_SIDE)
                if wparam in LEFT_EDGES:
                    rect.left = rect.right - side - chrome[0]
                else:
                    rect.right = rect.left + side + chrome[0]
                if wparam in TOP_EDGES:
                    rect.top = rect.bottom - side - chrome[1]
                else:
                    rect.bottom = rect.top + side + chrome[1]
        except Exception:
            pass
        return user32.CallWindowProcW(previous, handle, message, wparam, lparam)

    hook = WNDPROC(window_proc)
    previous = set_long(hwnd, GWLP_WNDPROC, ctypes.cast(hook, ctypes.c_void_p).value)
    if not previous:
        return None

    def restore():
        set_long(hwnd, GWLP_WNDPROC, previous)

    # The trampoline has to outlive this function: let it be collected and the
    # next resize calls into freed memory, so it rides along on the restorer.
    restore.hook = hook
    return restore


class SquareWindow:
    """A resizable highgui window held square, with the frame padded to fit.

    The plot is a disc, so the window is kept square rather than the picture
    stretched to whatever shape the window is dragged to. Windows is made to
    refuse a non-square drag outright (see keep_window_square); everything that
    hook never sees -- maximising, Win+arrow snapping, any platform without it --
    is squared up here after the fact. Until that lands the caller draws to
    `side` and the frame is padded, so the worst case is bars, never an ellipse.
    """

    def __init__(self, name, side):
        self.name = name
        self.size = (side, side)
        self._restore = None
        self._hooked = False
        # WINDOW_NORMAL is what makes the window resizable in the first place.
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, side, side)

    @property
    def side(self):
        """What to draw at: the shorter side, so the disc always fits."""
        return min(self.size)

    @property
    def closed(self):
        return cv2.getWindowProperty(self.name, cv2.WND_PROP_VISIBLE) < 1

    def show(self, plot):
        cv2.imshow(self.name, self._pad(plot))
        # The chrome can only be measured once a frame has been drawn, so the
        # constraint goes on here rather than in __init__ -- and only once,
        # whether or not this platform has one to install.
        if not self._hooked:
            self._hooked = True
            self._restore = keep_window_square(self.name)
        self._resquare()

    def close(self):
        if self._restore:  # highgui's own proc back before it tears the window down
            self._restore()
        try:
            cv2.destroyWindow(self.name)
        except cv2.error:  # the user already closed it
            pass

    def _pad(self, plot):
        width, height = self.size
        side = plot.shape[0]
        if (width, height) == (side, side):
            return plot
        left, top = (width - side) // 2, (height - side) // 2
        if left < 0 or top < 0:  # a frame drawn before the window shrank
            return plot
        return cv2.copyMakeBorder(
            plot, top, height - side - top, left, width - side - left,
            cv2.BORDER_CONSTANT, value=(0, 0, 0),
        )

    def _resquare(self):
        """Re-read the window, and pull it back to square if it is not.

        Whichever side moved further is taken as the one the user meant, so
        dragging the right edge widens *and* heightens rather than being undone
        by a snap back to the shorter side. This only runs when the rect has
        actually changed, so a window manager that declines the resize is asked
        once rather than every frame.

        A minimised window reports zeroes and a closed one raises; both keep the
        last known size, and the caller stops on `closed`.
        """
        try:
            _, _, width, height = cv2.getWindowImageRect(self.name)
        except cv2.error:
            return
        if width < MIN_SIDE or height < MIN_SIDE:
            return
        previous, self.size = self.size, (width, height)
        if self.size == previous or width == height:
            return
        moved_wider = abs(width - previous[0]) >= abs(height - previous[1])
        side = width if moved_wider else height
        cv2.resizeWindow(self.name, side, side)


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

        window = SquareWindow(WINDOW, INITIAL_CANVAS)

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

            # Checked before drawing, not after: imshow on a window the user has
            # closed silently builds a new one -- and an AUTOSIZE one, so the view
            # would come back unresizable and the loop would never end.
            if window.closed:
                break

            plot = render(
                points, packet_no, RANGE_STEPS_MM[range_index], colour_by_intensity,
                rotation_hz, window.side,
            )
            window.show(plot)

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
                cv2.imwrite(name, plot)  # the square plot, without the padding
                print(f"wrote {name}")

        window.close()


if __name__ == "__main__":
    main()
