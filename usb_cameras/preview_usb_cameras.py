"""Preview the system's USB (UVC) cameras in a single window.

Left-click the window to cycle to the next camera, right-click to go back,
press q or Esc to quit. Only one camera is held open at a time, so the preview
still works when several cameras share a USB controller's bandwidth.

Every camera is put into full auto on open -- autofocus, auto exposure (which on
UVC governs both shutter time and gain, there being no separate control for
either) and auto white balance -- because whatever app used the camera last may
have left it pinned to manual. The overlay reports what each driver actually
accepted; `a` re-applies it and `s` opens the driver's own settings dialog for
controls OpenCV cannot reach.

The rover's OAK-D-Lite is not a UVC device and will not show up here -- use
oak_camera/preview_rgb.py for that one.
"""

import subprocess
import sys

import cv2

WINDOW = "USB camera preview"
# DirectShow hands out dense indices from 0, so probing a small range is enough.
MAX_PROBE = 8
PROBE_READS = 3  # some webcams hand back nothing until the stream warms up
REQUEST_SIZE = (1280, 720)
# A camera that vanishes mid-stream (unplugged, or grabbed by another app)
# returns failed reads forever rather than erroring, so give up after a second.
MAX_READ_FAILURES = 30
# UVC has no separate auto-gain or auto-shutter control: one auto-exposure flag
# drives both shutter time and gain (ISO), so these three cover everything a
# webcam automates. Anything else the driver offers lives behind CAP_PROP_SETTINGS.
AUTO_PROPS = [
    ("focus", cv2.CAP_PROP_AUTOFOCUS),
    ("exposure", cv2.CAP_PROP_AUTO_EXPOSURE),
    ("white balance", cv2.CAP_PROP_AUTO_WB),
]


def backends():
    """Capture backends to try, cheapest first."""
    if sys.platform != "win32":
        return [("default", cv2.CAP_ANY)]
    # DirectShow opens in tens of milliseconds; MSMF can take seconds per
    # device, so it is only a fallback for cameras DirectShow cannot see.
    return [("DirectShow", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)]


def silence_opencv():
    """Probing missing indices is noisy and the warnings are expected."""
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except AttributeError:
        pass


def device_names():
    """Friendly camera names in Windows' enumeration order, best effort."""
    if sys.platform != "win32":
        return []
    query = (
        "Get-CimInstance Win32_PnPEntity "
        "-Filter \"PNPClass='Camera' or PNPClass='Image'\" "
        "| Select-Object -ExpandProperty Name"
    )
    try:
        done = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", query],
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in done.stdout.splitlines() if line.strip()]


def probe(backend):
    """Indices that actually deliver a frame on this backend."""
    found = []
    for index in range(MAX_PROBE):
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            if any(cap.read()[0] for _ in range(PROBE_READS)):
                found.append(index)
        cap.release()
    return found


def find_cameras():
    for name, backend in backends():
        found = probe(backend)
        if found:
            return name, backend, found
    return None, None, []


def enable_auto(cap):
    """Hand focus, exposure and white balance back to the camera's automatics.

    Returns one (name, status) pair per control:

    * ``on``          -- the driver confirms auto is engaged.
    * ``set``         -- the driver accepted it but will not report the flag
                         back, so this is its word rather than a reading.
                         DirectShow answers -1 for auto-exposure on every camera
                         here, which is why the distinction exists.
    * ``refused``     -- accepted the call, still reports manual.
    * ``unsupported`` -- no such control on this camera.
    """
    report = []
    for name, prop in AUTO_PROPS:
        accepted = cap.set(prop, 1)
        value = cap.get(prop)
        if value == 1:
            status = "on"
        elif value < 0:  # driver declines to report this one at all
            status = "set" if accepted else "unsupported"
        else:
            status = "refused"
        report.append((name, status))
    return report


def open_camera(index, backend):
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None, []
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_SIZE[1])
    # Some drivers only accept control changes once the stream is live, so warm
    # it up before asking for auto -- and discard that first frame, which was
    # exposed under whatever settings the previous user of the camera left.
    cap.read()
    return cap, enable_auto(cap)


def describe(cap, index, label, position, total):
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return f"[{position + 1}/{total}] index {index} {label} {width}x{height}"


def annotate(frame, lines):
    # putText stretches glyph advances once thickness exceeds 1, so drawing the
    # outline as a heavier pass makes it drift right of the text it should
    # bound. Build the outline from offset copies and keep every pass at 1.
    def draw(text, origin, colour):
        cv2.putText(
            frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1,
            cv2.LINE_AA,
        )

    for row, text in enumerate(lines):
        x, y = 12, 28 + row * 24
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2),
                       (-1, -1), (1, -1), (-1, 1), (1, 1)):
            draw(text, (x + dx, y + dy), (0, 0, 0))
        draw(text, (x, y), (255, 255, 255))


def main():
    silence_opencv()
    print("Scanning for USB cameras...")
    backend_name, backend, cameras = find_cameras()
    if not cameras:
        print("No USB cameras found.", file=sys.stderr)
        return 1

    # OpenCV exposes no device names, so borrow Windows' list positionally.
    # DirectShow enumerates in the same order, including devices that refuse to
    # stream (IR sensors), which is why we index by capture index rather than by
    # our own position -- but any oddity in the PnP list shifts every label, so
    # treat these as a hint, not an identity.
    names = device_names()
    labels = [names[i] if i < len(names) else "" for i in cameras]
    print(f"{len(cameras)} camera(s) via {backend_name}: {cameras}")

    step = [0]  # click handler nudges this; the main loop consumes it

    def on_mouse(event, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            step[0] += 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            step[0] -= 1

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)

    position = 0
    cap = None
    try:
        while True:
            if cap is None:
                index = cameras[position]
                cap, auto = open_camera(index, backend)
                if cap is None:
                    print(f"Could not open index {index}.", file=sys.stderr)
                    if len(cameras) == 1:
                        return 1
                    position = (position + 1) % len(cameras)
                    continue
                caption = describe(
                    cap, index, labels[position], position, len(cameras)
                )
                auto_line = "auto: " + ", ".join(f"{n} {s}" for n, s in auto)
                print(f"{caption}\n  {auto_line}")
                failures = 0

            ok, frame = cap.read()
            if ok:
                failures = 0
                annotate(frame, [
                    f"{caption} -- click to cycle, q to quit",
                    f"{auto_line} -- a re-applies, s opens driver settings",
                ])
                cv2.imshow(WINDOW, frame)
            else:
                failures += 1
                if failures >= MAX_READ_FAILURES:
                    print(f"Lost index {cameras[position]}.", file=sys.stderr)
                    step[0] += 1

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("n"), 32):  # keyboard alternatives to clicking
                step[0] += 1
            elif key == ord("p"):
                step[0] -= 1
            elif key == ord("a"):
                auto = enable_auto(cap)
                auto_line = "auto: " + ", ".join(f"{n} {s}" for n, s in auto)
                print(f"  {auto_line}")
            elif key == ord("s"):
                # Escape hatch for controls OpenCV cannot reach -- the Brio's
                # focus, for one. DirectShow only; a no-op elsewhere.
                cap.set(cv2.CAP_PROP_SETTINGS, 1)
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

            if step[0]:
                position = (position + step[0]) % len(cameras)
                step[0] = 0
                cap.release()
                cap = None
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
