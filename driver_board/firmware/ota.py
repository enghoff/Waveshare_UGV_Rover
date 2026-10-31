"""Update the driver board's firmware over Wi-Fi, from this desktop.

    python driver_board/firmware/ota.py <image.bin> [board address]

Uses the Arduino core's own espota.py (installed by build.sh) with the password
read from secrets/board-ota.key, so that it never appears on a command line. The
board writes the image into its other app slot and restarts into it; the daemon
loses the board for the few seconds of the restart and reconnects by itself. The
board connects back to this machine to fetch the image, so the desktop's
firewall must let it in on the port printed below.
"""
import importlib.util
import logging
import pathlib
import socket
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
ESPOTA = (pathlib.Path.home() / "AppData/Local/Arduino15/packages/esp32/hardware"
          / "esp32/3.0.4/tools/espota.py")
BOARD = "192.168.1.22"
BOARD_PORT = 3232
LOCAL_PORT = 3233


def main() -> int:
    image = pathlib.Path(sys.argv[1])
    board = sys.argv[2] if len(sys.argv) > 2 else BOARD
    password = (REPO / "secrets/board-ota.key").read_text().strip()
    spec = importlib.util.spec_from_file_location("espota", ESPOTA)
    espota = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(espota)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    espota.TIMEOUT = 10  # espota's main() sets these from its options
    espota.PROGRESS = False
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.connect((board, BOARD_PORT))  # picks the interface that reaches it
    local = probe.getsockname()[0]
    probe.close()
    print("updating %s from %s:%d with %s" % (board, local, LOCAL_PORT, image))
    return espota.serve(board, local, BOARD_PORT, LOCAL_PORT, password, str(image))


if __name__ == "__main__":
    sys.exit(main())
