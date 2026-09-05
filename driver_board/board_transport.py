"""Direct board transports shared by gamepad and face-tracking bench tools.

HTTP retries a stale connection once. Serial sends one compact JSON line and
discards telemetry these one-way tools do not consume. The daemon owns its own
bidirectional board connection; these tools must not share its open UART.
"""
import json

BAUD = 115200

def js_path(command):
    """A command as the board wants it: JSON in the query string of `/js`."""
    from urllib.parse import quote

    return "/js?json=" + quote(json.dumps(command, separators=(",", ":")), safe="")

class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint.

    A fresh connection per command would mean a TCP handshake 20 times a second
    at an ESP32; this keeps one open and rebuilds it only when it breaks, which
    also makes a lost link visible as an error rather than a silent stall.
    """

    def __init__(self, host, timeout=0.5):
        import http.client

        self._client = http.client
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = js_path(command)
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a command
            if self.connection is None:
                self.connection = self._client.HTTPConnection(self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", path)
                self.connection.getresponse().read()
                return True
            except Exception:
                self.close()
                if attempt == 2:
                    return False
        return False

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None

class SerialLink:
    """JSON commands over the ESP32's Type-C port -- the one *not* labelled LIDAR."""

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            self.link.reset_input_buffer()  # the board chatters; nothing here reads it
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass

class NoLink:
    """--no-move: everything runs, nothing is commanded."""

    def describe(self):
        return "nothing (--no-move)"

    def send(self, command):
        return True

    def close(self):
        pass
