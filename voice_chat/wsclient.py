"""A blocking WebSocket client with no dependencies beyond the standard library.

The rover runs Raspbian on an ARMv6 Pi 1 where `apt` needs a password we do not
have non-interactively, so [talk_pi.py](talk_pi.py) cannot have the `websockets`
package. The protocol it needs is small and fixed -- one connection, no TLS, no
compression, no extensions -- so RFC 6455 is cheaper to implement than to
install.

Two things here are not optional and are easy to get wrong:

* Every client frame must be masked. Masking byte-by-byte in Python costs
  seconds per megabyte on a 700MHz ARM11, so it is done as a single big-integer
  XOR, which runs in C.
* uvicorn pings every 20s and drops the connection if nothing comes back, so
  `recv` answers pings itself rather than handing them to the caller.
"""

from __future__ import annotations

import base64
import hashlib
import os
import socket
import struct
import threading
from urllib.parse import urlsplit

# RFC 6455's fixed handshake salt.
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

CONTINUATION, TEXT, BINARY, CLOSE, PING, PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA

# A turn can sit silent while the GPU thinks; only a much longer gap is a fault.
DEFAULT_RECV_TIMEOUT_S = 120.0


class WebSocketError(Exception):
    pass


def _xor(payload: bytes, mask: bytes) -> bytes:
    """Apply a 4-byte mask. One big-int XOR, because a loop here is too slow.

    Byte-at-a-time masking of a 15-second utterance (~480KB) takes several
    seconds of bytecode on the Pi; as an integer it is a single C operation.
    """
    length = len(payload)
    if not length:
        return b""
    repeated = (mask * (length // 4 + 1))[:length]
    return (int.from_bytes(payload, "big") ^ int.from_bytes(repeated, "big")).to_bytes(
        length, "big"
    )


class WebSocket:
    """One connection. `send_bytes`/`send_text` are safe to call from a thread."""

    def __init__(
        self,
        url: str,
        connect_timeout: float = 10.0,
        recv_timeout: float = DEFAULT_RECV_TIMEOUT_S,
    ) -> None:
        parts = urlsplit(url)
        if parts.scheme != "ws":
            raise WebSocketError(f"only ws:// is supported, got {parts.scheme!r}")
        host, port = parts.hostname, parts.port or 80
        path = (parts.path or "/") + (f"?{parts.query}" if parts.query else "")

        self.sock = socket.create_connection((host, port), timeout=connect_timeout)
        # The turn is a request/response ping-pong of small messages; Nagle would
        # sit on the {"type":"end"} that releases the whole turn.
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buf = b""
        self._send_lock = threading.Lock()
        self.closed = False
        self._handshake(host, port, path)
        self.sock.settimeout(recv_timeout)

    def _handshake(self, host: str, port: int, path: str) -> None:
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET {path} HTTP/1.1\r\n"
                f"Host: {host}:{port}\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                "Sec-WebSocket-Version: 13\r\n"
                "\r\n"
            ).encode()
        )
        header = b""
        while b"\r\n\r\n" not in header:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise WebSocketError("server closed during the handshake")
            header += chunk
            if len(header) > 65536:
                raise WebSocketError("handshake response too large")
        head, _, self._buf = header.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        if "101" not in lines[0]:
            raise WebSocketError(f"handshake refused: {lines[0]}")

        # Proves the peer actually speaks WebSocket rather than being some other
        # server that happened to answer 101.
        want = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
        got = None
        for line in lines[1:]:
            name, _, value = line.partition(":")
            if name.strip().lower() == "sec-websocket-accept":
                got = value.strip()
        if got != want:
            raise WebSocketError("handshake accept header did not match")

    def _read(self, count: int) -> bytes:
        while len(self._buf) < count:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise WebSocketError("connection closed mid-frame")
            self._buf += chunk
        data, self._buf = self._buf[:count], self._buf[count:]
        return data

    def _read_frame(self) -> tuple[bool, int, bytes]:
        first, second = self._read(2)
        fin, opcode = bool(first & 0x80), first & 0x0F
        masked, length = bool(second & 0x80), second & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._read(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._read(8))[0]
        mask = self._read(4) if masked else None
        payload = self._read(length) if length else b""
        return fin, opcode, _xor(payload, mask) if mask else payload

    def _send_frame(self, opcode: int, payload: bytes = b"") -> None:
        mask = os.urandom(4)
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", length)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", length)
        header += mask
        with self._send_lock:
            self.sock.sendall(bytes(header) + _xor(payload, mask))

    def send_bytes(self, data: bytes) -> None:
        self._send_frame(BINARY, data)

    def send_text(self, text: str) -> None:
        self._send_frame(TEXT, text.encode())

    def recv(self) -> tuple[bool, bytes] | None:
        """One whole message as (is_binary, payload); None once the peer closes.

        Fragments are reassembled and control frames answered in here, so the
        caller only ever sees the messages the protocol is actually about.
        """
        data, is_binary = b"", False
        while True:
            fin, opcode, payload = self._read_frame()
            if opcode == PING:
                self._send_frame(PONG, payload)
                continue
            if opcode == PONG:
                continue
            if opcode == CLOSE:
                self.closed = True
                try:
                    self._send_frame(CLOSE, payload[:2])
                except OSError:
                    pass
                return None
            if opcode in (TEXT, BINARY):
                is_binary, data = opcode == BINARY, payload
            elif opcode == CONTINUATION:
                data += payload
            else:
                raise WebSocketError(f"unexpected opcode 0x{opcode:x}")
            if fin:
                return is_binary, data

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            try:
                self._send_frame(CLOSE, struct.pack(">H", 1000))
            except OSError:
                pass
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self) -> WebSocket:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
