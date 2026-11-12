"""The rover's own sound: the conversation through Bluetooth speakers and headsets.

Until this file the browser was the only microphone and the only speaker. Now a
conversation started from the console's microphone button sends the model's
voice to whatever Bluetooth audio the rover has connected, and listens through
a connected headset's microphone instead of the browser's:

    a speaker connected        the voice plays there; the browser stays silent
    a headset connected        voice and microphone both on the headset
    both                       the headset, as if the speaker were not there
    neither                    the browser, as before

**A speaker and a headset are never streamed at once** ("One link at a time",
below).
The console's button still starts and ends the conversation; this only
changes where its sound goes.

**A speaker in the room is heard by every microphone in it**, the browser's
included, and the service's own turn detection takes the rover's voice for
somebody interrupting it. So while the voice is coming out of a speaker the
microphone is not listened to, and for SPEAKER_TAIL_S after; a worn headset or
pair of headphones is exempt, because nothing leaks from it. The cost is that
nobody can talk over the rover through a speaker -- the STOP button still works.

The setup that makes Bluetooth audio possible on the Orin at all is
`install_bt_audio.sh`; pairing is the daemon's (rover_daemon/rover_bluetooth.py).
Everything here talks to PipeWire through its own command-line tools --
`pw-dump` to read the graph, `wpctl` to put a headset into its microphone
profile, and ALSA's `aplay`/`arecord` through the `pipewire` device for raw PCM
-- so the console takes on no new library.
"""
from __future__ import annotations

import collections
import json
import os
import subprocess
import threading
import time
from typing import Any, Callable

import numpy as np

import _paths  # noqa: F401 - session.py, beside the console in the repository
from session import IN_RATE, OUT_RATE, _to_pcm16

#: How far ahead of the clock audio is handed to aplay. Everything written is
#: gone past recall -- an interruption cannot take it back -- so this is kept to
#: a few blocks: enough that a scheduling hiccup on a busy Orin is not a click.
LEAD_S = 0.08
#: What the Bluetooth link itself adds after that, for `played_ms`: A2DP is
#: commonly 150-250 ms from the source to the ear. Over-stating it makes the
#: model believe a little less was heard than was, which it recovers from;
#: under-stating it makes it believe it said words nobody heard yet.
LINK_LATENCY_S = 0.2
#: One block of output, in seconds, which is also the pacing step.
BLOCK_S = 0.02
#: How long after the voice stops before the microphone is listened to again,
#: when the voice came out of a speaker: the link's latency and the room's echo.
SPEAKER_TAIL_S = 0.5
#: Form factors that are worn, and so leak nothing into a microphone.
WORN = ("headset", "headphone", "hands-free")

# One link at a time: why a headset takes the voice off a speaker rather than
# sharing with it. The Orin's one radio card does Bluetooth and its 2.4 GHz
# wi-fi by taking turns, and on 2026-11-12 (the owner's JBL Flip and Sony
# WI-XB400, wi-fi on channel 6, no 5 GHz network in reach) the rover's wi-fi
# upload, alternated two or three times each way, was about 7 Mbit/s with no
# Bluetooth audio, 3 with the speaker's A2DP stream, 5 with the headset's
# hands-free microphone -- and 0.3 to 2.3 with both. A conversation sends
# 0.35 Mbit/s of microphone and the console's pictures about as much again;
# with both links up the service's keepalive went unanswered for 20 s at a
# time, replies came half a minute late and the voice broke up.
#: One read of the microphone: 20 ms at the model's input rate.
MIC_BLOCK = int(IN_RATE * 0.02) * 2


def _env() -> dict[str, str]:
    """The environment for a PipeWire tool, which finds the server by its user's
    runtime directory -- one a console started from cron does not have set."""
    env = dict(os.environ)
    if not env.get("XDG_RUNTIME_DIR") and hasattr(os, "getuid"):
        env["XDG_RUNTIME_DIR"] = f"/run/user/{os.getuid()}"
    return env


def pw_dump() -> list[dict[str, Any]]:
    """PipeWire's whole graph, as `pw-dump` prints it."""
    done = subprocess.run(["pw-dump"], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=5, env=_env())
    if done.returncode != 0:
        raise OSError(done.stderr.decode("utf-8", "replace").strip()
                      or f"pw-dump failed ({done.returncode})")
    return json.loads(done.stdout or b"[]")


def bt_audio(objects: list[dict[str, Any]]) -> dict[str, Any]:
    """The Bluetooth devices in a PipeWire graph, with their outputs and inputs."""
    devices: dict[int, dict[str, Any]] = {}
    nodes = []
    for obj in objects:
        kind = str(obj.get("type", "")).rsplit(":", 1)[-1]
        info = obj.get("info") or {}
        props = info.get("props") or {}
        if kind == "Device" and props.get("device.api") == "bluez5":
            params = info.get("params") or {}
            profiles = [{"index": p.get("index"), "name": str(p.get("name", "")),
                         "available": p.get("available", "unknown")}
                        for p in params.get("EnumProfile") or []]
            active = (params.get("Profile") or [{}])[0].get("name", "")
            devices[obj["id"]] = {
                "id": obj["id"],
                "address": str(props.get("api.bluez5.address", "")).upper(),
                "name": str(props.get("device.description")
                            or props.get("device.alias") or props.get("device.name")),
                "form": str(props.get("device.form-factor", "")),
                "profiles": profiles, "profile": str(active),
                "headset": any(p["name"].startswith("headset-head-unit")
                               and p["available"] != "no" for p in profiles)}
        elif kind == "Node":
            media = props.get("media.class")
            if media in ("Audio/Sink", "Audio/Source") and props.get("device.id"):
                nodes.append((int(props["device.id"]), media,
                              str(props.get("node.name", ""))))
    for device in devices.values():
        device["sinks"] = [n for d, m, n in nodes
                           if d == device["id"] and m == "Audio/Sink"]
        device["sources"] = [n for d, m, n in nodes
                             if d == device["id"] and m == "Audio/Source"]
    return {"devices": sorted(devices.values(), key=lambda d: d["name"])}


def choose(audio: dict[str, Any]) -> dict[str, Any]:
    """Where the voice goes and which microphone is listened to.

    The first headset (by name) is the microphone, and is asked into its
    hands-free profile if it is not in it: in A2DP a headset has no microphone
    at all. The voice then goes to that headset too, over the same hands-free
    link, and to nothing else -- see "One link at a time". Until the profile has changed
    its outputs are its music ones, which are about to go away, so the voice
    stays in the browser meanwhile. With no headset the voice goes to every
    speaker.
    """
    devices = audio.get("devices") or []
    mic_device = next((d for d in devices if d["headset"]), None)
    if mic_device is None:
        outputs = [(d, s) for d in devices for s in d["sinks"]]
    elif mic_device["profile"].startswith("headset-head-unit"):
        outputs = [(mic_device, s) for s in mic_device["sinks"]]
    else:
        outputs = []
    choice = {"outputs": [s for _, s in outputs],
              "out_names": [d["name"] for d, _ in outputs],
              "mic": None, "mic_name": "", "profile": None,
              "gate": any(d["form"] not in WORN for d, _ in outputs)}
    if mic_device is not None:
        choice["mic_name"] = mic_device["name"]
        if mic_device["sources"]:
            choice["mic"] = mic_device["sources"][0]
        if not mic_device["profile"].startswith("headset-head-unit"):
            wanted = [p for p in mic_device["profiles"]
                      if p["name"].startswith("headset-head-unit")
                      and p["available"] != "no"]
            wanted.sort(key=lambda p: p["name"] != "headset-head-unit")
            if wanted:
                choice["profile"] = (mic_device["id"], wanted[0]["index"])
    return choice


def _player(node: str) -> subprocess.Popen:
    env = _env()
    env["PIPEWIRE_NODE"] = node
    return subprocess.Popen(
        ["aplay", "-q", "-D", "pipewire", "-t", "raw", "-f", "S16_LE",
         "-r", str(OUT_RATE), "-c", "1", "--buffer-time=100000",
         "--period-time=25000", "-"],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, env=env)


def _recorder(node: str) -> subprocess.Popen:
    env = _env()
    env["PIPEWIRE_NODE"] = node
    return subprocess.Popen(
        ["arecord", "-q", "-D", "pipewire", "-t", "raw", "-f", "S16_LE",
         "-r", str(IN_RATE), "-c", "1", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env)


class LocalSpeaker:
    """The speaker interface `Session` calls, played on the rover's Bluetooth.

    The same four questions `BrowserSpeaker` answers. `played_ms` is worked out
    from the clock rather than reported, since nothing on the far side of a
    Bluetooth link reports anything: audio is handed over a block at a time, no
    more than LEAD_S ahead of when it is due, and counts as heard once it is due
    plus LINK_LATENCY_S. That keeps what an interruption cannot take back to a
    few blocks, which is the whole reason it is paced rather than piped.
    """

    rate = OUT_RATE

    def __init__(self, launch: Callable[[str], Any] = _player,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._launch = launch
        self._clock = clock
        self._cond = threading.Condition()
        self._blocks: collections.deque[bytes] = collections.deque()
        self._targets: list[str] = []
        self._players: dict[str, Any] = {}
        self._queued_ms = 0.0       # handed to this speaker for this reply
        self._written_ms = 0.0      # ...of which given to aplay
        self._cursor = 0.0          # when the last block written is due to end
        self._closed = False
        self._thread = threading.Thread(target=self._feed, daemon=True,
                                        name="bt-speaker")
        self._thread.start()

    # --- where it plays --------------------------------------------------------

    def route(self, nodes: list[str]) -> None:
        """Play to these outputs from the next block on."""
        with self._cond:
            self._targets = list(nodes)
            for node in [n for n in self._players if n not in nodes]:
                self._stop(self._players.pop(node))

    @property
    def available(self) -> bool:
        with self._cond:
            return bool(self._targets)

    # --- what Session calls ------------------------------------------------------

    def begin(self) -> None:
        with self._cond:
            self._blocks.clear()
            self._queued_ms = 0.0
            self._written_ms = 0.0

    def write(self, audio: np.ndarray) -> None:
        pcm = _to_pcm16(audio)
        step = int(OUT_RATE * BLOCK_S) * 2
        with self._cond:
            for start in range(0, len(pcm), step):
                self._blocks.append(pcm[start:start + step])
            self._queued_ms += len(pcm) / 2 * 1000.0 / OUT_RATE
            self._cond.notify()

    def played_ms(self) -> int:
        with self._cond:
            pending = max(0.0, self._cursor + LINK_LATENCY_S - self._clock())
            return int(max(0.0, min(self._queued_ms,
                                    self._written_ms - pending * 1000.0)))

    def flush(self) -> float:
        """Drop what has not been handed over, and say how many seconds went."""
        with self._cond:
            played = self._written_ms - max(
                0.0, self._cursor + LINK_LATENCY_S - self._clock()) * 1000.0
            dropped = max(0.0, self._queued_ms - max(0.0, played)) / 1000.0
            self._blocks.clear()
            self._queued_ms = self._written_ms
        return dropped

    @property
    def busy(self) -> bool:
        return self.played_ms() + 50 < self._queued_ms

    def quiet_since(self) -> float:
        """When the last of the voice reached the ear, on the monotonic clock;
        in the future while it is still playing."""
        with self._cond:
            if self._blocks:
                return float("inf")
            return self._cursor + LINK_LATENCY_S

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._blocks.clear()
            for player in self._players.values():
                self._stop(player)
            self._players.clear()
            self._cond.notify()

    # --- the feeding thread ----------------------------------------------------

    def _feed(self) -> None:
        while True:
            with self._cond:
                while not self._blocks and not self._closed:
                    self._cond.wait()
                if self._closed:
                    return
                now = self._clock()
                start = max(now, self._cursor)
                wait = start - LEAD_S - now
            if wait > 0:
                time.sleep(wait)
                continue
            with self._cond:
                if not self._blocks:
                    continue        # flushed while waiting
                block = self._blocks.popleft()
                targets = list(self._targets)
                start = max(self._clock(), self._cursor)
                self._cursor = start + len(block) / 2 / OUT_RATE
                self._written_ms += len(block) / 2 * 1000.0 / OUT_RATE
                players = []
                for node in targets:
                    if node not in self._players:
                        try:
                            self._players[node] = self._launch(node)
                        except OSError:
                            continue
                    players.append((node, self._players[node]))
            for node, player in players:
                try:
                    player.stdin.write(block)
                    player.stdin.flush()
                except (OSError, ValueError, AttributeError):
                    with self._cond:
                        if self._players.get(node) is player:
                            self._stop(self._players.pop(node))

    @staticmethod
    def _stop(player: Any) -> None:
        try:
            player.stdin.close()
        except Exception:
            pass
        try:
            player.terminate()
        except Exception:
            pass


class LocalMic:
    """A headset's microphone, read 20 ms at a time into a callback."""

    def __init__(self, node: str, deliver: Callable[[bytes], None],
                 launch: Callable[[str], Any] = _recorder) -> None:
        self.node = node
        self._deliver = deliver
        self._process = launch(node)
        self._thread = threading.Thread(target=self._read, daemon=True,
                                        name="bt-mic")
        self._thread.start()

    def _read(self) -> None:
        stream = self._process.stdout
        while True:
            try:
                pcm = stream.read(MIC_BLOCK)
            except (OSError, ValueError):
                return
            if not pcm:
                return
            self._deliver(pcm)

    @property
    def alive(self) -> bool:
        return self._process.poll() is None

    def close(self) -> None:
        try:
            self._process.terminate()
        except Exception:
            pass


class RoverSound:
    """Which Bluetooth audio the conversation uses, kept up to date.

    `refresh` is called every few seconds while a conversation is live, from a
    worker thread; everything else is cheap and called from anywhere.
    """

    def __init__(self, deliver: Callable[[bytes], None],
                 read: Callable[[], list] = pw_dump,
                 launch_player: Callable[[str], Any] = _player,
                 launch_mic: Callable[[str], Any] = _recorder,
                 set_profile: Callable[[int, int], None] | None = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._deliver = deliver
        self._read = read
        self._launch_mic = launch_mic
        self._set_profile = set_profile or _wpctl_profile
        self._clock = clock
        self.speaker = LocalSpeaker(launch_player, clock)
        self._lock = threading.Lock()
        self._mic: LocalMic | None = None
        self._choice: dict[str, Any] = choose({})
        self._asked_profile: tuple[int, int] | None = None
        # Between conversations nothing is recorded. `refresh` runs on a worker
        # thread that a conversation ending cannot interrupt, so it checks this
        # rather than trusting that it was not called after `stop`.
        self._active = False
        self.error = ""

    def start(self) -> None:
        with self._lock:
            self._active = True

    def refresh(self) -> None:
        with self._lock:
            if not self._active:
                return
        try:
            choice = choose(bt_audio(self._read()))
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            with self._lock:
                self.error = f"no sound server: {error}"[:160]
            choice = choose({})
        else:
            with self._lock:
                self.error = ""
        if choice["profile"] and choice["profile"] != self._asked_profile:
            # Asked once per headset: a profile switch takes a second or two,
            # and its microphone appears on a later refresh.
            self._asked_profile = choice["profile"]
            try:
                self._set_profile(*choice["profile"])
            except (OSError, subprocess.SubprocessError) as error:
                with self._lock:
                    self.error = f"the headset's microphone profile: {error}"[:160]
        with self._lock:
            if not self._active:
                return
            self.speaker.route(choice["outputs"])
            mic = self._mic
            if mic is not None and (mic.node != choice["mic"] or not mic.alive):
                mic.close()
                self._mic = mic = None
            if mic is None and choice["mic"]:
                try:
                    self._mic = LocalMic(choice["mic"], self._heard,
                                         self._launch_mic)
                except OSError as error:
                    self.error = f"the headset's microphone: {error}"[:160]
            self._choice = choice

    def stop(self) -> None:
        """Let go of the microphone and the outputs, at the end of a conversation."""
        with self._lock:
            self._active = False
            mic, self._mic = self._mic, None
            self._choice = choose({})
            self._asked_profile = None
        if mic is not None:
            mic.close()
        self.speaker.route([])

    @property
    def has_mic(self) -> bool:
        with self._lock:
            return self._mic is not None

    def deaf(self) -> bool:
        """Whether the microphone should not be listened to right now: the voice
        is coming out of a speaker, or did a moment ago."""
        with self._lock:
            gate = self._choice["gate"]
        return gate and self._clock() < self.speaker.quiet_since() + SPEAKER_TAIL_S

    def _heard(self, pcm: bytes) -> None:
        if not self.deaf():
            self._deliver(pcm)

    def describe(self) -> dict[str, Any]:
        with self._lock:
            return {"out": ", ".join(self._choice["out_names"]),
                    "mic": self._choice["mic_name"] if self._mic else "",
                    "error": self.error}


def _wpctl_profile(device: int, index: int) -> None:
    subprocess.run(["wpctl", "set-profile", str(device), str(index)],
                   check=True, timeout=5, env=_env(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
