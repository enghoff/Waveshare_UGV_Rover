"""Push-to-talk / hands-free voice client for the MEDIA voice-chat service.

Runs on whatever machine has the microphone. It does the two things that must
happen locally -- capture and playback -- plus endpointing, which belongs here
rather than on the GPU: deciding that the user has stopped talking needs no
model, and doing it locally means silence is never sent over the wire.

The service now binds the LAN, so no tunnel is needed:

    python voice_chat/talk.py --url ws://media.local:8767/ws

A line at the bottom of the terminal says what the microphone is doing --
listening, hearing you, thinking, speaking -- because the two states where it is
deliberately deaf, while the reply decodes and while it plays, are otherwise
indistinguishable from a client that has died with the stream open.

Ctrl-C to quit. This is the only chat client: an earlier one ran on the rover
itself, over Bluetooth, and was removed because the Pi's audio never became
reliable enough to hold a conversation through.

It also carries the rover's tools -- headlights, camera, face tracking. It
performs none of them: `rover_daemon.py` on the Pi owns the board and the camera,
and this asks it what it can do and passes calls along. The daemon is probed once
at startup and the tools are offered only if it answers, since tools that cannot
reach the rover are worse than none -- the model says out loud that it has
switched the lights on, and nothing happens.

    python voice_chat/talk.py --rover rpi.local:8769      # name it explicitly
    python voice_chat/talk.py --rover none                # conversation only

Dependencies are deliberately thin -- sounddevice, numpy, websockets. No torch,
no onnxruntime: a neural VAD would be better at rejecting a television playing
in the background, but it is a 100MB dependency to improve a decision that an
adaptive noise floor already gets right on a desk microphone.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import queue
import socket
import sys
import time
import urllib.parse

import numpy as np
import sounddevice as sd
import websockets

import rover_tools
from endpointing import BLOCK, IN_RATE, Endpointer

# The service answers a hello immediately -- there is no model behind it -- so
# this only has to outlast a hiccup on the LAN.
HELLO_TIMEOUT_S = 5.0

# Shorter than the library's 10s default. Nothing here is slow when it is
# working -- the service is on the LAN and answers a handshake without waking a
# model -- so a wait this long already means it is not coming, and the sooner
# that is said the sooner somebody can go and start it.
CONNECT_TIMEOUT_S = 8.0


# What the microphone is doing, in the four states that differ to a person
# sitting in front of it. "listening" and "hearing you" are both an open
# microphone; the other two are a closed one, and which of the two it is decides
# whether waiting or repeating yourself is the right move.
#
# The dot is only used where the terminal can encode it: a redirected stdout on
# Windows is cp1252, and U+25CF would raise UnicodeEncodeError on the first
# update -- an indicator that kills the conversation it is decorating.
try:
    "●○".encode(sys.stdout.encoding or "ascii")
    STATUS = {
        "listening": "● listening",
        "hearing": "● hearing you",
        "thinking": "○ thinking",
        "speaking": "○ speaking",
    }
except (UnicodeEncodeError, LookupError):
    STATUS = {
        "listening": "[ listening ]",
        "hearing": "[ hearing you ]",
        "thinking": "[ thinking ]",
        "speaking": "[ speaking ]",
    }
STATUS_WIDTH = max(len(text) for text in STATUS.values())


class Indicator:
    """One line at the bottom of the terminal saying whether the mic is open.

    Rewritten in place and rubbed out before anything else prints, so the
    transcript above it stays a transcript. Everything the conversation says goes
    through `say` for that reason.

    Silent when stdout is not a terminal: `\\r` is just a character in a log file,
    and a status that changes forty times a second would be most of the log.
    """

    def __init__(self) -> None:
        self.on = sys.stdout.isatty()
        self.shown = ""

    def set(self, state: str) -> None:
        text = STATUS[state]
        if not self.on or text == self.shown:
            return
        sys.stdout.write("\r" + text.ljust(STATUS_WIDTH))
        sys.stdout.flush()
        self.shown = text

    def clear(self) -> None:
        if self.on and self.shown:
            sys.stdout.write("\r" + " " * STATUS_WIDTH + "\r")
            sys.stdout.flush()
        # Forgotten rather than remembered, so the next `set` redraws it below
        # whatever was just printed instead of deciding nothing has changed.
        self.shown = ""

    def say(self, text: str, err: bool = False) -> None:
        self.clear()
        print(text, file=sys.stderr if err else sys.stdout, flush=True)

    # A context manager so that however the conversation ends -- Ctrl-C, a
    # dropped service, a raised exception -- the last thing on the terminal is
    # not a half-drawn "listening" that is no longer true.
    def __enter__(self) -> "Indicator":
        return self

    def __exit__(self, *_exc) -> None:
        self.clear()


class ServiceUnreachable(Exception):
    """The voice service could not be reached, said in a way somebody can act on.

    A stack trace out of `websockets` names asyncio internals and not the one
    thing that matters -- which host was not there, and how to start what should
    have been on it -- so every way the connection can fail is turned into one of
    these and printed as text.
    """


async def _open(url: str):
    """Connect to the voice service, or raise `ServiceUnreachable` explaining why not."""
    parts = urllib.parse.urlsplit(url)
    host = parts.hostname or url
    port = parts.port or (443 if parts.scheme == "wss" else 80)
    where = f"{host}:{port}"
    health = f"{'https' if parts.scheme == 'wss' else 'http'}://{where}/health"
    # The switcher is how the card is handed between this and the vision
    # services, so "not running" and "running, but it is dino's turn" have the
    # same fix and it is worth printing either way.
    start = f"  Start it with: ssh root@media ~/switch_service.sh voice\n  Then: curl {health}"

    try:
        return await websockets.connect(url, max_size=None, open_timeout=CONNECT_TIMEOUT_S)
    except websockets.InvalidURI as error:
        raise ServiceUnreachable(f"{url} is not a WebSocket URL ({error}).\n"
                                 "  It should look like: ws://media.local:8767/ws") from None
    except socket.gaierror:
        # Nearly always mDNS: the name is only resolvable by machines that speak
        # it, and this client runs on whatever desk has a microphone.
        raise ServiceUnreachable(
            f'cannot reach {where}: the name "{host}" does not resolve here.\n'
            "  mDNS may not be answering on this network; name the host by address:\n"
            "    python voice_chat/talk.py --url ws://192.168.1.x:8767/ws") from None
    except ConnectionRefusedError:
        # Reached the host, found no listener. Note the service binds late, so
        # this is also what a service that is still warming up looks like.
        raise ServiceUnreachable(
            f"nothing is listening on {where}.\n"
            "  The service does not bind its port until its ~5GB of weights are\n"
            "  loaded, so a start ~60s ago looks exactly like this too.\n"
            f"{start}") from None
    except (TimeoutError, asyncio.TimeoutError):
        # A dropped SYN rather than a refused one: the host is asleep, or the
        # port is filtered. Distinguishable from the above only by the silence.
        raise ServiceUnreachable(
            f"{where} did not answer within {CONNECT_TIMEOUT_S:.0f}s.\n"
            f"  The host is down or the port is being dropped rather than refused --\n"
            f"  check it is up (ping {host}) and that the firewall passes 8765-8774.\n"
            f"{start}") from None
    except websockets.InvalidStatus as error:
        # Something is there and it speaks HTTP, so this is nearly always the
        # path: the service serves /health and /chat on this same port, and only
        # /ws is a socket.
        raise ServiceUnreachable(
            f"{where} answered HTTP {error.response.status_code} rather than upgrading.\n"
            f"  Check the path -- only /ws is a WebSocket: ws://{where}/ws") from None
    except websockets.InvalidHandshake as error:
        raise ServiceUnreachable(
            f"{where} answered, but not as a voice service ({error}).\n"
            f"  Something else may have taken the port; check: curl {health}") from None
    except OSError as error:
        raise ServiceUnreachable(f"cannot reach {where}: {error}.\n{start}") from None


def _to_pcm16(audio: np.ndarray) -> bytes:
    """Float samples as the 16kHz mono s16le the service reads."""
    return (np.clip(audio, -1.0, 1.0) * 32767).astype("<i2").tobytes()


async def converse(
    url: str,
    device: int | None,
    out_device: int | None,
    rover: rover_tools.RoverClient | None,
    early: bool = True,
) -> None:
    blocks: queue.Queue[np.ndarray] = queue.Queue()
    indicator = Indicator()

    def on_audio(indata, _frames, _t, status) -> None:
        if status:
            # From PortAudio's thread, so it cannot touch the indicator's line;
            # an overrun report is rare enough that a stray newline is fine.
            print(f"\n  (input {status})", file=sys.stderr)
        blocks.put(indata[:, 0].copy())

    print(f"connecting to {url} ...", flush=True)
    async with await _open(url) as ws:
        print(f"connected to {url}")
        # What this client can perform, announced before anything is said. The
        # service holds no catalogue of its own -- it puts whatever it is given
        # into the prompt -- so a service that is too old to understand this
        # never answers, and the wait is what notices.
        #
        # Sent even when there is no rover and so no tools to declare, because
        # the ack carries the other half of the negotiation: whether this
        # service will take an utterance early. Guessing that wrong is not a
        # missing feature but a corrupt one -- a service that ignores
        # "speculate" keeps the audio buffered, and the real utterance then
        # arrives appended to it, so the model is asked a question with its
        # first half said twice.
        #
        # Asked for afresh on every connection rather than cached: the daemon is
        # the authority on what this rover can do, and it may have been
        # restarted with more tools since the last time anybody looked.
        tools = await asyncio.to_thread(rover.tools) if rover is not None else []
        early_ok = False
        await ws.send(json.dumps({"type": "hello", "tools": tools}))
        try:
            ack = json.loads(await asyncio.wait_for(ws.recv(), HELLO_TIMEOUT_S))
            # Both ends have to want it. --no-early is the client's half, and it
            # is here rather than behind an environment variable on the service
            # because the question it answers -- is speculation costing us
            # transcripts? -- can only be asked of a real microphone, and the
            # microphone is on this machine.
            early_ok = early and bool(ack.get("early"))
            if tools:
                print(f"tools: {', '.join(ack.get('tools') or []) or 'none accepted'}")
        except (asyncio.TimeoutError, ValueError):
            if tools:
                print("  the service did not take the tools; carrying on without them",
                      file=sys.stderr)
        print("just talk. Ctrl-C to quit.\n")

        stream = sd.InputStream(
            samplerate=IN_RATE,
            blocksize=BLOCK,
            channels=1,
            dtype="float32",
            device=device,
            callback=on_audio,
        )
        player: sd.OutputStream | None = None
        rate = 24000  # replaced by the service's declared rate on the first turn
        endpointer = Endpointer()
        # While the assistant is speaking, the microphone hears it. Without this
        # the reply endpoints itself and the two talk over each other forever.
        muted_until = 0.0

        with stream, indicator:
            indicator.set("listening")
            while True:
                try:
                    block = blocks.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.005)
                    continue

                if time.monotonic() < muted_until:
                    # Deaf on purpose while the reply plays. Said out loud
                    # because a microphone that is ignoring you looks exactly
                    # like one that has stopped working.
                    indicator.set("speaking")
                    continue

                utterance = endpointer.push(block)
                # Whether the endpointer has decided speech is under way, which
                # is the half of "listening" worth seeing: it is the difference
                # between a microphone that is on and one that can hear you.
                indicator.set("hearing" if endpointer.speaking else "listening")
                if utterance is None:
                    if early_ok:
                        # Still inside the hang window. Hand the utterance over
                        # now so Whisper runs while this loop is counting out
                        # the rest of it, or take it back if the speaker turned
                        # out to be pausing rather than finishing.
                        if endpointer.take_void():
                            await ws.send(json.dumps({"type": "cancel"}))
                        elif (guess := endpointer.pending()) is not None:
                            await ws.send(_to_pcm16(guess))
                            await ws.send(json.dumps({"type": "speculate"}))
                    continue

                indicator.set("thinking")
                if endpointer.spoke_early:
                    # Already sent, and already transcribed. The confirmed
                    # utterance differs only by the trailing silence, so there
                    # is nothing worth sending again.
                    await ws.send(json.dumps({"type": "end", "early": True}))
                else:
                    await ws.send(_to_pcm16(utterance))
                    await ws.send(json.dumps({"type": "end"}))

                # Drain one turn's worth of events.
                while True:
                    message = await ws.recv()
                    if isinstance(message, bytes):
                        audio = np.frombuffer(message, dtype="<i2").astype(np.float32) / 32768.0
                        if player is not None:
                            player.write(audio)
                            # Hold the mic closed for as long as this clip lasts,
                            # plus a little for the speakers to stop ringing.
                            muted_until = max(muted_until, time.monotonic()) + len(audio) / rate
                            # Said here rather than left to the outer loop: the
                            # reply starts playing while the rest of it is still
                            # decoding, and "thinking" over the top of the
                            # assistant's own voice reads as a hang.
                            indicator.set("speaking")
                        continue

                    event = json.loads(message)
                    kind = event.get("type")
                    if kind == "stt":
                        if event.get("empty"):
                            indicator.say("  (nothing heard)")
                        else:
                            indicator.say(f"you: {event['text']}")
                    elif kind == "start":
                        rate = event["rate"]
                        if player is None:
                            player = sd.OutputStream(
                                samplerate=rate,
                                channels=1,
                                dtype="float32",
                                device=out_device,
                            )
                            player.start()
                    elif kind == "text":
                        indicator.say(f"bot: {event['text']}")
                    elif kind == "tool":
                        name = event.get("name")
                        arguments = event.get("arguments") or {}
                        # On a thread: a call can take seconds -- count_faces has
                        # to start the camera -- and this loop is also the one
                        # writing to the speaker.
                        result = (
                            {"ok": False, "error": "no rover attached"}
                            if rover is None
                            else await asyncio.to_thread(rover.call, name, arguments)
                        )
                        indicator.say(
                            f"  [{name}{json.dumps(arguments)} -> {json.dumps(result)}]")
                        await ws.send(json.dumps(
                            {"type": "tool_result", "id": event.get("id"), "result": result}))
                    elif kind == "resend":
                        # The service took this utterance early and no longer
                        # has the transcript -- cancelled in between, or a
                        # restart under us. Say it again the ordinary way; the
                        # audio is still in hand.
                        await ws.send(_to_pcm16(utterance))
                        await ws.send(json.dumps({"type": "end"}))
                    elif kind == "error":
                        indicator.say(f"  error: {event['message']}", err=True)
                        break
                    elif kind == "done":
                        stats = event.get("stats") or {}
                        if stats.get("first_audio_ms"):
                            indicator.say(
                                f"  [stt {stats['stt_ms']}ms, "
                                f"first audio {stats['first_audio_ms']}ms, "
                                f"total {stats['total_ms']}ms]\n"
                            )
                        # Flush anything still queued before reopening the mic.
                        muted_until = max(muted_until, time.monotonic()) + 0.25
                        break


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://media.local:8767/ws")
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--rover", default="auto", metavar="HOST:PORT",
                        help='the rover daemon; "auto" looks for it, "none" for no tools')
    parser.add_argument("--no-early", action="store_true",
                        help="transcribe only after the turn ends, not during the "
                             "hang window; slower, and the comparison to make if "
                             "transcripts look wrong")
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return 0

    rover = None
    if args.rover != "none":
        # Probed rather than assumed, and searched for rather than probed at one
        # address: the rover answers on wlan0 or eth0 depending on whether it is
        # plugged in, and picking the wrong one looks exactly like a rover that
        # is not there. Offering tools that cannot reach it is worse than
        # offering none, so a miss means a plain conversation and a printed line.
        if args.rover == "auto":
            rover = rover_tools.discover()
        else:
            rover = rover_tools.RoverClient(args.rover)
            if not rover.probe():
                rover.close()
                rover = None
        if rover is not None:
            print(f"rover daemon at {rover.describe()}")
        else:
            where = ("any of " + ", ".join(rover_tools.DEFAULT_CANDIDATES)
                     if args.rover == "auto" else args.rover)
            print(f"  no rover daemon on {where}; no tools.\n"
                  f"  Start it with: ssh rpi 'cd ugv && python3 rover_daemon.py'",
                  file=sys.stderr)

    try:
        asyncio.run(converse(args.url, args.input_device, args.output_device, rover,
                             not args.no_early))
    except KeyboardInterrupt:
        print("\nbye")
    except ServiceUnreachable as error:
        print(f"\n{error}", file=sys.stderr)
        return 1
    except websockets.ConnectionClosed as error:
        # Caught here rather than around each await: whichever one it died on,
        # the reason is the same and so is what to do about it. A restart of the
        # service, or the card being switched to a vision service mid-sentence,
        # both arrive as this.
        print(f"\nthe voice service closed the connection ({error}).\n"
              "  It was probably restarted or switched off the card; run it again"
              " once it is back.", file=sys.stderr)
        return 1
    finally:
        if rover is not None:
            rover.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
