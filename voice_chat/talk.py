"""Push-to-talk / hands-free voice client for the MEDIA voice-chat service.

Runs on whatever machine has the microphone. It does the two things that must
happen locally -- capture and playback -- plus endpointing, which belongs here
rather than on the GPU: deciding that the user has stopped talking needs no
model, and doing it locally means silence is never sent over the wire.

The service now binds the LAN, so no tunnel is needed:

    python voice_chat/talk.py --url ws://media.local:8767/ws

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
import sys
import time

import numpy as np
import sounddevice as sd
import websockets

import rover_tools
from endpointing import BLOCK, IN_RATE, Endpointer

# The service answers a hello immediately -- there is no model behind it -- so
# this only has to outlast a hiccup on the LAN.
HELLO_TIMEOUT_S = 5.0


async def converse(
    url: str,
    device: int | None,
    out_device: int | None,
    rover: rover_tools.RoverClient | None,
) -> None:
    blocks: queue.Queue[np.ndarray] = queue.Queue()

    def on_audio(indata, _frames, _t, status) -> None:
        if status:
            print(f"  (input {status})", file=sys.stderr)
        blocks.put(indata[:, 0].copy())

    async with websockets.connect(url, max_size=None) as ws:
        print(f"connected to {url}")
        # What this client can perform, announced before anything is said. The
        # service holds no catalogue of its own -- it puts whatever it is given
        # into the prompt -- so a service that is too old to understand this
        # never answers, and the wait is what notices.
        if rover is not None:
            # Asked for afresh on every connection rather than cached: the daemon
            # is the authority on what this rover can do, and it may have been
            # restarted with more tools since the last time anybody looked.
            tools = await asyncio.to_thread(rover.tools)
            await ws.send(json.dumps({"type": "hello", "tools": tools}))
            try:
                ack = json.loads(await asyncio.wait_for(ws.recv(), HELLO_TIMEOUT_S))
                print(f"tools: {', '.join(ack.get('tools') or []) or 'none accepted'}")
            except (asyncio.TimeoutError, ValueError):
                print("  the service did not take the tools; carrying on without them",
                      file=sys.stderr)
        # The service loads ~5GB of weights on first start; say so rather than
        # looking hung.
        print("listening -- just talk. Ctrl-C to quit.\n")

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

        with stream:
            while True:
                try:
                    block = blocks.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.005)
                    continue

                if time.monotonic() < muted_until:
                    continue

                utterance = endpointer.push(block)
                if utterance is None:
                    continue

                pcm = (np.clip(utterance, -1.0, 1.0) * 32767).astype("<i2").tobytes()
                await ws.send(pcm)
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
                        continue

                    event = json.loads(message)
                    kind = event.get("type")
                    if kind == "stt":
                        if event.get("empty"):
                            print("  (nothing heard)")
                        else:
                            print(f"you: {event['text']}")
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
                        print(f"bot: {event['text']}")
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
                        print(f"  [{name}{json.dumps(arguments)} -> {json.dumps(result)}]")
                        await ws.send(json.dumps(
                            {"type": "tool_result", "id": event.get("id"), "result": result}))
                    elif kind == "error":
                        print(f"  error: {event['message']}", file=sys.stderr)
                        break
                    elif kind == "done":
                        stats = event.get("stats") or {}
                        if stats.get("first_audio_ms"):
                            print(
                                f"  [stt {stats['stt_ms']}ms, "
                                f"first audio {stats['first_audio_ms']}ms, "
                                f"total {stats['total_ms']}ms]\n"
                            )
                        # Flush anything still queued before reopening the mic.
                        muted_until = max(muted_until, time.monotonic()) + 0.25
                        break


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://media.local:8767/ws")
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--rover", default="auto", metavar="HOST:PORT",
                        help='the rover daemon; "auto" looks for it, "none" for no tools')
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

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
        asyncio.run(converse(args.url, args.input_device, args.output_device, rover))
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        if rover is not None:
            rover.close()


if __name__ == "__main__":
    main()
