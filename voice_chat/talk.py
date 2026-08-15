"""Push-to-talk / hands-free voice client for the MEDIA voice-chat service.

Runs on whatever machine has the microphone. It does the two things that must
happen locally -- capture and playback -- plus endpointing, which belongs here
rather than on the GPU: deciding that the user has stopped talking needs no
model, and doing it locally means silence is never sent over the wire.

The service now binds the LAN, so no tunnel is needed:

    python voice_chat/talk.py --url ws://192.168.1.3:8767/ws

Ctrl-C to quit. On the rover use [talk_pi.py](talk_pi.py) instead -- same
protocol and the same endpointer, but it drives PipeWire directly because the Pi
has no PortAudio.

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

from endpointing import BLOCK, IN_RATE, Endpointer


async def converse(url: str, device: int | None, out_device: int | None) -> None:
    blocks: queue.Queue[np.ndarray] = queue.Queue()

    def on_audio(indata, _frames, _t, status) -> None:
        if status:
            print(f"  (input {status})", file=sys.stderr)
        blocks.put(indata[:, 0].copy())

    async with websockets.connect(url, max_size=None) as ws:
        print(f"connected to {url}")
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
    parser.add_argument("--url", default="ws://192.168.1.3:8767/ws")
    parser.add_argument("--input-device", type=int, default=None)
    parser.add_argument("--output-device", type=int, default=None)
    parser.add_argument("--list-devices", action="store_true")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    try:
        asyncio.run(converse(args.url, args.input_device, args.output_device))
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
