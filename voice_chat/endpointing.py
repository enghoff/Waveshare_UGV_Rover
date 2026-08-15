"""Deciding when the speaker has stopped -- shared by every client.

This is the one piece of real logic on the client side, and there are now two
clients: [talk.py](talk.py) on a desktop with sounddevice, and
[talk_pi.py](talk_pi.py) on the rover, which has neither PortAudio nor an
ALSA bridge and drives PipeWire directly instead. They capture audio in
completely different ways and must still end a turn at the same moment, so the
decision lives here rather than in either of them.

Nothing in this module touches an audio device, which is also what lets
[selftest.py](selftest.py) exercise it on a machine with no microphone at all.
"""

from __future__ import annotations

from collections import deque

import numpy as np

IN_RATE = 16000  # what Whisper wants
BLOCK_MS = 20  # the granularity endpointing decides on
BLOCK = IN_RATE * BLOCK_MS // 1000  # 320 samples

# Speech is declared when the block's RMS sits this far above the running noise
# floor. A ratio rather than an absolute threshold, because the floor differs by
# 30dB between a laptop's built-in microphone and a headset.
SPEECH_FACTOR = 3.5
# ...but a room can be quiet enough that 3.5x the floor is still inaudible, so
# there is an absolute lower bound as well.
SPEECH_FLOOR = 0.006
# How much silence ends a turn. Under ~500ms it cuts people off mid-sentence
# when they pause for thought; over ~1s the conversation feels sluggish.
HANG_MS = 700
# Speech shorter than this is a cough, a keyboard, or a door.
MIN_SPEECH_MS = 250
# Keep this much audio from before the trigger. Without it the VAD eats the
# first consonant, and "start" arrives at Whisper as "art".
PREROLL_MS = 300


class Endpointer:
    """Adaptive-threshold VAD: tracks the noise floor, reports utterances.

    The floor only adapts on blocks judged to be silence, which is what keeps a
    long sentence from slowly raising the bar until it cuts itself off.

    Timing is counted in blocks, not in wall-clock seconds. The stream is a fixed
    16kHz, so a block *is* 20ms by definition, whereas the clock only measures
    when the callback happened to run -- and both sounddevice and a pipe from
    pw-record deliver in bursts after a scheduling hiccup, which a clock reads as
    a long silence and ends the turn mid-sentence. Counting blocks also makes the
    decision reproducible offline.
    """

    def __init__(self) -> None:
        self.floor = 0.01
        self.speaking = False
        self.preroll: deque[np.ndarray] = deque(maxlen=max(1, PREROLL_MS // BLOCK_MS))
        self.voiced: list[np.ndarray] = []
        self.silence_blocks = 0
        self.speech_blocks = 0
        self.hang_blocks = max(1, HANG_MS // BLOCK_MS)
        self.min_speech_blocks = max(1, MIN_SPEECH_MS // BLOCK_MS)

    def push(self, block: np.ndarray, rms: float | None = None) -> np.ndarray | None:
        """Feed one block; returns the utterance when the speaker stops.

        `rms` may be supplied by a caller that has already computed it for a
        whole batch of blocks at once. On the rover that matters: doing it here,
        one block at a time, is one numpy call per 20ms, and that rate of small
        allocations is enough to make PipeWire miss its deadline and chop the
        audio. The value must be the plain RMS of `block` -- see talk_pi.py.
        """
        if rms is None:
            rms = float(np.sqrt(np.mean(block.astype(np.float32) ** 2)))
        is_speech = rms > max(self.floor * SPEECH_FACTOR, SPEECH_FLOOR)

        if not is_speech:
            # Track the floor slowly; a fast filter would follow a fan spinning
            # up and stop hearing quiet speech along with it.
            self.floor = 0.95 * self.floor + 0.05 * rms

        if not self.speaking:
            self.preroll.append(block)
            if is_speech:
                self.speaking = True
                self.speech_blocks = 1
                self.silence_blocks = 0
                self.voiced = list(self.preroll)
                self.preroll.clear()
            return None

        self.voiced.append(block)
        if is_speech:
            self.speech_blocks += 1
            self.silence_blocks = 0
            return None

        self.silence_blocks += 1
        if self.silence_blocks < self.hang_blocks:
            return None

        self.speaking = False
        audio = np.concatenate(self.voiced)
        voiced_blocks = self.speech_blocks
        self.voiced = []
        self.speech_blocks = 0
        self.silence_blocks = 0
        # Count voiced blocks, not elapsed ones: a short word followed by a long
        # think should not qualify as an utterance just because it took a while.
        if voiced_blocks < self.min_speech_blocks:
            return None
        return audio

    def reset(self) -> None:
        """Drop any half-heard utterance -- used after the assistant speaks."""
        self.speaking = False
        self.voiced = []
        self.speech_blocks = 0
        self.silence_blocks = 0
        self.preroll.clear()
