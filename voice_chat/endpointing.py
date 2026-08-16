"""Deciding when the speaker has stopped.

This is the one piece of real logic on the client side, and it is kept apart
from [talk.py](talk.py) for two reasons. It is the decision that belongs on the
client rather than the GPU -- it needs no model, and making it locally means
silence never crosses the network -- and it is the part worth testing, which is
only possible because nothing in this module touches an audio device.
[selftest.py](selftest.py) exercises it on a machine with no microphone at all.

It was shared with a second client on the rover until that was removed; the
interface is still the one that suited two callers, which costs nothing and
would matter again if another turned up.
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
# How far into the hang window to hand the utterance over speculatively, so the
# GPU can transcribe it while the client is still deciding whether the turn is
# over. The whole HANG_MS is dead time on the card today, and STT costs 0.18s of
# it, so this hides the transcription entirely behind a wait that was happening
# anyway.
#
# Not zero: the endpointer decides a block is silence on RMS, and the tail of a
# trailing fricative can fall below that threshold while still being audible. A
# few blocks of margin keeps the "s" on the end of a word that Whisper would
# otherwise have to guess at. It still leaves ~580ms of the window to work in.
SPECULATE_AFTER_MS = 120


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
        self.speculate_blocks = max(1, SPECULATE_AFTER_MS // BLOCK_MS)
        # A speculation is outstanding: audio has been handed out for an
        # utterance that has not yet ended.
        self.speculated = False
        # The utterance `push` just returned was already sent speculatively, so
        # whatever was transcribed from it still stands.
        self.spoke_early = False
        self._void = False

    def push(self, block: np.ndarray, rms: float | None = None) -> np.ndarray | None:
        """Feed one block; returns the utterance when the speaker stops.

        `rms` may be supplied by a caller that has already computed it for a
        whole batch of blocks at once. On the rover that matters: doing it here,
        one block at a time, is one numpy call per 20ms, and that rate of small
        allocations is enough to make PipeWire miss its deadline and chop the
        audio. The value must be the plain RMS of `block`.
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
            if self.speculated:
                # The speaker paused for thought and carried on, so whatever was
                # sent ahead is half an utterance. Void it; the caller tells the
                # server to throw the transcript away.
                self.speculated = False
                self._void = True
            return None

        self.silence_blocks += 1
        if self.silence_blocks < self.hang_blocks:
            return None

        self.speaking = False
        audio = np.concatenate(self.voiced)
        voiced_blocks = self.speech_blocks
        # Only meaningful for the utterance being returned right now, and read
        # by the caller immediately after this call.
        self.spoke_early = self.speculated
        self.speculated = False
        self.voiced = []
        self.speech_blocks = 0
        self.silence_blocks = 0
        # Count voiced blocks, not elapsed ones: a short word followed by a long
        # think should not qualify as an utterance just because it took a while.
        if voiced_blocks < self.min_speech_blocks:
            # Unreachable while `pending` refuses to speculate below the same
            # threshold, since speech_blocks only grows -- but an orphaned
            # transcript on the server is a silent bug, so do not lean on that.
            if self.spoke_early:
                self.spoke_early = False
                self._void = True
            return None
        return audio

    def pending(self) -> np.ndarray | None:
        """The utterance so far, once, while the hang window is still running.

        Returns audio exactly once per utterance and only when it already
        qualifies as speech, so anything handed out here is something the
        endpointer would go on to confirm unless the speaker resumes.
        """
        if self.speculated or not self.speaking or not self.voiced:
            return None
        if self.silence_blocks < self.speculate_blocks:
            return None
        if self.speech_blocks < self.min_speech_blocks:
            return None
        self.speculated = True
        return np.concatenate(self.voiced)

    def take_void(self) -> bool:
        """Whether an outstanding speculation has just been invalidated."""
        void, self._void = self._void, False
        return void

    def reset(self) -> None:
        """Drop any half-heard utterance -- used after the assistant speaks."""
        if self.speculated:
            self._void = True
        self.speaking = False
        self.speculated = False
        self.spoke_early = False
        self.voiced = []
        self.speech_blocks = 0
        self.silence_blocks = 0
        self.preroll.clear()
