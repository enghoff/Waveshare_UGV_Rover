"""Voice chat service: speech in, speech out, on the MEDIA GPU host.

Three models stay resident and hand off to each other inside one process --
faster-whisper for STT, Qwen3-4B for the reply, Kokoro for TTS. Keeping them in
one service rather than three is not packaging convenience: a turn is a strict
chain (audio -> text -> text -> audio) with nothing to overlap between stages,
so splitting them would buy no parallelism and cost two extra hops plus two
extra CUDA contexts on a card that only has ~6.5GB free once Windows has taken
its share.

Binds to 127.0.0.1 by default -- reach it from another host via SSH tunnel, the
same way as the Grounding DINO and Qwen3-VL services. It shares the card with
those two and is not meant to run alongside them; ~/switch_service.sh enforces
that.

The latency-critical decision here is sentence chunking (see :func:`_sentences`):
audio starts playing after the model's FIRST sentence, not its last. On a
three-sentence reply that is the difference between ~0.9s and ~3s before the
user hears anything, and it costs nothing but a splitter.

Tools are executed by the *client*, not here. The rover's hardware hangs off an
ESP32 that only the Pi is wired to, so this service knows nothing about it: a
client announces what it can do when it connects, those schemas go into the
prompt, and a call the model makes goes back down the same socket to be
performed. `talk.py` on a desktop announces nothing and gets a plain
conversation, with no mention of rover hardware in its context at all.

With VOICE_VISION=1 the reply model is a vision-language one and the
conversation can hold pictures. The picture does **not** arrive through the
client. A tool the client performs makes whoever holds the camera POST a JPEG
to `/frame` here, and the tool result names it; the image then enters the
history on this side, having gone straight from the camera to this card. That
is the same path `face-detect` already takes, and it keeps a 35kB frame off the
desk that only has a microphone on it -- see :func:`frame`.

Vision is a switch, not a rewrite: with it off, this is the text service it has
always been, down to the model that is loaded. The rollback is
`VOICE_VISION=0` plus the text model in the unit, and a restart.
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import time
from contextlib import asynccontextmanager
from queue import Empty
from threading import Thread
from typing import Any, Iterator

import numpy as np
import torch
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

HOST = os.environ.get("VOICE_HOST", "127.0.0.1")
PORT = int(os.environ.get("VOICE_PORT", "8767"))
DEVICE = os.environ.get("VOICE_DEVICE", "cuda" if torch.cuda.is_available() else "cpu")

# --- STT ---------------------------------------------------------------------
# distil-large-v3 is 6 encoder layers where large-v3 has 32, at ~1% relative WER
# on English. It is English-only; swap to Systran/faster-whisper-large-v3 if you
# need another language and can spare the extra ~1.5GB and ~2x the time.
# int8_float16 quantizes weights to int8 and computes in fp16: 0.8GB resident,
# and a 5s utterance transcribes in ~120ms on this card.
STT_MODEL = os.environ.get("VOICE_STT_MODEL", "Systran/faster-distil-whisper-large-v3")
STT_COMPUTE = os.environ.get("VOICE_STT_COMPUTE", "int8_float16")
# Greedy. Beam search is the default at 5 and costs ~3x the time for a WER
# improvement that does not survive contact with a desk microphone.
STT_BEAM = int(os.environ.get("VOICE_STT_BEAM", "1"))

# --- LLM ---------------------------------------------------------------------
# 4B rather than 8B. Spoken turns are short and the reply is heard, not read, so
# the gap that shows up on long-form reasoning barely registers here -- while 8B
# at int4 would take ~5GB and leave no room for Whisper's CUDA context to grow.
LLM_MODEL = os.environ.get("VOICE_LLM_MODEL", "Qwen/Qwen3-4B-Instruct-2507")
# Same int4 recipe as /opt/qwen3-vl: torchao weight-only, tinygemm, tile-packed
# for sm86. See that service's server.py for the measurements behind it -- the
# short version is that it is the only 4-bit backend transformers will compile.
LLM_QUANT = os.environ.get("VOICE_LLM_QUANT", "int4")
INT4_GROUP = int(os.environ.get("VOICE_INT4_GROUP", "128"))
INT4_PACKING = os.environ.get("VOICE_INT4_PACKING", "tile_packed_to_4d")
# Modules left in bf16, comma-separated, and only meaningful for a vision model.
# Straight from /opt/qwen3-vl, where it was measured: int4 is a decode
# optimization and a prefill pessimization -- weight-only int4 has to unpack for
# the wide GEMMs prefill is made of -- so quantizing the vision tower costs
# 0.45s -> 0.90s on a frame to speed up a token loop it takes no part in.
# Excluding it buys that back for ~0.6GB, and that 0.6GB is the first thing to
# give back if this does not fit beside Whisper and Kokoro.
INT4_SKIP = [m for m in os.environ.get("VOICE_INT4_SKIP", "visual").split(",") if m]
# Attention runs over the whole static window every step whether or not it holds
# real tokens, so this is sized for a conversation, not for a context record:
# 2048 is ~12 turns of speech at this verbosity. History is trimmed to fit
# (see :func:`_trim`) rather than being allowed to silently overrun it.
CACHE_LEN = int(os.environ.get("VOICE_CACHE_LEN", "2048"))
COMPILE = os.environ.get("VOICE_COMPILE", "1") not in ("0", "false", "False")
# Compile for a *range* of prompt lengths rather than for one exact length.
# This is not the usual dynamic-shapes tradeoff, it is the difference between
# working and not: a conversation's prompt grows by a turn every turn, and with
# static shapes each new length is a fresh ~50s compile. Measured over four
# turns, static-shape compilation spent 54s, 76s, 71s and 68s; dynamic pays once.
COMPILE_DYNAMIC = os.environ.get("VOICE_COMPILE_DYNAMIC", "1") not in ("0", "false", "False")
MAX_NEW_TOKENS = int(os.environ.get("VOICE_MAX_NEW_TOKENS", "160"))
# How long a turn may produce nothing before it is called a failure. This is not
# a budget for slow decoding -- tokens arrive every ~25ms once they start -- it
# is how long to wait for the *first* one, and the only thing that takes minutes
# there is a compile that _load should already have paid for.
STREAM_TIMEOUT_S = float(os.environ.get("VOICE_STREAM_TIMEOUT", "180"))
# 0.2, not the 0.7 this started at, because whether the model *acts* turned out
# to be a sampled decision. Measured over six samples a cell through /chat, with
# the tool prompt below and the rover's nine schemas:
#
#     request                             t=0.7   t=0.3   t=0.2   t=0.0
#     "Hello, can you switch lights off?"   6/6     6/6     6/6     6/6
#     "Can you look to your left?"          6/6     5/6     6/6     6/6
#     "Start following me."                 3/6     6/6     6/6     6/6
#     "What is your name?"                  0/6     0/6     0/6     0/6   (want 0)
#
# Half the time at 0.7 the rover simply did not do what it was told, and said it
# had. Nothing over-calls at any of these, so the cost of turning it down is only
# variety -- which is worth very little in a one-to-three-sentence spoken reply,
# and much less than doing as it is asked.
TEMPERATURE = float(os.environ.get("VOICE_TEMPERATURE", "0.2"))

# Spoken replies, not written ones. Without this the model reaches for bullets,
# headings and code fences, and Kokoro reads the punctuation out loud.
SYSTEM_PROMPT = os.environ.get(
    "VOICE_SYSTEM_PROMPT",
    "You are the voice of a small tracked rover. You are speaking out loud, so "
    "reply in one to three short sentences of plain spoken English. Never use "
    "markdown, bullet points, headings, emoji or code. Write numbers and units "
    "as you would say them aloud. If you do not know something, say so briefly.",
)

# --- Vision ------------------------------------------------------------------
# Off by default, and off is the text service exactly as it was: nothing below
# is reached, no processor is loaded, /frame refuses, and no message in a
# conversation can hold an image. Turning it on means loading a model that can
# take one -- the two settings go together, and _load says so rather than
# failing somewhere less obvious.
VISION = os.environ.get("VOICE_VISION", "0") not in ("0", "false", "False", "")
# The rover's camera is 640x480 and a frame is ~35kB, so this is a ceiling for
# anything else that posts, not a resize of the usual case. Vision tokens go as
# the area, and they are spent out of the same window the conversation lives in.
VISION_MAX_SIDE = int(os.environ.get("VOICE_VISION_MAX_SIDE", "640"))
# A posted frame is held only long enough for the turn that asked for it to
# reach the model. Held by token rather than "the latest one" because two
# clients could be talking to this at once, and the wrong picture answered
# confidently is the failure this whole path exists to avoid.
FRAME_TTL_S = float(os.environ.get("VOICE_FRAME_TTL", "60"))
MAX_FRAMES = 4
# What one picture costs in the context window. Measured at load against a frame
# of the configured size (see :func:`_measure_image_tokens`); this is only the
# fallback for when that measurement cannot be taken.
IMAGE_TOKENS = int(os.environ.get("VOICE_IMAGE_TOKENS", "400"))
# Whether a picture outlives the turn that took it. It does not, by default: the
# camera is on a gimbal that sweeps while face tracking runs, so last turn's
# picture is of somewhere the rover is no longer pointing, and answering from it
# is answering about the past with complete confidence. Each turn that needs to
# see therefore takes its own frame -- ~2s, against an answer that may simply be
# out of date. Set to 0 to let one picture answer follow-up questions instead,
# which is cheaper and was the behaviour until it was measured against a moving
# camera.
FRESH_PICTURE = os.environ.get("VOICE_FRESH_PICTURE", "1") not in ("0", "false", "False", "")
# Appended to the system prompt when vision is on and the client announced
# tools. The model otherwise answers "what can you see" from the conversation,
# or from nothing, with complete confidence -- the same failure as the rover
# that said it had switched the lights on.
# Appended to the system prompt when vision is on and the client announced
# tools. The model otherwise answers "what can you see" from the conversation,
# or from nothing, with complete confidence -- the same failure as the rover
# that said it had switched the lights on.
#
# This wording is what measured best, not what reads best, and it has one known
# wart: asked to describe something in a picture it is already holding, it
# sometimes answers "I need to take a picture to see" and does not take one. The
# obvious repair makes things worse and was tried -- rewriting this to say a
# picture stays and that saying you will look is not looking took the first look
# from 3/3 to 1/3 and produced "I took a picture to show what's in front of me"
# from a model that had taken none, which is the lie this whole design exists to
# prevent. A prompt that talks about the act of looking gets the act narrated.
# Change it with numbers, and check the first call as well as the follow-ups.
VISION_PROMPT = os.environ.get(
    "VOICE_VISION_PROMPT",
    " You see by taking a picture with the tool that takes one, so if you are "
    "asked what you can see, or what something looks like, or to read or "
    "describe anything in front of you, take a picture first and answer from "
    "it. Describe only what is actually in the picture.",
)

# --- TTS ---------------------------------------------------------------------
# Kokoro-82M: small enough that it is rounding error next to the LLM, and it
# still beats every other open TTS in this size class on naturalness.
TTS_VOICE = os.environ.get("VOICE_TTS_VOICE", "af_heart")
TTS_SPEED = float(os.environ.get("VOICE_TTS_SPEED", "1.0"))
# Kokoro is trained at 24kHz and there is no reason to resample: the client is
# told the rate and opens its output stream to match.
TTS_RATE = 24000
# Whisper's input rate. Fixed by the model, not a preference -- the client
# resamples to this before sending.
STT_RATE = 16000

# --- Tools -------------------------------------------------------------------
# How long the client gets to perform a call. Most are a JSON line down a UART
# and answer in milliseconds, but not all: a rover asked how many people it can
# see has to start its camera and wait for a first buffer, which is seconds. So
# this is sized for the slowest tool rather than the typical one -- the point of
# it is only to stop a conversation hanging on a rover that has gone away.
TOOL_TIMEOUT_S = float(os.environ.get("VOICE_TOOL_TIMEOUT", "12"))
# Calls the model may make before it has to answer in words. The last decode of
# a turn is offered no tools at all (see :func:`_run_turn`), so a model that has
# decided everything is a tool call still ends the turn having said something.
MAX_TOOL_CALLS = int(os.environ.get("VOICE_MAX_TOOL_CALLS", "2"))
# A cap on what a client may announce, so a broken or hostile client cannot push
# the prompt past the cache window before anyone has spoken.
MAX_TOOLS = 16

# Appended to the system prompt only when a client has actually announced tools.
# Without it the "if you do not know something, say so" line above wins and the
# model explains that it has no way to reach the hardware it is holding.
TOOL_PROMPT = os.environ.get(
    "VOICE_TOOL_PROMPT",
    " You control this rover through the tools you have been given. Call a tool "
    "whenever you are asked to do something one of them covers, including when "
    "the request is phrased as a question such as 'can you turn the lights off'. "
    "You have done something only if you have called a tool for it: never say "
    "you have switched, moved, started or stopped anything unless the call was "
    "made and answered. Then say what you did in one short sentence, without "
    "reading the tool call or its result out loud.",
)

_stt = None
_llm = None
_tokenizer = None
_processor = None  # only with VISION; None keeps every image path unreachable
_tts = None
_cache = None
_quantization = "none"
_compiled = False
_image_tokens = IMAGE_TOKENS
# Frames posted by whoever holds the camera, waiting to be claimed by the tool
# result that names them. Small and short-lived by construction -- see /frame.
_frames: dict[str, tuple[Any, float]] = {}
_frame_seq = 0
# One turn at a time. The three models share a card and a CUDA stream, and two
# concurrent turns would interleave into thrash rather than throughput; this is
# a single-user assistant, so serialising is honest rather than limiting.
_gpu_lock = asyncio.Lock()


def _quant_config():
    """The 4-bit backend to load with, plus the label /health reports."""
    if LLM_QUANT in ("0", "none", ""):
        return None, "none"
    if LLM_QUANT == "int4":
        from torchao.quantization import Int4WeightOnlyConfig
        from transformers import TorchAoConfig

        cfg = Int4WeightOnlyConfig(group_size=INT4_GROUP, int4_packing_format=INT4_PACKING)
        skip = INT4_SKIP if VISION else []
        label = f"int4/{INT4_PACKING}" + (f" (bf16: {','.join(skip)})" if skip else "")
        return TorchAoConfig(quant_type=cfg, modules_to_not_convert=skip or None), label
    if LLM_QUANT == "nf4":
        from transformers import BitsAndBytesConfig

        return (
            BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.bfloat16,
            ),
            "nf4",
        )
    raise ValueError(f"unknown VOICE_LLM_QUANT: {LLM_QUANT}")


def _load() -> None:
    global _stt, _llm, _tokenizer, _processor, _tts, _cache
    global _quantization, _compiled, _image_tokens
    if _llm is not None:
        return

    from faster_whisper import WhisperModel
    from kokoro import KPipeline
    from transformers import AutoConfig, AutoTokenizer, CompileConfig

    t0 = time.perf_counter()
    _stt = WhisperModel(STT_MODEL, device=DEVICE, compute_type=STT_COMPUTE)
    t_stt = time.perf_counter()

    quant, _quantization = _quant_config()
    _tokenizer = AutoTokenizer.from_pretrained(LLM_MODEL)
    if VISION:
        # Refused here rather than left to fail somewhere less legible: a text
        # model loaded under VOICE_VISION=1 would start, serve, and then answer
        # questions about a picture it was never shown -- which is exactly the
        # confident-and-wrong failure this path is meant to remove.
        config = AutoConfig.from_pretrained(LLM_MODEL)
        if not (hasattr(config, "vision_config") or hasattr(config, "vision_encoder")):
            raise SystemExit(
                f"VOICE_VISION=1 but {LLM_MODEL} has no vision tower. Set "
                "VOICE_LLM_MODEL to a vision-language model (Qwen/Qwen3-VL-4B-Instruct) "
                "or set VOICE_VISION=0.")
        from transformers import AutoModelForImageTextToText, AutoProcessor

        # The generic class rather than Qwen3VLForConditionalGeneration, so that
        # trying a different VLM is an environment variable rather than an edit.
        _processor = AutoProcessor.from_pretrained(LLM_MODEL)
        _llm = AutoModelForImageTextToText.from_pretrained(
            LLM_MODEL,
            dtype=torch.bfloat16,
            device_map=DEVICE,
            quantization_config=quant,
        )
    else:
        from transformers import AutoModelForCausalLM

        _llm = AutoModelForCausalLM.from_pretrained(
            LLM_MODEL,
            dtype=torch.bfloat16,
            device_map=DEVICE,
            quantization_config=quant,
        )
    _llm.eval()
    t_llm = time.perf_counter()

    # lang_code 'a' is American English, which is what the af_* voices are.
    _tts = KPipeline(lang_code=TTS_VOICE[0], device=DEVICE)
    t_tts = time.perf_counter()

    if DEVICE == "cuda":
        # Let transformers own the static cache rather than building one here and
        # passing it as past_key_values. Two reasons, both learned the hard way:
        # it rejects an explicit cache alongside cache_implementation at all, and
        # more importantly it is the side that issues the cudagraph step markers.
        # A cache we allocate and reuse ourselves under "reduce-overhead" gets its
        # K/V buffers overwritten by the graph between turns -- which surfaces as
        # an index_copy_ error deep in the attention layer, not as anything that
        # points at the cache.
        #
        # max_length is what sizes that cache, so pinning it keeps the window
        # fixed at CACHE_LEN instead of being re-derived per prompt, which would
        # recompile on every new input length.
        _llm.generation_config.cache_implementation = "static"
        _llm.generation_config.max_length = CACHE_LEN
        _cache = "static"
        if COMPILE:
            # transformers wraps only the decode step; prefill stays eager, so a
            # variable-length prompt does not trigger a recompile every turn.
            # Compilation is an optimization, so a failure here degrades to eager
            # rather than taking the service down -- /health reports which it got.
            try:
                # No mode="reduce-overhead": it asks for CUDA graphs, and this
                # model does not get them anyway -- inductor reports "skipping
                # cudagraphs due to mutated inputs (108 instances)" because the
                # static cache is written in place. Asking for them costs capture
                # attempts and buys nothing; the 46 tok/s comes from the fused
                # inductor kernels, which the default mode also gives.
                _llm.generation_config.compile_config = CompileConfig(
                    fullgraph=True, dynamic=COMPILE_DYNAMIC
                )
                _compiled = True
            except Exception as exc:
                print(f"[voice] compile unavailable, staying eager: {exc}", flush=True)

    if VISION:
        _image_tokens = _measure_image_tokens()

    print(
        f"[voice] loaded in {t_tts - t0:.1f}s "
        f"(stt {t_stt - t0:.1f}s, llm {t_llm - t_stt:.1f}s, tts {t_tts - t_llm:.1f}s) "
        f"quant={_quantization} compile={_compiled} dynamic={COMPILE_DYNAMIC}"
        + (f" vision={LLM_MODEL} image={_image_tokens} tokens" if VISION else ""),
        flush=True,
    )

    # Spend the compile here rather than on the user's first question. It is
    # ~130s and it has to happen exactly once, so the only choice is whether the
    # person waiting for it is a person: the switcher already blocks on /health,
    # and this service does not bind the port until this returns.
    #
    # All three models are warmed, not just the LLM, because Whisper and Kokoro
    # each have their own first-call cost (0.55s and 1.5s measured) -- small, but
    # there is no reason to leave them on the first turn either.
    if _compiled or DEVICE == "cuda":
        t_warm = time.perf_counter()
        try:
            _speak("Ready.")
            _transcribe(np.zeros(STT_RATE, dtype=np.float32))
            for _ in _generate([{"role": "user", "content": "Say ready."}]):
                pass
            t_text = time.perf_counter()
            print(f"[voice] warmed in {t_text - t_warm:.1f}s", flush=True)
            # And again with a picture in front of it, which is a *different*
            # compile and not an optimisation of the same one. A conversation
            # holding an image takes another path through the model -- Qwen3-VL
            # positions image tokens with 3D rope -- so the first turn that
            # looks at anything recompiles the decode step from scratch. That
            # was measured the hard way: 2450 inductor artifacts written while
            # one turn sat there, and the streamer gave up at 180s before it
            # finished. Nobody should meet that mid-conversation, having just
            # asked what the rover can see.
            if VISION:
                from PIL import Image

                probe = Image.new(
                    "RGB", (VISION_MAX_SIDE, VISION_MAX_SIDE * 3 // 4), (128, 128, 128))
                for _ in _generate([_image_message(probe),
                                    {"role": "user", "content": "Say ready."}]):
                    pass
                print(f"[voice] warmed the picture path in "
                      f"{time.perf_counter() - t_text:.1f}s", flush=True)
        except Exception as exc:
            # A failed warmup is not a failed service -- the real request will
            # raise its own error, and that one has somewhere to be reported to.
            print(f"[voice] warmup failed ({exc}); first turn will be slow", flush=True)


def _template():
    """Whatever owns the chat template: the processor when there is one.

    Multimodal models keep their template on the processor rather than the
    tokenizer, and the two do not always both have one -- so everything that
    renders a conversation goes through this rather than reaching for
    `_tokenizer` and working by luck.
    """
    return _processor if _processor is not None else _tokenizer


def _image_message(image: Any) -> dict[str, Any]:
    """The picture, as a turn in the conversation.

    A user turn rather than the tool result it came from: a tool message holds a
    string, and the templates that would render an image inside one do not
    agree with each other. This says out loud whose picture it is, because the
    model is about to be asked what *it* can see.
    """
    return {
        "role": "user",
        "content": [
            {"type": "image", "image": image},
            # Nothing but what it is. This carried a sentence about the picture
            # staying in front of you and when another might be taken, which
            # measured no improvement and did come back out of the model's mouth
            # as a rule it had been given -- "I can't take a picture again
            # unless you ask me to look again or need a newer view" -- to
            # somebody who had asked for exactly that. Under FRESH_PICTURE it
            # would also have been a lie: the picture does not stay.
            {"type": "text", "text": "This is the picture your camera has just taken."},
        ],
    }


def _images(history: list[dict[str, Any]]) -> list[Any]:
    """Every image in the conversation, in the order the template will want them."""
    found = []
    for message in history:
        content = message.get("content")
        # Only a list can hold one. Iterating a string here would walk it a
        # character at a time, on every prompt measurement, for nothing.
        if not isinstance(content, list):
            continue
        for part in content:
            if isinstance(part, dict) and part.get("type") == "image" and part.get("image"):
                found.append(part["image"])
    return found


def _textual(history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The same conversation with every picture reduced to the fact that it existed.

    For counting and for trimming, both of which have to render the prompt and
    neither of which should pay for an image to do it.
    """
    plain = []
    for message in history:
        content = message.get("content")
        if not isinstance(content, list):
            plain.append(message)
            continue
        text = " ".join(
            part.get("text", "") for part in content
            if isinstance(part, dict) and part.get("type") == "text"
        )
        plain.append({**message, "content": text})
    return plain


def _forget_pictures(history: list[dict[str, Any]], keep_newest: bool = True) -> None:
    """Drop pictures out of the conversation, in place, leaving a note that one existed.

    Two reasons, and they stack. A picture costs a few hundred tokens of a
    window that holds about a dozen spoken turns, so a conversation that looked
    four times would be mostly photographs of a room nobody is asking about any
    more. And with `keep_newest` false -- which is what a new turn does, since
    the camera moves between them -- it is how the rover is stopped from
    answering today's question with yesterday's view.

    What is left behind is a sentence saying there was a picture, so the model
    knows it looked and does not claim to still be looking at something that is
    no longer in front of it.
    """
    newest = max((i for i, m in enumerate(history) if _images([m])), default=None)
    if newest is None:
        return
    # The call that fetched it goes with it. Leaving the call and its result
    # behind was measured twice and failed twice, under two different wordings
    # of what was left in the picture's place: the model reads its own earlier
    # `look` as having already looked, and then answers "I can't describe them,
    # I don't have a picture" -- or, worse, "I need to take a picture to see"
    # while taking none. 0/3 both times. A call whose result has been taken away
    # is a stranded call, which this service already knows better than to keep;
    # _trim cuts whole exchanges for the same reason. What is left is the
    # transcript of what was actually said, which is all a later turn needs.
    keep: list[dict[str, Any]] = []
    for index, message in enumerate(history):
        if _images([message]) and not (keep_newest and index == newest):
            # ...and the two messages in front of it, which are the call and
            # its result, appended together with the picture in _run_turn.
            if keep and keep[-1].get("role") == "tool":
                keep.pop()
                if keep and keep[-1].get("tool_calls"):
                    keep.pop()
            continue
        keep.append(message)
    history[:] = keep


def _measure_image_tokens() -> int:
    """What one frame of the configured size actually costs in the window.

    Measured rather than assumed, because it decides when history is trimmed and
    a wrong constant there is the quiet kind of wrong: too low and the prompt
    overruns the static cache and silently falls back to the dynamic one, too
    high and a conversation is cut short for room nobody needed.
    """
    try:
        from PIL import Image

        probe = Image.new("RGB", (VISION_MAX_SIDE, VISION_MAX_SIDE * 3 // 4), (128, 128, 128))
        message = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": "x"}]}]
        text = _template().apply_chat_template(
            message, tokenize=False, add_generation_prompt=True)
        with_image = _processor(text=[text], images=[probe], return_tensors="pt")
        return max(int(with_image["input_ids"].shape[-1]) - len(_tokenizer(text)["input_ids"]), 1)
    except Exception as exc:
        print(f"[voice] cannot measure an image's token cost ({exc}); "
              f"assuming {IMAGE_TOKENS}", flush=True)
        return IMAGE_TOKENS


# A sentence ends at .?!… or a newline, but only when the next character is
# whitespace or end-of-string -- otherwise "3.5 metres" and "192.168.1.4" split
# mid-number and Kokoro reads the fragments with falling intonation.
_SENTENCE_END = re.compile(r"(?<=[.!?…\n])(?=\s|$)")
# Do not hand Kokoro a two-word fragment just because it ended in a period; the
# prosody of a very short clause spoken alone is noticeably wrong. Below this,
# keep buffering.
_MIN_SENTENCE = int(os.environ.get("VOICE_MIN_SENTENCE", "12"))


def _sentences(stream: Iterator[str]) -> Iterator[str]:
    """Regroup a token stream into speakable sentences as they complete."""
    buf = ""
    for piece in stream:
        buf += piece
        while True:
            parts = _SENTENCE_END.split(buf, maxsplit=1)
            if len(parts) < 2:
                break
            head, buf = parts[0], parts[1]
            if len(head.strip()) < _MIN_SENTENCE:
                # Too short to speak alone -- glue it back onto what follows.
                buf = head + buf
                break
            yield head.strip()
    if buf.strip():
        yield buf.strip()


_TOOL_OPEN = "<tool_call>"
_TOOL_CLOSE = "</tool_call>"


class _ToolSniffer:
    """Passes prose through; swallows a tool call and everything after it.

    This sits between the token stream and the sentence splitter for one
    reason: a tool call is *text*, and the splitter would hand it to Kokoro,
    which would read the JSON out loud, brace by brace. Nothing may be spoken
    until it is known not to be a call.

    It watches for two shapes, because which one arrives depends on the
    tokenizer rather than on the model. Qwen wraps a call in `<tool_call>`
    markers, but those markers are added tokens, and a streamer built with
    `skip_special_tokens=True` may well eat them before this ever sees them --
    leaving a bare JSON object as the whole reply. So: a `<tool_call>` marker
    anywhere, *or* a reply that opens with a brace. Spoken English does neither.
    """

    def __init__(self) -> None:
        self.tail = ""  # the call, and anything the model wrote after it
        self._pending = ""  # a part-written marker, held back until it can be judged
        self._opened = False  # has the reply's first real character been seen?

    def feed(self, piece: str) -> str:
        """One chunk of decoded text in, the part of it that is prose out."""
        if self.tail:
            self.tail += piece
            return ""
        text = self._pending + piece
        self._pending = ""

        if not self._opened:
            head = text.lstrip()
            if not head:
                self._pending = text  # nothing but whitespace so far
                return ""
            if head.startswith("{"):
                self.tail = head
                return ""
            self._opened = True

        cut = text.find(_TOOL_OPEN)
        if cut >= 0:
            self.tail = text[cut:]
            return text[:cut]
        # Hold back a trailing fragment that could still become the marker --
        # the stream arrives in sub-word pieces, so "<tool" and "_call>" is a
        # perfectly ordinary way for one to turn up.
        for n in range(min(len(_TOOL_OPEN) - 1, len(text)), 0, -1):
            if text.endswith(_TOOL_OPEN[:n]):
                self._pending = text[-n:]
                return text[:-n]
        return text

    def flush(self) -> str:
        """Whatever was held back and turned out to be ordinary text."""
        text, self._pending = self._pending, ""
        return text


def _parse_tool_call(text: str) -> dict[str, Any] | None:
    """The first call in a swallowed block, or None if it will not parse."""
    body = text.strip()
    if body.startswith(_TOOL_OPEN):
        body = body[len(_TOOL_OPEN):]
    body = body.split(_TOOL_CLOSE, 1)[0].strip()
    # A second call in the same reply is dropped rather than queued: the tools
    # here are cheap and idempotent, and honouring only the first keeps the turn
    # to one round trip.
    body = body.split(_TOOL_OPEN, 1)[0].strip()
    try:
        call = json.loads(body)
    except ValueError:
        return None
    if not isinstance(call, dict):
        return None
    name = call.get("name")
    arguments = call.get("arguments", {})
    # Some templates emit the arguments as a JSON *string* rather than an
    # object. Both are common enough to accept.
    if isinstance(arguments, str):
        try:
            arguments = json.loads(arguments)
        except ValueError:
            return None
    if not isinstance(name, str) or not isinstance(arguments, dict):
        return None
    return {"name": name, "arguments": arguments}


def _reply_stream(
    history: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    system: str | None = None,
    temperature: float | None = None,
) -> Iterator[tuple[str, Any]]:
    """One decode as ("sentence", str) items, then at most one ("tool", call).

    The ordering is what makes a mid-reply call work: prose the model wrote
    before reaching for a tool is still spoken, and spoken while the call is
    still decoding, exactly as an ordinary sentence would be.
    """
    raw = _generate(history, tools, system=system, temperature=temperature)
    sniffer = _ToolSniffer()

    def prose() -> Iterator[str]:
        for piece in raw:
            if text := sniffer.feed(piece):
                yield text
        if text := sniffer.flush():
            yield text

    for sentence in _sentences(prose()):
        yield "sentence", sentence

    if sniffer.tail:
        if (call := _parse_tool_call(sniffer.tail)) is not None:
            yield "tool", call
        else:
            # It looked like a call and was not one. Say it rather than swallow
            # it -- a silent turn is a worse failure than an odd-sounding one.
            for sentence in _sentences(iter([sniffer.tail])):
                yield "sentence", sentence


def _prompt_len(history: list[dict[str, Any]], tools: list[dict[str, Any]] = ()) -> int:
    """How many tokens this history would become, tool schemas included.

    The unwrapping is not defensive style, it is the fix for a bug that made
    :func:`_trim` inert for the life of this service. `apply_chat_template` with
    `tokenize=True` returns a **BatchEncoding** on transformers 5, not a list of
    ids -- so `len()` of it is 2, the number of keys, and every "does this fit
    the cache" test read `2 <= 1856` and said yes. Nothing was ever trimmed, and
    a long conversation instead quietly fell through to the dynamic cache in
    :func:`_generate`, losing the compiled decode path and getting slower rather
    than failing. Older versions did return a flat list, so both are handled.
    """
    encoded = _template().apply_chat_template(
        [{"role": "system", "content": SYSTEM_PROMPT}] + _textual(history),
        tools=list(tools) or None,
        tokenize=True,
        add_generation_prompt=True,
    )
    ids = encoded["input_ids"] if hasattr(encoded, "keys") else encoded
    # A batch of one, if it was asked to return tensors rather than a flat list.
    if ids and isinstance(ids[0], (list, tuple)):
        ids = ids[0]
    # Images are counted rather than rendered. A placeholder is one token in the
    # prompt and several hundred by the time the processor has expanded it, so a
    # history holding a picture measures nearly empty if this is left out -- and
    # the whole point of measuring is to know when the window is full.
    return len(ids) + _image_tokens * len(_images(history))


def _trim(
    history: list[dict[str, Any]], tools: list[dict[str, Any]] = ()
) -> list[dict[str, Any]]:
    """Drop whole exchanges off the front until the prompt fits the static cache.

    Whole exchanges, not tokens: half a turn in the history reads to the model as
    the user being interrupted, and it starts apologising for things it did not
    do. Leaves room for the reply as well as the prompt, since both share the
    window.

    An exchange is a user message and everything answering it, which is no
    longer always one assistant message -- a turn that called a tool holds the
    call and its result too. Cutting a fixed two entries would strand a call
    with no result, or a result with no call, and a model shown either starts
    narrating tool plumbing out loud.
    """
    budget = CACHE_LEN - MAX_NEW_TOKENS - 32
    history = list(history)
    # Before dropping any turn, drop every picture but the newest: a photograph
    # of a room from four turns ago is worth less than the sentences it would
    # cost, and this is the cheaper cut of the two.
    _forget_pictures(history)
    while history:
        if _prompt_len(history, tools) <= budget:
            break
        # Where the next exchange starts. If there is not another one then this
        # is the last, and it is left alone however long it is: trimming it away
        # would erase the utterance being answered and hand the model an empty
        # conversation. _generate falls back to the dynamic cache for that case,
        # which is slower but correct.
        cut = 1
        while cut < len(history) and history[cut].get("role") != "user":
            cut += 1
        if cut >= len(history):
            break
        history = history[cut:]
    return history


def _transcribe(pcm: np.ndarray) -> str:
    segments, _info = _stt.transcribe(
        pcm,
        language="en",
        beam_size=STT_BEAM,
        # The client already endpointed this clip with its own VAD; running
        # Whisper's too would clip leading consonants off short replies.
        vad_filter=False,
        condition_on_previous_text=False,
    )
    return "".join(seg.text for seg in segments).strip()


def _generate(
    history: list[dict[str, Any]],
    tools: list[dict[str, Any]] = (),
    system: str | None = None,
    temperature: float | None = None,
) -> Iterator[str]:
    """Yield the reply as it decodes, one token-chunk at a time.

    `system` and `temperature` are overridable only so that /chat can sweep
    wordings without a restart; the voice path always passes neither.
    """
    from transformers import TextIteratorStreamer

    if system is None:
        # The vision line only with tools, since seeing is a tool: a client that
        # announced none has no way to take a picture, and telling that model it
        # has a camera is how it starts describing rooms it has never seen.
        system = SYSTEM_PROMPT + (TOOL_PROMPT + (VISION_PROMPT if VISION else "")
                                  if tools else "")
    if temperature is None:
        temperature = TEMPERATURE
    prompt = _template().apply_chat_template(
        [{"role": "system", "content": system}] + history,
        # The chat template writes the tool instructions and the call format
        # into the prompt itself; there is nothing to hand-roll here, and
        # hand-rolling it would only disagree with what the model was tuned on.
        tools=list(tools) or None,
        tokenize=False,
        add_generation_prompt=True,
    )
    images = _images(history) if VISION else []
    if _processor is not None:
        # One call for both, because the pictures are not independent of the
        # text: the processor expands each placeholder in the prompt into as
        # many tokens as that image actually takes, and the two must agree.
        inputs = _processor(
            text=[prompt], images=images or None, return_tensors="pt").to(_llm.device)
    else:
        inputs = _tokenizer(prompt, return_tensors="pt").to(_llm.device)
    # The timeout is the backstop for the failure below: if the worker dies in a
    # way that never reaches our handler, the iterator gives up rather than
    # holding the WebSocket open forever. Generous, because a compile of the
    # decode step lands on the first token of a turn whose shape is new -- which
    # _load now pays for both shapes, so reaching this means something else.
    streamer = TextIteratorStreamer(
        _tokenizer, skip_prompt=True, skip_special_tokens=True, timeout=STREAM_TIMEOUT_S
    )

    kwargs: dict[str, Any] = dict(
        **inputs,
        streamer=streamer,
        max_new_tokens=MAX_NEW_TOKENS,
        do_sample=temperature > 0,
        temperature=temperature or None,
        top_p=0.9,
        pad_token_id=_tokenizer.eos_token_id,
    )
    # A prompt that would overrun the static window falls back to the dynamic
    # cache -- correct, just slower -- rather than truncating the conversation.
    # _trim keeps this rare, but a single very long first utterance can still
    # reach it.
    if _cache is not None and inputs["input_ids"].shape[-1] + MAX_NEW_TOKENS > CACHE_LEN:
        kwargs["cache_implementation"] = None

    # generate() runs on a worker so its tokens can be consumed as they land, but
    # that also means an exception in it is invisible here: the streamer simply
    # never ends and the caller waits forever. Catch it, end the stream by hand,
    # and re-raise on the consuming side so the turn fails loudly instead.
    failure: list[BaseException] = []

    def run() -> None:
        try:
            _llm.generate(**kwargs)
        except BaseException as exc:  # noqa: BLE001 -- re-raised below
            failure.append(exc)
            streamer.end()

    thread = Thread(target=run, daemon=True)
    thread.start()
    # A silent worker and a dead one arrive here identically -- as `queue.Empty`
    # raised from inside transformers, which says nothing about this service and
    # nothing a user could act on. Both are named instead.
    quiet = False
    try:
        yield from streamer
    except Empty:
        quiet = True
    # Not waited on when it is still running: a worker that has spent the whole
    # timeout compiling will keep compiling, and blocking on it here would hold
    # the socket for as long again.
    thread.join(timeout=1.0)
    if failure:
        raise failure[0]
    if quiet:
        raise TimeoutError(
            f"the model produced nothing for {STREAM_TIMEOUT_S:.0f}s. If this is the first "
            "turn of a shape it has not seen -- the first one with a picture, say -- it is "
            "compiling, and _load is meant to have paid that already.")


def _speak(text: str) -> np.ndarray:
    """Render one sentence to 24kHz PCM."""
    chunks = [audio for _gs, _ps, audio in _tts(text, voice=TTS_VOICE, speed=TTS_SPEED)]
    if not chunks:
        return np.zeros(0, dtype=np.float32)
    audio = np.concatenate([np.asarray(c, dtype=np.float32).reshape(-1) for c in chunks])
    return audio


def _to_pcm16(audio: np.ndarray) -> bytes:
    return (np.clip(audio, -1.0, 1.0) * 32767.0).astype("<i2").tobytes()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    _load()
    yield


app = FastAPI(title="Voice Chat", version="1.0", lifespan=lifespan)


@app.get("/health")
def health() -> dict[str, Any]:
    return {
        "ok": True,
        "device": DEVICE,
        "cuda": torch.cuda.is_available(),
        "loaded": _llm is not None,
        "stt_model": STT_MODEL,
        "llm_model": LLM_MODEL,
        "tts_voice": TTS_VOICE,
        "quantization": _quantization,
        # As with the vision services, `compile` and `cache` report what is
        # actually in force, not what was requested.
        "cache": "static" if _cache is not None else "dynamic",
        "cache_len": CACHE_LEN if _cache is not None else None,
        "compiled": _compiled,
        "in_rate": STT_RATE,
        "out_rate": TTS_RATE,
        # What a client has to know before it offers a tool that takes a
        # picture: with vision off, /frame refuses and such a tool can only fail.
        "vision": VISION,
        "image_tokens": _image_tokens if VISION else None,
        "frames_held": len(_frames),
        # Tools are per-connection, announced by each client -- this is only the
        # ceiling on what one may announce.
        "max_tools": MAX_TOOLS,
        "max_tool_calls": MAX_TOOL_CALLS,
    }


@app.post("/chat")
async def chat(request: dict[str, Any]) -> Any:
    """Text in, text out, for checking what the model *decides* -- never spoken.

    This exists because the interesting failure is not audible. A model that
    narrates an action it never took sounds exactly like one that took it, and
    finding that out through the microphone costs a TTS round trip, a decode and
    a playback per attempt -- nine seconds to learn one bit. Here it is one
    request, nothing is synthesised, and no tool is performed: the call the model
    *wanted* to make is reported instead, so this can be pointed at the real
    schemas without touching the rover.

    `system` and `temperature` may be overridden per request, which is the whole
    point of the endpoint: prompt wording is the thing most likely to need
    twenty attempts, and restarting this service to try one costs ~150s.

        {"text": "can you turn the lights off", "tools": [...],
         "system": "...", "temperature": 0.2}
     -> {"reply": "...", "tool_calls": [{"name": ..., "arguments": {...}}]}
    """
    text = request.get("text") or ""
    tools = [t for t in (request.get("tools") or [])[:MAX_TOOLS]
             if isinstance(t, dict) and isinstance(t.get("function"), dict)]
    history = list(request.get("history") or []) + [{"role": "user", "content": text}]
    system = request.get("system")
    temperature = request.get("temperature")

    def run() -> dict[str, Any]:
        spoken, calls = [], []
        for kind, value in _reply_stream(history, tools, system=system,
                                         temperature=temperature):
            (spoken if kind == "sentence" else calls).append(value)
        return {"reply": " ".join(spoken), "tool_calls": calls}

    async with _gpu_lock:
        return await asyncio.to_thread(run)


def _stash(image: Any) -> str:
    """Hold one frame under a name, and forget the stale ones. Returns the name."""
    global _frame_seq
    now = time.monotonic()
    for token, (_image, at) in list(_frames.items()):
        if now - at > FRAME_TTL_S:
            del _frames[token]
    while len(_frames) >= MAX_FRAMES:
        del _frames[min(_frames, key=lambda t: _frames[t][1])]
    _frame_seq += 1
    token = f"frame-{_frame_seq}"
    _frames[token] = (image, now)
    return token


def _decode_frame(data: bytes) -> Any:
    """JPEG bytes -> an image the model can be shown, downscaled if it is huge."""
    import io

    from PIL import Image

    image = Image.open(io.BytesIO(data))
    image.load()  # decode here, so a truncated frame fails here and not mid-turn
    image = image.convert("RGB")
    if max(image.size) > VISION_MAX_SIDE:
        scale = VISION_MAX_SIDE / max(image.size)
        image = image.resize((max(int(image.width * scale), 1),
                              max(int(image.height * scale), 1)))
    return image


@app.post("/frame")
async def frame(request: Request) -> Any:
    """One JPEG from whoever holds a camera, kept until a tool result claims it.

    This is the whole reason a picture can reach the model at all. The camera is
    on the rover, the microphone is on a desk, and the conversation is here --
    so the frame goes straight from the rover to this card, exactly as it
    already does to `face-detect`, rather than being carried through the client
    that only ever wanted to talk. What crosses the client is the *name* the
    frame was given, in an ordinary tool result.

        POST /frame   body: one JPEG
          -> {"ok": true, "image": "frame-7", "w": 640, "h": 480}

    Decoded here rather than at the point of use, so that a truncated frame --
    which is what a camera that has only just been opened produces -- is
    reported to the rover, which can take another one, instead of failing in
    the middle of somebody's sentence.
    """
    if not VISION:
        return JSONResponse(
            {"ok": False,
             "error": "this service is not running a vision model; "
                      "set VOICE_VISION=1 and a vision-language VOICE_LLM_MODEL"},
            status_code=409,
        )
    data = await request.body()
    if not data:
        return JSONResponse({"ok": False, "error": "empty body; expected one JPEG"},
                            status_code=400)
    try:
        image = await asyncio.to_thread(_decode_frame, data)
    except Exception as exc:
        return JSONResponse(
            {"ok": False, "error": f"not a decodable image: {type(exc).__name__}"},
            status_code=400)
    token = _stash(image)
    return {"ok": True, "image": token, "w": image.width, "h": image.height,
            "bytes": len(data)}


@app.post("/say")
async def say(text: str) -> Any:
    """TTS on its own -- for checking a voice without holding a conversation."""
    async with _gpu_lock:
        audio = await asyncio.to_thread(_speak, text)
    from fastapi.responses import Response

    return Response(content=_to_pcm16(audio), media_type="application/octet-stream")


async def _run_tool(ws: WebSocket, call: dict[str, Any], index: int) -> dict[str, Any]:
    """Ask the client to perform one call and wait for its answer.

    Deliberately inside the GPU lock. Holding a card idle across a LAN round
    trip looks wasteful, but this is a single-user assistant and the alternative
    is worse: releasing the lock mid-turn lets another turn interleave into the
    same conversation, between a tool call and its result.
    """
    ident = f"{index}"
    await ws.send_json(
        {"type": "tool", "id": ident, "name": call["name"], "arguments": call["arguments"]}
    )
    deadline = time.monotonic() + TOOL_TIMEOUT_S
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            msg = await asyncio.wait_for(ws.receive(), remaining)
        except asyncio.TimeoutError:
            break
        if msg["type"] == "websocket.disconnect":
            raise WebSocketDisconnect(msg.get("code", 1005))
        if (text := msg.get("text")) is None:
            # Audio arriving mid-turn is the microphone running ahead of the
            # reply; it belongs to no utterance and is dropped.
            continue
        event = json.loads(text)
        if event.get("type") == "tool_result" and event.get("id") == ident:
            result = event.get("result")
            return result if isinstance(result, dict) else {"ok": False, "error": "no result"}
    # A tool that never answers is reported to the model rather than failing the
    # turn: it can say the rover did not respond, which is what the user needs
    # to hear, and the conversation carries on.
    return {"ok": False, "error": f"the rover did not answer within {TOOL_TIMEOUT_S:.0f} seconds"}


async def _decode_and_speak(
    ws: WebSocket,
    history: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    spoken: list[str],
) -> tuple[dict[str, Any] | None, float | None]:
    """One decode: speak what it says, and return any tool call it made.

    Decoding runs on a worker thread and hands sentences to the event loop as
    they complete, rather than collecting them into a list first. This is the
    whole point of the splitter: sentence 1 is being spoken and sent while
    sentence 2 is still decoding. Materialising the generator would put the
    entire reply's decode time in front of the first audio frame.
    """
    loop = asyncio.get_running_loop()
    queue: asyncio.Queue[tuple[str, Any] | None | Exception] = asyncio.Queue()

    def produce() -> None:
        try:
            for item in _reply_stream(history, tools):
                loop.call_soon_threadsafe(queue.put_nowait, item)
        except Exception as exc:
            loop.call_soon_threadsafe(queue.put_nowait, exc)
        finally:
            loop.call_soon_threadsafe(queue.put_nowait, None)

    Thread(target=produce, daemon=True).start()

    call: dict[str, Any] | None = None
    first_audio: float | None = None
    while True:
        item = await queue.get()
        if item is None:
            break
        if isinstance(item, Exception):
            raise item
        kind, value = item
        if kind == "tool":
            call = value
            continue
        spoken.append(value)
        await ws.send_json({"type": "text", "text": value})
        audio = await asyncio.to_thread(_speak, value)
        if audio.size:
            if first_audio is None:
                first_audio = time.perf_counter()
            await ws.send_bytes(_to_pcm16(audio))
    return call, first_audio


async def _run_turn(
    ws: WebSocket,
    history: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    pcm: np.ndarray,
) -> None:
    """One utterance in, one spoken reply out, with any tool calls in between."""
    t0 = time.perf_counter()
    async with _gpu_lock:
        heard = await asyncio.to_thread(_transcribe, pcm)
        t_stt = time.perf_counter()
        if not heard:
            await ws.send_json({"type": "stt", "text": "", "empty": True})
            await ws.send_json({"type": "done", "stats": {"stt_ms": int((t_stt - t0) * 1000)}})
            return

        await ws.send_json({"type": "stt", "text": heard})
        # A new question starts with nothing in front of it. The camera is on a
        # gimbal that sweeps while tracking runs, so the picture from the last
        # turn shows somewhere the rover is no longer pointing -- and a model
        # holding one does not take another, so it would answer about the past
        # in the present tense. Dropped here rather than after the last turn so
        # that the turn which took the picture keeps it for as long as it is
        # answering from it, including across its own tool calls.
        if VISION and FRESH_PICTURE:
            _forget_pictures(history, keep_newest=False)
        history.append({"role": "user", "content": heard})
        history[:] = _trim(history, tools)

        await ws.send_json({"type": "start", "rate": TTS_RATE})
        spoken: list[str] = []
        first_audio: float | None = None
        used = 0

        for round_index in range(MAX_TOOL_CALLS + 1):
            # The last pass is offered no tools, so the model has to answer in
            # words. Without that a model that has decided everything needs a
            # tool call can spend the whole turn calling them, and the user hears
            # nothing at all -- which is indistinguishable from a crash.
            offered = tools if round_index < MAX_TOOL_CALLS else []
            call, audio_at = await _decode_and_speak(ws, history, offered, spoken)
            if first_audio is None:
                first_audio = audio_at
            if call is None:
                break

            used += 1
            result = await _run_tool(ws, call, used)
            # A result naming a frame is a picture that arrived by the other
            # road: the rover posted it to /frame while this call was in
            # flight, and the name is how the two are tied together. Claimed
            # rather than copied, so the same frame cannot be shown twice.
            picture = _frames.pop(result["image"], (None, 0))[0] if (
                VISION and isinstance(result.get("image"), str)) else None
            if picture is not None:
                result = {k: v for k, v in result.items() if k != "image"}
            elif isinstance(result.get("image"), str):
                # The rover believes it sent one and this service does not have
                # it. Say so in the result: the model then tells the user it
                # could not see rather than inventing what was in front of it.
                result = {**{k: v for k, v in result.items() if k != "image"},
                          "ok": False,
                          "error": "the picture did not arrive; nothing was seen"}
            # Recorded as a call and a result rather than as prose, so the model
            # sees its own action in the form it was trained on -- and so the
            # next turn can answer "are they on?" from the history.
            history.append(
                {
                    "role": "assistant",
                    "content": "",
                    "tool_calls": [{"type": "function", "function": call}],
                }
            )
            history.append({"role": "tool", "content": json.dumps(result)})
            if picture is not None:
                history.append(_image_message(picture))
            history[:] = _trim(history, tools)

    reply = " ".join(spoken)
    history.append({"role": "assistant", "content": reply})
    await ws.send_json(
        {
            "type": "done",
            "text": reply,
            "stats": {
                "stt_ms": int((t_stt - t0) * 1000),
                "first_audio_ms": int(((first_audio or time.perf_counter()) - t0) * 1000),
                "total_ms": int((time.perf_counter() - t0) * 1000),
                "tools": used,
            },
        }
    )


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    """Conversation loop.

    Client -> server: binary frames of 16kHz mono s16le, then {"type":"end"} to
    close the utterance. {"type":"reset"} clears the history, and an opening
    {"type":"hello","tools":[...]} announces what this client can perform.
    Server -> client: JSON events, with each {"type":"text"} followed by one
    binary frame of 24kHz mono s16le holding that sentence, plus {"type":"tool"}
    for a call the client is to make and answer with {"type":"tool_result"}.
    """
    await ws.accept()
    history: list[dict[str, Any]] = []
    tools: list[dict[str, Any]] = []
    buf = bytearray()
    try:
        while True:
            msg = await ws.receive()
            if msg["type"] == "websocket.disconnect":
                break
            if (data := msg.get("bytes")) is not None:
                buf.extend(data)
                continue
            if (text := msg.get("text")) is None:
                continue

            event = json.loads(text)
            kind = event.get("type")
            if kind == "hello":
                # The client is the authority on what it can do, so its schemas
                # are taken as given rather than matched against a catalogue
                # here -- this service has no business knowing what a rover is.
                # Checked only for shape, and capped, so that a broken client
                # cannot fill the context window before anyone has spoken.
                tools = [
                    tool
                    for tool in (event.get("tools") or [])[:MAX_TOOLS]
                    if isinstance(tool, dict)
                    and isinstance(tool.get("function"), dict)
                    and isinstance(tool["function"].get("name"), str)
                ]
                # History is cleared with them: the tools available are part of
                # what the model was told, so a conversation cannot straddle a
                # change to the set without the earlier half becoming a lie.
                history.clear()
                await ws.send_json(
                    {"type": "hello", "tools": [t["function"]["name"] for t in tools]}
                )
            elif kind == "reset":
                history.clear()
                buf.clear()
                await ws.send_json({"type": "reset"})
            elif kind == "end":
                pcm = np.frombuffer(bytes(buf), dtype="<i2").astype(np.float32) / 32768.0
                buf.clear()
                # Under ~0.3s is a door slam or a cough, not speech; Whisper
                # hallucinates fluent sentences out of clips that short.
                if pcm.size < STT_RATE * 0.3:
                    await ws.send_json({"type": "stt", "text": "", "empty": True})
                    await ws.send_json({"type": "done", "stats": {}})
                    continue
                await _run_turn(ws, history, tools, pcm)
    except WebSocketDisconnect:
        pass
    except Exception as exc:  # keep the socket's failure off the service's logs as a crash
        try:
            await ws.send_json({"type": "error", "message": str(exc)})
        except Exception:
            pass
        raise


def main() -> None:
    import uvicorn

    _load()
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
