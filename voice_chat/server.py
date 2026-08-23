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
performed. A client that announces no tools gets a plain conversation, with no
mention of rover hardware in its context at all.

With VOICE_VISION=1 the reply model is a vision-language one and the
conversation can hold pictures. The picture does **not** arrive through the
client. A tool the client performs makes whoever holds the camera POST a JPEG
to `/frame` here, and the tool result names it; the image then enters the
history on this side, having gone straight from the camera to this card. That
is the same path `face-detect` already takes, and it keeps a 35kB frame off the
desk that only has a microphone on it -- see `voice_http.frame`.

Vision is a switch, not a rewrite: with it off, this is the text service it has
always been, down to the model that is loaded. The rollback is
`VOICE_VISION=0` plus the text model in the unit, and a restart.
"""

from __future__ import annotations

import asyncio
import os
import time
from queue import Empty
from threading import Thread
from typing import Any, Iterator

import numpy as np
import torch

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
# Capture the decode step as a CUDA graph, so a token costs one graph launch
# instead of several hundred kernel launches.
#
# Worth it because the card is not the constraint yet: 46 tok/s is 21.7ms a
# token, while 448 GB/s against 2.5GB of int4 weights is a 5.6ms roofline. The
# other 16ms is launch latency, and this host pays more of it than most -- media
# is Ubuntu under WSL2 (see docs/hosts.md), so every launch goes through the
# Windows WDDM scheduler.
#
# Inductor refuses this by default here, reporting "skipping cudagraphs due to
# mutated inputs (108 instances)": the static cache is written in place, which
# is exactly the aliasing a graph cannot normally tolerate. It is safe in this
# one case *because* the cache is static -- the buffers are allocated once and
# live at the same addresses for the life of the process, which is the property
# a graph needs. `cudagraph_support_input_mutation` is how that is asserted.
#
# Kept a switch because the failure mode is not a crash. Two turns can share a
# graph and quietly share K/V with it; if replies start referring to the
# previous question, turn this off first.
CUDAGRAPHS = os.environ.get("VOICE_CUDAGRAPHS", "1") not in ("0", "false", "False")
# Keep the K/V cache between turns and re-prefill only what actually changed.
#
# Every turn currently re-processes the whole prompt from the first token, and
# almost all of that prompt is identical to last turn's: the system prompt, the
# tool schemas, and every exchange before this one. Only the newest user message
# is new. Measured on this box, that waste is the single largest cost in a turn
# once a rover is attached -- ~134ms of prefill per tool schema, so ~1.4s for the
# rover's ten, paid again on every question. Prefill is where int4 hurts (see
# INT4_SKIP: weight-only int4 has to unpack for the wide GEMMs prefill is made
# of), which is why the number is that big.
#
# The mechanism is a longest-common-prefix compare against the tokens the cache
# already holds, then StaticCache.crop() down to that point. Nothing is assumed
# to be stable: what is reused is what is *verified* identical, token for token,
# so a changed system prompt or a trimmed history simply reuses less.
PREFIX_CACHE = os.environ.get("VOICE_PREFIX_CACHE", "1") not in ("0", "false", "False")
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
#
# The last sentence is aimed at one failure and was measured on the vision
# model, which the rest of this wording never was -- it was tuned on
# Qwen3-4B-Instruct-2507 and inherited unexamined. What Qwen3-VL does instead of
# calling is *promise*: "I will switch the lights on for you", no call, and a
# user who hears it standing next to a rover that did not move. Naming the words
# it uses beats arguing that a question is a request, which this prompt already
# does two sentences earlier and which is not enough on its own. Two independent
# six-sample runs, 66/90 -> 75/90 and 40/60 -> 51/60, nothing regressed:
#
#   "Well, can you switch the lights on?"  0/6 -> 6/6
#   "Can you switch the lights on?"        4/6 -> 6/6
#   "Follow me."                           2/6 -> 3/6
#
# Its **position is worth more than the sentence**, and the opposite way round
# from the vision line, where front was what worked: moved to the front of this
# prompt it scores 42/90, below saying nothing at all, and it stops being a
# missing call and starts being a lie -- "I switched the lights off" with no
# call made, on requests that pass 6/6 today. Re-measure both ends if it moves.
TOOL_PROMPT = os.environ.get(
    "VOICE_TOOL_PROMPT",
    " You control this rover through the tools you have been given. Call a tool "
    "whenever you are asked to do something one of them covers, including when "
    "the request is phrased as a question such as 'can you turn the lights off'. "
    "You have done something only if you have called a tool for it: never say "
    "you have switched, moved, started or stopped anything unless the call was "
    "made and answered. Then say what you did in one short sentence, without "
    "reading the tool call or its result out loud."
    " Do not say 'I will', 'I'll' or 'I am going to' about anything a tool "
    "does. Call the tool instead, and say what you did afterwards.",
)

_stt = None
_llm = None
_tokenizer = None
_processor = None  # only with VISION; None keeps every image path unreachable
_tts = None
_cache = None
_quantization = "none"
_compiled = False
_graphs = False  # whether the compiled decode step is captured as a CUDA graph
_image_tokens = IMAGE_TOKENS
# The K/V cache carried between turns, and the exact tokens it was filled from.
# The two are only ever written together: a cache whose contents are not
# described by _kv_ids is worse than no cache at all, because the reuse test
# would then be comparing against a claim rather than a fact.
_kv = None
_kv_ids: list[int] = []
# Frames posted by whoever holds the camera, waiting to be claimed by the tool
# result that names them. Small and short-lived by construction -- see /frame.
_frames: dict[str, tuple[Any, float]] = {}
_frame_seq = 0
# One turn at a time. The three models share a card and a CUDA stream, and two
# concurrent turns would interleave into thrash rather than throughput; this is
# a single-user assistant, so serialising is honest rather than limiting.
_gpu_lock = asyncio.Lock()

from voice_history import (
    _blind_refusal, _exchanges, _forget_pictures, _forget_promises,
    _forget_refusals, _image_message, _images, _measure_image_tokens,
    _promised, _prompt_len, _template, _textual, _trim,
)
from voice_stream import (
    _ToolSniffer, _parse_tool_call, _sentences,
)

def _quant_config():
    """The 4-bit backend to load with, plus the label /health reports."""
    if LLM_QUANT in ("0", "none", ""):
        return None, "none"
    if LLM_QUANT in ("checkpoint", "awq", "gptq"):
        # A checkpoint that arrives already quantized carries its own config and
        # its own kernels, and the only thing this service has to do is not
        # quantize it a second time. Worth having as a path because the fastest
        # 4-bit kernels on sm86 are Marlin's, which come with an AWQ or GPTQ
        # checkpoint rather than from torchao -- see the README for why none of
        # them is loadable here yet.
        return None, f"{LLM_QUANT} (from the checkpoint)"
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
    global _quantization, _compiled, _graphs, _image_tokens
    global _kv
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
        _llm.generation_config.max_length = CACHE_LEN
        if PREFIX_CACHE:
            # Owning the cache is the whole point: one that transformers
            # allocates per call cannot outlive the call, and outliving the call
            # is what makes the next turn cheap. The cost of taking it over is
            # that `cache_implementation` may no longer be set -- generate()
            # rejects being given both -- so this is either/or, not both.
            from transformers import StaticCache

            _kv = StaticCache(
                # A vision model's top-level config has no attention shape on
                # it; the text tower's does.
                config=_llm.config.get_text_config(),
                max_cache_len=CACHE_LEN,
            )
            _llm.generation_config.cache_implementation = None
            _cache = "static (kept between turns)"
        else:
            _llm.generation_config.cache_implementation = "static"
            _cache = "static"
        if COMPILE:
            # transformers wraps only the decode step; prefill stays eager, so a
            # variable-length prompt does not trigger a recompile every turn.
            # Compilation is an optimization, so a failure here degrades to eager
            # rather than taking the service down -- /health reports which it got.
            try:
                # "reduce-overhead" is what asks for CUDA graphs. On its own it
                # gets none -- inductor skips them because the static cache is
                # written in place -- so the flag below is the half that makes
                # the mode mean anything. See CUDAGRAPHS for why that is sound
                # here and what it looks like when it is not.
                mode = None
                if CUDAGRAPHS:
                    from torch._inductor import config as inductor_config

                    try:
                        inductor_config.triton.cudagraph_support_input_mutation = True
                        mode = "reduce-overhead"
                    except AttributeError:
                        # Older inductor without the knob. Fused kernels only,
                        # which is what this service ran on before.
                        print("[voice] no cudagraph_support_input_mutation; "
                              "staying with fused kernels only", flush=True)
                _llm.generation_config.compile_config = CompileConfig(
                    fullgraph=True, dynamic=COMPILE_DYNAMIC, mode=mode
                )
                _compiled = True
                _graphs = mode is not None
            except Exception as exc:
                print(f"[voice] compile unavailable, staying eager: {exc}", flush=True)

    if VISION:
        _image_tokens = _measure_image_tokens()

    print(
        f"[voice] loaded in {t_tts - t0:.1f}s "
        f"(stt {t_stt - t0:.1f}s, llm {t_llm - t_stt:.1f}s, tts {t_tts - t_llm:.1f}s) "
        f"quant={_quantization} compile={_compiled} dynamic={COMPILE_DYNAMIC} "
        f"cudagraphs={_graphs}"
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



def _pcm(buf: bytearray) -> np.ndarray:
    """The client's 16kHz mono s16le buffer as float32 in [-1, 1]."""
    return np.frombuffer(bytes(buf), dtype="<i2").astype(np.float32) / 32768.0


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


def _reuse_prefix(ids: list[int], kwargs: dict[str, Any]) -> None:
    """Point this call at the kept cache, keeping whatever prefix still matches.

    The comparison is token for token against what the cache is *known* to hold,
    so a changed system prompt, a trimmed history or a different tool set is not
    a special case -- each simply reuses less.
    """
    global _kv_ids

    keep = 0
    for held, wanted in zip(_kv_ids, ids):
        if held != wanted:
            break
        keep += 1
    # Leave generate at least one token to forward: a prompt served entirely
    # from cache has nothing to run the first decode step from.
    keep = min(keep, len(ids) - 1)
    # Rewinding is done by writing the length back, not by dropping anything:
    # a static cache's buffers are allocated once and the first `keep` positions
    # still hold what was written there, so moving the write cursor back is the
    # whole operation. StaticCache.crop() looks like the API for this and is
    # not -- it delegates to a per-layer crop that StaticLayer does not
    # implement, and raises. In-place `fill_` also keeps the tensor's identity,
    # which matters once CUDA graphs have captured its address.
    for layer in _kv.layers:
        # Untouched on the first turn, when nothing has been written yet and
        # `keep` is 0 in any case.
        if layer.is_initialized:
            layer.cumulative_length.fill_(keep)
    kwargs["past_key_values"] = _kv
    # Claim nothing until the call has actually written it. generate() is about
    # to fill this cache, and if it raises partway the contents describe some
    # prefix nobody can name -- so the honest record until it returns is "empty",
    # which costs the next turn a full prefill and cannot corrupt it.
    _kv_ids = []


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
    prompt_ids: list[int] = []
    overlong = inputs["input_ids"].shape[-1] + MAX_NEW_TOKENS > CACHE_LEN
    if PREFIX_CACHE and _kv is not None:
        # A prompt that would overrun the window is handed no cache at all, and
        # transformers falls back to a dynamic one for that call. The kept cache
        # is left untouched rather than reset, so the *next* turn -- which _trim
        # will have brought back under the limit -- still reuses its prefix.
        if not overlong:
            prompt_ids = inputs["input_ids"][0].tolist()
            _reuse_prefix(prompt_ids, kwargs)
    elif _cache is not None and overlong:
        # A prompt that would overrun the static window falls back to the
        # dynamic cache -- correct, just slower -- rather than truncating the
        # conversation. _trim keeps this rare, but a single very long first
        # utterance can still reach it.
        kwargs["cache_implementation"] = None

    # generate() runs on a worker so its tokens can be consumed as they land, but
    # that also means an exception in it is invisible here: the streamer simply
    # never ends and the caller waits forever. Catch it, end the stream by hand,
    # and re-raise on the consuming side so the turn fails loudly instead.
    failure: list[BaseException] = []

    def run() -> None:
        global _kv_ids
        try:
            _llm.generate(**kwargs)
            if kwargs.get("past_key_values") is not None:
                # Written, so now it can be claimed -- and only the prompt is
                # claimed, though the cache also holds the reply that followed
                # it. Describing less than is there is always safe; describing
                # more is the bug this ordering exists to prevent.
                _kv_ids = prompt_ids
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


def __getattr__(name: str):
    """Load the FastAPI app on first use, so desk tests need no fastapi."""
    if name == "app":
        from voice_http import app
        return app
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def main() -> None:
    import uvicorn
    from voice_http import app

    _load()
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
