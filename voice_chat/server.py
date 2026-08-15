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
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import time
from contextlib import asynccontextmanager
from threading import Thread
from typing import Any, Iterator

import numpy as np
import torch
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
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
TEMPERATURE = float(os.environ.get("VOICE_TEMPERATURE", "0.7"))

# Spoken replies, not written ones. Without this the model reaches for bullets,
# headings and code fences, and Kokoro reads the punctuation out loud.
SYSTEM_PROMPT = os.environ.get(
    "VOICE_SYSTEM_PROMPT",
    "You are the voice of a small tracked rover. You are speaking out loud, so "
    "reply in one to three short sentences of plain spoken English. Never use "
    "markdown, bullet points, headings, emoji or code. Write numbers and units "
    "as you would say them aloud. If you do not know something, say so briefly.",
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

_stt = None
_llm = None
_tokenizer = None
_tts = None
_cache = None
_quantization = "none"
_compiled = False
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
        return TorchAoConfig(quant_type=cfg), f"int4/{INT4_PACKING}"
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
    global _stt, _llm, _tokenizer, _tts, _cache, _quantization, _compiled
    if _llm is not None:
        return

    from faster_whisper import WhisperModel
    from kokoro import KPipeline
    from transformers import AutoModelForCausalLM, AutoTokenizer, CompileConfig

    t0 = time.perf_counter()
    _stt = WhisperModel(STT_MODEL, device=DEVICE, compute_type=STT_COMPUTE)
    t_stt = time.perf_counter()

    quant, _quantization = _quant_config()
    _tokenizer = AutoTokenizer.from_pretrained(LLM_MODEL)
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

    print(
        f"[voice] loaded in {t_tts - t0:.1f}s "
        f"(stt {t_stt - t0:.1f}s, llm {t_llm - t_stt:.1f}s, tts {t_tts - t_llm:.1f}s) "
        f"quant={_quantization} compile={_compiled} dynamic={COMPILE_DYNAMIC}",
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
            print(f"[voice] warmed in {time.perf_counter() - t_warm:.1f}s", flush=True)
        except Exception as exc:
            # A failed warmup is not a failed service -- the real request will
            # raise its own error, and that one has somewhere to be reported to.
            print(f"[voice] warmup failed ({exc}); first turn will be slow", flush=True)


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


def _trim(history: list[dict[str, str]]) -> list[dict[str, str]]:
    """Drop whole turns off the front until the prompt fits the static cache.

    Whole turns, not tokens: half a turn in the history reads to the model as
    the user being interrupted, and it starts apologising for things it did not
    do. Leaves room for the reply as well as the prompt, since both share the
    window.
    """
    budget = CACHE_LEN - MAX_NEW_TOKENS - 32
    while len(history) > 1:
        text = _tokenizer.apply_chat_template(
            [{"role": "system", "content": SYSTEM_PROMPT}] + history,
            tokenize=True,
            add_generation_prompt=True,
        )
        if len(text) <= budget:
            break
        # Two at a time keeps user/assistant pairing intact.
        history = history[2:]
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


def _generate(history: list[dict[str, str]]) -> Iterator[str]:
    """Yield the reply as it decodes, one token-chunk at a time."""
    from transformers import TextIteratorStreamer

    prompt = _tokenizer.apply_chat_template(
        [{"role": "system", "content": SYSTEM_PROMPT}] + history,
        tokenize=False,
        add_generation_prompt=True,
    )
    inputs = _tokenizer(prompt, return_tensors="pt").to(_llm.device)
    # The timeout is the backstop for the failure below: if the worker dies in a
    # way that never reaches our handler, the iterator gives up rather than
    # holding the WebSocket open forever. Generous, because a cold compile of the
    # decode step lands on the first token of the first turn.
    streamer = TextIteratorStreamer(
        _tokenizer, skip_prompt=True, skip_special_tokens=True, timeout=180
    )

    kwargs: dict[str, Any] = dict(
        **inputs,
        streamer=streamer,
        max_new_tokens=MAX_NEW_TOKENS,
        do_sample=TEMPERATURE > 0,
        temperature=TEMPERATURE or None,
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
    yield from streamer
    thread.join()
    if failure:
        raise failure[0]


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
    }


@app.post("/say")
async def say(text: str) -> Any:
    """TTS on its own -- for checking a voice without holding a conversation."""
    async with _gpu_lock:
        audio = await asyncio.to_thread(_speak, text)
    from fastapi.responses import Response

    return Response(content=_to_pcm16(audio), media_type="application/octet-stream")


async def _run_turn(ws: WebSocket, history: list[dict[str, str]], pcm: np.ndarray) -> None:
    """One utterance in, one spoken reply out."""
    t0 = time.perf_counter()
    async with _gpu_lock:
        heard = await asyncio.to_thread(_transcribe, pcm)
        t_stt = time.perf_counter()
        if not heard:
            await ws.send_json({"type": "stt", "text": "", "empty": True})
            await ws.send_json({"type": "done", "stats": {"stt_ms": int((t_stt - t0) * 1000)}})
            return

        await ws.send_json({"type": "stt", "text": heard})
        history.append({"role": "user", "content": heard})
        history[:] = _trim(history)

        await ws.send_json({"type": "start", "rate": TTS_RATE})
        reply_parts: list[str] = []
        first_audio: float | None = None

        # Decode on a worker thread and hand sentences to the event loop as they
        # complete, rather than collecting them into a list first. This is the
        # whole point of the splitter: sentence 1 is being spoken and sent while
        # sentence 2 is still decoding. Materialising the generator would put the
        # entire reply's decode time in front of the first audio frame.
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[str | None | Exception] = asyncio.Queue()

        def produce() -> None:
            try:
                for sentence in _sentences(_generate(history)):
                    loop.call_soon_threadsafe(queue.put_nowait, sentence)
            except Exception as exc:
                loop.call_soon_threadsafe(queue.put_nowait, exc)
            finally:
                loop.call_soon_threadsafe(queue.put_nowait, None)

        Thread(target=produce, daemon=True).start()

        while True:
            item = await queue.get()
            if item is None:
                break
            if isinstance(item, Exception):
                raise item
            reply_parts.append(item)
            await ws.send_json({"type": "text", "text": item})
            audio = await asyncio.to_thread(_speak, item)
            if audio.size:
                if first_audio is None:
                    first_audio = time.perf_counter()
                await ws.send_bytes(_to_pcm16(audio))

    reply = " ".join(reply_parts)
    history.append({"role": "assistant", "content": reply})
    await ws.send_json(
        {
            "type": "done",
            "text": reply,
            "stats": {
                "stt_ms": int((t_stt - t0) * 1000),
                "first_audio_ms": int(((first_audio or time.perf_counter()) - t0) * 1000),
                "total_ms": int((time.perf_counter() - t0) * 1000),
            },
        }
    )


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    """Conversation loop.

    Client -> server: binary frames of 16kHz mono s16le, then {"type":"end"} to
    close the utterance. {"type":"reset"} clears the history.
    Server -> client: JSON events, with each {"type":"text"} followed by one
    binary frame of 24kHz mono s16le holding that sentence.
    """
    await ws.accept()
    history: list[dict[str, str]] = []
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
            if kind == "reset":
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
                await _run_turn(ws, history, pcm)
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
