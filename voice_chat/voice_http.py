"""HTTP and WebSocket surface of the voice service.

Kept beside [server.py](server.py) so the model-loading file stays near the
prompt literals that prompts.py reads with ast. Tests still patch generate
and history helpers on the server module; this file only owns the routes.
"""
from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager
from threading import Thread
from typing import Any

import numpy as np
import torch
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

import server as S


@asynccontextmanager
async def lifespan(_app: FastAPI):
    S._load()
    yield


app = FastAPI(title="Voice Chat", version="1.0", lifespan=lifespan)


@app.get("/health")
def health() -> dict[str, Any]:
    return {
        "ok": True,
        "device": S.DEVICE,
        "cuda": torch.cuda.is_available(),
        "loaded": S._llm is not None,
        "stt_model": S.STT_MODEL,
        "llm_model": S.LLM_MODEL,
        "tts_voice": S.TTS_VOICE,
        "quantization": S._quantization,
        # As with the vision services, `compile` and `cache` report what is
        # actually in force, not what was requested.
        "cache": S._cache or "dynamic",
        "cache_len": S.CACHE_LEN if S._cache is not None else None,
        "compiled": S._compiled,
        "cudagraphs": S._graphs,
        "in_rate": S.STT_RATE,
        "out_rate": S.TTS_RATE,
        # What a client has to know before it offers a tool that takes a
        # picture: with vision off, /frame refuses and such a tool can only fail.
        "vision": S.VISION,
        "image_tokens": S._image_tokens if S.VISION else None,
        "frames_held": len(S._frames),
        # Tools are per-connection, announced by each client -- this is only the
        # ceiling on what one may announce.
        "max_tools": S.MAX_TOOLS,
        "max_tool_calls": S.MAX_TOOL_CALLS,
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
    tools = [t for t in (request.get("tools") or [])[:S.MAX_TOOLS]
             if isinstance(t, dict) and isinstance(t.get("function"), dict)]
    history = list(request.get("history") or []) + [{"role": "user", "content": text}]
    system = request.get("system")
    temperature = request.get("temperature")

    def run() -> dict[str, Any]:
        spoken, calls = [], []
        for kind, value in S._reply_stream(history, tools, system=system,
                                         temperature=temperature):
            (spoken if kind == "sentence" else calls).append(value)
        return {"reply": " ".join(spoken), "tool_calls": calls}

    async with S._gpu_lock:
        return await asyncio.to_thread(run)


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
    if not S.VISION:
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
        image = await asyncio.to_thread(S._decode_frame, data)
    except Exception as exc:
        return JSONResponse(
            {"ok": False, "error": f"not a decodable image: {type(exc).__name__}"},
            status_code=400)
    token = S._stash(image)
    return {"ok": True, "image": token, "w": image.width, "h": image.height,
            "bytes": len(data)}


@app.post("/say")
async def say(text: str) -> Any:
    """TTS on its own -- for checking a voice without holding a conversation."""
    async with S._gpu_lock:
        audio = await asyncio.to_thread(S._speak, text)
    from fastapi.responses import Response

    return Response(content=S._to_pcm16(audio), media_type="application/octet-stream")


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
    deadline = time.monotonic() + S.TOOL_TIMEOUT_S
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
    return {"ok": False, "error": f"the rover did not answer within {S.TOOL_TIMEOUT_S:.0f} seconds"}


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
            for item in S._reply_stream(history, tools):
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
        audio = await asyncio.to_thread(S._speak, value)
        if audio.size:
            if first_audio is None:
                first_audio = time.perf_counter()
            await ws.send_bytes(S._to_pcm16(audio))
    return call, first_audio


async def _run_turn(
    ws: WebSocket,
    history: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    pcm: np.ndarray | None,
    heard: str | None = None,
) -> None:
    """One utterance in, one spoken reply out, with any tool calls in between.

    `heard` short-circuits the transcription: the client sent this utterance
    ahead during its hang window and the text is already known. `pcm` is then
    None, because there is nothing left to do with the audio.
    """
    t0 = time.perf_counter()
    async with S._gpu_lock:
        early = heard is not None
        if not early:
            heard = await asyncio.to_thread(S._transcribe, pcm)
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
        # in the present tense. The whole exchange goes, not just the picture:
        # the answer spoken from a picture is read back as an answer about now
        # (see _forget_pictures). Dropped here rather than after the last turn so
        # that the turn which took the picture keeps it for as long as it is
        # answering from it, including across its own tool calls.
        if S.VISION and S.FRESH_PICTURE:
            S._forget_pictures(history, keep_newest=False)
            # And the turns that refused to see, for the same reason: they are
            # the other thing the model reads back to itself instead of looking.
            S._forget_refusals(history)
        # And the turns that promised and did not act, which is the same rule
        # again pointed at the tools rather than at the camera. Only with tools
        # attached: "I'll be here" is not a promise about hardware when there is
        # no hardware to reach, and a client that announced nothing cannot act
        # whatever it says.
        if tools:
            S._forget_promises(history)
        history.append({"role": "user", "content": heard})
        history[:] = S._trim(history, tools)

        await ws.send_json({"type": "start", "rate": S.TTS_RATE})
        spoken: list[str] = []
        first_audio: float | None = None
        used = 0

        for round_index in range(S.MAX_TOOL_CALLS + 1):
            # The last pass is offered no tools, so the model has to answer in
            # words. Without that a model that has decided everything needs a
            # tool call can spend the whole turn calling them, and the user hears
            # nothing at all -- which is indistinguishable from a crash.
            offered = tools if round_index < S.MAX_TOOL_CALLS else []
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
            picture = S._frames.pop(result["image"], (None, 0))[0] if (
                S.VISION and isinstance(result.get("image"), str)) else None
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
                history.append(S._image_message(picture, heard))
            history[:] = S._trim(history, tools)

    reply = " ".join(spoken)
    history.append({"role": "assistant", "content": reply})
    await ws.send_json(
        {
            "type": "done",
            "text": reply,
            "stats": {
                # Zero when the client sent the utterance ahead: the cost was
                # real, it was just paid during the hang window. Reported as a
                # flag rather than folded into the number, so a turn that got
                # STT for free is not confused with one where Whisper was fast.
                "stt_ms": int((t_stt - t0) * 1000),
                "stt_early": early,
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

    A client may also send the utterance *early*, while it is still counting out
    its hang window, so that transcription happens on a card that would be idle
    anyway: binary frames then {"type":"speculate"}, followed by either
    {"type":"cancel"} if the speaker turned out to be mid-sentence, or
    {"type":"end","early":true} with no audio to confirm it. If the held
    transcript is gone by then the server answers {"type":"resend"} and the
    client falls back to the ordinary road. A client that does none of this is
    unaffected -- the plain path is untouched.
    """
    await ws.accept()
    history: list[dict[str, Any]] = []
    tools: list[dict[str, Any]] = []
    buf = bytearray()
    # The transcript of an utterance the client sent ahead, held until it says
    # whether the turn really ended. None means nothing is held; "" means
    # something was transcribed and it came to nothing, which is a different
    # answer and must not be confused with the first.
    held: str | None = None
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
                    for tool in (event.get("tools") or [])[:S.MAX_TOOLS]
                    if isinstance(tool, dict)
                    and isinstance(tool.get("function"), dict)
                    and isinstance(tool["function"].get("name"), str)
                ]
                # History is cleared with them: the tools available are part of
                # what the model was told, so a conversation cannot straddle a
                # change to the set without the earlier half becoming a lie.
                history.clear()
                held = None
                await ws.send_json(
                    {
                        "type": "hello",
                        "tools": [t["function"]["name"] for t in tools],
                        # How a client knows it may send an utterance early. A
                        # service without this key is one that would buffer the
                        # speculation and then answer it twice over.
                        "early": True,
                    }
                )
            elif kind == "reset":
                history.clear()
                buf.clear()
                held = None
                await ws.send_json({"type": "reset"})
            elif kind == "speculate":
                # The speaker has stopped, but the client has not yet decided
                # the turn is over -- it is counting out the hang window. That
                # window is dead time on a card with nothing else to do, so
                # transcribe now and hold the text until the client either
                # confirms the turn or takes it back.
                #
                # Under the GPU lock like any other work: nothing else can be
                # running, since the person is mid-utterance, but the lock is
                # what makes that a fact rather than an assumption.
                pcm = S._pcm(buf)
                buf.clear()
                held = None
                if pcm.size >= S.STT_RATE * 0.3:
                    async with S._gpu_lock:
                        held = await asyncio.to_thread(S._transcribe, pcm)
            elif kind == "cancel":
                # The speaker paused and carried on, so what arrived early was
                # half a sentence. Nothing to say about it; the full utterance
                # will arrive by the ordinary road.
                held = None
            elif kind == "end":
                if event.get("early"):
                    # Confirming a speculation, so no audio came with this and
                    # none is wanted: the confirmed utterance differs from the
                    # one already transcribed only by trailing silence.
                    if held is None:
                        # Cancelled in between, or this connection never saw the
                        # speculation at all. Ask for the audio rather than
                        # answering an utterance nobody made.
                        await ws.send_json({"type": "resend"})
                        continue
                    heard, held = held, None
                    await _run_turn(ws, history, tools, None, heard=heard)
                    continue
                pcm = S._pcm(buf)
                buf.clear()
                held = None
                # Under ~0.3s is a door slam or a cough, not speech; Whisper
                # hallucinates fluent sentences out of clips that short.
                if pcm.size < S.STT_RATE * 0.3:
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

