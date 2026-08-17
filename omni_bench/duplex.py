"""Does the tool call arrive before the model starts saying it?

The second half of step 0, and the one nobody has published an answer to. In a
duplex model the reply is text and speech at once, and a tool call is text -- so
the question is not whether `<tool_call>` appears in the stream but whether it
appears with enough margin to be intercepted before a speech decoder reads it out
loud, brace by brace. `_ToolSniffer` in the deployed service has a sentence
splitter standing between it and Kokoro; a Talker generating from the same stream
may not leave that much room.

What this measures, per attempt:

  * when the marker appeared, in seconds from the first token
  * when the first audio arrived, on the same clock
  * whether any audio was produced *after* the marker, which is the failure --
    audio generated from text that was a tool call is a rover reading JSON aloud

The shape of the stream is not documented, so the first attempt dumps what it
actually yields rather than assuming. That dump is worth as much as the numbers.

    python duplex.py --path /workspace/MiniCPM-o-4_5 --cases 4
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

from corpus import PRIMARY  # noqa: E402
from sniff import Sniffer  # noqa: E402


def describe(item) -> dict:
    """What one stream item is, without assuming what it is."""
    if isinstance(item, str):
        return {"kind": "str", "text": item}
    fields = {}
    for name in ("text", "audio", "audio_wav", "sampling_rate"):
        if hasattr(item, name):
            value = getattr(item, name)
            if hasattr(value, "shape"):
                fields[name] = f"array{tuple(value.shape)}"
            elif isinstance(value, (str, int, float)) or value is None:
                fields[name] = value
            else:
                fields[name] = type(value).__name__
    if isinstance(item, dict):
        for key, value in item.items():
            fields[key] = f"array{tuple(value.shape)}" if hasattr(value, "shape") else str(value)[:40]
    return {"kind": type(item).__name__, **fields}


def text_of(item) -> str:
    if isinstance(item, str):
        return item
    if isinstance(item, dict):
        return item.get("text") or ""
    return getattr(item, "text", "") or ""


def audio_of(item):
    if isinstance(item, dict):
        return item.get("audio_wav", item.get("audio"))
    return getattr(item, "audio_wav", None) or getattr(item, "audio", None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", default="/workspace/MiniCPM-o-4_5")
    parser.add_argument("--cases", type=int, default=4)
    parser.add_argument("--audio-dir", default=str(HERE / "audio" / "zira"))
    parser.add_argument("--out", default="duplex.jsonl")
    parser.add_argument("--dump", type=int, default=6, help="stream items to describe on the first case")
    parser.add_argument("--speak", action="store_true",
                        help="the other path: generate the reply in full, then synthesise it")
    args = parser.parse_args()

    import librosa
    import torch
    from transformers import AutoModel, AutoTokenizer

    system = json.loads((HERE / "prompt.json").read_text(encoding="utf-8"))["system"]
    tokenizer = AutoTokenizer.from_pretrained(args.path, trust_remote_code=True)
    model = AutoModel.from_pretrained(
        args.path, trust_remote_code=True, attn_implementation="sdpa",
        torch_dtype=torch.bfloat16, init_vision=True, init_audio=True, init_tts=True,
    ).eval().cuda()
    model.init_tts()
    print("tts ready", flush=True)

    # Cases that should call, and the one that should not: a model that speaks
    # its refusal is the control for a model that speaks its tool call.
    chosen = [c for c in PRIMARY if c.want][: args.cases - 1] + [PRIMARY[-1]]

    if args.speak:
        # The other path through the same `chat`: no streaming, speech
        # synthesised from the finished text. Whether a WAV appears at all, and
        # how long it is, says whether the Talker would have read the call out
        # loud -- which is the failure the sniffer exists to prevent.
        import soundfile as sf

        for case in chosen:
            samples, _ = librosa.load(str(Path(args.audio_dir) / f"{case.key}.wav"), sr=16000, mono=True)
            msgs = [{"role": "system", "content": [system]},
                    {"role": "user", "content": [samples]}]
            out_wav = f"/workspace/spoken-{case.key}.wav"
            Path(out_wav).unlink(missing_ok=True)
            started = time.monotonic()
            answer = model.chat(
                msgs=msgs, tokenizer=tokenizer,
                use_tts_template=True, generate_audio=True, output_audio_path=out_wav,
                omni_mode=True, do_sample=True, temperature=0.2, max_new_tokens=256,
            )
            seconds = time.monotonic() - started
            spoken = "none"
            if Path(out_wav).exists():
                wave, rate = sf.read(out_wav)
                spoken = f"{len(wave) / rate:.2f}s"
            sniffer = Sniffer()
            sniffer.feed(answer if isinstance(answer, str) else str(answer))
            sniffer.flush()
            print(f"{case.text:<40} reply={str(answer)[:70]!r} calls={[c['name'] for c in sniffer.calls]} "
                  f"wav={spoken} in {seconds:.1f}s", flush=True)
        return 0

    with open(args.out, "a", encoding="utf-8") as sink:
        for index, case in enumerate(chosen):
            samples, _ = librosa.load(str(Path(args.audio_dir) / f"{case.key}.wav"), sr=16000, mono=True)
            msgs = [{"role": "system", "content": [system]},
                    {"role": "user", "content": [samples]}]

            sniffer = Sniffer()
            started = time.monotonic()
            first_audio = None
            audio_after_marker = 0
            audio_chunks = 0
            shapes = []

            stream = model.chat(
                msgs=msgs, tokenizer=tokenizer, stream=True,
                use_tts_template=True, generate_audio=True, omni_mode=True,
                do_sample=True, temperature=0.2, max_new_tokens=256,
            )
            for item in stream:
                now = time.monotonic() - started
                if index == 0 and len(shapes) < args.dump:
                    shapes.append({"at": round(now, 3), **describe(item)})
                piece = text_of(item)
                if piece:
                    sniffer.feed(piece)
                wave = audio_of(item)
                if wave is not None and getattr(wave, "size", len(wave) if hasattr(wave, "__len__") else 0):
                    audio_chunks += 1
                    if first_audio is None:
                        first_audio = now
                    if sniffer.marker_at_time is not None:
                        audio_after_marker += 1
            sniffer.flush()

            record = {
                "case": case.key,
                "text": case.text,
                "want": case.want,
                "calls": sniffer.calls,
                "prose": sniffer.prose.strip(),
                "marker_at_time": sniffer.marker_at_time,
                "marker_at_char": sniffer.marker_at_char,
                "first_audio_at": first_audio,
                "audio_chunks": audio_chunks,
                "audio_chunks_after_marker": audio_after_marker,
                "stream_shape": shapes,
            }
            sink.write(json.dumps(record) + "\n")
            sink.flush()

            if shapes:
                print("stream items:", flush=True)
                for shape in shapes:
                    print("   ", shape, flush=True)
            margin = (
                f"{first_audio - sniffer.marker_at_time:+.2f}s"
                if first_audio is not None and sniffer.marker_at_time is not None else "n/a"
            )
            print(
                f"{case.text:<40} calls={[c['name'] for c in sniffer.calls]} "
                f"marker={sniffer.marker_at_time} audio_at={first_audio} "
                f"chunks={audio_chunks} after_marker={audio_after_marker} margin={margin}",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
