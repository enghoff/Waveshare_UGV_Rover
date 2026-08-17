"""The measurement itself, run on the rented card.

Audio in, tool call out -- and the same phrases typed, in the same process,
against the same weights, so that the drop from one to the other is a property of
the modality rather than of the model. That pairing is the whole reason this file
exists rather than a comparison against the 66/90 already in the repository:
that number was taken on `Qwen3-VL-4B`, and a spoken score from a different model
measured against it would confound the two changes we care least about telling
apart.

Everything it needs travels with it -- the corpus, the sniffer, and a frozen copy
of the rover's ten schemas and system prompt, rendered on the workstation from
[rover_daemon.py] and [voice_chat/server.py] so that the rented machine needs
neither repository nor a serial port.

    python runner.py --probe                    # one case, loudly, to check the API
    python runner.py --arm text  --out text.jsonl
    python runner.py --arm audio --out audio.jsonl

Results are one JSON object per attempt, appended as they happen. A run that dies
halfway is still worth what it collected, which matters when the machine
underneath it is rented by the hour.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

from corpus import ALL, PRIMARY, SAMPLES, Case, judge  # noqa: E402
from sniff import Sniffer  # noqa: E402

FROZEN = HERE / "prompt.json"


def frozen() -> dict:
    """The schemas and system prompt as they were on the workstation."""
    return json.loads(FROZEN.read_text(encoding="utf-8"))


class MiniCPM:
    """MiniCPM-o 4.5 through its own `chat` entry point.

    Its multimodal path does not take an OpenAI-style `tools` argument, so the
    tools go in as the system turn instead -- rendered into exactly the `# Tools`
    block its chat template would have produced, since the template it inherits
    from Qwen is the same one. The model therefore sees the string it would have
    seen either way.
    """

    def __init__(self, path: str, *, duplex: bool = False):
        import torch
        from transformers import AutoModel, AutoTokenizer

        self.torch = torch
        self.tokenizer = AutoTokenizer.from_pretrained(path, trust_remote_code=True)
        self.model = AutoModel.from_pretrained(
            path,
            trust_remote_code=True,
            attn_implementation="sdpa",
            torch_dtype=torch.bfloat16,
            init_vision=True,
            init_audio=True,
            init_tts=duplex,
        ).eval().cuda()
        if duplex:
            self.model.init_tts()

        # Which knobs this checkpoint's remote code actually exposes. Guessing
        # costs a model load to find out, and a model load is four minutes of a
        # rented card, so ask once and adapt.
        import inspect

        self.accepts = set(inspect.signature(self.model.chat).parameters)

    def signature(self) -> str:
        import inspect

        return str(inspect.signature(self.model.chat))

    def _system(self, msgs: list, system: str) -> dict:
        """Put the system turn wherever this checkpoint will accept it.

        `system_prompt=` is the documented way and the one that lands in the
        right place in the template. If it is not there, a leading system message
        is the next best thing, and prepending the text to the user turn is the
        last resort -- worse, because the tools then arrive after the audio
        rather than before it.
        """
        if "system_prompt" in self.accepts:
            return {"system_prompt": system}
        msgs.insert(0, {"role": "system", "content": [system]})
        return {}

    def generate(self, case: Case, audio_path: Path | None, system: str,
                 temperature: float, max_new_tokens: int) -> tuple[str, Sniffer, float]:
        content: list = []
        if audio_path is not None:
            import librosa

            samples, _ = librosa.load(str(audio_path), sr=16000, mono=True)
            content.append(samples)
        else:
            content.append(case.text)

        msgs = [{"role": "user", "content": content}]
        started = time.monotonic()
        sniffer = Sniffer()

        extra = self._system(msgs, system)
        if "stream" in self.accepts:
            extra["stream"] = True
        if "max_new_tokens" in self.accepts:
            extra["max_new_tokens"] = max_new_tokens
        # 4.5 spells it `do_sample`, the way `generate` does; 2.6 spelled it
        # `sampling`. Anything unrecognised here lands in **kwargs and is passed
        # to `generate` unchallenged, so getting the name wrong would quietly
        # sample at the default temperature rather than at ours.
        extra["do_sample" if "do_sample" in self.accepts else "sampling"] = temperature > 0

        stream = self.model.chat(
            msgs=msgs,
            tokenizer=self.tokenizer,
            temperature=max(temperature, 0.01),
            **extra,
        )
        if isinstance(stream, str):  # a checkpoint without streaming
            stream = [stream]
        raw = []
        for piece in stream:
            piece = piece if isinstance(piece, str) else getattr(piece, "text", str(piece))
            raw.append(piece)
            sniffer.feed(piece)
        sniffer.flush()
        return "".join(raw), sniffer, time.monotonic() - started


class Qwen3Omni:
    """Qwen3-Omni's Thinker, which is where a tool call is text before it is speech.

    Only the Thinker is loaded. The Talker turns that text into speech and cannot
    change which tool was chosen, so for this measurement it is weight we would be
    paying to hold.
    """

    def __init__(self, path: str, **_):
        import torch
        from transformers import AutoProcessor, Qwen3OmniMoeForConditionalGeneration

        self.torch = torch
        self.processor = AutoProcessor.from_pretrained(path)
        self.model = Qwen3OmniMoeForConditionalGeneration.from_pretrained(
            path, dtype="auto", device_map="auto", attn_implementation="sdpa",
        ).eval()
        self.model.disable_talker()

        # Recent transformers routes mixture-of-experts through `torch._grouped_mm`,
        # which is Hopper-only -- on anything else it raises "only supported on CUDA
        # devices with compute capability = 9.0" after the weights are already on the
        # card. Falling back to the plain per-expert loop is slower and works
        # everywhere, which is the right trade for a benchmark rented by the hour.
        for module in self.model.modules():
            config = getattr(module, "config", None)
            if config is not None and hasattr(config, "_experts_implementation"):
                config._experts_implementation = "eager"

    def signature(self) -> str:
        return "Qwen3OmniMoeForConditionalGeneration.generate"

    def generate(self, case: Case, audio_path: Path | None, system: str,
                 temperature: float, max_new_tokens: int) -> tuple[str, Sniffer, float]:
        if audio_path is not None:
            user = [{"type": "audio", "audio": str(audio_path)}]
        else:
            user = [{"type": "text", "text": case.text}]
        messages = [
            {"role": "system", "content": [{"type": "text", "text": system}]},
            {"role": "user", "content": user},
        ]

        text = self.processor.apply_chat_template(
            messages, add_generation_prompt=True, tokenize=False,
        )
        import librosa

        audios = None
        if audio_path is not None:
            audios = [librosa.load(str(audio_path), sr=16000, mono=True)[0]]
        inputs = self.processor(
            text=text, audio=audios, return_tensors="pt", padding=True,
        ).to(self.model.device)

        started = time.monotonic()
        with self.torch.no_grad():
            out = self.model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                do_sample=temperature > 0,
                temperature=max(temperature, 0.01),
                return_audio=False,
            )
        # With the Talker disabled this is a tensor of ids, but the same call
        # returns (text_ids, waveform) when it is not, and the difference is a
        # runtime detail rather than something worth asserting about.
        if isinstance(out, (tuple, list)):
            out = out[0]
        reply = self.processor.batch_decode(
            out[:, inputs["input_ids"].shape[1]:], skip_special_tokens=False,
        )[0]
        # Not a stream, so the timing fields stay empty and the sniffer is used
        # only for its parsing. The duplex question is MiniCPM-o's to answer.
        sniffer = Sniffer()
        sniffer.feed(reply)
        sniffer.flush()
        return reply, sniffer, time.monotonic() - started


BACKENDS = {"minicpm": MiniCPM, "qwen3omni": Qwen3Omni}


def run(backend, cases: list[Case], arm: str, samples: int, temperature: float,
        max_new_tokens: int, audio_dir: Path, out: Path) -> dict:
    system = frozen()["system"]
    tally: dict[str, int] = {}

    with out.open("a", encoding="utf-8") as sink:
        for case in cases:
            audio = audio_dir / f"{case.key}.wav" if arm == "audio" else None
            if audio is not None and not audio.exists():
                raise SystemExit(f"no audio for {case.text!r} at {audio}")
            for sample in range(samples):
                reply, sniffer, seconds = backend.generate(
                    case, audio, system, temperature, max_new_tokens,
                )
                calls = sniffer.calls
                passed, why = judge(case, calls)
                tally[case.key] = tally.get(case.key, 0) + int(passed)
                record = {
                    "arm": arm,
                    "case": case.key,
                    "text": case.text,
                    "want": case.want,
                    "sample": sample,
                    "passed": passed,
                    "why": why,
                    "calls": calls,
                    "prose": sniffer.prose.strip(),
                    "raw": reply,
                    "seconds": round(seconds, 2),
                    "marker_at_char": sniffer.marker_at_char,
                    "marker_at_chunk": sniffer.marker_at_chunk,
                    "marker_at_time": sniffer.marker_at_time,
                }
                sink.write(json.dumps(record) + "\n")
                sink.flush()
            print(f"{case.text:<42} {tally[case.key]}/{samples}  {why}", flush=True)
    return tally


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="minicpm")
    parser.add_argument("--path", default="openbmb/MiniCPM-o-4_5")
    parser.add_argument("--arm", choices=["text", "audio"], default="text")
    parser.add_argument("--cases", choices=["primary", "all"], default="primary")
    parser.add_argument("--samples", type=int, default=SAMPLES)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--max-new-tokens", type=int, default=256)
    parser.add_argument("--audio-dir", default=str(HERE / "audio" / "zira"))
    parser.add_argument("--out", default="results.jsonl")
    parser.add_argument("--probe", action="store_true", help="one case, loudly")
    args = parser.parse_args()

    print(f"loading {args.path} ...", flush=True)
    started = time.monotonic()
    backend = BACKENDS[args.backend](args.path)
    print(f"loaded in {time.monotonic() - started:.0f}s; chat{backend.signature()}", flush=True)

    cases = PRIMARY if args.cases == "primary" else ALL
    if args.probe:
        cases, args.samples = cases[:1], 1

    tally = run(
        backend, cases, args.arm, args.samples, args.temperature,
        args.max_new_tokens, Path(args.audio_dir), Path(args.out),
    )
    total = sum(tally.values())
    print(f"\n{args.backend} {args.arm}: {total}/{len(cases) * args.samples}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
