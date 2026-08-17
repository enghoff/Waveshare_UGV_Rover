"""The same fifteen phrases, put to Alibaba's hosted omni models.

This is the upper bound [omni-build.md](../docs/omni-build.md) wanted and briefly
could not have: if the best omni model reachable through any API cannot call the
rover's ten schemas when spoken to, no local 9B model was ever going to. It is
worth saying plainly, since the premise of the whole design is that the
conversation stays on a network we control, that running this sends the fifteen
synthesised phrases to Alibaba and nothing else -- no pictures, no telemetry, and
no real conversation.

One difference from the local runs, deliberately. These models take tools through
the OpenAI `tools` parameter, which is their intended interface, so that is what
they are given; the local models were handed the same schemas rendered into the
system turn because their multimodal entry points accept nothing else. Both end up
as the same `# Tools` block in the same template. `--tools prompt` runs it the
local way if the difference is ever worth measuring.

    python cloud.py --model qwen3.5-omni-flash --arm audio
"""

from __future__ import annotations

import argparse
import base64
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

from corpus import ALL, PRIMARY, SAMPLES, Case, judge
from credentials import read
from schemas import full_system, system_prompt, tools
from sniff import Sniffer

HERE = Path(__file__).resolve().parent
BASE = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"


def key() -> str:
    return read("alibaba.key", "DASHSCOPE_API_KEY")


def post(path: str, body: dict, timeout: int = 120) -> dict:
    request = urllib.request.Request(
        f"{BASE}{path}",
        data=json.dumps(body).encode(),
        headers={
            "Authorization": f"Bearer {key()}",
            "Content-Type": "application/json",
            "User-Agent": "ugv-omni-bench/1.0",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f"HTTP {exc.code}: {exc.read().decode(errors='replace')[:300]}") from exc


def stream(path: str, body: dict, timeout: int = 120, attempts: int = 6):
    """Server-sent events, because the omni models refuse anything else.

    Retries on 429. Six samples a cell against 20 cases is 120 requests as fast
    as they will go, which is well inside the free tier's patience for one model
    and not for another -- and losing an hour's run to a rate limit two thirds of
    the way through is a silly way to fail.
    """
    for attempt_number in range(attempts):
        request = urllib.request.Request(
            f"{BASE}{path}",
            data=json.dumps({**body, "stream": True}).encode(),
            headers={
                "Authorization": f"Bearer {key()}",
                "Content-Type": "application/json",
                "User-Agent": "ugv-omni-bench/1.0",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                for raw in response:
                    line = raw.decode("utf-8", "replace").strip()
                    if not line.startswith("data:"):
                        continue
                    payload = line[5:].strip()
                    if payload == "[DONE]":
                        return
                    yield json.loads(payload)
            return
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode(errors="replace")[:300]
            if exc.code == 429 and attempt_number < attempts - 1:
                wait = 5 * (attempt_number + 1)
                print(f"    rate limited, waiting {wait}s", flush=True)
                time.sleep(wait)
                continue
            raise RuntimeError(f"HTTP {exc.code}: {detail}") from exc


def audio_part(path: Path) -> dict:
    encoded = base64.b64encode(path.read_bytes()).decode()
    return {"type": "input_audio",
            "input_audio": {"data": f"data:audio/wav;base64,{encoded}", "format": "wav"}}


def attempt(model: str, case: Case, audio: Path | None, native: bool,
            temperature: float, max_tokens: int) -> tuple[list[dict], str, str, float]:
    content = [audio_part(audio)] if audio else [{"type": "text", "text": case.text}]
    body: dict = {
        "model": model,
        "messages": [
            {"role": "system", "content": [
                {"type": "text", "text": system_prompt() if native else full_system()}]},
            {"role": "user", "content": content},
        ],
        "modalities": ["text"],
        "temperature": temperature,
        "max_tokens": max_tokens,
    }
    if native:
        body["tools"] = tools()

    sniffer = Sniffer()
    native_calls: list[dict] = []
    partial: dict[int, dict] = {}
    started = time.monotonic()

    for chunk in stream("/chat/completions", body):
        for choice in chunk.get("choices") or []:
            delta = choice.get("delta") or {}
            if piece := delta.get("content"):
                sniffer.feed(piece if isinstance(piece, str) else "")
            for call in delta.get("tool_calls") or []:
                slot = partial.setdefault(call.get("index", 0), {"name": "", "arguments": ""})
                function = call.get("function") or {}
                slot["name"] += function.get("name") or ""
                slot["arguments"] += function.get("arguments") or ""
    sniffer.flush()

    for slot in partial.values():
        try:
            arguments = json.loads(slot["arguments"] or "{}")
        except ValueError:
            arguments = {}
        native_calls.append({"name": slot["name"], "arguments": arguments})

    # A model given `tools` may still answer with the marker in its text -- the
    # two paths are the same template underneath -- so both are collected and the
    # native one wins when both are present.
    calls = native_calls or sniffer.calls
    return calls, sniffer.prose.strip(), "native" if native_calls else "text", time.monotonic() - started


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="qwen3.5-omni-flash")
    parser.add_argument("--arm", choices=["text", "audio"], default="audio")
    parser.add_argument("--cases", choices=["primary", "all"], default="all")
    parser.add_argument("--samples", type=int, default=SAMPLES)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--max-tokens", type=int, default=256)
    parser.add_argument("--tools", choices=["native", "prompt"], default="native")
    parser.add_argument("--audio-dir", default=str(HERE / "runs" / "audio" / "zira"))
    parser.add_argument("--out", default=None)
    parser.add_argument("--probe", action="store_true")
    args = parser.parse_args()

    cases = PRIMARY if args.cases == "primary" else ALL
    samples = args.samples
    if args.probe:
        cases, samples = cases[:1], 1

    out = Path(args.out or HERE / "runs" / f"{args.model}-{args.arm}.jsonl")
    native = args.tools == "native"
    total = 0

    with out.open("a", encoding="utf-8") as sink:
        for case in cases:
            audio = Path(args.audio_dir) / f"{case.key}.wav" if args.arm == "audio" else None
            passes = 0
            for sample in range(samples):
                calls, prose, how, seconds = attempt(
                    args.model, case, audio, native, args.temperature, args.max_tokens,
                )
                passed, why = judge(case, calls)
                passes += passed
                sink.write(json.dumps({
                    "arm": args.arm, "model": args.model, "case": case.key, "text": case.text,
                    "want": case.want, "sample": sample, "passed": passed, "why": why,
                    "calls": calls, "prose": prose, "raw": prose, "via": how,
                    "seconds": round(seconds, 2),
                    "marker_at_char": None, "marker_at_chunk": None, "marker_at_time": None,
                }) + "\n")
                sink.flush()
            total += passes
            print(f"{case.text:<42} {passes}/{samples}  {why}", flush=True)

    print(f"\n{args.model} {args.arm}: {total}/{len(cases) * samples}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
