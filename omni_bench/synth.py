"""Turning the fifteen phrases into speech, so the model can be asked out loud.

The whole point of step 0 is that every tool-calling number this repository holds
was measured by *typing* at a model that will be *spoken* to, so the corpus has to
exist as audio before anything can be learned. This module makes that audio.

It uses the speech voices Windows already has, through `System.Speech`, for one
reason: they need no account, no download and no key, so the harness can be built
and debugged to completion before a card is rented. They are also, audibly, worse
than a person -- and that is the right direction to be wrong in. A model that
calls the right tool on a synthetic voice may still fail on a real one in a room;
a model that fails here was never going to work. Both are upper bounds, which is
what the decision rule in [docs/omni-build.md] already assumes.

One voice makes the headline number, because mixing speakers into the six samples
of a cell would confound speaker variation with the model's own sampling. The
other voices are there to be run as a separate, clearly-labelled robustness pass.

    python omni_bench/synth.py            # the default voice, every case
    python omni_bench/synth.py --all-voices
"""

from __future__ import annotations

import json
import subprocess
import wave
from pathlib import Path

from corpus import ALL, Case

HERE = Path(__file__).resolve().parent
AUDIO = HERE / "runs" / "audio"

# Three voices ship with Windows 11. Zira is the default because the deployed
# rover is spoken to in American-accented English more often than not, and
# because a single voice for the headline keeps the comparison honest.
DEFAULT_VOICE = "Microsoft Zira Desktop"
VOICES = ["Microsoft Zira Desktop", "Microsoft David Desktop", "Microsoft Hazel Desktop"]

# 16 kHz mono is what every one of these models resamples to anyway -- Whisper in
# MiniCPM-o, the audio tower in Qwen3-Omni -- so synthesising at the target rate
# avoids one resample and makes the files small enough to commit if we ever want
# to.
RATE = 16000

# The synthesiser is driven from one PowerShell process for the whole manifest.
# Starting powershell.exe costs about half a second, and per-phrase that would
# dominate the run.
SCRIPT = r"""
param([string]$Manifest)
Add-Type -AssemblyName System.Speech
$jobs = Get-Content -Raw -Encoding UTF8 $Manifest | ConvertFrom-Json
$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
foreach ($job in $jobs) {
    $fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
        %RATE%,
        [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
        [System.Speech.AudioFormat.AudioChannel]::Mono)
    $synth.SetOutputToWaveFile($job.path, $fmt)
    $synth.SelectVoice($job.voice)
    $synth.Speak($job.text)
    $synth.SetOutputToNull()
    Write-Output ("ok " + $job.path)
}
$synth.Dispose()
"""


def path_for(case: Case, voice: str) -> Path:
    """One file per (phrase, voice). Deterministic, so a re-run is free."""
    folder = voice.replace("Microsoft ", "").replace(" Desktop", "").lower()
    return AUDIO / folder / f"{case.key}.wav"


def synthesise(cases: list[Case], voices: list[str], force: bool = False) -> list[Path]:
    """Write a WAV for every (case, voice) that does not already have one."""
    jobs = []
    made = []
    for voice in voices:
        for case in cases:
            target = path_for(case, voice)
            made.append(target)
            if target.exists() and not force:
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            jobs.append({"path": str(target), "voice": voice, "text": case.text})

    if jobs:
        AUDIO.mkdir(parents=True, exist_ok=True)
        manifest = AUDIO / "manifest.json"
        manifest.write_text(json.dumps(jobs), encoding="utf-8")
        script = AUDIO / "synth.ps1"
        script.write_text(SCRIPT.replace("%RATE%", str(RATE)), encoding="utf-8")
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
             "-File", str(script), "-Manifest", str(manifest)],
            capture_output=True, text=True, timeout=600,
        )
        if result.returncode != 0:
            raise RuntimeError(f"speech synthesis failed:\n{result.stderr[:800]}")
    return made


def describe(path: Path) -> str:
    with wave.open(str(path)) as wav:
        seconds = wav.getnframes() / wav.getframerate()
        return f"{seconds:5.2f}s  {wav.getframerate()} Hz  {wav.getnchannels()}ch"


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all-voices", action="store_true", help="the robustness pass, not the headline")
    parser.add_argument("--force", action="store_true", help="re-synthesise files that exist")
    args = parser.parse_args()

    voices = VOICES if args.all_voices else [DEFAULT_VOICE]
    made = synthesise(ALL, voices, force=args.force)

    total = 0.0
    for path in made:
        with wave.open(str(path)) as wav:
            total += wav.getnframes() / wav.getframerate()
    print(f"{len(made)} files, {total:.1f}s of speech, under {AUDIO}")
    for path in made[:len(ALL)]:
        print(f"  {describe(path)}  {path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
