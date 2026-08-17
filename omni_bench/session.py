"""Bring a card up, put the harness on it, and start the long download.

Kept separate from `runner.py` because the two have different failure costs. This
part is slow and boring -- a pod appears, files copy, 20 GB arrives -- and none of
it should need a person. The measuring is the part worth watching.

    python session.py --up          # rent, upload, bootstrap; leaves it running
    python session.py --push        # re-upload the harness to a pod already up
    python session.py --exec "..."  # run something on it

The pod is left running on purpose, since the next step is another script; pair it
with `python pod.py --sentinel <minutes>` in the background so that an
interrupted session still ends with the meter stopped.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import pod

HERE = Path(__file__).resolve().parent
REMOTE = "/workspace/omni_bench"

# The remote end says things this console cannot spell -- progress bars, and a
# model that occasionally answers in Chinese. Losing a run's output to a codec is
# a silly way to spend a rented card.
for stream in (sys.stdout, sys.stderr):
    stream.reconfigure(encoding="utf-8", errors="replace")

# What the card needs: the harness, the frozen prompt, and the speech. Not the
# repository -- the whole point of freezing the schemas was that this machine
# never has to see it.
UPLOAD = ["corpus.py", "sniff.py", "runner.py", "bootstrap.sh"]


def current() -> dict:
    live = pod.running()
    if not live:
        raise SystemExit("no pod running. python session.py --up")
    return pod._request(f"{pod.REST}/pods/{live[0]['id']}")


def scp(sources: list[str], target: str, host: str, port: int) -> None:
    subprocess.run(
        ["scp", "-P", str(port), *pod.SSH_OPTS, "-r", *sources, f"root@{host}:{target}"],
        check=True,
    )


def push(pod_info: dict) -> None:
    host, port = pod.ssh_target(pod_info)
    pod.ssh(pod_info, f"mkdir -p {REMOTE}/audio")
    scp([str(HERE / name) for name in UPLOAD], REMOTE, host, port)
    scp([str(HERE / "runs" / "prompt.json")], REMOTE, host, port)
    scp([str(HERE / "runs" / "audio" / "zira")], f"{REMOTE}/audio", host, port)
    listing = pod.ssh(pod_info, f"ls {REMOTE} {REMOTE}/audio/zira | head -30")
    print(listing.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--up", action="store_true")
    parser.add_argument("--push", action="store_true")
    parser.add_argument("--exec", metavar="CMD")
    parser.add_argument("--gpu", default="L40S")
    parser.add_argument("--minutes", type=int, default=120)
    parser.add_argument("--disk", type=int, default=80)
    parser.add_argument("--cloud", default="COMMUNITY", choices=["COMMUNITY", "SECURE"])
    parser.add_argument("--model", default="openbmb/MiniCPM-o-4_5")
    parser.add_argument("--image", default="runpod/pytorch:1.1.0-rc.154-cu1281-torch280-ubuntu2204-cluster")
    args = parser.parse_args()

    if args.up:
        info = pod.up(args.gpu, minutes=args.minutes, image=args.image, disk_gb=args.disk, cloud=args.cloud)
        push(info)
        print("=== bootstrap", flush=True)
        result = pod.ssh(info, f"bash {REMOTE}/bootstrap.sh {args.model} 2>&1 | tee /workspace/bootstrap.log",
                         timeout=3600)
        print(result.stdout[-4000:] or result.stderr[-2000:])
        host, port = pod.ssh_target(info)
        print(json.dumps({"id": info["id"], "host": host, "port": port}))
        return result.returncode

    info = current()
    if args.push:
        push(info)
        return 0
    if args.exec:
        result = pod.ssh(info, args.exec, timeout=3600)
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        return result.returncode

    host, port = pod.ssh_target(info)
    print(f"{info['id']}  root@{host}:{port}  ${info.get('costPerHr')}/hr")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
