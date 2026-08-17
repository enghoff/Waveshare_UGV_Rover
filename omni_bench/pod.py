"""Renting a GPU, and giving it back.

The measurement this harness exists for costs about five dollars of inference.
A pod that outlives the measurement costs more than that by lunchtime, so this
module is written around one idea: **the pod is terminated in a `finally`, and a
watchdog terminates it again if the `finally` never runs.**

    with rented("L40S", minutes=20) as pod:
        run(ssh_command(pod, "nvidia-smi"))
    # terminated here, whatever happened in between

Three guards, because the failure is silent and the meter is not:

  * `rented` refuses to start if a pod is already running. An orphan is the thing
    that empties the account, and the only moment anybody reliably looks for one
    is when they are about to make another.
  * a watchdog thread terminates the pod once `minutes` have elapsed, whether or
    not the body has finished, so a hung SSH costs a bounded amount.
  * the pod id is written to `runs/active_pod.json` before the pod is created and
    cleared after it dies, so a later session -- or `--reap` -- can find one that
    this process was killed too abruptly to clean up.

Terminating twice is harmless: the second call answers "pod not found".
"""

from __future__ import annotations

import atexit
import json
import subprocess
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from pathlib import Path

from credentials import runpod_key

REST = "https://rest.runpod.io/v1"
GRAPHQL = "https://api.runpod.io/graphql"

# Cloudflare fronts both APIs and rejects the default `Python-urllib/x.y`
# signature with HTTP 403 and a body of `error code: 1010`, which reads exactly
# like a rejected key and is not one.
USER_AGENT = "ugv-omni-bench/1.0"

HERE = Path(__file__).resolve().parent
RUNS = HERE / "runs"
ACTIVE = RUNS / "active_pod.json"
KNOWN_HOSTS = RUNS / "known_hosts"

# A pod is a fresh machine every time, so its host key is always new and always
# unknown. Trusting it on first sight is the only workable setting here; the
# separate file keeps that decision out of the workstation's real known_hosts.
SSH_OPTS = [
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", f"UserKnownHostsFile={KNOWN_HOSTS}",
    "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=15",
]

# A typo in a GPU name that resolved to an H200 would be a $3.59/hr typo, so the
# ceiling is checked against the quoted price before the pod is created rather
# than after. It sits at the RTX Pro 6000's $1.69 because Qwen3-Omni's 70 GB of
# bf16 weights do not fit on the L40S the rest of this work uses, and renting 96
# GB for an hour turned out to be both cheaper and far less risky than an hour
# spent teaching vLLM to quantise a mixture-of-experts to fp8 on the fly.
MAX_HOURLY = 1.75


def _request(url: str, method: str = "GET", body: dict | None = None) -> object:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={
            "Authorization": f"Bearer {runpod_key()}",
            "Content-Type": "application/json",
            "User-Agent": USER_AGENT,
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            raw = resp.read()
            return json.loads(raw) if raw.strip() else None
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")[:300]
        raise RuntimeError(f"{method} {url} -> HTTP {exc.code}: {detail}") from exc


def _graphql(query: str) -> dict:
    """Only for what REST v1 does not expose -- the GPU catalogue, in practice."""
    req = urllib.request.Request(
        f"{GRAPHQL}?api_key={runpod_key()}",
        data=json.dumps({"query": query}).encode(),
        headers={"Content-Type": "application/json", "User-Agent": USER_AGENT},
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = json.load(resp)
    if payload.get("errors"):
        raise RuntimeError(f"RunPod: {payload['errors']}")
    return payload["data"]


def gpu_types() -> list[dict]:
    """The catalogue, with the community-cloud on-demand price attached."""
    query = """
    query { gpuTypes { id displayName memoryInGb communityCloud
                       lowestPrice(input: {gpuCount: 1}) { uninterruptablePrice } } }
    """
    return _graphql(query)["gpuTypes"]


def resolve_gpu(fragment: str) -> tuple[str, float]:
    """A GPU id and its hourly price, from a fragment of its display name.

    Exact-match first: "RTX PRO 6000" is a prefix of "RTX PRO 6000 MaxQ", and
    picking a variant by luck is how a run ends up on a card it was not costed
    for.
    """
    catalogue = gpu_types()
    want = fragment.strip().lower()
    exact = [g for g in catalogue if (g.get("displayName") or "").lower() == want]
    hits = exact or [g for g in catalogue if want in (g.get("displayName") or "").lower()]
    if not hits:
        names = ", ".join(sorted(g.get("displayName") or g["id"] for g in catalogue))
        raise RuntimeError(f"no GPU matching {fragment!r}. Known: {names}")
    if len(hits) > 1:
        names = ", ".join(g.get("displayName") or g["id"] for g in hits)
        raise RuntimeError(f"{fragment!r} is ambiguous: {names}")
    gpu = hits[0]
    price = (gpu.get("lowestPrice") or {}).get("uninterruptablePrice") or 0.0
    return gpu["id"], float(price)


def pods() -> list[dict]:
    return _request(f"{REST}/pods") or []


def running() -> list[dict]:
    return [p for p in pods() if p.get("desiredStatus") == "RUNNING"]


def terminate(pod_id: str) -> None:
    try:
        _request(f"{REST}/pods/{pod_id}", method="DELETE")
        print(f"[pod] terminated {pod_id}")
    except RuntimeError as exc:
        # A pod that is already gone is the outcome we wanted, not a failure.
        if "404" in str(exc) or "not found" in str(exc).lower():
            print(f"[pod] {pod_id} was already gone")
        else:
            raise


def reap() -> int:
    """Terminate everything running, plus anything `active_pod.json` remembers.

    The file matters because a process killed between `create` and its `finally`
    leaves a pod that `running()` will show but nobody is watching.
    """
    victims = {p["id"] for p in running()}
    if ACTIVE.exists():
        try:
            victims.add(json.loads(ACTIVE.read_text())["id"])
        except Exception:
            pass
    for pod_id in victims:
        terminate(pod_id)
    ACTIVE.unlink(missing_ok=True)
    return len(victims)


def _remember(pod_id: str, note: str) -> None:
    RUNS.mkdir(exist_ok=True)
    ACTIVE.write_text(json.dumps({"id": pod_id, "note": note, "at": time.time()}))


def stop(pod_id: str) -> None:
    """Stop a pod without destroying it.

    Worth knowing what this does and does not keep. A stopped pod is not billed
    for its GPU or for its container disk -- and the container disk is *erased*,
    which is why anything meant to survive has to sit on a volume. The volume is
    billed while stopped, at twice the running rate: $0.20 per GB per month
    against $0.10. A 10 GB volume is therefore about two dollars a month to keep
    a finished piece of work reachable.
    """
    _request(f"{REST}/pods/{pod_id}/stop", method="POST")
    print(f"[pod] stopped {pod_id} (volume still billed, GPU not)")


def create(gpu: str, *, minutes: int, image: str, disk_gb: int = 40,
           name: str = "omni-bench", env: dict | None = None,
           cloud: str = "COMMUNITY", volume_gb: int = 0) -> dict:
    """Create one on-demand pod and wait until it is RUNNING with SSH mapped.

    `gpu` may be several names separated by commas. A card being listed is not
    the same as a card being free -- "There are no instances currently
    available" is an ordinary answer at any given minute -- so offering the
    variants of one card together (a Pro 6000 and its MaxQ and WK siblings are
    the same 96 GB) turns a hard failure into a slightly different receipt.
    """
    wanted = [resolve_gpu(name.strip()) for name in gpu.split(",")]
    dearest = max(price for _, price in wanted)
    if dearest > MAX_HOURLY:
        raise RuntimeError(f"{gpu} reaches ${dearest}/hr, over the ${MAX_HOURLY:.2f} ceiling")
    budget = dearest * minutes / 60
    print(f"[pod] {gpu} at up to ${dearest:.2f}/hr, {minutes} min -> ${budget:.2f} at most")

    body = {
        "name": name,
        "imageName": image,
        "gpuTypeIds": [gpu_id for gpu_id, _ in wanted],
        "gpuTypePriority": "availability",
        "gpuCount": 1,
        "cloudType": cloud,
        "computeType": "GPU",
        "containerDiskInGb": disk_gb,
        "volumeInGb": volume_gb,
        "ports": ["22/tcp"],
        "supportPublicIp": True,
        # RunPod's own images read this and write it into root's authorized_keys.
        "env": {"PUBLIC_KEY": _public_key(), **(env or {})},
    }
    pod = _request(f"{REST}/pods", method="POST", body=body)
    pod_id = pod["id"]
    _remember(pod_id, f"{gpu} {name}")
    print(f"[pod] created {pod_id}")
    return pod_id


def _public_key() -> str:
    return (Path.home() / ".ssh" / "id_ed25519.pub").read_text().strip()


def wait_ready(pod_id: str, timeout: int = 600) -> dict:
    """Block until the pod is RUNNING and answering SSH.

    RUNNING means the container was started, not that sshd has bound, and the two
    are minutes apart on a cold image pull. Waiting for the port rather than the
    status is the difference between a smoke test that works and one that fails
    on connection refused every time.
    """
    deadline = time.time() + timeout
    pod = {}
    while time.time() < deadline:
        pod = _request(f"{REST}/pods/{pod_id}") or {}
        host, port = ssh_target(pod)
        if pod.get("desiredStatus") == "RUNNING" and host and port:
            probe = subprocess.run(
                ["ssh", *SSH_OPTS, "-p", str(port), f"root@{host}", "true"],
                capture_output=True, text=True, encoding="utf-8", errors="replace",
            )
            if probe.returncode == 0:
                print(f"[pod] ssh up on {host}:{port}")
                return pod
        time.sleep(10)
    raise RuntimeError(f"pod {pod_id} was not reachable within {timeout}s (last: {pod.get('desiredStatus')})")


def ssh_target(pod: dict) -> tuple[str | None, int | None]:
    """The public host and mapped port for SSH, or (None, None) if not up yet."""
    mappings = pod.get("portMappings") or {}
    port = mappings.get("22")
    return pod.get("publicIp"), int(port) if port else None


def ssh(pod: dict, command: str, *, timeout: int = 600) -> subprocess.CompletedProcess:
    host, port = ssh_target(pod)
    return subprocess.run(
        ["ssh", *SSH_OPTS, "-p", str(port), f"root@{host}", command],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout,
    )


@contextmanager
def rented(gpu: str, *, minutes: int, image: str, disk_gb: int = 40,
           name: str = "omni-bench", env: dict | None = None):
    """A pod for the duration of the block, and gone afterwards."""
    if existing := running():
        ids = ", ".join(p["id"] for p in existing)
        raise RuntimeError(
            f"a pod is already running ({ids}). Terminate it first: "
            f"python omni_bench/pod.py --reap"
        )

    started = time.time()
    pod_id = create(gpu, minutes=minutes, image=image, disk_gb=disk_gb, name=name, env=env)

    # Belt and braces. The `finally` handles the ordinary path; the watchdog
    # handles a body that hangs, and `atexit` handles an exception on the way out
    # of the interpreter. All three converge on the same idempotent call.
    done = threading.Event()

    def watchdog():
        if not done.wait(minutes * 60):
            print(f"[pod] watchdog: {minutes} min elapsed, terminating {pod_id}")
            terminate(pod_id)

    threading.Thread(target=watchdog, daemon=True).start()
    atexit.register(terminate, pod_id)

    try:
        yield wait_ready(pod_id)
    finally:
        done.set()
        terminate(pod_id)
        ACTIVE.unlink(missing_ok=True)
        held = (time.time() - started) / 3600
        _, price = resolve_gpu(gpu)
        print(f"[pod] held {held * 60:.1f} min, about ${held * price:.2f}")


def up(gpu: str, *, minutes: int, image: str, disk_gb: int, name: str = "omni-bench",
       cloud: str = "COMMUNITY", volume_gb: int = 0) -> dict:
    """Create a pod and leave it running, for work that spans more than one script.

    `rented` is the right shape for a job that finishes on its own; a model that
    takes twenty minutes to download and then has to be poked at is not that job.
    The safety that `rented` gets from a `finally` has to come from somewhere
    else here, so the pod id goes to `active_pod.json` before creation and the
    caller is expected to start `--sentinel` alongside it.
    """
    if existing := running():
        raise RuntimeError(f"already running: {', '.join(p['id'] for p in existing)}")
    pod_id = create(gpu, minutes=minutes, image=image, disk_gb=disk_gb, name=name, cloud=cloud,
                    volume_gb=volume_gb)
    try:
        pod = wait_ready(pod_id)
    except Exception:
        # A pod that never became reachable is the worst of both worlds: billing,
        # and no way in to stop it doing so. Not every community machine hands out
        # a public IP as promptly as it hands out the GPU.
        print(f"[pod] {pod_id} never came up; terminating rather than leaving it billing")
        terminate(pod_id)
        raise
    host, port = ssh_target(pod)
    print(f"[pod] {pod_id} up at root@{host}:{port}")
    return pod


def sentinel(minutes: int) -> int:
    """Sleep, then terminate whatever is running. The dead man's handle.

    Run this in the background next to `--up`. It is the only thing standing
    between a session that gets interrupted and a card billing by the hour until
    somebody notices.
    """
    print(f"[sentinel] armed for {minutes} min")
    time.sleep(minutes * 60)
    print(f"[sentinel] {minutes} min elapsed")
    return reap()


def smoke(gpu: str = "L40S") -> int:
    """Prove the whole loop -- create, SSH in, terminate -- for a few cents.

    Worth doing before the first real run, because the thing most likely to be
    wrong is the SSH key on the account, and that failure only shows up once a
    pod exists. On the smallest RunPod base image so the pull is under a minute.
    """
    image = "runpod/base:1.1.0-rc.154-ubuntu2204"
    with rented(gpu, minutes=10, image=image, disk_gb=20, name="omni-smoke") as pod:
        host, port = ssh_target(pod)
        print(f"[smoke] {host}:{port}")
        for command in ("hostname", "nvidia-smi --query-gpu=name,memory.total --format=csv,noheader",
                        "nproc", "df -h /workspace | tail -1", "python3 -V"):
            result = ssh(pod, command)
            output = (result.stdout or result.stderr).strip().replace("\n", " | ")
            print(f"[smoke] {command:<52} {output}")
    return 0


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="what is running right now")
    parser.add_argument("--reap", action="store_true", help="terminate everything")
    parser.add_argument("--smoke", action="store_true", help="rent briefly, SSH in, give it back")
    parser.add_argument("--up", action="store_true", help="rent and leave running (pair with --sentinel)")
    parser.add_argument("--sentinel", type=int, metavar="MIN", help="reap everything after MIN minutes")
    parser.add_argument("--gpu", default="L40S", help="card for --smoke and --up")
    parser.add_argument("--minutes", type=int, default=90, help="budget for --up")
    parser.add_argument("--disk", type=int, default=80, help="container disk GB for --up")
    parser.add_argument("--image", default="runpod/pytorch:1.1.0-rc.154-cu1281-torch280-ubuntu2204-cluster")
    args = parser.parse_args()

    if args.reap:
        print(f"reaped {reap()} pod(s)")
        return 0
    if args.sentinel:
        return sentinel(args.sentinel)
    if args.smoke:
        return smoke(args.gpu)
    if args.up:
        pod = up(args.gpu, minutes=args.minutes, image=args.image, disk_gb=args.disk)
        host, port = ssh_target(pod)
        print(json.dumps({"id": pod["id"], "host": host, "port": port}))
        return 0

    live = running()
    if not live:
        print("nothing running")
        return 0
    for pod in live:
        print(f"{pod['id']}  {pod.get('name')}  ${pod.get('costPerHr')}/hr  {pod.get('desiredStatus')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
