"""Does the RunPod key work, is anything already running, and what does an L40S cost?

Run this first, before anything is rented. It answers three questions and spends
nothing:

  1. Is the key valid? An invalid key fails here in a second rather than halfway
     through a pod creation, where the failure looks like a quota problem.
  2. Is a pod already running? A pod nobody remembers is the expensive failure
     mode of this whole exercise -- the meter runs whether or not a measurement
     is being taken -- so every session starts by looking.
  3. What is on offer, and at what price? `omni-build.md` quotes $0.79/hr for a
     community-cloud L40S as of 2026-08-16. Advertised rates move, so this reads
     the live number rather than trusting the document.

    python omni_bench/check_runpod.py
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request

from credentials import MissingCredential, describe, runpod_key

GRAPHQL = "https://api.runpod.io/graphql"

# Cloudflare sits in front of the API and rejects the default `Python-urllib/x.y`
# signature outright -- HTTP 403, body `error code: 1010`, which reads exactly
# like a rejected key and is not one. Any ordinary agent string is accepted.
USER_AGENT = "ugv-omni-bench/1.0"

# The cards omni-build.md costs out, in the order it lists them. Anything else
# the account can see is reported too, but these are the ones with a decision
# attached.
OF_INTEREST = ("L40S", "RTX 4090", "RTX 5090", "RTX PRO 6000", "H200")

GPU_QUERY = """
query GpuTypes {
  gpuTypes {
    id
    displayName
    memoryInGb
    secureCloud
    communityCloud
    lowestPrice(input: {gpuCount: 1}) {
      minimumBidPrice
      uninterruptablePrice
    }
  }
}
"""

POD_QUERY = """
query Pods {
  myself {
    pods {
      id
      name
      desiredStatus
      costPerHr
      machine { gpuDisplayName }
    }
  }
}
"""


def graphql(key: str, query: str) -> dict:
    """One GraphQL call. Errors come back in the body with HTTP 200, so check both."""
    body = json.dumps({"query": query}).encode()
    req = urllib.request.Request(
        f"{GRAPHQL}?api_key={key}",
        data=body,
        headers={"Content-Type": "application/json", "User-Agent": USER_AGENT},
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            payload = json.load(resp)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")[:200]
        raise SystemExit(f"RunPod refused the request: HTTP {exc.code} {detail}") from exc
    if payload.get("errors"):
        raise SystemExit(f"RunPod returned an error: {payload['errors']}")
    return payload["data"]


def main() -> int:
    try:
        key = runpod_key()
    except MissingCredential as exc:
        print(f"no credential: {exc}", file=sys.stderr)
        return 2
    print(f"key: {describe(key)}")

    pods = graphql(key, POD_QUERY)["myself"]["pods"] or []
    running = [p for p in pods if p.get("desiredStatus") == "RUNNING"]
    if running:
        total = sum(p.get("costPerHr") or 0 for p in running)
        print(f"\n*** {len(running)} pod(s) RUNNING, ${total:.2f}/hr ***")
        for pod in running:
            card = (pod.get("machine") or {}).get("gpuDisplayName", "?")
            print(f"    {pod['id']}  {pod.get('name', '')}  {card}  ${pod.get('costPerHr')}/hr")
    else:
        print(f"\npods: {len(pods)} known, none running")

    print("\ncard                      VRAM   community   secure")
    for gpu in graphql(key, GPU_QUERY)["gpuTypes"]:
        name = gpu.get("displayName") or gpu["id"]
        if not any(want.lower() in name.lower() for want in OF_INTEREST):
            continue
        price = gpu.get("lowestPrice") or {}
        rate = price.get("uninterruptablePrice")
        rate = f"${rate:.2f}" if isinstance(rate, (int, float)) else "-"
        print(
            f"{name:<24}  {gpu.get('memoryInGb', '?'):>3} GB   "
            f"{'yes' if gpu.get('communityCloud') else 'no':<9}   "
            f"{'yes' if gpu.get('secureCloud') else 'no':<6}   lowest {rate}/hr"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
