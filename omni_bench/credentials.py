"""Where the harness finds its keys, and nowhere else.

One rule: a secret is read from a file under `secrets/`, which `.gitignore`
excludes wholesale, or from the environment if the environment has one. The
environment wins, so a shell can override a file for one run without editing it.

Nothing here ever prints a key. `describe` exists so that a run log can record
*which* credential was used and whether it looked plausible, without the key
itself ending up in a transcript that gets pasted into a document later.
"""

from __future__ import annotations

import os
from pathlib import Path

# The repo root, from this file, so the path does not depend on the working
# directory a run was started from.
ROOT = Path(__file__).resolve().parent.parent
SECRETS = ROOT / "secrets"


class MissingCredential(RuntimeError):
    """Raised with the exact path to write, rather than a generic 'not configured'."""


def read(name: str, env: str) -> str:
    """The secret called `name`, from $`env` or from `secrets/<name>`.

    Trailing newlines are stripped, because a key pasted into an editor almost
    always acquires one and a key with a newline in it fails as an opaque 401.
    """
    from_env = os.environ.get(env, "").strip()
    if from_env:
        return from_env

    path = SECRETS / name
    if not path.exists():
        raise MissingCredential(
            f"no {env} in the environment and no file at {path}. "
            f"Write the key into that file as a single line, or set {env}."
        )
    key = path.read_text(encoding="utf-8").strip()
    if not key:
        raise MissingCredential(f"{path} is empty.")
    return key


def runpod_key() -> str:
    """The RunPod API key: $RUNPOD_API_KEY, or `secrets/runpod.key`."""
    return read("runpod.key", "RUNPOD_API_KEY")


def describe(key: str) -> str:
    """A safe-to-log fingerprint: length and last four, never the key."""
    return f"{len(key)} chars, ending {key[-4:]}" if len(key) > 8 else "too short to be a key"
