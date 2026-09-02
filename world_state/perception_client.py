"""Asking the perception sidecar what is in a frame, and what comes back.

One small interface, a deterministic fake for tests, and a client that treats the
sidecar being absent as an ordinary answer rather than as an exception. The
caller is an inspection running inside the process that owns STOP, so nothing
here may raise at it. The shape is inherited from the language-model client this
replaced, which is worth knowing only because it is why a backend swap costs
nothing here.

**What comes back is a measurement, not a claim.** Each region is a box in
fractions of the frame and two vectors, and there is deliberately no name among
them: naming a region by the nearest phrase in a word list was measured to be
worth nothing, and what the semantic vector is genuinely good for is answering a
phrase somebody types. None of it says which lasting thing anything is; that
needs a bearing from a second place, which is `locate.py`'s business.

The vectors arrive base64-encoded and are kept as raw float32 bytes all the way
into the database, because that is what they are for: a stored BLOB and a numpy
dot product, with no vector database anywhere in the design.
"""
from __future__ import annotations

import base64
import http.client
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import urlparse

#: Where the sidecar listens. 8769 is the daemon, 8770 the depth camera, 8771 the
#: console, 8772 and 8773 the ROS bridges, 8774 the frame service, and 8775 was
#: the language model that used to answer inspections and is gone.
DEFAULT_URL = "http://127.0.0.1:8776"
ENV_URL = "UGV_PERCEPTION_URL"

#: The wall clock on one look. Generous beside the 0.2 s a look costs on the GPU,
#: because the first call after a restart also pays for loading three models, and
#: because a board that has fallen back to the CPU takes ten times longer and is
#: still worth waiting for.
TIMEOUT_S = 60.0
#: How long to wait on health. It answers at once or it is loading; neither is
#: worth blocking an inspection on.
PROBE_S = 3.0


@dataclass
class Sighting:
    """One region of one frame, as measured. The box is the measurement.

    There is no name on this and there is not going to be one. A region used to
    carry the nearest phrase in a fixed word list to `siglip`, scored between
    0.08 and 0.12 whatever was in the picture; it named a sofa "a computer
    monitor" on the rover's own frame and nothing downstream could safely read
    it. `siglip` itself is kept, because a phrase a person types compares against
    it honestly.
    """

    bbox: list[float]
    region_score: float = 0.0
    area: float = 0.0
    dino: bytes = b""
    siglip: bytes = b""
    #: Perception has no categories, so everything it finds is an object. The
    #: field exists because the store records a kind and because a later phase may
    #: want openings kept apart from furniture.
    kind: str = "object"

    @property
    def raw(self) -> dict[str, Any]:
        """What was measured about this region, for the row that keeps it.

        The vectors are not in here. They are columns of their own, because a
        768-float BLOB inside a JSON string would be neither readable nor usable.
        """
        return {"region_score": round(self.region_score, 3),
                "area": round(self.area, 4)}


@dataclass
class Look:
    """What one call produced.

    `ok` says the sidecar answered. `backend` says which of the two answered it,
    and it is not decoration: the GPU and CPU backends do not produce comparable
    vectors, so a stored vector is only ever worth comparing with another from the
    same backend.
    """

    ok: bool = False
    error: str = ""
    backend: str = ""
    regions: list[Sighting] = field(default_factory=list)
    found: int = 0
    kept: int = 0
    #: How many regions were dropped for having no picture in them -- a blown-out
    #: window, a bare wall. Carried so the diagnostics can say so: on the rover
    #: this is a sixth of what the region finder proposes, and a person reading
    #: "5 of 20 regions kept" deserves to know why.
    blank: int = 0
    timings: dict[str, Any] = field(default_factory=dict)
    took_s: float = 0.0
    duration_s: float = 0.0


class Eyes:
    """Look at one picture and say what regions are in it."""

    name = "perception"

    def look(self, jpeg: bytes) -> Look:
        raise NotImplementedError

    def embed(self, phrases: list[str]) -> tuple[list[bytes], str]:
        return [], "this perception backend cannot embed text"

    def available(self) -> tuple[bool, str]:
        return True, ""

    def describe(self) -> str:
        return self.name


class FakeEyes(Eyes):
    """Perception that answers from a script, for tests and for a workstation.

    Enough to build and exercise the store, the placement arithmetic and the
    console without a GPU or a gigabyte of models. It proves nothing about any
    real model, and an experiment about whether the rover keeps track of a real
    chair cannot be run against it.
    """

    name = "fake"

    def __init__(self, looks: list[Any] | None = None, fail: str = "") -> None:
        self.looks = list(looks or [])
        self.fail = fail
        self.calls: list[int] = []

    def look(self, jpeg: bytes) -> Look:
        self.calls.append(len(jpeg))
        if self.fail:
            return Look(ok=False, error=self.fail, backend=self.name)
        if not self.looks:
            return Look(ok=True, backend=self.name)
        answer = self.looks.pop(0)
        if isinstance(answer, Look):
            return answer
        regions = [region if isinstance(region, Sighting) else Sighting(**region)
                   for region in answer]
        return Look(ok=True, backend=self.name, regions=regions,
                    found=len(regions), kept=len(regions), took_s=0.01)

    def embed(self, phrases: list[str]) -> tuple[list[bytes], str]:
        """A deterministic stand-in: each phrase becomes a vector of its own.

        Enough to prove that a search ranks, that an empty result is reported and
        that the wiring is right, and nothing at all about whether SigLIP2 finds
        a spray bottle.
        """
        import hashlib
        import struct

        if self.fail:
            return [], self.fail
        vectors = []
        for phrase in phrases:
            digest = hashlib.sha256(phrase.encode("utf-8")).digest()[:32]
            vectors.append(struct.pack("<8f", *[b / 255.0 for b in digest[:8]]))
        return vectors, ""

    def available(self) -> tuple[bool, str]:
        return (False, self.fail) if self.fail else (True, "")


class SidecarEyes(Eyes):
    """The perception sidecar on loopback, over HTTP.

    No exception escapes `look`. A sidecar that is not running, one still loading
    its models, a connection dropped mid-answer and a reply that is not JSON all
    come back as a `Look` with `ok` false and a sentence saying which.
    """

    name = "perception-sidecar"

    def __init__(self, url: str | None = None,
                 timeout_s: float = TIMEOUT_S) -> None:
        self.url = (url or os.environ.get(ENV_URL) or DEFAULT_URL).rstrip("/")
        parsed = urlparse(self.url)
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 80
        self.timeout_s = timeout_s

    def describe(self) -> str:
        return f"{self.name} at {self.url}"

    def available(self) -> tuple[bool, str]:
        payload, error = self._request("GET", "/health", None, PROBE_S)
        if error:
            return False, (f"the perception sidecar at {self.url} is not "
                           f"answering ({error}). Start it with "
                           f"~/ugv/world_state/run_perception.sh")
        if not payload.get("ready"):
            return False, str(payload.get("detail") or "the models are not installed")
        return True, ""

    def look(self, jpeg: bytes) -> Look:
        began = time.monotonic()
        payload, error = self._request("POST", "/look", jpeg, self.timeout_s)
        took = round(time.monotonic() - began, 3)
        if error:
            return Look(ok=False, error=error, duration_s=took)
        if not payload.get("ok"):
            return Look(ok=False, duration_s=took,
                        error=str(payload.get("error") or "the sidecar refused"))
        regions = []
        for region in payload.get("regions") or []:
            try:
                regions.append(Sighting(
                    bbox=[float(value) for value in region["bbox"]],
                    region_score=float(region.get("region_score") or 0.0),
                    area=float(region.get("area") or 0.0),
                    dino=base64.b64decode(region.get("dino") or ""),
                    siglip=base64.b64decode(region.get("siglip") or "")))
            except (KeyError, TypeError, ValueError) as bad:
                # One malformed region does not throw the frame away. The rest of
                # it is still a measurement, and the count that is short says so.
                continue
        return Look(ok=True, backend=str(payload.get("backend") or ""),
                    regions=regions,
                    found=int(payload.get("found") or 0),
                    kept=int(payload.get("kept") or 0),
                    blank=int(payload.get("blank") or 0),
                    timings=payload.get("timings") or {},
                    took_s=float(payload.get("took_s") or 0.0),
                    duration_s=took)

    def embed(self, phrases: list[str]) -> tuple[list[bytes], str]:
        """Text vectors for phrases, in the same space as the stored ones.

        (vectors, error) rather than an exception, for the reason everything else
        here answers that way: the caller is a console request handler inside the
        daemon, and a sidecar that is restarting is an ordinary Tuesday.
        """
        body = json.dumps({"phrases": list(phrases)}).encode()
        payload, error = self._request("POST", "/embed", body, self.timeout_s,
                                       content_type="application/json")
        if error:
            return [], error
        if not payload.get("ok"):
            return [], str(payload.get("error") or "the sidecar refused")
        try:
            return [base64.b64decode(one) for one in payload["vectors"]], ""
        except (KeyError, TypeError, ValueError) as bad:
            return [], f"the sidecar sent vectors that could not be read: {bad}"

    # --- the wire -------------------------------------------------------------

    def _request(self, method: str, path: str, body: bytes | None,
                 timeout: float,
                 content_type: str = "image/jpeg") -> tuple[dict[str, Any], str]:
        """(payload, error). Never raises."""
        connection = None
        try:
            connection = http.client.HTTPConnection(self.host, self.port,
                                                    timeout=timeout)
            headers = {"Content-Type": content_type} if body else {}
            connection.request(method, path, body=body, headers=headers)
            reply = connection.getresponse()
            raw = reply.read()
        except OSError as error:
            return {}, f"{type(error).__name__}: {error}"
        except Exception as error:                     # never past here
            return {}, f"{type(error).__name__}: {error}"
        finally:
            if connection is not None:
                connection.close()
        try:
            return json.loads(raw.decode("utf-8", "replace")), ""
        except ValueError:
            return {}, (f"the sidecar answered {reply.status} with "
                        f"{len(raw)} bytes that were not JSON")


def describe_eyes(eyes: Eyes) -> str:
    """What to write in the diagnostics row for this perception backend."""
    describe = getattr(eyes, "describe", None)
    return describe() if callable(describe) else getattr(eyes, "name", "unknown")
