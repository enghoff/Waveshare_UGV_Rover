"""The model boundary: one small interface, a deterministic fake, and a client for
the local Cosmos sidecar.

Cosmos Reason 2 is maintenance-only upstream and NVIDIA points new work at Cosmos
3, so a replacement is a normal event rather than a rewrite. Everything that knows
this particular model is in `CosmosReasoner`; everything else in this package
knows only `inspect(jpeg, known) -> Answer`.

The model runs in its own process for the reason that decides most of the shape of
this rover: `rover_daemon` owns the driver-board UART, the gimbal camera and STOP,
and a model that runs out of memory or takes a fault must not be able to take those
down with it. So the sidecar is a `llama.cpp` server on loopback, started and
restarted by its own script, and this file is a client that treats it being absent
as an ordinary answer.

Three separate things stop a model that will not stop talking, because runaway
generation is the likeliest way an inspection hangs: a grammar built from the
response schema, a cap on output tokens, and a wall clock that gives up on the
whole call.
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

from .contract import RESPONSE_SCHEMA, build_prompt

#: Where the sidecar listens. Loopback only: 8769 is the daemon, 8770 the depth
#: camera, 8771 the console, 8772 and 8773 the ROS bridges and 8774 the frame
#: service the voice session uses, so the next free number is this one.
DEFAULT_URL = "http://127.0.0.1:8775"
ENV_URL = "UGV_COSMOS_URL"
ENV_MODEL = "UGV_COSMOS_MODEL"

#: The wall clock on one inspection. Generous because this is a 2B model doing
#: vision on four CPU cores and the first call after a restart also pays for the
#: image encoder warming up; hard because the console is holding a connection open
#: behind it and a call that never returns is worse than one that fails.
TIMEOUT_S = 180.0
#: The output cap, and it has to be big enough for the biggest answer the schema
#: allows -- otherwise a model doing exactly what it was asked runs out mid-object
#: and its work is thrown away as truncated, which is what happened here twice
#: before these two numbers were made to agree. Measured on the rover: about
#: sixty-five tokens per observation, six of them, plus the scene sentence, is
#: around five hundred. This is that with room to spare, and still well inside the
#: wall clock below at the seven or so tokens a second this board manages.
MAX_TOKENS = 900
#: How long to wait on the sidecar's health endpoint. It either answers at once or
#: it is loading a model, and neither case is worth blocking an inspection on.
PROBE_S = 3.0


@dataclass
class Answer:
    """What one call to a physical reasoner produced.

    `ok` says the model answered; whether what it said was usable is
    `contract.validate`'s question, deliberately not this one. Keeping the two
    apart is what lets the popup say "the model is not running" and "the model
    answered with prose" as different things, which are fixed in different places.
    """

    ok: bool = False
    text: str = ""
    error: str = ""
    model_id: str = ""
    backend: str = ""
    duration_s: float = 0.0
    usage: dict[str, Any] = field(default_factory=dict)


class PhysicalReasoner:
    """Look at one picture and say what is in it.

    `known` is the bounded list of entities the caller is prepared to have the
    answer refer to. It is passed in rather than fetched here because the store,
    not the model client, decides how much of the world the model is shown.
    """

    name = "physical-reasoner"

    def inspect(self, jpeg: bytes, known: list[dict[str, Any]]) -> Answer:
        raise NotImplementedError

    def available(self) -> tuple[bool, str]:
        """(ready, why not). Cheap enough to call before every inspection."""
        return True, ""

    def close(self) -> None:
        pass


class FakeReasoner(PhysicalReasoner):
    """A reasoner that answers from a script, for tests and for development.

    It exists so the store, the association rules and the console can be built and
    exercised on a workstation with no GPU and no two-gigabyte download. It does
    not finish the task: an experiment about whether a real model keeps track of a
    real sofa cannot be run against this.

    Each entry in `answers` is either a dict, which is returned as JSON, or a
    string, which is returned verbatim -- the second being how the malformed and
    truncated cases get tested.
    """

    name = "fake"

    def __init__(self, answers: list[Any] | None = None,
                 fail: str = "", model_id: str = "fake-reasoner") -> None:
        self.answers = list(answers or [])
        self.fail = fail
        self.model_id = model_id
        self.calls: list[dict[str, Any]] = []

    def inspect(self, jpeg: bytes, known: list[dict[str, Any]]) -> Answer:
        self.calls.append({"bytes": len(jpeg), "known": [e["id"] for e in known],
                           "prompt": build_prompt(known)})
        if self.fail:
            return Answer(ok=False, error=self.fail, backend=self.name,
                          model_id=self.model_id)
        if not self.answers:
            return Answer(ok=True, text=json.dumps({"scene": "", "observations": []}),
                          backend=self.name, model_id=self.model_id)
        answer = self.answers.pop(0)
        text = answer if isinstance(answer, str) else json.dumps(answer)
        return Answer(ok=True, text=text, backend=self.name,
                      model_id=self.model_id, duration_s=0.01)

    def available(self) -> tuple[bool, str]:
        return (False, self.fail) if self.fail else (True, "")


class CosmosReasoner(PhysicalReasoner):
    """Cosmos Reason 2 2B, quantized, behind a local `llama.cpp` server.

    Everything specific to this model and this runtime is here. The rest of the
    package sees an `Answer`, and a Cosmos 3 or an entirely different VLM replaces
    this class without touching the schema or the console.

    No exception escapes `inspect`. A sidecar that is not running, one that is
    still loading the model, a connection dropped mid-generation and a model that
    talks past its budget all come back as an answer with `ok` false and a sentence
    saying which -- because the caller is an inspection that must leave the world
    state untouched and say why, not a program that can afford to raise.
    """

    name = "cosmos-reason2-llamacpp"

    def __init__(self, url: str | None = None, model_id: str = "",
                 timeout_s: float = TIMEOUT_S,
                 max_tokens: int = MAX_TOKENS) -> None:
        self.url = (url or os.environ.get(ENV_URL) or DEFAULT_URL).rstrip("/")
        parsed = urlparse(self.url)
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 80
        self.model_id = model_id or os.environ.get(ENV_MODEL, "")
        self.timeout_s = timeout_s
        self.max_tokens = max_tokens
        #: Set once the server has refused a JSON-schema response format, so the
        #: fallback is paid for once rather than on every inspection.
        self._no_schema = False

    def describe(self) -> str:
        return f"{self.name} at {self.url}"

    def available(self) -> tuple[bool, str]:
        body, error = self._get("/health", PROBE_S)
        if error:
            return False, f"the Cosmos sidecar at {self.url} is not answering: {error}"
        status = (body or {}).get("status", "")
        if status and status != "ok":
            return False, f"the Cosmos sidecar says it is {status}"
        return True, ""

    def model(self) -> str:
        """The model build actually loaded, which is what gets stamped on the rows.

        Asked of the server rather than configured here, so that a database written
        across a change of quantization says so by itself.
        """
        if self.model_id:
            return self.model_id
        body, error = self._get("/v1/models", PROBE_S)
        if not error and isinstance(body, dict):
            data = body.get("data") or []
            if data and isinstance(data[0], dict):
                self.model_id = str(data[0].get("id") or "")
        return self.model_id

    def inspect(self, jpeg: bytes, known: list[dict[str, Any]]) -> Answer:
        began = time.monotonic()
        model_id = self.model()
        request = {
            "model": model_id or "cosmos-reason2-2b",
            "temperature": 0.0,
            "top_p": 1.0,
            "max_tokens": self.max_tokens,
            "stream": False,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": build_prompt(known)},
                    {"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64,"
                               + base64.b64encode(jpeg).decode("ascii")}},
                ],
            }],
        }
        if not self._no_schema:
            # A grammar rather than a hope. This is what turns "answer in JSON"
            # from an instruction a small model may ignore into something the
            # sampler cannot leave.
            request["response_format"] = {
                "type": "json_schema",
                "json_schema": {"name": "world_state", "schema": RESPONSE_SCHEMA},
            }
        body, error = self._post("/v1/chat/completions", request, self.timeout_s)
        took = round(time.monotonic() - began, 2)
        if error and not self._no_schema and _schema_refused(error):
            # An older or differently built server. Pay for the discovery once and
            # go on without the grammar; the answer then has to survive
            # `extract_json`, which is written for exactly this.
            self._no_schema = True
            request.pop("response_format", None)
            body, error = self._post("/v1/chat/completions", request,
                                     max(5.0, self.timeout_s - took))
            took = round(time.monotonic() - began, 2)
        if error:
            return Answer(ok=False, error=error, backend=self.name,
                          model_id=model_id, duration_s=took)
        try:
            choice = body["choices"][0]
            text = choice["message"]["content"]
        except (KeyError, IndexError, TypeError):
            return Answer(ok=False, backend=self.name, model_id=model_id,
                          duration_s=took,
                          error="the sidecar answered in an unexpected shape: "
                                + json.dumps(body)[:200])
        finish = str(choice.get("finish_reason") or "")
        answer = Answer(ok=True, text=text or "", backend=self.name,
                        model_id=model_id, duration_s=took,
                        usage=body.get("usage") or {})
        if finish == "length":
            # Said out loud rather than left for the JSON parser to trip over. A
            # model cut off mid-sentence is a budget that is too small, which is a
            # different fix from a model that answered in prose.
            answer.usage["truncated"] = True
        return answer

    # --- the wire -------------------------------------------------------------

    def _get(self, path: str, timeout_s: float):
        return self._request("GET", path, None, timeout_s)

    def _post(self, path: str, payload: dict[str, Any], timeout_s: float):
        return self._request("POST", path, payload, timeout_s)

    def _request(self, method: str, path: str, payload: dict[str, Any] | None,
                 timeout_s: float) -> tuple[dict[str, Any] | None, str]:
        """One request, with a deadline that covers the whole of it.

        `http.client`'s timeout is per socket operation, so a server producing one
        token every few seconds would reset it for ever. The body is therefore read
        in chunks against a deadline, and the connection is dropped the moment the
        budget is gone -- which is the difference between one failed inspection and
        a console connection held open until somebody notices.
        """
        deadline = time.monotonic() + timeout_s
        connection = http.client.HTTPConnection(self.host, self.port,
                                                timeout=timeout_s)
        try:
            headers = {"Accept": "application/json"}
            body = None
            if payload is not None:
                body = json.dumps(payload).encode()
                headers["Content-Type"] = "application/json"
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            raw = bytearray()
            while True:
                if time.monotonic() > deadline:
                    return None, (f"the sidecar was still answering after "
                                  f"{timeout_s:.0f} s and was given up on")
                chunk = response.read(65536)
                if not chunk:
                    break
                raw += chunk
            if response.status != 200:
                return None, (f"the sidecar answered {response.status} "
                              f"{response.reason}: "
                              f"{raw.decode('utf-8', 'replace')[:300]}")
            try:
                return json.loads(raw.decode("utf-8", "replace")), ""
            except ValueError as error:
                return None, f"the sidecar's reply was not JSON: {error}"
        except (OSError, http.client.HTTPException) as error:
            return None, f"{type(error).__name__}: {error}"
        finally:
            connection.close()


def _schema_refused(error: str) -> bool:
    lowered = error.lower()
    return ("response_format" in lowered or "json_schema" in lowered
            or "grammar" in lowered)


def describe_backend(reasoner: PhysicalReasoner) -> str:
    describe = getattr(reasoner, "describe", None)
    return describe() if callable(describe) else reasoner.name
