"""The fifteen phrases, and what each one should make the rover do.

This file exists because the corpus did not, as a corpus. It was distributed
across the tables in [voice_chat/README.md] -- nine rows in the one that totals
66/90, the rest named in the prose around it -- and a measurement whose case list
has to be reassembled by reading an essay is a measurement nobody can repeat.
Every phrase below carries the score it got through `/chat` against
`Qwen3-VL-4B-Instruct` at temperature 0.2, six samples a cell, so that a spoken
result can be read beside the typed one it is meant to be compared with.

Two blocks, and the distinction matters:

  * `PRIMARY` is the fifteen. It is what totals out of 90, and it is the only
    thing the decision rule in [docs/omni-build.md] applies to. Do not add to it:
    the whole value of these fifteen is that a number taken today is comparable
    with one taken in August.
  * `COVERAGE` is five more, added here and *not* counted in the total, because
    the fifteen exercise only five of the daemon's ten tools. A model that never
    sees `count_faces`, `look_at`, `center_camera`, `track_next` or
    `tracking_status` in a test has not really been asked whether it can read a
    ten-tool schema list. These are new phrasings with no text baseline, which is
    exactly why they are kept out of the headline figure.

The `want` field is the tool that should be called, or `None` for the one case
that must call nothing -- a corpus with no negative in it cannot tell a model
that calls correctly from a model that calls constantly.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Case:
    """One phrase, the call it should produce, and where its baseline came from."""

    text: str
    want: str | None
    baseline: str = ""          # what /chat scored on the deployed text model
    want_args: dict = field(default_factory=dict)
    note: str = ""

    @property
    def key(self) -> str:
        """A short stable id for filenames and result tables."""
        return self.text.lower().translate(str.maketrans("", "", "?.,'")).replace(" ", "-")[:40]


# The fifteen. Nine come from the table that totals 66/90 with the deployed
# prompt and 75/90 with the "do not say 'I will'" sentence added; the remaining
# six are named in the prose of the same section, with their scores.
PRIMARY: list[Case] = [
    Case("Well, can you switch the lights on?", "set_lights", "0/6 -> 6/6",
         {"level": 255},
         "the case that started it: it promised and called nothing"),
    Case("Can you switch the lights on?", "set_lights", "4/6 -> 6/6", {"level": 255}),
    Case("Switch the lights on.", "set_lights", "6/6", {"level": 255}),
    Case("Can you switch the lights off?", "set_lights", "6/6", {"level": 0}),
    Case("Well, can you switch the lights off?", "set_lights", "6/6", {"level": 0},
         "a leading marker only tips a phrasing that was already marginal"),
    Case("Could you dim the lights a bit?", "set_lights", "0/6", {"level": "dim"},
         "0/6 under every wording tried, and it lies rather than stalls"),
    Case("Are the lights on?", "get_lights", "6/6 on its own"),
    Case("Follow me.", "start_tracking", "2/6 -> 3/6",
         note="announcing instead of acting, with and without the fix"),
    Case("Start following me.", "start_tracking", "6/6"),
    Case("Start tracking people.", "start_tracking", "6/6"),
    Case("Then start tracking people.", "start_tracking", "0/6",
         note="one leading word against the line above, and nothing explains it"),
    Case("Would you stop following me?", "stop_tracking", "6/6"),
    Case("What do you see?", "look", "6/6"),
    Case("So, what do you see?", "look", "6/6"),
    Case("What is your name?", None, "0/6, and 0 is the wanted score",
         note="the negative case: answering in words is the correct behaviour"),
]

# Not counted. These reach the five tools the fifteen never touch.
COVERAGE: list[Case] = [
    Case("How many people can you see?", "count_faces", "6/6 once count_faces was reworded"),
    Case("Can you look to your left?", "look_at", "6/6 on the older case list",
         {"pan": "negative"}),
    Case("Look straight ahead.", "center_camera"),
    Case("Are you tracking anyone?", "tracking_status"),
    Case("Track someone else.", "track_next"),
]

ALL = PRIMARY + COVERAGE

# Six, because three will mislead you. The README records an investigation where
# a 3-sample spoken test said a prompt change had made things worse while 6-sample
# text runs said it had taken two cases from 0/6 to 6/6.
SAMPLES = 6

# 15 cases x 6 samples. The denominator the decision rule is written against.
DENOMINATOR = len(PRIMARY) * SAMPLES


def judge(case: Case, calls: list[dict]) -> tuple[bool, str]:
    """Did this attempt do what the phrase asked? Returns (passed, why).

    Deliberately strict about the negative case and deliberately forgiving about
    everything else: extra chatter around a correct call is not a failure, but a
    second, different tool call is, because on a real rover it is a second
    physical act.
    """
    names = [c.get("name") for c in calls]

    if case.want is None:
        return (not names, "called nothing" if not names else f"called {names}, wanted none")

    if not names:
        return False, "called nothing"
    if case.want not in names:
        return False, f"called {names}, wanted {case.want}"
    if len(set(names)) > 1:
        return False, f"called {names} -- more than the one act asked for"

    call = next(c for c in calls if c.get("name") == case.want)
    args = call.get("arguments") or {}
    for key, expected in case.want_args.items():
        got = args.get(key)
        if expected == "dim":
            # "a bit" is not a number, so anything strictly between off and full
            # counts. The failure being tested for is a model that says it dimmed
            # and sets 0 or 255, or sets nothing at all.
            if not isinstance(got, (int, float)) or not 0 < got < 255:
                return False, f"{key}={got!r}, wanted a level between off and full"
        elif expected == "negative":
            if not isinstance(got, (int, float)) or got >= 0:
                return False, f"{key}={got!r}, wanted a negative angle"
        elif got != expected:
            return False, f"{key}={got!r}, wanted {expected!r}"

    return True, f"{case.want}({', '.join(f'{k}={v}' for k, v in args.items())})"


if __name__ == "__main__":
    print(f"{len(PRIMARY)} primary x {SAMPLES} samples = {DENOMINATOR}")
    for block, cases in (("PRIMARY", PRIMARY), ("COVERAGE", COVERAGE)):
        print(f"\n{block}")
        for case in cases:
            want = case.want or "(nothing)"
            args = f" {case.want_args}" if case.want_args else ""
            print(f"  {case.text:<40} -> {want}{args:<22} {case.baseline}")
