# Step 0, run

[omni-build.md](omni-build.md) ends by saying that nothing in it has been run and
that all of it should be treated as a shopping list. This is the first item off
that list, measured on 2026-08-17 with the harness in
[omni_bench](../omni_bench/), and it answers the question the whole plan was made
contingent on.

The short version: the fear behind step 0 was misplaced. Speech costs these models
nothing in tool calling — every one of the four scored the same or *better* spoken
than typed. What separates them is something the plan never thought to ask about,
which is judgement about *which* tool, and whether to call one at all. And the
schemas they were given are partly to blame, because every word of those
descriptions was tuned against a model with the opposite fault.

## What was asked, and how

The rover's ten real schemas, read straight out of
[rover_daemon.py](../rover_daemon/rover_daemon.py), and the deployed system prompt
read out of [voice_chat/server.py](../voice_chat/server.py) — the 75/90 wording,
with the sentence about not saying "I will" left in the position that was worth
nine points of it. Fifteen phrases, six samples each, temperature 0.2. Each phrase
typed and spoken, in the same process against the same weights.

That pairing is a deliberate departure from the plan, which compares the spoken
score against the 66/90 already in the repository. That number was measured on
`Qwen3-VL-4B`, and setting an omni model's spoken score beside a different model's
typed score confounds the two changes we care most about telling apart. The old
figure is background here, not the comparator.

Five further phrases reach the five tools the fifteen never touch — `count_faces`,
`look_at`, `center_camera`, `track_next`, `tracking_status`. They are scored
separately and never folded into the ninety, because they have no typed baseline
to be compared against. They turned out to be the most informative part of the
run.

Speech is the voices Windows already has, 16 kHz mono. They are worse than a
person, which is the right direction to be wrong in: a model that fails on clean
synthetic speech was never going to survive a mic array in a room.

## The four

| | typed | spoken | the five extra tools, spoken |
|---|---|---|---|
| **Qwen3.5-Omni-Plus** (hosted) | **90/90** | **90/90** | 30/30 |
| **Qwen3.5-Omni-Flash** (hosted) | **90/90** | **90/90** | 30/30 |
| **MiniCPM-o 4.5** (9B, local) | 84/90 | **89/90** | 12/30 typed, 12/30 spoken |
| **Qwen3-Omni-30B-A3B** (local) | 57/90 | 66/90 | 30/30 |
| `Qwen3-VL-4B`, typed, for reference | 75/90 | — | — |

**Nobody lost anything to speech.** MiniCPM-o gained five points, Qwen3-Omni
gained nine, and the hosted models were already perfect. The decision rule
anticipated a drop of up to ten and allowed for twenty; there was no drop at all.

**The upper bound exists and is 90/90.** Alibaba's hosted Qwen3.5-Omni answers
every phrase correctly, spoken and typed, including the four that defeated
`Qwen3-VL-4B` and the one that must call nothing. So the task is not intrinsically
hard, and any local model that fails it is failing for reasons that can be worked
on rather than because omni-modal tool calling does not work yet.

**Track A's verdict is proceed.**

## MiniCPM-o 4.5, the one that would actually be deployed

| request | typed | spoken | Qwen3-VL-4B, typed |
|---|---|---|---|
| "Well, can you switch the lights on?" | 6/6 | 6/6 | 0/6 → 6/6 |
| "Can you switch the lights on?" | 6/6 | 6/6 | 4/6 → 6/6 |
| "Switch the lights on." | 6/6 | 6/6 | 6/6 |
| "Can you switch the lights off?" | 6/6 | 6/6 | 6/6 |
| "Well, can you switch the lights off?" | 6/6 | 6/6 | 6/6 |
| "Could you dim the lights a bit?" | 6/6 | 6/6 | **0/6** |
| "Are the lights on?" | 6/6 | 6/6 | 6/6 |
| "Follow me." | 6/6 | 6/6 | **2/6 → 3/6** |
| "Start following me." | 6/6 | 6/6 | 6/6 |
| "Start tracking people." | 6/6 | 6/6 | 6/6 |
| "Then start tracking people." | 6/6 | 6/6 | **0/6** |
| "Would you stop following me?" | 6/6 | 6/6 | 6/6 |
| "What do you see?" | 6/6 | 6/6 | 6/6 |
| "So, what do you see?" | 6/6 | 6/6 | 6/6 |
| "What is your name?" *(want 0)* | **0/6** | **5/6** | 6/6 |
| **total** | **84/90** | **89/90** | 75/90 |

Every phrasing that defeated `Qwen3-VL-4B` — the dimming request, "Follow me.",
the leading "Then" — is 6/6 here, typed and spoken alike. The entire difference
between its two columns is the one case that is supposed to call nothing.

### Its failure is over-calling, and it is new

In 235 of 240 attempts it called a tool. The five that did not were "What is your
name?", spoken. Asked the same question in writing it called `look` six times out
of six: it would rather photograph the room than admit it has no name.

The five coverage cases show where that goes.

| request | wanted | MiniCPM-o typed | spoken | Qwen3-Omni-30B | hosted |
|---|---|---|---|---|---|
| "How many people can you see?" | `count_faces` | 3/6 | **0/6** | 6/6 | 6/6 |
| "Can you look to your left?" | `look_at` | **0/6** | **0/6** | 6/6 | 6/6 |
| "Look straight ahead." | `center_camera` | **0/6** | **0/6** | 6/6 | 6/6 |
| "Are you tracking anyone?" | `tracking_status` | 6/6 | 6/6 | 6/6 | 6/6 |
| "Track someone else." | `track_next` | 6/6 | 6/6 | 6/6 | 6/6 |

Every one of those failures is the same failure: it called `look`. Asked to aim
the camera left, it takes a photograph. Asked to centre the camera, it takes a
photograph. Asked how many people are there, it photographs them instead of
counting them. The other three models get all five right.

This is [the neighbour
effect](../voice_chat/README.md#getting-it-to-call-look--a-tool-is-read-against-its-neighbours)
running backwards. `look`'s description is the most argued-over text in this
repository, and every sentence of it exists to overcome a model that would not
look: it opens by naming the questions that should trigger it and it claims to be
"the only way you can see anything at all". Put in front of a model that reaches
for tools by default, those same words make `look` swallow its neighbours.

So the schema wording does not transfer, and it fails in the most expensive
direction possible: tuned to fix under-calling, handed to a model that over-calls.
The tools survive; the prose inside them does not, and every `look` table in
[voice_chat](../voice_chat/README.md#getting-it-to-actually-call-them) has to be
re-run against whichever model is actually deployed.

## Qwen3-Omni-30B, and a familiar lie

The big local model scores worst of the four, and it is worth seeing why, because
the failure is one this repository has met before.

It is perfect on all five of the extra tools, in both arms, where MiniCPM-o
manages two. What it cannot do is start or stop tracking. "Follow me.", "Start
following me.", "Start tracking people.", "Would you stop following me." — 0/6
each, in both arms, and not because it picked the wrong tool:

```
  "Follow me."                   -> "I am following you now."        no call
  "Start tracking people."       -> "Started tracking people."       no call
  "Would you stop following me?" -> "I am stopping face tracking."   no call
```

That is precisely the *announcing instead of acting* that
[voice_chat](../voice_chat/README.md#the-re-run-it-does-not-refuse-it-promises)
documents at length for `Qwen3-VL-4B`, and the sentence in the system prompt
written to stop it — "never say you have switched, moved, started or stopped
anything unless the call was made" — does not stop it here. It has simply
concentrated on one family of tools instead of being spread across all of them.

Two smaller notes. Its lights and vision tools are perfect. And it is the only
model whose typed column is *worse* than its spoken one by a wide margin (57
against 66), driven by "What do you see?" splitting 3/6 typed against 6/6 spoken.

## Can the tool call be caught before it is spoken?

Yes, and on this serving path it is not even close.

Across 235 MiniCPM-o calls the tool call was the *entire* reply: zero characters
of prose before the marker, which arrived a median of 0.17 seconds into the
decode. There is nothing to hold back, because the model never says anything
first. The sniffer's careful business of buffering a partial `<tool` fragment is
still needed for the sub-word split, but there is no prose to race against.

Then the other half of the question. Asked to produce speech as well as text, the
model happily synthesised the tool call. Replies that were nothing but
`<tool_call>{"name": "set_lights", "arguments": {"level": 255}}</tool_call>` came
back with 9, 13, 15 and 44 seconds of audio attached, and transcribing that audio
gives, among other things:

```
  "set recites, arguments sap level dot two hundred fifty five ...
   he turned the lights on."
```

That is the schema being read aloud, badly, by a rover that did at least make the
call. `_ToolSniffer` therefore transfers intact and matters more than before,
exactly as [omni-build.md](omni-build.md) predicted.

**One caveat about how far that generalises.** This was measured through
`transformers`, and on that path MiniCPM-o 4.5 is not duplex at all: `chat` with
`stream=True` returns the text iterator and returns *before* any speech exists,
and speech is generated afterwards from the finished text by
`_generate_speech_non_streaming`. The interception margin measured here is
therefore infinite by construction. The genuinely interleaved Omni-Flow behaviour
the architecture document leans on lives in `vllm-omni`'s realtime runtime, which
is a different serving stack and was not exercised. What is established is that
the call is textual, that it arrives first, and that a speech decoder given the
chance will read it out loud.

## What it cost

About $2.50 against a five-dollar target, by the stopwatch:

| | |
|---|---|
| smoke test, L40S, 1.2 min | $0.02 |
| MiniCPM-o 4.5, L40S, ~50 min | ~$0.66 |
| Qwen3-Omni-30B, RTX Pro 6000, ~65 min | ~$1.83 |
| the keeper pod, ~5 min | ~$0.06 |
| Qwen3.5-Omni via DashScope, 537 requests | $0.22, and free in practice |

Read the RunPod rows as estimates rather than receipts. Their per-pod billing
records settle with a lag — hours after the last pod was terminated the account
still listed only the smoke test — so the balance is the number to trust, and it
will keep falling for a while after a run ends.

The DashScope row is exact, and it is worth breaking down because of what it says
about the shape of this workload. Flash cost $0.05 and Plus $0.17, on 787K input
tokens against 15.6K output. Of the roughly 1,470 input tokens in a request,
**1,450 are the system prompt and the ten schemas and 23 are the audio.** The
question is free; the tools are the bill. Anything that runs this often enough to
care should be reaching for prompt caching rather than for a cheaper model, and
the free tier's 1M input tokens — 79 per cent of which this one afternoon spent —
is really a budget of about one more sweep.

Two notes on the receipts. The RTX Pro 6000 was rented instead of the L40S the
plan costed because Qwen3-Omni's 70 GB of bf16 weights do not fit in 48 GB, and
renting 96 GB for an hour was both cheaper and far less risky than an hour spent
teaching vLLM to quantise a mixture-of-experts to fp8 on the fly. That also
removes a confound the plan flagged as unmeasured: this is bf16, so nothing here
is a quantisation artefact.

And the harness was estimated at "days, not an afternoon". It was an afternoon,
because the corpus already existed and the schemas could be read out of the daemon
rather than rewritten.

## What this changes upstream

* **"Tool calling is not documented anywhere for this model"** — true of
  MiniCPM-o's documentation, false of the checkpoint. It ships the Qwen chat
  template wholesale, `# Tools` block and `<tool_call>` markers and all.
* **"Qwen3-Omni is a quality ceiling to measure against"** — it is not. It is the
  worst of the four at this task. The ceiling is the hosted Qwen3.5-Omni, at
  90/90, and the 9B model that would actually be deployed sits one point below it.
* **The question worth asking now is the opposite one.** Not whether an omni model
  can call a tool when spoken to — it can — but whether it can be persuaded *not*
  to, and whether ten schemas tuned for a reluctant model can be retuned for an
  eager one without breaking the neighbours. That is a `/chat`-style sweep against
  MiniCPM-o, and it is the cheapest useful thing to do next.
* **Nothing here tested a room.** Synthetic speech is clean, one speaker, no echo,
  no barge-in. Every number above is still an upper bound on what the rover will
  get, which is what the decision rule assumed.
