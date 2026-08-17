# Building the omni rover

[omni-architecture.md](omni-architecture.md) describes what the system would be.
This document describes what it would take to build it: which parts of the
existing code survive, which parts have to be written from scratch, which model
to run, which card to run it on, and what order to do all of that in. It was
written on 2026-08-16, and although the model landscape it surveys is only a few
months old, the prices in it will go stale within weeks.

Like the architecture document, none of this has been built. The difference is
that this one has been costed.

**This step has since been run.** [omni-step0.md](omni-step0.md) has the numbers,
and they say proceed: no model lost anything to being spoken to, and two of the
four gained. Read the rest of this section as the reasoning that led to the
measurement rather than as a live question — in particular, the fear that speech
would cost tool reliability was misplaced, and the thing that actually separates
the candidates is whether they can be persuaded *not* to call a tool.

## The question that has to be answered first

Whether an omni model calls the rover's tools reliably when spoken to is the
question that decides whether the rest of this document is worth acting on,
and the measurement does not exist anywhere — not in this repository, and not
in the field either.
[TOBench](https://arxiv.org/abs/2605.16909), the closest thing to a benchmark
for omni-modal agents using real-world tools, finds the field bad at the job —
the best proprietary model completes 32% of its tasks and the best open one
41%, with cross-modal fusion failures and multimodal hallucination prominent in
the error analysis — and even it delivers every user request as text. Its audio
lives in the task materials, never in the instruction. Nobody is measuring what
happens to tool calling when the instruction itself is spoken.

The same gap runs through what this repository believes it knows.
Every tool-calling measurement we have — the 66/90 and 75/90 totals, and all the
`look` tables in
[voice_chat](../voice_chat/README.md#getting-it-to-actually-call-them) — was
taken through the `/chat` endpoint, which accepts text and returns a decision.
For a text model that was the correct instrument, but against an omni model
those numbers become an upper bound that the deployed path never actually
reaches, because the deployed path is audio. We have spent four months
measuring the easy modality, and without a deliberate correction we would carry
that habit straight into the new design.

Step 0 is therefore to run the same corpus again, spoken rather than typed:
fifteen phrases, six samples per cell, against the real daemon schemas, with
the phrases synthesised as audio. Two measurements come out of it, not one:

* **Does the model call the right tool when the instruction is spoken?** The
  number that matters is the drop against the text column we already have,
  because that drop is what choosing an omni model costs in tool reliability,
  and at the moment nobody knows what it is.
* **For MiniCPM-o specifically: can a tool call be caught before it is
  spoken?** Its Omni-Flow interleaves text and speech tokens inside one-second
  chunks, and whether a tool call surfaces in the text stream early enough to
  intercept — or gets read out loud by the Talker — is unknown, and it is the
  mechanism everything downstream leans on. Qwen's Thinker/Talker split makes
  that interception explicit; MiniCPM-o may not.

The decision rule is written down here, before the run, because whoever runs
it will be motivated to continue: a spoken total within ten points of the text
column (out of 90) proceeds; a drop of more than twenty ends the project; in
between, the failures are read one by one before any hardware is ordered,
because a model that fails on phrasing can be met halfway by the schemas and a
model that fails on hearing cannot. The rule is deliberately asymmetric —
synthesised speech is clean, a mic array in a room is not, so even the spoken
column is an upper bound on what the rover will get.

And honesty about the cost: five dollars is what the *runs* cost. The
instrument does not exist and is the real deliverable of step 0 — audio in,
tool call out, against three models with three different serving paths — and
building it is days, not an afternoon. It is still the cheapest item in this
document, and everything below it is contingent on the result.

## What we already have

Considerably more survives than the architecture document suggests. That
document is written from the model's point of view, where almost everything gets
deleted; seen from the rover's point of view, most of the suite carries over
untouched.

| what | size | in the new design |
|---|---|---|
| [rover_daemon.py](../rover_daemon/rover_daemon.py) | 1068 | the mode and command tier, extended rather than replaced |
| [aiming.py](../face_tracking/aiming.py) | 545 | unchanged, since a calibrated control law is not a model concern |
| [face_detect/server.py](../face_detect/server.py) | 322 | unchanged, and more important than before: the admission gate needs a cheap salience signal |
| `_ToolSniffer` and `_parse_tool_call` | ~90 | unchanged, and needed more than before |
| `_stash`, `_decode_frame`, `POST /frame` | ~75 | becomes the full-resolution-on-demand path |
| `_forget_pictures`, `_forget_refusals`, `_forget_promises` | ~170 | the rules survive, but the mechanism becomes an eviction policy |
| `Endpointer` | 130 | the audio half of the admission gate, with the speculation removed |
| the failsafe in `drive_gamepad.py` | ~30 | the deadman that the safety supervisor is built around |
| `parse` and `crc8` in `lidar_view.py` | ~55 | the supervisor's stop input |
| [talk.py](../voice_chat/talk.py) | 460 | deleted, because there is no client |
| [rover_tools.py](../voice_chat/rover_tools.py) | 164 | deleted, though its discovery ordering survives as a fact worth remembering |

Four of those entries deserve more than a table row.

The first is that `rover_daemon.py` already implements most of the mode tier,
and nobody has previously said so. `Rover._loop` is a control loop that runs on
the rover, driven by `face_detect` and `aiming.py`, which the model can start,
stop and query the status of. That is exactly the `mode` row of the architecture
document's tool-shape table, and it is already built and working. What it is
missing is not the concept but the protocol: `Handler` writes precisely one
reply per request, so a mode can be started and polled but cannot report
anything on its own initiative, which means it has no way of saying that it lost
its target or hit a limit. Adding an event channel to a line protocol that
already exists is a small job, which is why the asynchronous tool protocol is
the smallest row in the components table below — and why it sits in track B
rather than waiting on the model.

The second is that the tool sniffer survives, and this is a specific technical
claim rather than wishful thinking. The Qwen3-Omni
[technical report](https://arxiv.org/html/2509.17765v1) describes external
modules, function calling among them, intervening on the textual output of the
Thinker. That means a tool call is still text in a stream, still wrapped in
markers that may or may not be special tokens depending on the tokenizer, and
still liable to arrive in sub-word fragments. Everything `_ToolSniffer` was
originally written to handle is still true, and one aspect of the problem gets
worse rather than better, because a Talker synthesising speech from that same
stream will read a leaked tool call out loud without a sentence splitter
standing in between to catch it first.

The third is that the `_forget_*` rules transfer even though the code that
implements them does not. The architecture document already makes this point
about pictures, but it applies equally to refusals and promises, and the regexes
behind them (`_UNABLE`, `_SEEING`, `_PROMISING`, `_DOING`) represent the
distilled result of a great many six-sample experiments. What remains unknown is
whether the underlying law — that whatever this model said last, it will say
again — is a property of `Qwen3-VL-4B` specifically or of small
instruction-tuned models in general, and finding that out is one of the cheaper
experiments available to us.

The fourth is that the fifteen-phrase corpus is the single most valuable asset
this repository holds for the work ahead. It is the only reason step 0 is a
harness plus some runs rather than a corpus-design project on top of both, and
it exists solely because somebody took the trouble to record six samples per
cell rather than three.

Beyond the code, a number of measurements carry over as facts rather than as
lines of source: 266 ms of gimbal dead time, 9.65 px of image shift per
commanded degree, the 0.85 and 0.60 thresholds for acquiring and keeping a
target, a sweep rate of 25°/s, roughly 2° of backlash, and the finding that a
process waking fifty times a second breaks audio on the current Pi. That last
one is not a curiosity; it is a hard requirement on where the admission gate can
run.

## What has to be written

There are nine new components. The size estimates below are relative to one
another rather than expressed in days.

| component | tier | size | depends on |
|---|---|---|---|
| session process | GPU | large | the model choice |
| safety supervisor | rover | medium | nothing; it can be built today |
| admission gate | rover | medium | new rover hardware |
| watermark and truncation | both | medium | the session process |
| driving mode controllers | rover | medium | the safety supervisor |
| asynchronous tool protocol | both | small | nothing |
| compactor and checkpoint | GPU | medium | the session process |
| signalling and TURN | — | small | only needed if the GPU is remote |
| spatial store | GPU | large | nothing, and it can be deferred |

The session process is the one component with no counterpart in what exists
today. It holds a live context, owns a WebRTC peer connection, and runs a model
that is never invoked in the request-response sense. Everything about the shape
of [server.py](../voice_chat/server.py) — the notion of a turn, the GPU lock,
the request itself — is wrong for that job, which is why this has to be a
rewrite rather than a refactor.

The safety supervisor is worth singling out because it depends on nothing at
all. It needs no GPU, no rental, no model and no new hardware, and a version of
its deadman already exists in miniature: `drive_gamepad.py` sets the firmware
heartbeat to 500 ms when it connects, feeds it every 167 ms while it runs, and
restores the 3000 ms default on the way out, so that a script which dies leaves
the rover stopped rather than moving. Generalising that mechanism, adding the
lidar and bumper stops and the speed and acceleration clamps, and driving the
whole thing from a script is track B's first step, and it can run in parallel
with step 0 at no marginal cost.

The spatial store is both the largest of the new components and the most easily
deferred. It is also, as the architecture document argues, where most of the
product value sits, which is a reason to build it last and deliberately rather
than last and by accident.

## Which model to run

There are three candidates, and they are not the same kind of thing as one
another.

| | params | licence | duplex | VRAM | tools |
|---|---|---|---|---|---|
| MiniCPM-o 4.5 | 9B dense | Apache-2.0 | native, end to end | ~28 GB torch, 19 GB bf16, 11 GB int4 | undocumented |
| Qwen3-Omni-30B-A3B | 35B total, ~3B active | Apache-2.0 | streaming, not full duplex | 78.9 GB bf16 at 15 s of video | via the Thinker's text |
| Qwen3.5-Omni-Flash | undisclosed | not open | semantic interruption | — | native, OpenAI format |

[MiniCPM-o 4.5](https://github.com/OpenBMB/MiniCPM-o) is the candidate to
deploy, and it fits the architecture document's requirements better than the
Qwen3-Omni that document assumed, for one reason above all others: the duplex
behaviour lives inside the model rather than having to be built around it. Its
Omni-Flow framework aligns video, audio and output on a shared one-second
timeline, TAIL keeps the speech from lagging behind the visuals by adjusting the
text generation rate, and its Listen-Speak control already handles being
interrupted mid-utterance and then resuming. Between them those account for most
of the mechanism the architecture document proposes to construct by hand around
a model that lacks it. Architecturally it is SigLIP2, Whisper-medium, a Qwen3-8B
backbone and CosyVoice2 speech decoders, for 9B parameters in total, released
under Apache-2.0, and `vllm-omni` ships an experimental full-duplex realtime
runtime built specifically for it.

Its weakness is precisely the thing step 0 exists to measure. Tool calling is
not documented anywhere for this model, and while a 9B model built on Qwen3-8B
inherits a backbone that is unusually good at function calling, there is no
guarantee that the omni training run preserved that capability.

[Qwen3-Omni-30B-A3B](https://github.com/QwenLM/Qwen3-Omni) is best treated as a
quality ceiling to measure against rather than as something to deploy. It is
better than MiniCPM-o at almost everything except the two things that matter
most here, in that it needs roughly four times the VRAM — 78.9 GB against
19 GB at bf16 — and its duplex is streaming rather than genuinely full.
Upstream vLLM still serves only the Thinker, so getting audio output from it
means either the transformers path or `vllm-omni`.

Qwen3.5-Omni cannot be deployed and is nonetheless worth using. It was released
on 2026-03-30 with native function calling in the OpenAI tools format and with
semantic interruption, which is exactly the shape we want, but its open-weight
status has not been confirmed and no first-party repository exists on the Hub
today, only third-party finetunes that carry the name. It therefore cannot be
planned around. What it can do is give step 0 an upper bound, on the reasoning
that if the best omni model available through any API cannot call the rover's
ten schemas when spoken to, then no local 9B model is going to manage it either.
That measurement involves sending fifteen synthetic phrases to Alibaba's cloud
and nothing else, which is worth stating plainly given that the entire premise
of this system is that the conversation stays on a network we control.

Two smaller consequences follow from the choice. MiniCPM-o's speech decoders are
CosyVoice2, which supports voice cloning, so the loss of a known and chosen
voice that [scaling-voice-chat.md](scaling-voice-chat.md#what-it-would-cost-this-codebase)
identifies as a real cost of going omni may turn out to be recoverable, which it
would not be with Qwen3-Omni. And 9B of bf16 weights is roughly 18 GB to
download against roughly 70 GB for the 30B model, which materially changes how
much the cold-start problem hurts in practice.

## Where to run it

The provider question does not come out in AWS's favour, and the margin is not
close enough to be worth debating.

| for | card | VRAM | RunPod | AWS |
|---|---|---|---|---|
| MiniCPM-o at int4, llama.cpp duplex | RTX 4090 | 24 GB | $0.34 | not offered |
| MiniCPM-o at int4 with more headroom | RTX 5090 | 32 GB | $0.69 | not offered |
| MiniCPM-o at bf16 and Qwen3-Omni at fp8 | L40S | 48 GB | $0.79 | `g6e.xlarge`, $1.86 |
| Qwen3-Omni at bf16, 15 s of video | RTX Pro 6000 | 96 GB | $1.69 | `g7e.2xlarge`, $3.36 |
| Qwen3-Omni at 120 s of video | H200 | 141 GB | $3.59 | impractical |

Those are RunPod community-cloud rates, with secure cloud running roughly 25 to
50 per cent higher, against AWS us-east-1 on-demand.

The recommendation is to rent an L40S at $0.79 an hour, because it is the only
rung on the ladder that holds both candidates at once: MiniCPM-o in bf16 with
room to spare, and Qwen3-Omni at fp8, the L40S being an Ada part and therefore
having the FP8 tensor cores that
[scaling-voice-chat.md](scaling-voice-chat.md#the-vram-is-not-the-weights)
established the 3070 does not have. A single pod answers both questions.
Stepping up to the RTX Pro 6000 is only justified if fp8 Qwen3-Omni turns out to
be visibly worse than bf16, and that is a comparison nobody here has made.

AWS has nothing to offer this particular workload. It charges 2.4 times as much
for an identical card, spot instances are ruled out because an interrupted
conversation is a worse failure than a slow one, and there is no ecosystem
dependency that would justify paying the difference. The account is still worth
keeping for the day something needs a VPC.

Three costs are specific to RunPod. None of them is fatal and all of them are
irritating. The first is that the weights have to live on a network volume,
because a pod without one re-downloads 18 GB, or 70 GB, on every start, which
layers a cold start of the pod on top of the cold start of the session that the
architecture document already identifies as a problem. The second is that the
pod sits behind NAT, so WebRTC from a rover on a home LAN to a rented pod
requires either exposed ports or a TURN relay together with a signalling path,
whereas on a local network it requires none of that, which is a genuine argument
for doing all development locally and renting only when there is something to
measure. The third is that pods are ephemeral, so a pod that disappears takes
the session checkpoint with it unless the checkpoint is also written to the
volume.

### The card to buy has changed

[scaling-voice-chat.md](scaling-voice-chat.md#the-vram-is-not-the-weights)
concludes that choosing an omni model deletes the cheap rungs of the ladder
rather than moving along it. That conclusion was drawn against Qwen3-Omni's
78.9 GB floor and it no longer holds, because MiniCPM-o 4.5 is a 9B model rather
than a 35B one: it needs 11 GB at int4 and roughly 19 GB at bf16, and
llama.cpp-omni will run its full-duplex mode on 12 GB or more. A 5090's 32 GB
holds it comfortably in bf16, and even a used 3090's 24 GB holds it at int4.

The cheap rungs are therefore back on the list, and the rent-then-buy conclusion
survives intact with a smaller number at the end of it. At $0.79 an hour an
always-on assistant costs about $570 a month, which is the wrong shape for
hourly billing for exactly the reasons already set out.

## The hardware that is not the GPU

Three items, two of which have lead times attached.

The Pi has to be replaced, and the reason is sharper than simply saying it is
slow. The admission gate makes its decisions at VAD granularity, which means
20 ms blocks and fifty wakeups a second, and the measurement recorded in
[voice_chat](../voice_chat/README.md#the-rover-client-and-why-there-is-not-one)
is that a process waking fifty times a second breaks audio on this box while a
CPU-bound process does not. The gate is precisely the workload the Pi 1 cannot
host, and that is before encoding or acoustic echo cancellation enter the
argument at all.

There may already be an Orin available. The host `jetson-orin` was installed on
2026-08-16 from a headless NVMe image and is not currently reachable on the LAN.
Establishing which Orin it actually is settles something structural, because an
Orin Nano or NX would be the rover tier the design calls for, while an AGX Orin
with 32 or 64 GB could hold MiniCPM-o 4.5 at int4 on the rover itself. That
second possibility collapses the two-tier split entirely, and with it the
transport, the NAT problem and the rental. It is worth ten minutes of
investigation before anything else here is planned around a remote GPU.

A ReSpeaker-class microphone array is the one purchase that is certain, both for
the hardware acoustic echo cancellation that duplex operation requires and for
the direction-of-arrival information that [face_tracking](../face_tracking/)
would like to have as a prior. It should be ordered early, since it gates the
session process and the admission gate — steps 4 and 5 — and nothing else on
this list is waiting on a delivery.

## What has to be decided

The open questions, in one place rather than split across the two documents,
each priced by what it costs to answer.

* **Whether an omni model calls tools when spoken to.** The project-deciding
  one, and the subject of
  [step 0](#the-question-that-has-to-be-answered-first). Costs the harness
  plus about five dollars of inference.
* **Whether a tool call can be caught before it is spoken** in MiniCPM-o's
  duplex mode. The second half of step 0; costs nothing extra once the harness
  exists.
* **Whether one model is the right shape at all.** A single omni model
  handling both banter and deliberate perception may be worse at both than a
  fast conversational model plus a heavier VLM the modes can call. Cannot be
  argued from first principles; costs an afternoon of side-by-side on the same
  rented L40S.
* **What interruption should do to a tool call in flight.** "Abandon it" and
  "finish it silently" are both defensible, and nothing can settle it before a
  real model is in the loop at step 6 — so the session process should carry
  the policy as a switch rather than a structure.
* **Whether CosyVoice2 cloning can hold a consistent voice** across a session
  and across restarts, or whether every checkpoint recovery produces a
  different person. Matters more than it sounds, since the known, chosen voice
  is the one real loss
  [scaling-voice-chat.md](scaling-voice-chat.md#what-it-would-cost-this-codebase)
  charges against going omni. Costs an hour on the step 0 pod.
* **Whether the `_forget_*` laws are a property of `Qwen3-VL-4B` or of small
  instruction-tuned models generally**, which decides whether the compactor
  must implement them. Costs a re-run of the existing six-sample corpus
  against one other model.
* **What the admission gate actually costs.** A rover parked in an empty
  corridor should cost close to nothing per minute, but nobody has established
  what fraction of frames and seconds a real room admits, and that fraction is
  the running cost of the whole system. Costs step 5 plus a day of logging.

Finally, nothing in this document had been run when it was written. The VRAM
figures are published minimums, the prices are advertised rates, and the
tool-calling scores come from a benchmark's published tables. All of it should be
treated as a shopping list, because that is what it is.

Step 0 is the exception as of 2026-08-17 — see [omni-step0.md](omni-step0.md).
Three of its claims did not survive contact: MiniCPM-o's tool calling is
undocumented but present and good, Qwen3-Omni is not the quality ceiling but the
worst of the four candidates at this task, and the RunPod prices quoted above are
exactly right to the cent.

## The order of work

The work splits into two tracks, because the expensive question and the useful
work do not block one another. Where a step has a finish line that can be
stated, it is stated, because "built" is not a criterion.

**Track A decides. Its deliverable is the harness; the runs cost five
dollars.**

0. Build the audio-in, tool-call-out harness, synthesise the fifteen phrases
   from
   [voice_chat](../voice_chat/README.md#the-re-run-it-does-not-refuse-it-promises),
   and run them against Qwen3.5-Omni-Flash through DashScope to establish an
   upper bound, then against MiniCPM-o 4.5 and Qwen3-Omni at fp8 on a single
   rented RunPod L40S. Six samples per cell, the real daemon schemas, both
   measurements from
   [the section above](#the-question-that-has-to-be-answered-first), judged
   against the decision rule written there — before the run, not after.

**Track B builds what is worth having under every design, including the one
running today, and it can start immediately.**

1. The safety supervisor and the driving mode controllers, with no model
   involved at all. Done when a scripted drive can be killed at any point —
   process, network or power — and the rover is stationary within the deadman
   window every time.
2. The event channel on the daemon's line protocol, so that a running mode can
   report that it lost its target or hit a limit. It depends on nothing, and
   step 1 is not trustworthy without it: a mode that fails silently is exactly
   what the supervisor exists to prevent.
3. Establish which Orin `jetson-orin` is, and order the microphone array. Ten
   minutes and a purchase order, and the first answer may collapse the
   two-tier design entirely.

**The remaining steps assume track A came back positive.**

4. The session process with a fake model that simply echoes, on the LAN
   against the new rover host, no rental needed. Done when barge-in truncates
   at the played-samples watermark — the committed transcript shows what the
   user heard, not what the model generated — over a wired link and a lossy
   WiFi link both. This proves the transport half of interruption; the context
   half cannot be proven before step 6.
5. The admission gate, measured in a real room. The deliverable is a number —
   frames and seconds admitted per idle minute — because that number is the
   running cost of the whole system.
6. The real model, on a rented L40S to begin with.
7. The compactor and the checkpoint. Done when a session can run for a day,
   survive a process restart, and still answer "what did I ask you this
   morning" — coherence over hours is the bet this design makes, and no
   earlier step tests it.
8. The spatial store, last and deliberately, because it is where the product
   value sits and deserves better than arriving by accident.

The order embodies one principle: the two genuinely hard parts — duplex
truncation and a supervisor that can be trusted — do not depend on which
checkpoint wins and are miserable to retrofit afterwards, so they come first
and the model comes last.

## Sources

The model and price claims in this document, none of which have been verified
here:

* [MiniCPM-o](https://github.com/OpenBMB/MiniCPM-o) and the
  [4.5 technical report](https://arxiv.org/abs/2604.27393)
* [Qwen3-Omni](https://github.com/QwenLM/Qwen3-Omni) and its
  [technical report](https://arxiv.org/html/2509.17765v1)
* [vllm-omni](https://github.com/vllm-project/vllm-omni), for the full-duplex
  realtime runtime and the omni serving support matrix
* [TOBench](https://arxiv.org/abs/2605.16909), for how badly omni-modal agents
  score on real-world tool use, and for the fact that no benchmark speaks its
  instructions aloud
* [RunPod pricing](https://www.runpod.io/pricing) and
  [EC2 on-demand pricing](https://aws.amazon.com/ec2/pricing/on-demand/)
