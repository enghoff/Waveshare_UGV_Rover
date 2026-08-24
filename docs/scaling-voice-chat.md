# Scaling voice chat

What it would take to answer faster than the 3070 does, and to run something
larger or of a different shape than the model it runs now. Surveyed 2026-08-16;
prices are us-east-1 on-demand and rot quickly.

Nothing here is deployed. This is the shopping list and the reasoning behind it.

It is also written against a client that has since gone. Where it says `talk.py`
or `endpointing.py` below, both were deleted on 2026-08-24 when the conversation
moved onto the rover: the microphone is a page served by the rover's own console,
the realtime service decides where a turn ends, and the surviving client is
[voice_chat/session.py](../voice_chat/session.py). Nothing in the argument here
turns on which file it was.

## The number that matters is bandwidth, not FLOPs

[voice_chat](../voice_chat/README.md) serves one person at a time: one utterance,
one reply, batch size one throughout. A batch-1 decode step reads the whole active
weight set out of VRAM to produce a single token and does almost no arithmetic per
byte, so it is memory-bound. Tokens per second is roughly

```
  memory bandwidth / bytes read per forward pass
```

and the GPU's compute number barely enters. Prefill — the 300-token image from
`look`, the growing prompt — *is* compute-bound, but it is a fraction of a turn.

This is why a card is worth buying for two reasons only: **more bandwidth** (a
faster reply) and **more VRAM** (a bigger model, and no interlock). Anything sold
on TFLOPs is being sold for training.

That framing holds for a text or vision model. It does **not** survive an omni
model, whose memory is not mostly weights at all — see [An omni model is a
different shape of thing to buy](#an-omni-model-is-a-different-shape-of-thing-to-buy).

## Read this first: some of the case above was a software problem

This document was written before the turn was profiled properly, and the
profiling moved the goalposts. On 2026-08-16 the largest single cost in a turn
turned out not to be the card at all: with the rover's ten tools attached, ~1.4s
of every question was spent re-prefilling tool schemas that had not changed since
startup. Keeping the K/V cache between turns removed it, and steady-state turns
went ~3.8x faster on the same 3070 — see
[voice_chat](../voice_chat/README.md#where-it-is-now).

So the honest ordering is: **profile, then buy.** The bandwidth argument below is
still correct about decode, and decode is still ~25% of roofline. But a card is
now the second lever rather than the first, and the numbers to compare it against
are the post-2026-08-16 ones, not the ones in this document's opening.

## Where the 3070 sits

Measured, from [voice_chat](../voice_chat/README.md#measured): 46 tok/s on
`Qwen3-VL-4B-Instruct` at int4, ~5.4GB of 8GB resident with all three models
loaded, ~1.3s to first audio.

The card is 448 GB/s and the quantized weights are ~2.5GB, so the roofline is
around 180 tok/s and the service is getting **a quarter of it**. That gap is not
bandwidth — it is torchao's `tile_packed_to_4d` dequantizing on every step. Two
consequences worth carrying into any purchase:

* Bandwidth ratios **understate** the gain. A card with room for bf16 weights can
  beat int4 outright despite moving three times the bytes, because the compiled
  bf16 path has no dequant in it.
* The int4 machinery — `VOICE_INT4_GROUP`, `VOICE_INT4_PACKING`, and the
  give-ground ladder that starts at `VOICE_INT4_SKIP=` — exists because of 8GB. On
  a 24GB card it is all deletable.

## The EC2 options

| Instance | GPU | VRAM | Bandwidth | $/hr | $/mo at 24/7 |
|---|---|---|---|---|---|
| *(current)* | RTX 3070 | 8 GB | 448 GB/s | — | — |
| `g6.xlarge` | L4 | 24 GB | 300 GB/s | $0.80 | ~$580 |
| `g5.xlarge` | A10G | 24 GB | 600 GB/s | $1.01 | ~$730 |
| **`g6e.xlarge`** | **L40S** | **48 GB** | **864 GB/s** | **$1.86** | **~$1,340** |
| `g7e.2xlarge` | RTX Pro 6000 | 96 GB | ~1.6 TB/s | $3.36 | ~$2,420 |
| `p5.4xlarge` | H100 | 80 GB | 3.35 TB/s | $6.88 | ~$4,950 |

**`g6.xlarge` is the trap.** It is the cheapest 24GB card on the list and its
bandwidth is *below* the 3070's, so it would buy a bigger model and a slower one.
The L4 is a video-transcode part.

`p5` and above are eight-GPU training boxes with capacity reservations attached;
they have nothing to offer a single conversational stream.

## Why `g6e.xlarge`

It is the only rung where both axes move at once: **~2× the bandwidth and 6× the
usable VRAM**. `g5.xlarge` is the cheap version of the same idea — 1.3× bandwidth,
24GB — and is the right pick if the model size matters more than the latency.

48GB changes what can be loaded. Sizes below are computed from parameter counts,
not measured:

| Model | int4 | fp8 | bf16 |
|---|---|---|---|
| `Qwen3-VL-4B-Instruct` *(current)* | ~2.5 GB | ~4 GB | ~8 GB |
| `Qwen3-VL-8B-Instruct` | ~5 GB | ~8 GB | ~16 GB |
| `Qwen3-VL-30B-A3B-Instruct` | ~17 GB | ~30 GB | ~60 GB |
| `Qwen3-VL-32B-Instruct` | ~18 GB | ~32 GB | ~64 GB |

**The 30B-A3B is the interesting one.** It is a mixture of experts: 30B total
parameters but only ~3B active per token, and decode reads only the active set. So
its bytes-per-forward-pass is in the same class as the 4B dense model running now,
while its answers are a tier better. The dense 32B fits in the same VRAM and would
decode roughly five times slower for it. If the point of the exercise is "bigger
model, not slower", MoE is the shape to buy.

Those are parameter counts and nothing else. An omni model of the same nominal
size needs far more than the row it would sit in, for reasons that are not about
the weights — the next section is the correction.

The other 48GB win is not about the model at all: `switch_service.sh` stops being
necessary. The interlock exists because `voice`, `dino` and `qwen` cannot coexist
in 6.5GB — see [Running it](../voice_chat/README.md#running-it). At 48GB they all
sit resident and the card stops being a thing to schedule.

## An omni model is a different shape of thing to buy

One model takes the microphone and gives back speech, so `distil-large-v3` and
Kokoro stop existing as stages. `Qwen3-Omni-30B-A3B-Instruct` is Apache-2.0 and is
the same Thinker–Talker MoE shape argued for above — 35B total, ~3B active — and
takes text, images, audio and video **interleaved in one turn**, which is exactly
the shape of a `look` turn: a spoken question about a picture. Its cookbooks
include audio-visual question answering and an audio function call.

**The latency case is weak, and it is worth writing down why so nobody re-makes
it.** STT and TTS are 0.32s of a 1.3s turn, and STT is already hidden behind
`HANG_MS` by the speculative transcription in `endpointing.py` — see
[Transcribing before the turn
is over](../voice_chat/README.md#transcribing-before-the-turn-is-over). So what an
omni model recovers is Kokoro's 0.14s. That is not a reason to buy anything.

Two things it buys that no card can:

* **The transcript stops being a lossy stage.** Today the model reads Whisper's
  text and separately sees a JPEG. An omni model hears the question and sees the
  picture in one representation, so a mis-heard name or a question whose meaning
  was in its tone stops being lost before the model ever gets it.
* **Barge-in.** The microphone is muted while the reply plays — that is what the
  `○ speaking` state in `talk.py` is. Full duplex is the only way that changes,
  and it is a property of the model, not of the client.

### The VRAM is not the weights

Published floors, not computed ones:

| Model | Params | bf16 | int4 | Stated Nvidia minimum |
|---|---|---|---|---|
| `Qwen3-Omni-30B-A3B-Instruct` | 35B / ~3B active | 78.9 GB † | — | — |
| `MiniCPM-o 4.5` | 9B dense | 19.0 GB | 11.0 GB | 12 GB, half-duplex speech |
| `Qwen2.5-Omni-3B` | 3B dense | 18.4 GB | — | ≥18 GB |

† Thinker + Talker, 15s of video, transformers with `flash_attention_2`. That is
the *shortest* row Qwen publishes; 120s of video is 144.8 GB.

**Look at the 3B row.** Three billion parameters in bf16 is 6 GB of weights, and
the floor is 18.4 GB. The missing ~12 GB is the audio and vision encoders, the
Talker, and the streaming codec — and none of that shrinks when the LLM is
quantized. The give-ground ladder that makes a 4B vision model fit 8 GB today does
not transfer, because the parts that would have to give are the parts that grew.

Two purchase consequences follow, and they are not small:

* **78.9 GB does not fit the 48 GB `g6e.xlarge` recommended above.** fp8 does —
  ~35 GB of weights plus the encoders — and the L40S is Ada, which *has* FP8
  tensor cores. That is worth stating plainly because the note under
  `VOICE_LLM_QUANT` says the opposite about this card: Ampere has none, which is
  why the official FP8 checkpoint is useless on the 3070 and useful on an L40S.
* **Ampere is out at any size.** The `g5.xlarge` / used-3090 route — 24 GB, no
  FP8 — cannot hold a 35B omni model in any precision anyone has published. So
  choosing omni deletes the cheap rungs of the ladder rather than moving along it.
  A 5090's 32 GB is arguably reachable at int4 if such a checkpoint appears; that
  is arithmetic, not a measurement, and no int4 omni checkpoint exists today.

One more thing the table does not show: vLLM's audio *output* is not shipped for
`Qwen3-Omni` as of this survey, so the transformers path is the only one — which
is the slow path, and the one everything below is about.

### What it would cost this codebase

None of this is a config change, and three of the four are things the service
already learned the hard way:

* **`VOICE_PREFIX_CACHE` is the single largest win in the service** and it is a
  hand-rolled `StaticCache` rewind — `cumulative_length.fill_(keep)`, ids recorded
  after `generate` returns. A Thinker–Talker pair is two models with two caches.
  There is no obvious reason it cannot be done, and it is certainly not free.
* **Every tool-calling measurement transfers to nothing.** The README already says
  this about the text→vision swap and it is more true here: whether a model acts
  on *"can you turn the lights off"* was a sampled decision on `Qwen3-4B`, and the
  `look` wording was found against its nine neighbours. Six samples a cell, again,
  on a model family whose function-calling is far less exercised.
* **Compile shapes multiply.** [The first picture
  compiles](../voice_chat/README.md#the-first-picture-compiles) cost ~52s of extra
  warm for *one* extra graph. Omni adds audio-in and speech-out to that set, on
  top of a cold start already near 150s, and the service does not bind its port
  until it is warm.
* **Kokoro is 0.14s and a known voice**, selected by `VOICE_TTS_VOICE`. An omni
  Talker is neither, and its voice is whatever the checkpoint shipped with.

### Testing one before renting anything

**Nothing omni fits the 3070.** The card has ~6.5GB free once Windows has taken
its share and the smallest published int4 floor is 11 GB, so even stopping every
other service does not get there.

What can be run there is a `MiniCPM-o 4.5` GGUF at Q4 through llama.cpp with
layers on the CPU. Timings from that are meaningless — different model, different
runtime, half of it on a CPU — but the question worth asking first is not a timing
question. It is the one [`/chat`](../voice_chat/README.md#getting-it-to-actually-call-them)
exists for: **does an omni model call the rover's ten schemas at all**, six
samples a cell. MiniCPM-o's tool calling is undocumented, so a "no" there is free
and ends the idea.

The better order, though, is to skip that. MiniCPM-o is not the model that would
be deployed, and **two hours of `g6e.xlarge` is $3.72** — less than the afternoon
the llama.cpp detour costs, against the model that actually is the candidate. Run
the four `look` questions and the four tool questions from the README's tables,
compare cells, and decide.

## What moving off the LAN costs

Two round trips get added to a turn that currently measures 1.3s to first audio:

* **The utterance.** 16kHz s16le, so a two-second question is ~64KB. Negligible on
  any upstream, and it is already crossing a network — the microphone is not on the
  GPU host today either.
* **The picture.** `POST /frame` sends one JPEG from the rover when the model calls
  `look`. A few hundred KB on a home upstream is tens of milliseconds; on a slow
  one it is the only part of this worth measuring first.

Call it 50–150ms added, against a ~700ms saving on decode. Real, and small enough
that it does not change the recommendation — but it is the number to check before
committing, because it is the one that cannot be fixed by spending more.

An omni model changes the first bullet and not the second: the utterance stops
being transcribed at the far end and starts being *the input*, so the audio is on
the critical path rather than beside it. At 64KB for two seconds that is still
nothing, but it is no longer hidden behind `HANG_MS` the way STT is today.

## Two operational notes

* **Cold start.** The service does not bind its port until it is warm, and a first
  start downloads ~10GB of weights at roughly one file per five minutes
  unauthenticated. Stop/start on EC2 must keep the HF cache on a persistent EBS
  volume, or bake it into an AMI. Otherwise every start pays the download on top of
  the ~60s warm. An omni checkpoint is a larger download and more shapes to warm,
  so this gets worse rather than better.
* **Not spot.** An interrupted instance ends a conversation mid-sentence and costs
  the full cold start to come back. The interruption discount is not worth it for
  something meant to answer when spoken to.

## Rent or buy

The honest conclusion: **`g6e.xlarge` only wins if this is bursty.**

At 24/7 it is ~$1,340/month. A used 3090 is 24GB at 936 GB/s — more bandwidth than
the A10G — for roughly one month of that instance, once. A 5090 is 32GB at
~1.8 TB/s and beats everything on the list below `g7e`, for about two months of it.

So:

* **Renting** is right for deciding *which* model to run. That is now three
  questions rather than one — is `30B-A3B` actually better here, does an omni
  model call the rover's tools, and is full duplex worth anything to a rover — and
  a few days on a `g6e.xlarge` answers all three without buying anything. The
  answers determine how much VRAM to pay for, and the omni answer determines
  whether the cheap rungs are even candidates.
* **Buying** is right for the steady state. This is an always-on assistant on a
  LAN, which is the worst possible shape for hourly cloud billing.

The path that wastes least: rent `g6e.xlarge` long enough to pick the model, then
buy the smallest card that holds it. Settle the omni question during that rental
rather than after it — it is the one answer that changes *which cards are on the
list*, and finding it out after a 3090 has been bought is the expensive order.
