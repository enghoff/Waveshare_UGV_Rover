# Voice chat

Speech in, speech out. The models run on the MEDIA GPU host; the microphone and
speakers stay on whatever machine you are sitting at.

```
  the machine you are sitting at        root@media  (RTX 3070, 8GB)
  ------------------------------        ---------------------------
  mic -> VAD/endpointing --16k s16le-->  faster-whisper distil-large-v3
                                              |  text
                                         Qwen3-VL-4B-Instruct (int4)
                                              |  sentences, streamed
  speakers <-- playback <--24k s16le---  Kokoro-82M

  the rover ------------ one JPEG, POST /frame -----------^
     (only when the model asks to look; never via the desk)
```

One client, [talk.py](talk.py), wherever there is a microphone. Endpointing —
deciding the speaker has stopped — is in [endpointing.py](endpointing.py) beside
it rather than on the GPU; see below for why.

It also carries **tools**: the model can switch the headlights, aim the camera,
look through it, count the people it can see and start or stop face tracking.
None of that happens here or on the desk — it is performed by
[rover_daemon/](../rover_daemon/) on the rover, which owns the hardware. See
[Tools](#tools) and [Seeing](#seeing).

## Why it is split here

Endpointing — deciding the speaker has stopped — is the one piece that could
live on either side, and it goes local. It needs no model, and doing it locally
means silence never crosses the network: the link only ever carries a real
utterance and a real reply. The client therefore needs no torch, no CUDA and no
model weights, just `sounddevice`, `numpy` and `websockets`.

Everything downstream of that is one process rather than three services, because
a turn is a strict chain (audio → text → text → audio) with nothing to overlap
between stages. Splitting it would buy no parallelism and cost two extra network
hops plus two extra CUDA contexts on a card that has ~6.5GB free once Windows
has taken its share.

The one thing that *is* overlapped is TTS against decode. The reply is split into
sentences as it decodes and each is spoken as soon as it is complete, so audio
starts after the model's first sentence rather than its last.

## Measured

Warm, on the 3070, for a two-second question answered in three sentences:

| stage | time |
|---|---|
| STT (`distil-large-v3`, int8_float16) | 0.18s |
| LLM (`Qwen3-4B`, int4, compiled) | 0.6–0.9s, 46 tok/s |
| TTS (`Kokoro-82M`) | 0.14s |
| **first audio out** | **~1.3s** |
| whole turn | ~1.7s |

~5.4GB of the card's 8GB, all three models resident. Cold start is ~60s and the
service does not bind its port until it is warm, so `/health` answering means
the next turn is a fast one.

The first turn used to cost 56s and the second 76s: `torch.compile` was
rebuilding for every new prompt length, and a conversation's prompt grows every
turn. `VOICE_COMPILE_DYNAMIC=1` compiles once for a range of lengths instead, and
`_load()` spends that cost at startup rather than on the first question.

## Running it

The service shares the card with `grounding-dino` and `qwen3-vl` and is not meant
to run alongside them. Switch with the interlock, which now has three options:

```bash
ssh root@media ~/switch_service.sh voice     # or: dino, qwen
```

(That script was `switch_vision_service.sh` until this landed — renamed because
voice-chat is not vision, and what the set has in common is the card. Its source
of truth is `services/switch_service.sh` in the **mt4** repo, not this one.)

Then, from the machine with the microphone:

```bash
python voice_chat/talk.py
```

Just talk; it endpoints on its own. `Ctrl-C` to quit. `--list-devices` and
`--input-device N` / `--output-device N` if it picks the wrong hardware.

One line at the bottom says what the microphone is doing, rewritten in place:

```
● listening     open, room tone only
● hearing you   the endpointer has decided a turn is under way
○ thinking      utterance sent, waiting on the card
○ speaking      the reply is playing, and the mic is muted so it does not
                endpoint the assistant's own voice
```

The filled half is an open microphone and the hollow half a closed one. Both
closed states are deliberate and neither is visible any other way — a muted
client looks exactly like a dead one — which is the whole reason the line
exists. It is drawn only on a terminal: redirected to a file it is silent, and
where the encoding cannot take the dots (a piped stdout on Windows is cp1252)
it falls back to `[ listening ]`.

If the service is not up — which is the usual state, since the card is shared —
the client says which host it tried and how to start it, rather than a `websockets`
traceback. It distinguishes the four cases that need different answers: a name
that will not resolve, a port that refuses (which is also what a service still
loading its weights looks like), a host that answers nothing at all, and
something on the port that is not this. A connection lost mid-conversation —
a restart, or the card being switched away — ends the same way, with a line
rather than a stack trace.

Client dependencies: `pip install -r voice_chat/client-requirements.txt`.

The service binds `0.0.0.0` rather than loopback, so there is no tunnel. The
microphone is on a different machine from the card and a tunnel between them is a
process and a reconnect loop sitting in the middle of a conversation. The port is
open on the LAN with no authentication in front of it, the same as `face-detect`
beside it; the Hyper-V inbound rule covers 8765-8774. Do not do this on a network
you do not own.

## The rover client, and why there is not one

There was a second client on the rover itself — Bluetooth headset in, JBL Flip
out, PipeWire driven through `pw-record`/`pw-play` pipes because the Pi has no
PortAudio, and a hand-rolled RFC 6455 WebSocket because `apt` there needs a
password we do not have from a script. It worked, and it was never reliable
enough to hold a conversation through: the Pi 1 runs its Bluetooth dongle, its
wifi dongle and the camera off one weakly fused USB bus, and an always-open SCO
microphone alongside A2DP is more than that radio comfortably does.

It was removed on 2026-08-15 along with `wsclient.py`. Speech now happens only
where there is a desk and a real microphone; what the rover still does is
everything in [rover_daemon/](../rover_daemon/), which needs no audio at all.

Two findings from that work outlive the client and are kept, because they are
properties of the machine rather than of the code that went:

- **PipeWire on the Pi had no realtime priority**, because `admin` was not in the
  `pipewire` group and so the `rtprio 95` in
  `/etc/security/limits.d/25-pw-rlimits.conf` never applied. Fixed with
  `usermod -aG pipewire admin` and a reboot. See [docs/hosts.md](../docs/hosts.md).
- **A process waking 50 times a second breaks audio on that box, and a CPU hog
  does not.** A deliberate spin loop cost 0–2 dropouts; a 20 ms read loop cost 36
  in 15 seconds. Throughput the scheduler handles, latency it does not — so
  anything on that Pi that reads a pipe should read it in bulk. That is why
  `track_face_pi.py` forwarding whole frames is fine and why its 4 kB read chunk
  would not have been.

## Tools

The rover can be asked to do things, not just talked to: *turn the lights on*,
*dim them a bit*, *look to your left*, *how many people can you see*, *follow
that person*, *find somebody else*, *stop following*.

**Nothing here performs them.** The rover's hardware is a single UART and a
single camera, so exactly one process may own it, and that process is
[rover_daemon.py](../rover_daemon/rover_daemon.py) on the Pi. A call travels
from the model, back down this WebSocket to `talk.py`, and on to the daemon:

```
  you --speech--> MEDIA: model decides ---{"type":"tool"}--> talk.py
                         model answers <-{"type":"tool_result"}-  |
  speakers <--audio--                                             | TCP 8769
                                              rover_daemon.py <---+
                                              UART -> ESP32,  camera -> MEDIA
```

**Nobody but the daemon knows what the tools are.** `talk.py` asks it with
`list_tools` on connect and passes the schemas straight up in a `hello`; this
service puts whatever it is given into the prompt. So `server.py` contains no
mention of lights, cameras or serial ports, `rover_tools.py` is ~120 lines of
socket and no schemas at all, and adding a tool is a change to the daemon alone
— nothing else is redeployed. A client that announces nothing gets a plain
conversation with no rover in its context, which is what `--rover none` is for.

The daemon is probed once at startup and the tools are offered only if it
answers. Tools that cannot reach the rover are worse than no tools: the model
says out loud that it has switched the lights on, and nothing happens.

Measured over the LAN, answering with a stub board rather than the rover, so
these are the model's costs with no serial time in them:

| said | tool | first audio |
|---|---|---|
| "Please switch the lights on." | `set_lights{"level":255}` | 2.90s |
| "Are the lights on right now?" | none — answered from history | 1.60s |
| "Dim them to about half." | `set_lights{"level":128}` | 3.24s |

So a tool turn costs about **1.3–1.6s more** than a plain one — a whole second
decode, not the round trip, which is milliseconds. The card is held across that
round trip rather than released, because letting another turn interleave between
a call and its result is worse than leaving the GPU idle for a moment.

Note the middle row: it answered "are they on?" from the previous exchange
instead of calling `get_lights`, which is sensible and also means `get_lights` is
rarely reached. A conversation will therefore keep asserting whatever it last
set, so if the gamepad has moved the lights since, the answer is confidently
wrong. That is the same staleness as the tool's own, one step further removed.

### Getting it to actually call them

The first version of this shipped a rover that lied. Asked *"can you switch the
lights off?"* it answered *"I'm turning the lights off. The headlights are now
off."* and called nothing — which is indistinguishable from working, right up
until you look at the rover. Two independent causes, and it took both fixes.

**The wording of the tool prompt.** The original said to use the tools "when you
are asked to do something they cover", and the model read a question as a
question. Measured over six samples a cell:

| request | original prompt | current prompt |
|---|---|---|
| "Hello, can you switch lights off?" | **0/6** | 6/6 |
| "Can you look to your left?" | **0/6** | 6/6 |
| "Start following me." | 3/6 | 6/6 |
| "What is your name?" | 0/6 | 0/6 *(want 0)* |

Zero out of six, at every temperature. What fixed it was saying two things
explicitly: that a request phrased as a question is still a request, and that it
has done something *only* if it called a tool — never to claim it switched, moved
or started anything otherwise.

**And the temperature.** Whether the model acts turned out to be a sampled
decision, not a determined one: at 0.7, *"Start following me"* called 3/6. See
`VOICE_TEMPERATURE` in [server.py](server.py) for the table; the default is now
0.2. Nothing over-calls at any temperature, so this costs only variety.

Both of those were found through `/chat`, which exists for exactly this: text in,
the *decision* out, no speech synthesised and no tool performed. A spoken attempt
costs a TTS round trip, a decode and a playback — nine seconds to learn one bit,
and a bit that is noisy enough that three samples will mislead you. It did:
mid-investigation a 3-sample spoken test said the new prompt had made things
worse, and 6-sample text runs said it had taken two cases from 0/6 to 6/6. Do not
draw conclusions here from single attempts.

```bash
curl -s -X POST http://media.local:8767/chat -H 'Content-Type: application/json'   -d '{"text": "can you turn the lights off", "tools": [...], "temperature": 0.2}'
# -> {"reply": "...", "tool_calls": [{"name": "set_lights", "arguments": {"level": 0}}]}
```

`system` and `temperature` are overridable per request, which is the point — a
restart to try one wording costs ~150s, and this way a four-prompt sweep is one
script.

Four things about this end are not obvious:

- **A tool call is text, and Kokoro will read it out loud.** It has to be caught
  before the sentence splitter, which is what `_ToolSniffer` in
  [server.py](server.py) does. It watches for two shapes, because which one
  arrives depends on the *tokenizer*, not the model: Qwen wraps a call in
  `<tool_call>` markers, and a streamer built with `skip_special_tokens=True`
  could eat them before this ever sees them, leaving a bare JSON object as the
  whole reply. Measured on the deployed tokenizer, the markers **do** survive —
  they are not special tokens in Qwen3-4B-Instruct-2507 — but both are handled,
  since that is a property of a model file rather than of this code.
- **The markers arrive in sub-word pieces.** `<tool` and `_call>` is an ordinary
  way for one to turn up, so the sniffer holds back any trailing fragment that
  could still become a marker. Getting this wrong speaks the word "tool" and
  swallows the rest of the reply.
- **The last decode of a turn is offered no tools.** Otherwise a model that has
  decided everything is a tool call spends the whole turn calling them and the
  user hears nothing, which is indistinguishable from a crash.
- **`_trim` cuts whole exchanges**, since a turn that called a tool is four
  messages rather than two, and a call stranded without its result makes the
  model start narrating plumbing.

## Seeing

With `VOICE_VISION=1` the reply model is `Qwen3-VL-4B-Instruct` and the rover can
be asked what it can see. The interesting part is not the model, it is where the
picture goes:

```
  rpi (the camera)                desk (talk.py)              media (this)
  ----------------                --------------              ------------
                       <--{"call":"look"}--  tool call  <--{"type":"tool"}--
  one MJPEG frame ------------ POST /frame ------------------------> held as
       (~35 kB)                                                     "frame-7"
                       --{"ok":true,"image":"frame-7"}--> tool_result -->  |
                                                        claimed by the turn |
  speakers  <--------- "A desk with two monitors on it." <-----------------+
```

**The image never touches the client.** It goes straight from the rover to this
card, the same road [face_detect](../face_detect/) frames already take, and what
crosses the desk is the *name* it was filed under, in an ordinary tool result.
That is why the machine with the microphone needs no new code at all: `talk.py`
is unchanged, because to it `look` is a tool like `set_lights`.

Frames are decoded at `POST /frame` rather than at the point of use, so the
half-frame a just-opened camera gives is reported to the rover — which can take
another one — instead of failing in the middle of somebody's sentence. A frame
is held under its name for 60s and at most four at a time; a name is claimed
once, so the same picture cannot be shown twice.

No picture outlives the turn that took it — see [A picture does not outlive the
turn that took it](#a-picture-does-not-outlive-the-turn-that-took-it), which is
also where a frame lives and for how long. What one costs in the context window
is measured at startup against a frame of the configured size rather than
assumed, because that number decides when history is trimmed, and a wrong
constant there fails silently: too low and the prompt overruns the static cache
and quietly falls back to the dynamic one.

`look` is offered by the rover, not by this service — as with every other tool,
nothing here knows what a rover is. The daemon adds it only when started with
`--vision`, since a picture with nowhere to go is a tool that can only fail.

### Rolling it back

Two independent switches, and neither needs a deploy:

```bash
# the model: text again, on the box
ssh root@media 'sed -i "s/^Environment=VOICE_LLM_MODEL=.*/Environment=VOICE_LLM_MODEL=Qwen\/Qwen3-4B-Instruct-2507/; s/^Environment=VOICE_VISION=.*/Environment=VOICE_VISION=0/" /etc/systemd/system/voice-chat.service'
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'

# the tool: drop --vision from the rover's crontab line, then
ssh rpi 'pkill -f ugv/rover_daemon.py'      # run_daemon.sh restarts it
```

With `VOICE_VISION=0` this is the text service it always was: no processor is
loaded, `/frame` answers 409 with a sentence saying why, and no message in a
conversation can hold an image. Both models stay in the HF cache on the box, so
the swap either way costs a restart (~150s) and no download. `curl
media.local:8767/health` reports `vision`, and the model it is actually running.

If it starts but does not fit — the card has ~2.1GB free with the text model
loaded and the vision one wants most of that — the order to give ground in is
`VOICE_INT4_SKIP=` (quantize the vision tower too, ~0.6GB, and prefill on a
frame goes 0.45s → 0.90s), then `VOICE_CACHE_LEN=2048`, then a smaller
`VOICE_STT_MODEL`.

**The tool-calling measurements below were taken on Qwen3-4B-Instruct-2507 and do
not transfer for free.** Whether a model acts on "can you turn the lights off"
was a sampled decision on that one, and the prompt wording that fixed it was
found through `/chat`. Re-run that sweep against the vision model before
trusting it — six samples a cell, not three.

### Getting it to call `look` — a tool is read against its neighbours

The first version of this shipped a rover that would not look. Asked *"what can
you see right now?"* it answered **0/6** with the tool sitting right there, and
said instead that it could not see anything because it had not taken a picture —
which is the failure this whole path exists to remove, arriving one step later
than before. Six samples a cell at temperature 0.2, through `/chat`:

| request | as first written | `look` reworded | + `count_faces` reworded |
|---|---|---|---|
| "What can you see right now?" | **0/6** | 6/6 | **6/6** |
| "Can you describe what is in front of you?" | **0/6** | 5/6 | 3/6 |
| "How many people can you see?" | **nothing at all** | nothing at all | **6/6** `count_faces` |
| "Is there anybody there?" | 6/6 `count_faces` | 6/6 | 6/6 `count_faces` |
| "Please switch the lights on." | 6/6 `set_lights` | 6/6 | 6/6 `set_lights` |

Three system-prompt wordings were tried first and all three scored 0/6, which is
what makes the finding worth writing down: **the prompt was not the variable.**
The same schema *alone* scored 6/6 on every phrasing. It was the other nine that
suppressed it — a model does not read a tool on its own, it reads it against its
neighbours, and "take a photograph through the camera and look at it" lost to a
list already full of looking. Dropping `look_at` recovered one phrasing and not
the others, so the name is part of it and not the whole of it.

What fixed it was saying what the tool *is* rather than what it does — "this is
the only way you can see anything at all" — and, for the same reason, pointing
the counting question at the tool that counts. That third column also fixes a
bug that predates any of this: `count_faces` opened with *"Look through the
camera once…"*, and beside a tool that actually looks it stopped being called at
all, so **"how many people can you see" called nothing**. It had presumably been
answering that question by luck for as long as the wording had a monopoly on the
word "look".

One weak cell is left, honestly: *"can you describe what is in front of you"* is
3/6, and the wording that made it 5/6 is the one that costs the people question.
Do not tune it on a single attempt — that is the lesson recorded above.

### A tool result is context, and an instruction in it is an order

The result of `look` came back as `{"ok": true, "image": …, "note": "the picture
is in front of you; describe what is actually in it"}`. That note was written as
a comment. It is not a comment — it is a sentence handed to the model
immediately before the picture, on **every** look, and the model read it as the
instruction for the turn. So every follow-up got a fresh photograph and a
description of the whole scene, whatever had been asked:

| turn | with the note | without it |
|---|---|---|
| "What's in the picture?" | looks 3/3 | looks 3/3 |
| "Describe the shapes." | **looks 3/3** | 0/3 — answers from the picture it has |
| "What colour is the one on the left?" | **looks 3/3** | 0/3 |
| "Look again and tell me what is there." | looks 3/3 | looks 3/3 |

Removing it fixes all of that and costs nothing: it still looks when it has no
picture, and still looks again when asked to. What the model should *do* with a
picture belongs in the system prompt, where it is said once — the tool result
should carry facts and nothing else.

Two attempts to also fix the remaining wart — asked to describe something in a
picture it is holding, it sometimes answered "I need to take a picture to see" —
are worth recording because both were worse:

- **Putting the missing fact in the result instead** (*"this picture stays in
  front of you"*): identical numbers, no improvement to the wart. Not kept.
- **Rewriting the system prompt** to say a picture stays and that saying you
  will look is not looking: took the first look from **3/3 to 1/3** and produced
  *"I took a picture to show what's in front of me"* from a model that had taken
  none. A prompt that talks about the act of looking gets the act narrated
  instead of performed — the same failure as the rover that said it had switched
  the lights on, and the reason that clause exists in the tool prompt at all.

What did help was smaller: deleting *"You have no eyes of your own"* from the
front of the vision prompt, which the model had been reciting verbatim as its
reason for not answering. Description follow-ups now answer from the picture —
*"The circle is red and solid, positioned to the left of the square"* — and a
question about something that is genuinely not there is answered from the
picture too, rather than with an apology about cameras.

Under all of it the plumbing was never the problem: `look` posts a frame in
**0.8s cold, 0.1s warm**, and end to end through the rover's own camera a turn
runs **~8.3s** — *"I see a living room with two black leather sofas, a glass
table, and a dining area in the background."*

### A picture does not outlive the turn that took it

The camera is on a gimbal that sweeps while face tracking runs — mid-test it sat
at `pan: -49, tilt: 44` and two consecutive photographs showed a balcony and
then a living room. A picture from the previous turn is therefore a picture of
somewhere the rover is no longer pointing, and answering from it is answering
about the past in the present tense. So each new utterance drops every picture
before the model sees it (`VOICE_FRESH_PICTURE=1`, the default), and a turn that
needs to see takes its own frame. It costs ~2s a turn; set it to `0` to let one
picture answer follow-ups instead.

Dropping the picture is not enough on its own, and the two ways that fail are
worth knowing, because both leave the model *narrating* the tool instead of
calling it:

| what is left where the picture was | "what colour is the floor?" |
|---|---|
| "(a picture… which you can no longer see)" | 0/3 — *"I can't, I don't have a picture of it"* |
| "(…it is gone, so take another if you need to see)" | 0/3 — *"I need to take a picture first"* |
| nothing, and the `look` call removed with it | **3/3 — takes one** |

A `look` call whose picture has been taken away is a **stranded call**, and the
model reads it as having already looked. `_trim` has always cut whole exchanges
for exactly this reason; the same rule now applies to a picture that is
withdrawn. What is left in the history is the transcript of what was said aloud,
which is all a later turn needs.

Measured over three fresh conversations, every visual follow-up now takes its
own photograph: *"what colour is the floor?"* 3/3, *"is there anything on the
table?"* 3/3, after a first *"what's in the picture?"* at 3/3.

**Where a picture actually lives.** Posted to `/frame` it sits in a stash of at
most four, for at most 60s, until the turn claims it by name — and a name is
claimed once. From there it is one decoded image in that conversation's history,
in memory, dropped at the start of the next turn. `hello`, `reset` and a closed
socket drop it too. **Nothing is written to disk at any point**, on either host.

Two warts, measured and left in rather than smoothed over. A question about
something that is not in front of the camera — "describe the person" with nobody
there — is answered with a refusal rather than a look, and those refusals then
make the *next* few turns refuse as well: the model reads its own "I can't" back.
And a late "what can you see now?" is sometimes answered from the transcript,
repeating an earlier description word for word, 0/3.

### The first picture compiles

A turn carrying an image is a different compile from a text one, not a warmer
version of the same one — Qwen3-VL positions image tokens with 3D rope, so the
decode step is a new graph. The first such turn therefore recompiles, and that
is not a few seconds: measured, one turn sat there writing **2450 inductor
artifacts** with the GPU at 1% and the streamer gave up at 180s before it
finished, in the middle of somebody asking what the rover could see.

So `_load` warms **both** shapes, the same argument as everything else it warms:
the only choice is whether the person waiting for a compile is a person. It
costs ~52s on top of the text warm, and the service still does not bind its port
until it is done. A turn that produces nothing for `VOICE_STREAM_TIMEOUT` now
says so in those terms rather than raising `queue.Empty` from inside
transformers.

## Protocol

WebSocket at `/ws`. Client → server: binary frames of 16kHz mono s16le, then
`{"type":"end"}` to close the utterance; `{"type":"reset"}` clears history.
Optionally `{"type":"hello","tools":[…]}` first, with OpenAI-style function
schemas, announcing what this client can perform; it is answered with the names
that were accepted and clears any history, since the tools on offer are part of
what the model was told.

Server → client, all JSON except the audio:

| event | meaning |
|---|---|
| `{"type":"stt","text":…}` | what it heard (`"empty":true` if nothing) |
| `{"type":"start","rate":24000}` | reply beginning, at this sample rate |
| `{"type":"text","text":…}` | one sentence, followed by **one binary frame** of its audio |
| `{"type":"tool","id":…,"name":…,"arguments":{…}}` | perform this and answer |
| `{"type":"done","stats":{…}}` | turn over, with `stt_ms` / `first_audio_ms` / `total_ms` / `tools` |

A tool call is answered with `{"type":"tool_result","id":…,"result":{…}}`, the
result being whatever JSON object the model should see — including a failure, as
`{"ok":false,"error":…}`, which it will paraphrase out loud. A client that does
not answer within `VOICE_TOOL_TIMEOUT` (5s) gets that reported to the model as a
rover that did not respond, rather than the turn failing.

`GET /health` is what the switcher polls. `POST /say?text=…` returns raw PCM for
checking a voice without holding a conversation. `POST /frame` takes one JPEG
from whoever holds a camera and answers with the name it is held under, which is
what a `look`-shaped tool result carries back — see [Seeing](#seeing).

## Tuning

Server knobs are environment variables in
[voice-chat.service](voice-chat.service); the reasoning behind each default is in
the comments in [server.py](server.py). The ones worth knowing:

- `VOICE_LLM_MODEL` / `VOICE_VISION` — which model answers, and whether it can be
  shown a picture. They go together: vision on with a text model refuses to
  start rather than serving a rover that describes rooms it never saw. See
  [Seeing](#seeing) for the rollback. 8B at int4 fits (~5GB) but leaves no room
  for Whisper's context to grow; try it only with the vision services stopped.
- `VOICE_COMPILE=1` — compiled decode, ~46 tok/s against ~12 uncompiled. Falls
  back to eager if compilation fails rather than refusing to start; `/health`
  reports which it got. Note this service does *not* ask for `reduce-overhead`
  the way `qwen3-vl.service` does: inductor skips CUDA graphs here anyway
  ("mutated inputs"), because the static cache is written in place.
- `VOICE_COMPILE_DYNAMIC=1` — compile once for a range of prompt lengths. Turning
  this off costs a ~60s recompile on *every* turn of a conversation, since the
  prompt grows each time. Only worth it if the prompt length is somehow fixed.
- `VOICE_STREAM_TIMEOUT=180` — how long a turn may produce *nothing* before it is
  called a failure. Not a budget for slow decoding: tokens arrive every ~25ms
  once they start, and the only thing that takes minutes before the first one is
  a compile, which `_load` is meant to have paid for both shapes already.
- `VOICE_CACHE_LEN=2048` — the static-cache window, ~12 spoken turns (3072 with
  vision, where a picture takes ~300 of them). History is trimmed a whole
  exchange at a time to fit it.

  **This did nothing at all until 2026-08-15.** `apply_chat_template(tokenize=True)`
  returns a `BatchEncoding` on transformers 5, not a list of ids, so `len()` of it
  is **2** — the number of keys — and every "does this fit the cache" test read
  `2 <= 1856` and said yes. No history was ever trimmed. Nothing failed, which is
  why it went unnoticed: an overlong prompt falls through to the dynamic cache in
  `_generate` instead, so a long conversation got quietly slower and lost the
  compiled decode path rather than erroring. `_prompt_len` now unwraps the
  encoding, and `selftest.py` checks the token count itself rather than only the
  trimming built on top of it. Anything that measures a prompt should assume this
  return type has changed under it.
- `VOICE_TTS_VOICE=af_heart` — any Kokoro voice. The first letter picks the
  language pack, so keep it consistent with the language you are speaking.
- `VOICE_SYSTEM_PROMPT` — constrains replies to short spoken English. Without it
  the model emits markdown and Kokoro reads the punctuation aloud.
- `VOICE_TOOL_PROMPT` — appended to that, but only when a client has announced
  tools. Without it the "if you do not know something, say so" line wins and the
  model explains that it cannot reach hardware it is in fact holding.
- `VOICE_MAX_TOOL_CALLS=2` / `VOICE_TOOL_TIMEOUT=5` — how many calls a turn may
  make before it has to answer in words, and how long the rover gets to perform
  one. The timeout is not a work budget — a call is a JSON line down a UART — it
  is how long to wait before telling the model the rover is not answering.

Client knobs are constants at the top of [endpointing.py](endpointing.py):
`SPEECH_FACTOR` and `SPEECH_FLOOR` for sensitivity, `HANG_MS` for how much
silence ends a turn.

Endpointer timing is counted in 20ms blocks rather than wall-clock seconds —
`sounddevice` delivers in bursts after a scheduling hiccup, and a clock reads
that burst as a long silence and ends the turn mid-sentence.

## Checks

`selftest.py` covers the pieces where a bug is silent rather than loud: the
sentence splitter, the endpointer, history trimming, the sniffer that keeps a
tool call from being spoken aloud, and the line to the rover daemon. What the
daemon *does* with a call has its own checks, in
[rover_daemon/selftest.py](../rover_daemon/selftest.py), which run on the rover.
It has no GPU or microphone dependency, and each part skips where its
dependencies are absent, so run it anywhere:

```bash
python voice_chat/selftest.py               # endpointer + the rover client
ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py
ssh rpi 'cd ugv && python3 selftest.py'     # the daemon's own, on the rover
```

## Deploying

Source of truth is this directory; the guest copy is not authoritative.

```bash
scp voice_chat/{server.py,requirements.txt,selftest.py} root@media:/opt/voice_chat/
scp voice_chat/voice-chat.service root@media:/etc/systemd/system/
# pillow is new with vision; the rest of the venv is unchanged.
ssh root@media 'VIRTUAL_ENV=/opt/voice_chat/.venv /root/.local/bin/uv pip install pillow'
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'

scp rover_daemon/{rover_daemon.py,selftest.py} rpi:~/ugv/
```

`talk.py`, `endpointing.py` and `rover_tools.py` are not deployed anywhere —
they run from this repo on whichever desk has the microphone. The rover copy is
flat in `~/ugv/` alongside the face-tracking scripts, which is the layout already
there; nothing on the Pi needs installing.

First start downloads ~10GB of weights. Unauthenticated HF Hub requests are rate
limited to roughly one file per five minutes — set `HF_TOKEN` in the unit if you
would rather not wait.

Two things about that venv are load-bearing and not obvious:

- `en_core_web_sm` is pinned as a wheel in `requirements.txt`. Kokoro's G2P
  otherwise tries to `spacy download` it at startup, which fails under systemd
  because a uv-built venv has no pip ("No package installer found").
- `LD_LIBRARY_PATH` in the unit points at the venv's `nvidia/{cublas,cudnn}/lib`.
  faster-whisper's CTranslate2 dlopens CUDA 12 while torch 2.13 brings CUDA 13,
  so both runtimes have to be installed and the loader needs the path *before*
  the process starts. Without it the LLM and TTS work fine and only STT fails,
  with `libcublas.so.12 is not found`.
