# Voice chat

Speech in, speech out. The models run on the MEDIA GPU host; the microphone and
speakers stay on whatever machine you are sitting at.

```
  this machine, or the rover            root@media  (RTX 3070, 8GB)
  --------------------------            ---------------------------
  mic -> VAD/endpointing --16k s16le-->  faster-whisper distil-large-v3
                                              |  text
                                         Qwen3-4B-Instruct (int4)
                                              |  sentences, streamed
  speakers <-- playback <--24k s16le---  Kokoro-82M
```

Two clients, because the audio hardware has nothing in common: [talk.py](talk.py)
on a desktop with sounddevice, and [talk_pi.py](talk_pi.py) on the rover, which
drives PipeWire through pipes. The decision they must agree on — when a turn has
ended — is in [endpointing.py](endpointing.py) so it cannot drift between them.

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

Client dependencies: `pip install -r voice_chat/client-requirements.txt`.

The service binds `0.0.0.0` rather than loopback, so there is no tunnel any
more. That is a deliberate trade: the microphone now lives on a Pi 1, and an
SSH tunnel from it is a process and a reconnect loop sitting between us and the
conversation. The port is open on the LAN with no authentication in front of
it, the same as `face-detect` beside it; the Hyper-V inbound rule covers
8765-8774. Do not do this on a network you do not own.

## From the rover

The rover talks and listens over Bluetooth, with the models still on MEDIA:

```
  WI-XB400  --HFP/mSBC 16k-->  Pi  --wifi-->  MEDIA
  JBL Flip  <---A2DP SBC-----  Pi  <--------
```

```bash
ssh admin@rpi 'cd ugv && python3 talk_pi.py'
```

`--list` shows the PipeWire nodes, `--mic` / `--speaker` take a node name, and
`--mic auto` falls back to whatever `wpctl set-default` points at.

Four things about that client are not obvious:

- **It has no `sounddevice` and no `websockets`.** `apt` on the rover needs a
  password we do not have from a script, so both are avoided rather than
  installed: audio goes through `pw-record`/`pw-play` pipes, and
  [wsclient.py](wsclient.py) is ~180 lines of RFC 6455. PortAudio would not have
  helped anyway — there is no ALSA-to-PipeWire bridge on the box, so it would be
  talking to raw ALSA and would not see the Bluetooth devices at all.
- **Client frames are masked with one big-integer XOR.** Masking a 15-second
  utterance byte-at-a-time is seconds of bytecode on a 700MHz ARM11.
- **A thread sits in `recv` the whole time, not just during a turn.** uvicorn
  pings every 20s and hangs up if nothing answers, and a conversation is idle
  between turns — so reading only while a turn was in flight got the connection
  closed and the next utterance died with `Broken pipe` about half a minute in.
  The `websockets` library does this for `talk.py` invisibly; here it is a
  thread. It is also why a dropped link is noticed while waiting for speech
  rather than only when there is something to send.
- **The mic is muted while the assistant speaks**, for the queued audio plus
  `PLAY_LATENCY_S + MUTE_TAIL_S`, and everything captured during that window is
  thrown away on unmute. Without it the reply endpoints itself and the two talk
  over each other forever.

### Measured, on the rover

A turn from the Pi, with the models warm on MEDIA:

| stage | time |
|---|---|
| STT | 0.39–0.47s |
| first audio out | 1.10–1.37s |
| whole turn | 1.43–1.71s |

Queue-to-heard for the JBL, measured by putting a burst through it and finding
it in the headset mic:

| | |
|---|---|
| stream already running | **151 ms** |
| stream never played before | **2057 ms** |

That second number is why the speaker is built at startup rather than on the
first `start` event. Built lazily, the first reply's mute window was short by
~1.7s, the last sentence was still coming out of the JBL when the mic reopened,
and the assistant transcribed itself and answered its own question — turn two of
the first live run was literally `you: What do you need?`.

A2DP also gets slower while the HFP mic is open, which is expected on one radio:
2s of audio took **2.85s** with the mic shut and **3.51s** with it open. Slower,
but it does not stall.

### Why playback was choppy

Playback broke up while the Pi looked idle. The cause was **this client reading
the microphone one 20ms block at a time** — 50 wakeups a second, each with its
own small numpy allocations. That is enough to make PipeWire, which gets no
realtime priority on this box, miss its deadline; every miss is one quantum of
silence spliced into whatever is playing.

How to measure it, since almost every obvious approach here lies:

- **Capture the sink monitor, not a microphone.** `pw-record --target <sink>`
  silently records a *source* instead. Proved with a known-amplitude tone: peak
  4356 against the tone's exact 9000. The real thing is
  `pw-record -P "stream.capture.sink=true" --target <sink>`. An underrun on the
  monitor is unambiguous — the gap is literal zeros.
- **Ignore xrun counters.** This client holds `pw-play` open for the whole
  conversation, so between turns it underruns every quantum by design. A 3s gap
  scores 3.0 / 0.0213 = 141 "xruns" at every buffer size. It measures silence,
  not damage.
- **Play the probe tone with `cat`.** `cat` cannot starve `pw-play`, so any hole
  in the monitor is caused by whatever else is running.

That harness gives a clean bisection — 15s of `cat`-fed tone, varying only what
runs alongside it. **Two independent faults, and it took both fixes:**

| running alongside | before | after |
|---|---|---|
| nothing | 0 (0.00%) | 0 (0.00%) |
| an idle `pw-play` stream on the same sink | 0 (0.00%) | — |
| `pw-record` on the USB mic | 1 (0.27%) | — |
| `pw-record` on the Bluetooth mic (SCO) | 3 (0.82%) | — |
| the old 640-byte-at-a-time read loop | 36 (11.72%) | 6 (1.64%) |
| the whole client | 54 (39.59%) | **0 (0.00%)** |

And on a real spoken turn, measuring the reply the client feeds itself:

| | holes per sentence | longest hole |
|---|---|---|
| before | 6.49%, 8.00%, 11.66% | 41-100ms |
| after | 0.66%, 0.79%, 1.51% | **9ms** |

41ms is one quantum and plainly audible; 9ms is under one and is not. Measure
the *reply*, not a tone played into the room while the client is listening —
nothing plays over the speaker while someone is talking to it, so that case is a
property of the test rig rather than of the rover.

**Fault one, in this client: it woke too often, and in bursts.** Three separate
places, all of which had to go:

- `Recorder._read` asked for one 20ms block at a time — 50 wakeups a second,
  each with its own small numpy allocations. It now takes whatever `pw-record`
  offers (~100ms at a time, so no added latency), converts the batch in one
  numpy call, and hands the endpointer a precomputed RMS so it does none itself.
- The utterance went out as **one** WebSocket frame, and masking is a single
  big-integer XOR — a ~500KB integer operation holding the GIL for hundreds of
  milliseconds, right where the reply is about to play. Split into 32KB frames;
  the service buffers binary frames until `{"type":"end"}` so it sees no
  difference. This alone took the holes in a reply from 41-100ms down to 9ms.
- `Speaker._pump` treated `write()` returning `None` as a broken pipe. On a raw
  stream `None` means "would block, retry" — so a reply could stop dead partway
  and every later one be silently dropped, which looked like the speaker cutting
  out mid-sentence. It retries now, and says so on a real failure rather than
  dying quietly.

**Fault two, on the box.** PipeWire's `data-loop.0` was not realtime, so it had
no way to defend that 21ms deadline. `admin` was not in the `pipewire` group, so
the `rtprio 95` in `/etc/security/limits.d/25-pw-rlimits.conf` never applied.
Fixed permanently with `usermod -aG pipewire admin` and a reboot — group
membership is only read at login, and `systemctl --user restart` will not pick it
up. Check it with `chrt -p` on the *right* thread:

```bash
for t in /proc/$(pgrep -u admin -x pipewire | head -1)/task/*; do
    echo "$(cat $t/comm): $(chrt -p $(basename $t) | tr '\n' ' ')"
done
# want: data-loop.0: ... SCHED_FIFO ... priority 88
```

`ps -eLo cls,rtprio,comm | grep pipewire` does **not** answer this — the thread
is called `data-loop.0`, so that pattern greps straight past the only one that
matters, and reports `TS` for the idle main threads instead.

Blind alleys, recorded so they are not re-run: it is **not** CPU load (a
deliberate spin loop costs 0–2 holes — a hog competes for throughput, which the
scheduler handles, while waking 50 times a second competes for latency, which it
does not); **not** the SCO link, cleared three times over; **not** the pipe size
(200ms/500ms/1s/2s identical, and widening the pipe to 1MB changed nothing on its
own); and **not** a missing `rtkit`, which was installed and running throughout,
so `apt install rtkit` is a no-op — the rlimit was the thing it lacked.

### Getting the Bluetooth up

The adapter came up `off-blocked` — an rfkill soft block that `systemd-rfkill`
had been restoring at every boot. `rfkill` is not installed and `sudo` wants a
password, but a udev ACL leaves `/dev/rfkill` writable by `admin`, so the block
can be cleared by writing the 8-byte `struct rfkill_event` directly. After that:

```bash
bluetoothctl power on
bluetoothctl connect 20:18:5B:7C:2E:44   # JBL Flip
bluetoothctl connect 30:53:C1:A4:66:86   # WI-XB400
```

Both are trusted, so they reconnect on their own when switched on. WirePlumber
picks `headset-head-unit` with **mSBC** for the WI-XB400, which is 16kHz mono —
exactly the rate Whisper wants, with no resampling in between. Node names, which
is what the client addresses:

| device | PipeWire node |
|---|---|
| WI-XB400 mic | `bluez_input.30_53_C1_A4_66_86.0` |
| JBL Flip | `bluez_output.20_18_5B_7C_2E_44.1` |

The Pi 1 runs the CSR dongle, the wifi dongle and the camera off one weakly
fused USB bus, and an always-open SCO mic keeps the Bluetooth radio busy
alongside A2DP. If audio breaks up or the wifi drops during a run, that is the
first thing to suspect — see [docs/hosts.md](../docs/hosts.md).

## Protocol

WebSocket at `/ws`. Client → server: binary frames of 16kHz mono s16le, then
`{"type":"end"}` to close the utterance; `{"type":"reset"}` clears history.

Server → client, all JSON except the audio:

| event | meaning |
|---|---|
| `{"type":"stt","text":…}` | what it heard (`"empty":true` if nothing) |
| `{"type":"start","rate":24000}` | reply beginning, at this sample rate |
| `{"type":"text","text":…}` | one sentence, followed by **one binary frame** of its audio |
| `{"type":"done","stats":{…}}` | turn over, with `stt_ms` / `first_audio_ms` / `total_ms` |

`GET /health` is what the switcher polls. `POST /say?text=…` returns raw PCM for
checking a voice without holding a conversation.

## Tuning

Server knobs are environment variables in
[voice-chat.service](voice-chat.service); the reasoning behind each default is in
the comments in [server.py](server.py). The ones worth knowing:

- `VOICE_LLM_MODEL` — 4B by default. 8B at int4 fits (~5GB) but leaves no room
  for Whisper's context to grow; try it only with the vision services stopped.
- `VOICE_COMPILE=1` — compiled decode, ~46 tok/s against ~12 uncompiled. Falls
  back to eager if compilation fails rather than refusing to start; `/health`
  reports which it got. Note this service does *not* ask for `reduce-overhead`
  the way `qwen3-vl.service` does: inductor skips CUDA graphs here anyway
  ("mutated inputs"), because the static cache is written in place.
- `VOICE_COMPILE_DYNAMIC=1` — compile once for a range of prompt lengths. Turning
  this off costs a ~60s recompile on *every* turn of a conversation, since the
  prompt grows each time. Only worth it if the prompt length is somehow fixed.
- `VOICE_CACHE_LEN=2048` — the static-cache window, ~12 spoken turns. History is
  trimmed a whole turn at a time to fit it.
- `VOICE_TTS_VOICE=af_heart` — any Kokoro voice. The first letter picks the
  language pack, so keep it consistent with the language you are speaking.
- `VOICE_SYSTEM_PROMPT` — constrains replies to short spoken English. Without it
  the model emits markdown and Kokoro reads the punctuation aloud.

Client knobs are constants at the top of [endpointing.py](endpointing.py), shared
by both clients: `SPEECH_FACTOR` and `SPEECH_FLOOR` for sensitivity, `HANG_MS`
for how much silence ends a turn. The rover client adds `PLAY_LATENCY_S` and
`MUTE_TAIL_S` in [talk_pi.py](talk_pi.py) — how long after the last sample is
queued the JBL is still audible. Both are deliberately generous: too short and
the assistant hears itself and answers in a loop, too long only costs a moment
of responsiveness.

Endpointer timing is counted in 20ms blocks rather than wall-clock seconds —
`sounddevice` delivers in bursts after a scheduling hiccup, and a clock reads
that burst as a long silence and ends the turn mid-sentence.

## Checks

`selftest.py` covers the three pieces where a bug is silent rather than loud: the
sentence splitter, the endpointer, and the WebSocket framing. It has no GPU or
microphone dependency, and each part skips where its dependencies are absent, so
run it anywhere:

```bash
python voice_chat/selftest.py                                        # endpointer + framing
ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py   # splitter
ssh admin@rpi 'cd ugv && python3 selftest.py'                        # what the rover runs
```

The framing checks matter more than they look: a client frame that is not masked,
or a length field one byte out, makes the server hang up without a word, which
reads as a network fault rather than a bug.

## Deploying

Source of truth is this directory; the guest copy is not authoritative.

```bash
scp voice_chat/{server.py,requirements.txt,selftest.py} root@media:/opt/voice_chat/
scp voice_chat/voice-chat.service root@media:/etc/systemd/system/
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'

scp voice_chat/{endpointing.py,wsclient.py,talk_pi.py,selftest.py} admin@rpi:~/ugv/
```

The rover copy is flat in `~/ugv/` alongside the face-tracking scripts, which is
the layout already there; nothing on the Pi needs installing.

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
