# Asking the rover's questions out loud

**Results: [docs/omni-step0.md](../docs/omni-step0.md).** Four models, run on
2026-08-17. Nobody lost anything to being spoken to.


Every tool-calling number this repository holds was measured by typing at a model
that is spoken to. The `/chat` endpoint takes text and returns a decision, which
was the right instrument for a text model, and four months of measurement went
through it — the 66/90, the 75/90, every `look` table in
[voice_chat](../voice_chat/README.md#getting-it-to-actually-call-them). Against an
omni model those become an upper bound the deployed path never reaches, because
the deployed path is audio.

This is the instrument that closes that gap: the same fifteen phrases, spoken
rather than typed, against the same ten daemon schemas, with the answer scored the
same way. It is step 0 of [docs/omni-build.md](../docs/omni-build.md), and the
document is blunt about why it comes first — everything below it in that plan is
contingent on the result, and the result decides whether the omni design is worth
building at all.

## What it measures, and against what

Two numbers, from one run.

The first is whether the model calls the right tool when the instruction is
spoken. The comparison is not against the 66/90 already in this repository: that
was `Qwen3-VL-4B`, typed, and setting an omni model's spoken score beside it would
confound the change of modality with the change of model. So the harness runs both
arms — the same phrases typed and spoken, the same weights, the same process — and
the drop between them is the number that matters. The old figure is carried
alongside as background, which is what it is.

The second is whether a tool call can be caught before it is spoken. A tool call
is text, and in a duplex model that text is on its way to a speech decoder, so the
question is not whether it appears but whether it appears with enough margin to
intercept. The sniffer records where the marker showed up — how many characters,
how many chunks, how many milliseconds into the reply — so the margin is a
measurement rather than an impression.

## The pieces

| file | what it is |
|---|---|
| [corpus.py](corpus.py) | the fifteen phrases, what each should call, and the score it got typed |
| [schemas.py](schemas.py) | the ten tools and the system prompt, read out of the daemon and the voice service by `ast` |
| [synth.py](synth.py) | the phrases as speech, from the voices Windows already has |
| [sniff.py](sniff.py) | catching a tool call in a stream, and timing when it became catchable |
| [runner.py](runner.py) | the measurement, run on the rented card |
| [cloud.py](cloud.py) | the same measurement against Alibaba's hosted omni models |
| [duplex.py](duplex.py) | whether the call arrives before the speech, and what the speech says if it does not |
| [score.py](score.py) | the tables, and the decision rule that was written before the run |
| [pod.py](pod.py) | renting a GPU, and giving it back |
| [session.py](session.py) | bringing a card up and putting the harness on it |

Two of those deserve a note.

**Nothing here holds a copy of a schema.** The daemon is the one place a tool is
described, and every word of those descriptions was arrived at through six-sample
runs; a benchmark that pasted them here would be measuring a fossil the first time
somebody improved one. `schemas.py` therefore parses
[rover_daemon.py](../rover_daemon/rover_daemon.py) and
[voice_chat/server.py](../voice_chat/server.py) with `ast` rather than importing
them, because one pulls in `serial` and the other pulls in `torch`, and neither
belongs on a rented card.

**The corpus is now a corpus.** It was distributed across the tables and prose of
one long README, and a measurement whose case list has to be reassembled by
reading an essay is one nobody can repeat. The fifteen are fixed and are what
totals out of 90. Five more reach the tools the fifteen never touch —
`count_faces`, `look_at`, `center_camera`, `track_next`, `tracking_status` — and
are deliberately kept out of the headline number, because they have no typed
baseline to be compared against.

## Running it

```bash
python synth.py                          # the phrases as speech, once
python session.py --up                   # rent a card, upload, download weights
python pod.py --sentinel 150 &           # the dead man's handle
python session.py --exec "cd /workspace/omni_bench && python runner.py --arm text --out text.jsonl"
python score.py runs/*.jsonl
python pod.py --reap                     # always
```

The money is the reason this is split up. An L40S is $0.79 an hour and the whole
measurement is costed at about five dollars, so the harness is built and debugged
against a mock on the workstation, and the card is rented only when there is
something to run on it. [pod.py](pod.py) is written around the assumption that the
expensive failure is not a run that goes wrong but a pod nobody remembers: it
terminates in a `finally`, terminates again from a watchdog thread if the body
hangs, refuses to start while another pod is alive, and writes the pod id to disk
before creating it so that a later session can reap an orphan.
