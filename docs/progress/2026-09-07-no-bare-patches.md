# The bare-patch fault does not reproduce, and two of my labels were wrong

**Of the 43 things on the acceptance recording, one is arguably a patch of
nothing, and it was seen twice.** The two entities the drive's own review named
as blown-out wall and window are a ceiling light fitting and a doorway through to
the next room — both real objects, and both mislabelled by me off a contact sheet
that was too small to read. [R-WS-12](../requirements/world-state.md#r-ws-12) has
no clear instance on this recording.

That matters because the next thing on the list was a filter to make such patches
ineligible as somewhere worth driving to. **There is no reproduced fault to write
that filter against**, and building one anyway is the guess this repository's
rules exist to prevent.

## How the labels went wrong, twice

The first review looked at 394 crops as strips on one downscaled sheet. It got
two observation identifiers wrong, which was
[corrected already](2026-09-07-m0-acceptance-drive.md), and it got these two
entity labels wrong in the same way and for the same reason.

This time each entity is one labelled row of five crops at 150 pixels, spread
across its own history rather than taken from the start of it, seven rows to a
sheet, with the identifier burnt into the row. At that size there is no doubt:

| entity | called | actually |
|---|---|---|
| `object:2` | a blown-out wall patch | **a ceiling light panel**, seen 8 times, with one bare-ceiling crop among them |
| `object:24` | a blown-out window patch | **a doorway to a teal-walled room**, seen 5 times, with two bare-wall crops among them |

The bare crops inside each are wrong attachments — a merge fault, which is
already recorded and already has a deployed remedy. They are not an eligibility
fault, and the difference is the whole of R-WS-12.

## What a bare patch would have looked like, and what was there instead

Every entity with five or more looks, and every one with fewer, was gone through
again. The nearest thing to a patch of nothing in the whole recording:

- `object:18`, two looks, both a stretch of plain wall carrying a painted stripe.
  Even this has a feature in it, and two looks is below what the resolver will
  place.
- `object:6`, eight looks, the rug seen at such a grazing angle that most of each
  crop is floor beside it. A rug is an object, as the owner has already said, so
  this is a bad viewpoint rather than a bad thing.

Everything else is furniture, framed pictures, doors, a ceiling fan, a ceiling
light, an air vent, a rug, two shoes, a green tissue box, a person, and — pleasingly
— the ChArUco calibration board itself as `object:36`.

## Why the earlier measurement disagreed

[The M0 baseline](2026-09-07-m0-semantic-world-state.md) found two of fourteen
reviewed entities to be floor, and that is what R-WS-12 has been open on. This
recording was taken afterwards, through the corrected mount and the two capture
gates, and with `perceive._blank` refusing crops that carry no picture — a filter
measured on the drive of 2026-09-02, where 58 of 338 regions were a flat wall or
a blown-out window.

**Which of those explains the difference is not established here**, and this entry
does not claim the fault is fixed. What it claims is narrower and is what was
measured: on the most recent driven recording, the fault does not appear.

## What the re-review turned up instead

Merges, which are the known fault and are the one with a deployed remedy:

- `object:5`, 19 looks, holds **several different framed pictures** and a dark
  cabinet — not one picture with an intruder, but distinct paintings pooled.
- `object:34`, 7 looks, holds a framed picture and two unrelated blown-out crops.
- `object:29`, 11 looks, is mostly blown-out white with an armchair in it.

None of these is a picture-behind-a-chair, so the collapse test is not obviously
the remedy for them, and they should be looked at again on the held-out drive
before anything is built for them.

## Requirements

- [R-WS-12](../requirements/world-state.md#r-ws-12) stays `open` and its named
  instances are corrected: it has none on this recording. It is not settled,
  because one recording that fails to reproduce a fault is not a demonstration
  that it is gone, and the held-out drive is what would settle it.
- No code changed and nothing was deployed.
- M0 criterion 9 has no counter-example on current data.

## Next

1. Do **not** write a bare-patch filter. Carry the question to the held-out drive
   and settle it there.
2. Count splits, which nothing reports the way a merge is reported — `object:5`
   and the several entities sharing one painting make it concrete.
3. Widen the recorded position uncertainty to what the drive measured.
4. The navigation-restart demonstration, then the drive.
