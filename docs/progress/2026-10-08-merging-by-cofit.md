# Joining duplicate records by co-fit: the rug's seven records become two, about one join in 25 wrong

**A record pair is now scored by how many of each one's looks would also have
fitted the other -- the test same-object suspects use, applied to every look --
and joining pairs where at least 0.3 of each does finds duplicates the existing
merge proposer cannot.** On a copy of the rover's store it takes the rug under
the dining table from seven records to two, where the proposer leaves seven. On
three replays of map session 67, run after the proposer it links more of the
trial set's same-object looks in every run (48, 46, 40 against 43, 39, 37), with
no added link between different labelled objects. Of 25 joins drawn at random
and judged by photograph, 17 are one object, 1 is two, and 7 cannot be told
(alike chairs and fans, records already mixed). Measured offline; nothing is
deployed. Joining records is an identity action, so it waits on a case
(R-WS-13); it would serve R-WS-17.

## Why the proposer missed them

`merging.propose` scores a pair by appearance plus position. On today's store
(`experiments/entity_association/pair_evidence.py`, and the pair scores behind
it):

- **The black cabinet's seven records look alike** (appearance +0.4 to +2.1) but
  each claims its position to well under a metre, and they stand up to a metre
  apart, so position scores them as two (down to -3.7). The proposer joins
  three of the six.
- **The rug's seven records do not even look alike**: seen in pieces from
  different sides, record-to-record appearance is negative for most pairs. The
  proposer joins none.

Co-fit compares one look with the other record, not one record with another,
and both groups pass it.

## The evidence on labelled pairs

Every pair of placed records whose identity the frozen label sets settle
(the three `score_session.py` reads, a record taking the object most of its
labelled looks show), within 4 m:

| | one object | two objects |
|---|---|---|
| pairs | 15 | 98 |
| hold regions of one picture | 7 | 79 |
| existing proposer | 3 | 0 |
| co-fit at least 0.3 both ways, no shared picture | 7 | 1 |
| co-fit at least 0.3 either way, no shared picture | 8 | 1 |

The one wrong pair joins the grey gold-framed painting's record to a record that
already mixes it with the snowy painting over the cabinet. Seven of the 15
one-object pairs are paintings cut by chair backs into two regions of one
picture; a thing holds one region of a picture, and no rule joins those. With
the 2026-10-08 photograph groups added (the rug and the cabinet, found through
suspects and so favouring co-fit) the counts are 25 of 41 joinable pairs
against 9 for the proposer, still with one wrong.

## On replays of a whole session

`merge_after_session.py` joins records on a replayed session's final store and
`score_session.py` scores the result; three replays of map session 67, the
resolver as committed and two with a share of unlabelled regions dropped, give
the noise. Same-object look pairs linked, and different-object look pairs
linked, per label set:

| Replay | Pass | 10-03 set: same / cross | 10-07 trial: same / cross | 10-03 split objects |
|---|---|---|---|---|
| committed | none | 2,600 / 10 | 36 / 0 | 6 of 16 |
| | proposer | 3,167 / 11 | 43 / 0 | 5 |
| | co-fit 0.3 | 2,677 / 11 | 51 / 0 | 5 |
| | proposer, then co-fit 0.3 | 3,167 / 11 | 48 / 0 | 5 |
| noise 1 | none | 2,309 / 5 | 25 / 0 | 6 |
| | proposer | 3,072 / 5 | 39 / 0 | 6 |
| | co-fit 0.3 | 3,061 / 5 | 40 / 0 | 6 |
| | proposer, then co-fit 0.3 | 3,072 / 5 | 46 / 0 | 6 |
| noise 2 | none | 2,519 / 5 | 27 / 0 | 6 |
| | proposer | 2,704 / 5 | 37 / 0 | 4 |
| | co-fit 0.3 | 3,162 / 5 | 37 / 0 | 5 |
| | proposer, then co-fit 0.3 | 2,760 / 5 | 40 / 0 | 4 |

The 10-03 set is the one the proposer's appearance weights were fitted on, so
the trial set is the fairer comparison. Wrong looks rise by up to 0.9 points
after any pass. At 0.1 co-fit joins more but linked four different-object look
pairs in one run's trial set.

## On today's store

| | Rug (7 records) | Black cabinet (7) | Snowy painting (4) |
|---|---|---|---|
| proposer, round after round | 7 | 4 | 4 |
| co-fit 0.3 | 2 | 4 | 3 |
| proposer, then co-fit 0.3 | 2 | 4 | 4 |

None joined two of these groups, or the record of a person in front of the
cabinet into it. 344 pairs pass co-fit at 0.3; 116 joins result.

## Judged by photograph

25 of the 344 pairs at random (seed 20261008), three crops of each record
(`experiments/entity_association/cofit_review_20261008.json`, analyst's
judgment):

| | Pairs |
|---|---|
| one object | 12 -- the floor lamp (at least seven records), the blue painting over the shelf (four), the purple armchair, a window, the corridor doorway, the snowy painting |
| probably one | 5 -- one record mixed or blurred |
| two objects | 1 -- the corridor doorway and the open white door beside it |
| alike, cannot be told | 3 -- dining chairs, ceiling fans |
| both records already mixed | 4 -- the dark door with dining chairs |

The proposer's own review on 2026-10-04 found 20 of 26 plainly one object and
none plainly two.

## What it does not do

- It cannot join an object cut into two regions of one picture.
- It cannot tell a duplicate from a record that already mixes two objects, and
  joins into such records.
- Alike objects -- the dining chairs, the two ceiling fans -- pass it when no
  picture holds both.
