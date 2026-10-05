# Tentative evidence can be preserved, but this confirmation rule loses too much

Separating an observation's proposed identity from its influence on the model is
mechanically feasible. The tested rule is too costly to deploy: on the fresh
recording it keeps only 77% of existing correct pairs as confirmed and 93% when
tentative associations count too. The known painting/table error is removed, but
an armchair fragments. Freezing model updates outright also loses useful identity
links. Keep the distinction between tentative and confirmed evidence as a design
option; reject this two-witness candidate. R-WS-13 stays open, R-WS-17 and R-WS-18
stay proposed, and the proposed 5% contamination target has not been relaxed.

## The experiment was fixed before its replay

Commit c0b1c42 fixed the policy and success criteria. The existing resolver proposes
the destination using its ordinary appearance and geometry gates. A new view of an
existing entity retains that destination as a tentative association. It cannot
update appearance or enter a position refit until both plain and masked DINO score
at least the existing 0.70 recognition value against two already confirmed views
in distinct other inference frames. The latest 24 confirmed peers are considered.
Missing features, incompatible vector widths and different backends abstain.
Promotion uses a snapshot of confirmed peers; views cannot confirm each other in
the same round. Later support is reconsidered after every resolver pass.

The proxy filters confirmed history before applying its history limit, so tentative
views cannot evict all older position evidence. Every measured field remains
byte-identical in every replay arm. Proposed associations still claim their frame
slot and are retained in the archive. This prototype does not reconsider a tentative
destination, clean inherited evidence, or verify discovery founders: those founders
and all attached before-snapshot evidence start confirmed. Two inference frames
also do not guarantee independent viewpoints. None of these flags is production
state, and no existing reader consumes the prototype's confirmed-only view.

The criteria required fewer wrong confirmed pairs without newly introduced ones,
at least 90% retention of correct confirmed pairs, at least 95% when tentative
associations count, and no increase in missing or fragmented labelled identities.
These development criteria are separate from R-WS-18's observation-level tolerance.
Labels never select confirmation or influence; no threshold was adjusted to the
outcomes. The threshold is a tested heuristic, not calibrated confidence.

## Reproduction is explicit

The fresh control reproduces all 4,026 live end memberships and every entity's
placement and both appearance exemplar sets. It starts from the saved before-state,
adds the 232 later observations, and uses the previously reconstructed recorded
inspection schedule, six background passes and frozen occupancy map. This remains
an inferred schedule, not a newly recovered live call log.

The 516-frame older control reproduces every membership of the established older
baseline replay. It is **not** an exact reproduction of the older raw live snapshot:
the replay assigns 2,887 of 3,540 observations, whereas that snapshot assigns 2,979,
and record names differ. The labelled control scores match the established baseline.
This distinction prevents calling repeatability of a benchmark proof of live state.

## Fresh result and the cost of withholding evidence

| Fresh clear draft | Correct pairs together | Wrong pairs together | Views pending or unconfirmed |
|---|---:|---:|---:|
| Ordinary control | 284 | 3 | 39 |
| Candidate, tentative associations included | 264 | 0 | 42 |
| Candidate, confirmed only | 218 | 0 | 69 |
| Freeze appearance, all associations | 250 | 0 | 41 |
| Freeze appearance and position, all associations | 256 | 0 | 42 |

The confirmed-only column combines genuinely pending views with associated views
not promoted; the archived observations have not disappeared. In the candidate,
167 new attachments to existing records start tentative, 109 are promoted, and
four discovery founders are exempt. Across newly associated rows, 58 observations
remain tentative: 57 fresh and one previously pending historical observation.
The fresh clear draft covers 146 observations of 16 identities;
that small labelled subset does not establish purity of the inherited whole store.

The tentative-label sensitivity has 381 correct and 25 wrong pairs with proposed
associations included, against 407 and 34 in the control. Sixteen wrong tentative
relations are new relative to the control despite the lower net wrong total. The
confirmed-only sensitivity has 298 correct and zero wrong pairs. Uncertain chair,
door and part identities remain development judgments, not independent truth.

Twenty-seven associated clear-labelled object views remain unconfirmed. At the end,
each still has fewer than two eligible confirmed witnesses, so another idle pass
alone would not promote them. They include seven rug views, four table views,
five green-painting views and three opposite-wall armchair views. The preserved
diagnostic images show useful table/rug parts and a recognisable armchair in glare.
They also include the previously disputed painting-with-chair region 64625: its
selected pixels are mixed even though the frozen draft names the painting.
The scores retain the original labels; this caveat is not silently corrected away.

The cabinet-wall armchair gains extra records containing its fresh views. The rug
and opposite-wall armchair have no confirmed fresh-labelled views, although their
old evidence and tentative associations remain. That is a fresh-support coverage
gap, not proof that those physical objects have vanished from the whole store.

## The explanatory controls do not rescue the policy

After the first fresh result, commit 441bf76 predeclared two explanatory extremes:
freeze existing appearance while still refitting position, and freeze both.
Ordinary associations and discovery founders are retained; neither uses labels.
Both remove the fresh clear target error and both lose more correct pairs than
the candidate's complete proposed associations. Thus immediate model feedback
matters, but stopping adaptation wholesale is costly too. These controls were
specified after the first candidate result and are not independent acceptance.

## Cold start exposes a bootstrap restriction

| Older development recording | Correct pairs together | Wrong pairs together | Main views pending or unconfirmed |
|---|---:|---:|---:|
| Ordinary control | 3,716 | 710 | 0 |
| First candidate, tentative associations included | 2,004 | 216 | 26 |
| First candidate, confirmed only | 131 | 2 | 296 |

Without the large inherited confirmed archive, the first rule retains only 3.5%
of correct pairs as confirmed and 53.9% including tentative associations. Labelled
identity groups split across records with two views increase from four to nine.
One wrong hypothetical relation is newly introduced; none of the wrong confirmed
relations is new. There are 936 confirmed and 1,861 tentative assigned observations
across the 3,540-row replay. Both older reporting folds fail retention; neither is
an independent recording. The floor/skirting group remains in these development
scores and is not an acceptance object census.

This is more than a threshold score: requiring two **already confirmed** views
prevents unfamiliar but mutually consistent views from establishing a new appearance.
Warm-start evidence concealed the severity. It would be wrong to use this failure
to condemn the whole tentative-evidence idea without testing that restriction.

Commit febca34 therefore predeclared one corrected path. Two tentative views of
the same proposed entity can corroborate each other when both plain and masked
appearance match at 0.70; each connects to pre-round confirmed evidence at 0.55,
with at least one connection at 0.70. Their independently measured rays must yield
an ordinary valid fix with at least 0.4 m baseline and 12 degrees parallax, and
that fix must agree with the current placement within squared ellipse distance
13.8155. This retains range, height and map-visibility checks. The latest 24 tentative
peers are tried; anchors never come from the pair being promoted in that round.
No threshold or label changed. This revision is development learning from a
mechanism, not independent validation.

On the fresh recording it recovers one additional armchair view (64624), corroborated
by 64668 across a 0.732 m baseline and 25.9 degrees parallax. The clear draft then
has 270 correct pairs with all associations and 224 confirmed, zero wrong pairs in
either view, and respectively 41 and 68 pending/unconfirmed views. Those are still
95.1% and 78.9% correct-pair retention; confirmed retention fails the predeclared
90% floor even after the bootstrap correction.

## Conclusion and the next evidence

The corrected cold-start replay raises confirmed correct pairs from 131 to 1,241,
showing that the bootstrap restriction was real. It still falls far short:

| Older recording, anchored bridge | Correct pairs together | Wrong pairs together | Main views pending or unconfirmed |
|---|---:|---:|---:|
| Tentative associations included | 1,806 | 217 | 17 |
| Confirmed only | 1,241 | 77 | 174 |

This retains 33.4% of correct confirmed pairs and 48.6% with tentative associations
included. Forty-one wrong hypothetical relations are new, despite fewer wrong
pairs overall. None of the wrong confirmed relations is newly introduced. The
geometry-supported bridge accounts for 155 promotions; later direct confirmations
bring all promotions to 1,536. These are development results and do not establish
the 5% observation-level target or independently validate the founder exemptions.
Both recordings fail the fixed retention criteria; neither variant is approved.

There is a firm conclusion about this implementation: two strong confirmed-frame
matches do not supply a useful confirmation policy on these recordings, and the
anchored correction recovers support without recovering enough. Separating
archive association from learning does not erase the ambiguity of poor views or
make a contaminated founder reliable. This is a rejection of the tested rule,
not proof that multi-view confirmation or later repair can never help.

Do not continue lowering thresholds or changing phrase lists against these same
examples. The next decisive evidence requires the
[controlled drive](../plans/entity-evidence-drive.md): obtain clean founding views,
intermediate views connecting them to glare/part views, and a side view separating
the chair from its background painting. Preserve exact resolver calls and all
failed/mixed detections. Judge the actual selected pixels without showing entity
assignments before scoring. Clean masks failing across verified views would point
at viewpoint representation; mixed masks even in clear views would point at region
separation. Those are different changes and the current trial does not choose one.

The prototype and the [reproducible scorer](../../experiments/entity_association/assess_tentative.py)
are workstation experiments. No registered component changed; no deployment,
restart, perception request or driving was performed. All 35 experiment tests pass.
The [measurement summary](2026-10-05-tentative-evidence.json) records source hashes,
all eight arms, byte-preservation proofs and each predeclared check.
