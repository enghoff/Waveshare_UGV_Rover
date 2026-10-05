# The masked semantic feature does not improve the broader repair

The promising six-pair diagnostic did not survive fitting and full-store testing.
Adding masked semantic similarity to the existing three-channel identity score
loses 47 correct pairs and leaves 49 more wrong pairs than the control on the older
recording. It also leaves all six fresh chair/table mistakes. This fitted candidate
will not be deployed. R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed.

## The missing evidence is recovered

Once the rover returned online, a read-only transfer recovered 570 historical
photographs covering both saved snapshots. One startup photograph was absent:
`19700101-053038-1bc7b8.jpg`, affecting observations 61541 through 61551. None is
labelled in the development evidence. Their original assignments are retained;
missing features do not become zero-valued evidence or support reader joins.

The local CPU encoded all 4,015 available observations with the same SigLIP2 vision
export used in the previous probe. Each received a plain-crop control and a crop
masked to the reconstructed saved outline. Median agreement of plain features
with stored features is 0.9999987; the minimum is 0.93965, and 116 observations
fall below 0.99. The photos and model output are now available locally. Another
rover connection is not needed to repeat this experiment.

The source database stayed unchanged. Each new feature is tied to a fingerprint
of its observation ID, frame ID, bounding box, outline, original semantic vector
and backend. Those fingerprints match when the later snapshot's feature bank is
used with the older snapshot. Entity assignments are deliberately not a feature.
Hashes for the frame manifest, model, source and result artifacts are recorded in
the [measurement summary](2026-10-05-masked-semantic-repair.json).

## A controlled comparison

The control uses the existing plain DINO, masked DINO and plain SigLIP channels.
The candidate adds masked SigLIP as a fourth channel, retaining plain context.
Both are fitted by the same logistic procedure on the original development labels.
Two physical-object folds hold out all records of each test-fold object, with
the disputed observation 61656 excluded and head/body treated as one person.
Cross-fold evaluation pairs can compare a held-out object with a training-fold
object; this is not both-object holdout for every pair. Each pair is counted once.

The repair's tiling, linkage, appearance threshold, geometry veto, singleton
handling and second-stage reader grouping are unchanged. Both arms have the same
missing-feature policy. The existing reader's sampled appearance score receives
the new channel through a temporary experiment hook; normal rover code is untouched.
The zero-weight control reproduces **every proposal assignment and grouped
assignment** from the earlier two development folds and refitted fresh control.

| Older recording | Correct pairs together | Wrong pairs together | Main observations waiting |
|---|---:|---:|---:|
| Original records | 3,716 | 710 | 0 |
| Three-channel repair | 3,390 | 197 | 9 |
| Four-channel repair | 3,339 | 201 | 8 |
| Three-channel repair then grouping | 3,658 | 282 | 9 |
| Four-channel repair then grouping | 3,611 | 331 | 8 |

Against the control after grouping, the candidate removes 51 correct pairs and
adds four, while adding 51 wrong pairs and removing two. The net totals therefore
hide both costs and gains. The extra wrong relations concern the green painting's
mixed history (45 pairs) and the blurred dark armchair's history (six). Correct
relations lost are within the cow painting (45) and dark door (six); four correct
relations are gained between the cow painting's duplicate records. The dark door
splits into two pieces, reducing its pair recall from 1.0 to 0.4.

There are still no wrong pairs newly introduced relative to the original older
recording: these are errors the control removed but the candidate retains or
restores. That distinction does not make the candidate preferable to the control.

| Fresh recording, after repair and grouping | Clear correct pairs | Clear wrong pairs | Clear waiting | Tentative wrong pairs | Target chair/table errors |
|---|---:|---:|---:|---:|---:|
| Three-channel control | 475 | 0 | 42 | 78 | 6 |
| Four-channel candidate | 475 | 0 | 41 | 78 | 6 |

Fresh evaluation uses weights fitted on the full older development evidence;
the fresh labels never enter fitting. Clear and tentative labels remain the frozen
analyst drafts. Freeing one observation from waiting, without improving correct
pair counts, does not establish better identity. The clear score still excludes
the tentative chair identities and must not conceal the six inspected errors.

## Why the earlier observation was insufficient

The [earlier probe](2026-10-04-appearance-provenance.md) showed lower masked-semantic
similarity for six wrong pairs while three same-table pairs remained similar.
It did not show that this feature would receive a useful weight after accounting
for the other three channels, or that the resulting assignments would improve.

The fitted fourth coefficient is +3.38 in one object fold, -3.43 in the other,
and -1.18 when fitting all development objects for fresh evaluation. These are
conditional coefficients among correlated measurements, not a claim that masked
similarity intrinsically means different objects. Their changing sign cautions
against drawing a general rule from a few favourable examples. This result rejects
the tested four-channel linear score; it does not prove masked semantics can never
help, nor invalidate the earlier gain from joint repair itself.

## Decision and next work

Keep reader grouping diagnostic and leave the normal resolver unchanged. Stop
adjusting global appearance weights in response to these six examples. The next
bounded experiment should separate removing mixed observations from joining
previously separate records: test split-only proposals, which cannot introduce
cross-record joins, and measure the correct identities they fragment. Such a
restriction is a hypothesis to evaluate, not an accepted repair. Do not silently
restore the same joins in a later grouping stage.

Before any application, proposals still need enough visual evidence to distinguish
outliers, valid partial views, and mixed detections. The remaining review questions
are the object/part policy, the cost of conservative abstention, and whether a
proposal's support persists across views. The owner has delegated analyst judgment;
that does not turn these labels into independent acceptance evidence.

All 24 experiment contract tests passed, including source-fingerprint rejection,
explicit missing features, zero-weight reader equivalence, correct additional
channel arithmetic and restoration of temporary hooks after failure. Documentation
links and requirement identifiers resolve. Only experiments and documentation
changed; no deployment, restart or driving was performed. The rover was used only
to retrieve historical photographs.
