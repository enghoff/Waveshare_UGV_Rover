# A painting's outlined depth can select the chair in front of it

Keep the proposed quality target. The failed release and confirmation rules remain
unsuitable for automatic use, but their failure does not establish that better
association is impossible. A fresh controlled recording exposes ambiguity in the
measurements that those rules were being asked to trust. R-WS-13 remains open;
R-WS-17 and R-WS-18 remain proposed. No identity policy was deployed.

## What was recorded and reviewed

Main now contains both the newer navigation work and the entity studies at 509251a.
The deployer reported no outstanding component changes; the running Orin answered
`world_state_groups` with `preview_only=true`, `accepted=false`, and its current
world generation. Local world-state and daemon suites passed 1,052 and 1,107 checks.

Using the unchanged resolver, the rover moved from its parked position through an
intermediate position and two side positions, then looked at the same scene from
another heading. Before and after SQLite backups preserve 125 new regions in 14
frames. All 14 photographs and the five available depth maps are archived under
`captures/2026-10-05-evidence-drive`. Physical subjects were frozen from photographs
and boxes, and outline judgments from masked crops, before scoring or reading the
entity assignments. These are analyst development judgments, not independent
acceptance labels. The two foreground chairs were tracked through their fixed
relative order and the continuous short path.

The 18 selected regions cover the painting behind the chairs, both foreground
chairs and the rug. The table has no two clearly isolated retained regions, and
no armchair was recorded. This fulfils only part of the planned coverage. Current
diagnostics do not persist every background resolver call, so this is a visual
feature and depth diagnosis, not an exact resolver replay. No reset, rebuild,
merge or experimental confirmation policy ran.

## Masking separates objects, but does not guarantee cross-view recognition

Across all 50 selected painting/chair pairs, the maximum plain DINO similarity is
0.753, enough to pass the existing 0.70 recognition threshold. The maximum masked
similarity is only 0.385. This supports the usefulness of masking for excluding the
foreground chair; it does not license discarding masked evidence because some
previous repairs failed.

Conversely, the initial painting and the last side-position view (68581, 68700)
are the same physical picture, separated by 1.756 m and 13.0 degrees bearing
parallax. Their plain/masked similarities are only 0.574/0.639. Their ordinary
geometric fix is valid, but the tested requirement that both appearances exceed
0.70 refuses their support. It also refuses both independent rug pairs. Of the
seven independent front-left chair pairs, two pass both appearances and only one
has a valid ordinary fix; for the front-right chair those counts are five of eight
and six of eight. Thresholds were not fitted to this recording.

These observations show why a blanket confirmation requirement loses genuine
support. They do not separate intrinsic viewpoint sensitivity from residual mask
pixels and crop changes. Saved outlines are clipped at half resolution; the actual
appearance mask can include pixels outside them. No perfect pixel-purity claim is
made, and no new admission threshold is selected here.

## The ambiguous depth is reproduced, not guessed

All 16 available selected ranges reproduce exactly from the saved depth maps and
the production sampling code, including the box fallbacks; the two missing ranges
remain outside the depth view. The suspicious painting region 68640 returns
**1.852 m with sigma 0.030 m** from its outline. At the final production projection,
417 valid depth samples split into a near group and a much farther group: 34.1%
are below 2.5 m, 64.0% above 3 m. The overall median is 3.983 m. Production selects
the nearest band (140 samples, 33.6%), whose median supplies the 1.852 m answer;
its narrow spread supplies the small sigma. Initial projection guesses from one
to six metres all return the same result, so changing that guess is not a repair.

The nearer distance agrees roughly with the foreground chair ranges at that view.
That, the photographs and the other painting readings support the inference that
foreground depth contaminated the reading. There is no tape truth and no saved
OAK colour frame to prove which individual projected depth pixels belong to which
surface. The firm finding is that a small within-band sigma does not express the
ambiguity between these surfaces. Selecting the overall median would merely pick
a different surface without establishing its physical identity.

The front-left chair's outline also fails to range in one view, and the box fallback
returns 4.133 m with only 9.9% valid pixels. This provides a second concrete reason
not to treat every supplied distance as reliable identity support. It does not
justify refusing every fallback or every low-validity measurement: glass furniture
and narrow parts need a broader test.

## Decision and remaining work

Do not relax R-WS-18's provisional target or deploy either failed confirmation rule.
Prioritise checking whether a measured surface belongs to the proposed object,
and representing ambiguity honestly, before spending another experiment on a
stricter identity gate. Test any range-abstention or multiple-surface proposal on
these preserved failures and on correct narrow/glass/occluded examples; measure
lost useful ranges and identity support as well as removed mistakes. Masking and
appearance support remain useful, conditional evidence rather than compulsory
agreement between all channels. The exact matching recorder and missing target
coverage remain owed before the next resolver-policy trial.

At the owner's request the rover returned to its starting place after the visual
snapshot. Final navigation reported trusted position, zero motor PWM, 0.136 m
from its original position and 6.6 degrees from its original heading; the camera
again showed the original scene. Battery read about 70%. It was left stopped for
the owner to connect the charger. No further drive is needed for this diagnosis.

The companion [measurement](2026-10-05-visual-evidence-depth.json) includes hashes,
frozen judgments, all 33 same-subject pairs, all 50 painting/chair pairs and depth
reproduction details. Reproduce with the experiment README command.
