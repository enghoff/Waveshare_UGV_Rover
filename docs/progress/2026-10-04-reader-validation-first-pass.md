# First-pass labels reduce the owner's review to six repeated issues

The owner requested an analyst first pass and review where doubtful. All 232
regions in the fresh recording now have draft judgments: 153 appear clear and
79 need review, grouped into six illustrated questions. These remain coding-agent
labels, not owner-confirmed ground truth. R-WS-13 remains open; R-WS-17 and
R-WS-18 remain proposed. No candidate was scored and no new observations were
used for training or tuning.

## What was inspected

All 27 full photographs were inspected without consulting current entity
assignments, earlier labels or candidate results. Raw crops were checked for
ambiguous cases, followed by 101 saved outlines for overlapping, uncertain or
non-object regions. The original blank sheet was preserved. Draft labels are in
`captures/2026-10-04-reader-validation/owner-review/labels-first-pass.csv`, with
confidence, review group, author and unconfirmed status recorded explicitly.

The outlines changed several initial interpretations: five broad dining boxes
primarily select table legs/apron while excluding the chair backs, two others
isolate one chair, and five ceiling boxes select the red conduit. Crops alone
would have wrongly called some of those multi-object regions. Four clipped
painting regions and three lampshade views became clear after the outline check.
These are analyst judgments rather than measured acceptance results.

The 232 draft verdicts are 201 object regions, 18 mixed, 8 surface/structural
features and 5 unclear. There are 23 proposed object identities, only 16 of which
have a clear first-pass example. Those counts are not an independent object
census and do not establish the planned minimum of 20 distinguishable objects.
The head and body boxes 64552 and 64560 in one frame are drafted as one person;
same-frame detections must not silently be assumed to be different physical
identities when evaluating this working person policy.

## Review needed

`owner-review/first-pass-review.html` contains raw and outlined examples, full-frame
links, all affected observation IDs and an answer field for each repeated issue:

1. Cross-view identity of the front-left, front-right and side-on dining chairs.
2. Whether door regions with chair/table overlaps, and a person/chair region,
   have one clear principal object or should be mixed.
3. Which kitchen regions represent several fixtures versus one assembly. The
   pictures show sink/counter/cabinetry; calling them a refrigerator is unsupported.
4. Whether side-on and underneath pendant views show the same ceiling fixture.
5. A clipped glass/cup, a black angled item and a blurred floor power extension.
6. Physical scope/category of the hallway frame, glazed opening, upper kitchen
   opening and red ceiling conduit.

The reviewer can reply by question number or download free-text answers from
the page. Group answers must be interpreted against their examples before
changing any labels; no automatic confirmation or mass reassignment is built in.
Checking only doubts does not make the other coding-agent labels independent.
Further independent scrutiny is required before any acceptance claim.

All source IDs, frame IDs and boxes were verified unchanged, the original CSV
remained blank, and all 71 page links/images resolved. Nine existing evaluation
and review tests pass; documentation links and identifiers resolve. The
[compact provenance](2026-10-04-reader-validation-first-pass.json) records hashes
and counts. Only workstation tooling and documents changed; no deployment or
rover movement was needed.
