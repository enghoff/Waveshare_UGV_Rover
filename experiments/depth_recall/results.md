# Blind depth recall: what came out

The stop rule triggered. The progress entry on main,
`docs/progress/2026-10-13-blind-depth-recall.md`, is the account of record;
this file is the branch's own summary, for whoever reruns it.

| Target | Sources | Measured area | Warped useful | Unavailable | Wrong | p50 error | Copy useful |
|---|---|---|---|---|---|---|---|
| 091624-3b7439 | 091400 | 57% | 5% | 79% | 16% | 0.36 m | 11% |
| 091626-f16692 | 091400, 091411 | 53% | 17% | 55% | 27% | 0.27 m | 15% |
| 091637-12cb55 | 091400, 091411 | 59% | 15% | 61% | 24% | 0.26 m | 14% |
| 091652-c624cd | 091409, 091417 | 54% | 24% | 41% | 35% | 0.24 m | 3% |
| 091657-e6d359 | 091400, 091411 | 45% | 32% | 32% | 37% | 0.21 m | 4% |
| 091705-5c76e6 | 091331 | 68% | 31% | 35% | 33% | 0.18 m | 4% |

Two of six reach 30%; warping beats copying on five, by a median of 12
points. Scored on raw valid depth instead of the surface rule, none of the six
reaches 30%. "Beats copying" was made precise in the protocol as better on at
least four targets with a positive median difference; the plan said only
"beats".

Reproduce, from this directory, with the evidence paths of the main checkout:
`predict.py` then `score.py` (protocol.md has the arguments), and
`diagnose.py development` and `diagnose.py after` for the checks before and
after. Codex regenerated the six predictions to the committed hashes and
reproduced every score.

Limits: six correlated targets from one four-minute recording, sharing their
sources; baselines 0.73-0.94 m; three of the six targets and three of the five sources taken while
moving, with depth within 29 ms of the shutter. It says nothing about recall
from 25 cm, or about memory fused from many looks.
