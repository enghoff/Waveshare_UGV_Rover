# Letting a standing record answer ahead of a gone one gains one right answer and two wrong ones on the benchmark's 54

A gone record of the red bean bag outscored its standing one and answered
([2026-10-21](2026-10-21-what-the-world-model-switches-do-to-the-answers.md)).
So `find_thing`'s ordering was tried with one change: when the record that would
answer is seen gone or set aside, a standing record scoring within a margin of
it answers instead. Standing records are ordered among themselves as before, which
is what broke three of the six controls when the tie was widened for every
record ([2026-10-20](2026-10-20-recency-does-not-let-a-record-give-way.md)).

Scored on the rover's own rankings of the benchmark's 54 answers
(`tmp/md-days/md-day*-ranked.json`: per state, each question's 25 best records
as the rover had them), right within 1.0 m:

| Margin | Right | Wrong | Unresolved | Controls right, of 36 |
|---|---|---|---|---|
| 0 (deployed) | 41 | 8 | 5 | 36 |
| 0.02 or 0.03 | 42 | 10 | 2 | 36 |
| 0.05 or more | 42 | 12 | 0 | 36 |

The answer gained is day 2's red bean bag in the morning's state, from its
standing record at P1. The answers lost are day 1's stool in the morning's
state and in autonomy's. Its records there were seen gone, and the record that
answers instead is another, standing, in the wrong place. The rover used to say
the stool had gone from where it was; it would send someone to the wrong place.
The benchmark's third criterion asks for more right answers and no more wrong
ones, so this is not adopted.

`gone_margin_rule.py` in the session's scratchpad ran it; it reads only the
rankings file.

No requirement changed state. R-WS-17 stays proposed.
