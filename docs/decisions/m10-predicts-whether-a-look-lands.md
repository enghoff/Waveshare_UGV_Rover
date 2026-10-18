# M10 predicts whether a look lands on its record, and is tested on a trial run after the predictor was frozen

Status: re-aimed with the owner on 2026-10-18; the acceptance below set the same
day, before the trial it is judged on. Narrows what
[M10](../plans/autonomous-curiosity.md#milestone-m10-learned-outcome-prediction-improves-planning-on-held-out-experience)
predicts for the goal type that exists, a look at one of the world model's
records; its six criteria stand.

**The outcome M10 predicts is whether a look lands**: whether the world model
files a region of its picture to the record the look was aimed at. Not how many
metres the look takes off the record's placement.

## Why

**Predicting the gain in metres would mostly teach the rover not to look.**
Replayed over the history, 20 of 502 looks improved the placement of the record
they were aimed at. 93 of the 115 that reached it found it already at the
model's floor of 0.10 m, and 382 never reached it at all
([the measurements](../../experiments/m10_outcomes/README.md)). A predictor of
the gain would learn that a look is worth nothing and the planner would stop
looking, which saves battery and leaves the rover knowing no more.

**Whether a look lands can be learned, and it is the step that fails.** 115 of
430 looks landed, and what decides it is mostly how long ago the record was
last seen -- 41% under three hours, 2% after a day, by the model's own rule for
records it has not seen for three hours -- and then whether a record at least as
established stands within 0.8 m to take the picture. On the history a logistic
regression on those and five other features beat a constant and the planner's
old rule (a half for a record seen on six visits or more) on looks held out by
time: Brier 0.131 against 0.169 and 0.234. That is development, because the
features were chosen after looking at all 430 looks.

## The predictor, frozen

`autonomy/landing.py` with the numbers in `autonomy/landing.json`, digest
`088b4d51a9a9`, deployed at `87a89bf` on 2026-10-18. The planner takes a
look's predicted gain as that chance times what filing it would gain, records
both and the digest with each goal, and uses the old half when the numbers are
missing or unfit.

## The acceptance, set before the trial

**The test is a look trial**, `experiments/m10_outcomes/look_trial.py`: the
survey loop out and back, and at each stop up to six looks, drawn at random among
the records in reach (placed, a height known, seen on two visits or more, 0.5 to
2.5 m away with nothing mapped between, in the depth camera's view), each
recorded with its predicted chance before it is taken. A look is scored after
the model has filed the trial's pictures (`landed.py`).

1. **Held out by time.** Every look of the trial is taken after the predictor
   was frozen.
2. **The baselines** are the history's share of looks that landed, 115 of 430,
   as a constant, and the old rule.
3. **It beats them** when its Brier score is below both, and the 95% bootstrap
   interval of its difference from the constant is below zero.
4. **Ranking, offline:** at each stop, the look the new ranking would choose
   (chance times what it gains if it lands) lands at least as often over the
   stops as the old ranking's choice (what it gains if it lands), on the looks
   taken there, whose outcomes are all known. Physical benefit is claimed only
   after supervised runs with and without the predictor confirm it, planned
   after this.
5. **Calibrated** when every bin of the prediction holding 20 looks or more
   lands within 0.10 of what it predicted; every bin is reported.
6. **A predictor that is missing or unfit leaves the old half in charge**,
   which the tests check (`autonomy/test_goals.py`).

At least 80 scored looks. One trial gives about 94; if it gives fewer, a
second is run before anything is judged.

## What it leaves

A predictor of landing tells the rover which looks are worth taking. It does
not make more of them land. Most of the misses are the world state's: second
and stray records standing beside the one a thing is really held under, and
records unseen for a day that the model will not take a new picture into.
That is *Let looks reach their things* in the plan's next steps.
