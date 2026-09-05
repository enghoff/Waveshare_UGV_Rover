# Working on the web console

The repository-wide rules are in [../AGENTS.md](../AGENTS.md); what the console
is and how it runs is in [README.md](README.md).

## The console shows state, not explanations

The person at the console owns the rover and has used it before. **Do not add
descriptive text to the web console**: no help line under a control, no sentence
saying what a button does, no tooltip restating a label. That belongs in the README.

Status stays: a reading, a count, what is happening now, what a button did, what
failed and why. If a line could have been written before the rover was switched on,
it is documentation, not status. Prefer `cleared -- 4 entities, 96 observations`
over a sentence.
