# Test a bridge from established evidence to a new appearance

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. The first two-confirmed-
witness candidate has now failed both recordings. Its cold-start replay confirms
only 131 of the baseline's 3,716 correct pairs, exposing a bootstrap restriction:
two unfamiliar views cannot corroborate each other, even when geometrically sound.
Do not treat that as evidence that a tentative-evidence architecture is impossible.

Fix this specific restriction with one additional, predeclared promotion path.
Two tentative views proposing the same entity may be promoted together when:

- their plain and masked vectors both match at the unchanged 0.70 recognition
  threshold, with compatible backend/width and distinct inference frames;
- each is connected to at least one pre-existing confirmed view in both channels
  at the unchanged 0.55 different-object floor;
- at least one of the pair has a strong 0.70 connection in both channels to a
  pre-existing confirmed view;
- their rays produce a valid ordinary `locate.fix`: at least 0.4 m baseline,
  12 degrees parallax, forward intersections, visibility, range and height checks;
- that fix agrees with the current entity placement within the existing hard
  squared ellipse limit 13.8155, including both uncertainties and extents.

Retain the first candidate's direct promotion path; all new confirmation decisions
are staged against pre-round confirmed peers. Geometry and strong connections must
not be supplied by the very pair being promoted. The latest 24 tentative peers per
entity are tried for a bridge; archived observations remain intact. No labels enter
decisions, no cutoff is tuned, and no unconnected pair creates a second model.

Use exactly the same recorded schedules, baseline controls, labels, source snapshots,
founder exemptions and success criteria as commit c0b1c42. Evaluate both fresh and
cold-start older replay, not just the warm fresh result. In addition, preserve
promotion witnesses and distinguish bridge from direct promotions. This is a
development revision motivated by an observed mechanism; it is not independent
acceptance, even if it passes.

If the bridge still fails retention or introduces wrong confirmed relations, stop
automatic integration and proceed to the controlled evidence drive. That drive is
then needed to test clean founding views and genuine viewpoint connections before
another confidence policy is chosen. The next step is not a lowered threshold or
a phrase list tuned to these scores.
