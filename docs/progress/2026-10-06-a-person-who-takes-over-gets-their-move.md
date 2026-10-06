# A person who takes the rover back mid-leg now gets the move they asked for

**A person's move sent while an autonomous leg is driving ends the run, as it
always did, and is now carried out rather than refused.** The record of
2026-10-02's takeover trial (S3, `captures/m3-stop-2026-10-02/driver_S3.log`)
shows the fault. The console-style `drive_to` sent mid-leg latched autonomy off
and stopped the rover, and then came back `{"ok": false, "reason": "busy"}`.
The stopped leg had not yet let go of navigation's move lock. So the person
had stopped the rover and not moved it. The voice model's `drive` and
`turn_in_place` have no hand-over of their own and would have met the same
refusal. The console's clicks and `go_to_thing` already wait for one, 6 s and
3 s. This is M3's criterion 12, "concurrent voice/manual requests follow the
declared priority". The priority is declared in
[rover_daemon/README.md](../../rover_daemon/README.md): any person takes the
rover back.

## The fix

`Rover.call` is the one place every person's call passes. When the call has
just ended an open run and is itself a move (`PERSON_MOVES`), it now waits for
the navigator to have no move running before going ahead: at most 3 s, the
same as `go_to_thing`, polled every 50 ms. The wait is outside the autonomy
lock, because the stopped leg reports its end through that lock. It holds no
move lock and adds nothing to any move that is not taking over from a run. A
leg that has still not let go after 3 s is refused as busy, as before.

## Evidence

- **Reproduced first.** `test_a_person_who_takes_the_rover_back_gets_what_they_asked_for`
  uses a navigator whose stopped move holds the wheels for 0.3 s, as the real
  one does. A person's turn mid-leg ended the run and was refused as busy: the
  check failed on the code before the fix. It passes now.
- rover_daemon 1119, autonomy 785, drive_web 608 passed here.
- Deployed at the commit after this entry's (rover_daemon only), with the
  suite passing on the rover. **Not yet seen on the rover.** The next
  session's voice trial is where it will be: the owner tells the voice model to
  turn while a leg is moving.
