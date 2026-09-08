from test_executive import Session
from test_harness import check
import client
import events
import summary


def test_recovery_stop_does_not_forgive_failed_goals():
    session = Session()
    original = session.rover._autonomy_status
    def failed(args):
        if session.rover.driving:
            session.rover.arrive(reason="blocked")
        return original(args)
    session.rover._autonomy_status = failed
    result = session.executive.loop(turns=4)
    check("three failed goals exhaust the failure budget", len(session.rover.moves), 3)
    check("recovery stops do not reset failures", session.rover.permission.run.failures, 3)
    session.rover.watchdog()
    check("failure run ends", session.rover.permission.status()["enabled"], False)
    session.close()


def test_interruption_keeps_dispatched_move():
    session = Session()
    original = session.rover._autonomy_status
    def stopped(args):
        if session.rover.driving:
            session.rover.driving = False
            session.rover.permission.stop(by="review")
        return original(args)
    session.rover._autonomy_status = stopped
    got = session.executive.once()
    calls = session.calls(got["episode"])
    check("interrupted move is recorded", len(calls), 1)
    check("interruption is not reported as completed", calls[0]["result"]["completion_known"], False)
    kinds = [e["kind"] for e in session.events(got["episode"])]
    check("intent precedes the result", kinds.index("dispatch") < kinds.index("call"), True)
    session.close()


def test_lost_dispatch_reply_leaves_intent_and_unknown_outcome():
    session = Session()
    original = session.rover._autonomy_act
    def lost(args):
        answer = original(args)
        if args["action"] == "drive_to":
            raise client.Unreachable("reply lost after dispatch")
        return answer
    session.rover._autonomy_act = lost
    got = session.executive.once()
    check("lost reply still records dispatched drive", session.calls(got["episode"])[0]["call"], "drive_to")
    check("lost reply closes episode interrupted", got["outcome"], "interrupted")
    session.close()


def test_kill_between_intent_and_answer_is_not_called_nothing():
    session = Session()
    episode = session.store.open_episode("review")
    session.store.append(episode, events.make("dispatch", {
        "action_id": episode + "#1", "call": "drive_to", "params": {"x_m": 1., "y_m": 0.}}))
    said = summary.of(session.store, episode)
    check("unanswered dispatch is visible after a kill", "dispatch/completion unknown" in said, True)
    check("unanswered dispatch is not absence of movement", "called nothing" in said, False)
    session.close()


TESTS = (test_recovery_stop_does_not_forgive_failed_goals,
         test_interruption_keeps_dispatched_move,
         test_lost_dispatch_reply_leaves_intent_and_unknown_outcome,
         test_kill_between_intent_and_answer_is_not_called_nothing)
