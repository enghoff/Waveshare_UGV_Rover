"""Run the bridge's real dispatch method without importing ROS on the desk."""
import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import autonomy_guard
from test_harness import check


def test_guard_checks_detour_adjusted_goal_and_stop():
    guard = {"stop_seq": 4, "geofence": {"radius_m": 3.}}
    check("autonomous goal cannot survive a stop",
          bool(autonomy_guard.refusal(guard, 5, pose=(0,0))), True)
    check("inside destination remains admissible",
          autonomy_guard.refusal(guard, 4, pose=(0,0), goal=(1,0)), "")
    check("fitted destination outside inset is refused",
          bool(autonomy_guard.refusal(guard, 4, pose=(0,0), goal=(2.8,0))), True)
    path = NS(poses=[NS(pose=NS(position=NS(x=x,y=y)))
                    for x,y in [(0,0),(1,0),(3.2,0),(1,1)]])
    check("route detour is refused despite inside endpoints",
          "planned route" in autonomy_guard.refusal(guard,4,pose=(0,0),goal=(1,1),path=path), True)
    check("missing pose fails closed for a fenced journey",
          bool(autonomy_guard.refusal(guard,4)), True)


def test_stop_during_goal_acceptance_cancels_late_handle():
    source = ast.parse((Path(__file__).parent / "nav_moves.py").read_text())
    cls = next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='NavMoves')
    method = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run_goal')
    scope = {"autonomy_guard":autonomy_guard,"time":time,"PROGRESS_S":1.}
    exec(compile(ast.Module(body=[method],type_ignores=[]),"nav_moves.py","exec"),scope)
    class Future:
        def done(self): return True
        def result(self): return handle
    class Handle:
        accepted = True
        cancelled = 0
        def cancel_goal_async(self): self.cancelled += 1
        def get_result_async(self): return Future()
    handle = Handle()
    def send(*args,**kwargs):
        node.stop_seq += 1  # halt arrived before active_goal existed
        return Future()
    client=NS(wait_for_server=lambda **kw:True, send_goal_async=send)
    node=NS(actions={"goto":client},_lock=threading.Lock(),stop_seq=0,estop=False,
            pose=lambda:(0,0,0),dead_reckoned=lambda:(0,0,0),
            wait=lambda *args:True,finish=lambda *args:{"reason":"stopped"})
    scope['run_goal'](node,"goto",object(),5,lambda *a,**kw:None,lambda x:{},
                      guard={"stop_seq":0})
    check("late accepted goal is cancelled after stop",handle.cancelled,1)


TESTS=(test_guard_checks_detour_adjusted_goal_and_stop,
       test_stop_during_goal_acceptance_cancels_late_handle)
