"""Everything checked before a staged session, in one go, on the rover.

    python3 session_preflight.py

Prints one line per check and READY or what is wrong: position trusted and
the map settled, no gyro fault, the drift check's fit, the battery, the
camera's looking loop writing pictures (a dead GPU shows as none), and the
shadow runner running, which it starts if it is not (it does not survive a
reboot). Written after M5 session 5, whose checks were typed one by one while
the rover sat unplugged (2026-10-16).
"""
import glob
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import session  # noqa: E402

FRESH_S = 120.0


def main():
    problems = []
    check = session.pose_check()
    nav = session.call("nav_status")
    print("pose %s, trusted %s, settled %s, drift %s m %s deg, fit %s" % (
        check["pose"], check["trusted"], check["settled"], check["drift_m"], check["drift_deg"], check["fit"]))
    if not (check["trusted"] and check["settled"]):
        problems.append("navigation does not trust its position: refit")
    if session.disagrees(check):
        problems.append("the drift check disagrees with the pose")
    if nav.get("gyro_fault"):
        problems.append("gyro fault: %s (power-cycle the rover)" % nav["gyro_fault"])
    pct = session.battery()
    print("battery %s%%" % pct)
    if pct is None or pct < 60:
        problems.append("battery %s%%: charge first" % pct)
    frames = sorted(glob.glob(os.path.expanduser("~/.ugv/world/frames/*.jpg")), key=os.path.getmtime)
    age = time.time() - os.path.getmtime(frames[-1]) if frames else None
    print("newest picture %s s old" % (round(age) if age is not None else None))
    if age is None or age > FRESH_S:
        problems.append("the camera has not written a picture in %s s: GPU or camera down" % age)
    runner = subprocess.run(["pgrep", "-f", "python3 shadow.py"], capture_output=True, text=True).stdout.split()
    runner = [p for p in runner if p != str(os.getpid())]
    if not runner:
        subprocess.Popen("cd ~/m5/what_where && nohup ./run_shadow.sh > ~/.ugv/whatwhere/shadow.out 2>&1 < /dev/null &",
                         shell=True)
        time.sleep(5)
        runner = subprocess.run(["pgrep", "-f", "python3 shadow.py"], capture_output=True, text=True).stdout.split()
        print("shadow runner started: %s" % (runner or "FAILED"))
        if not runner:
            problems.append("the shadow runner did not start")
    else:
        print("shadow runner running: %s" % runner)
    print("READY" if not problems else "NOT READY: " + "; ".join(problems))


if __name__ == "__main__":
    main()
