"""Read out the crash dump the device stores after a firmware crash, then clear it.

The RVC2 keeps its last firmware crash dump until something reads it out. Two
reasons to run this:

* The report names what the firmware tripped on -- an assert with file and line,
  or a hardware trap -- which is the only direct evidence of a device-side fault.
* Reading it with `clearCrashDump=True` erases it. Under depthai 3.x a stored
  dump makes every subsequent `close()` segfault the host process while trying to
  archive it, so a run that worked perfectly still ends in a stack trace. depthai
  2.x does not have that bug, but clearing a stale dump keeps the next diagnostic
  honest -- otherwise `hasCrashDump()` stays true and points at an old crash.
"""

import json
from pathlib import Path

import depthai as dai

OUT_DIR = Path(__file__).parent / "crash_dumps"


def main() -> int:
    # USB2-only firmware; the USB3 build usually fails to boot here. See docs/oak-usb-link.md.
    with dai.Device(dai.Pipeline(), maxUsbSpeed=dai.UsbSpeed.HIGH) as device:
        mxid = device.getMxId()
        if not device.hasCrashDump():
            print(f"{mxid}: no crash dump stored, nothing to clear.")
            return 0

        dump = device.getCrashDump(clearCrashDump=True)
        record = json.loads(dump.serializeToJson())

        # The schema differs by major version: 2.x nests the cause under
        # crashReports[].errorSourceInfo, 3.x flattens it into reports[].
        # Reading only one key silently prints nothing for the other.
        reports = record.get("crashReports") or record.get("reports") or []
        for report in reports:
            info = report.get("errorSourceInfo", report)
            assert_ctx = info.get("assertContext", {})
            trap_ctx = info.get("trapContext", {})
            print(f"processor={report.get('processor')}"
                  f" errorSource={info.get('errorSource')}"
                  f" errorId={info.get('errorId')}")
            if assert_ctx.get("fileName"):
                print(f"  ASSERT {assert_ctx['fileName']}:{assert_ctx.get('line')}"
                      f" in {assert_ctx.get('functionName')}")
            if trap_ctx.get("trapName") or trap_ctx.get("trapAddress"):
                print(f"  TRAP {trap_ctx.get('trapName')}"
                      f" number={trap_ctx.get('trapNumber')}"
                      f" address={trap_ctx.get('trapAddress')}")
            # The firmware console buffer -- board init, sensor enumeration and
            # whatever it managed to say before dying.
            for line in report.get("prints", []):
                print(f"  print: {line}")

        OUT_DIR.mkdir(exist_ok=True)
        # The timestamp reads "2026-08-11 17:14:33"; colons and spaces are not
        # legal in a Windows filename.
        stamp = str(record.get("crashdumpTimestamp", "unknown"))
        stamp = stamp.replace(":", "").replace(" ", "T")
        path = OUT_DIR / f"{mxid}-{stamp}.json"
        path.write_text(json.dumps(record, indent=2))
        print(f"saved {path}")
        print(f"crash dump still present: {device.hasCrashDump()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
