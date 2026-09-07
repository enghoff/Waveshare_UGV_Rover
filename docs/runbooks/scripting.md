# Rover-side scripting

The daemon can run short Python programs that compose its existing tools. Scripts
are useful for sequences and repetition that do not justify a permanent tool.
They orchestrate behavior; they do not implement servo, camera or navigation
control loops.

## Execution model

A script runs as a child process under the same `jetson` account as the daemon.
It reaches hardware only through the daemon's newline-delimited JSON protocol on
loopback port 8769. The daemon remains the sole owner of the UART, camera and
navigation state.

This is process isolation, not a filesystem sandbox. A script can access files
available to the rover account. Code submission and control are therefore
restricted to loopback clients.

The daemon permits one script at a time. A blocking `run_script` has a default
15-second execution limit. A background `start_script` has no deadline unless
the caller supplies one. Both are watched against a 96 MB memory limit. Ending,
failing or stopping a script asks the daemon to stop the wheels.

## Calls

- `run_script` runs source and waits for the result;
- `start_script` starts a background behavior and returns a handle;
- `script_status` reports the active or most recent run;
- `script_stop` stops the active run;
- `list_api` returns the current scripting reference.

Submission and stopping are loopback-only. The realtime voice session runs on
the rover, so it can use the scripting tools without exposing code execution on
the LAN.

## API

Programs start with the public API already in their namespace. Imports remain
supported but are unnecessary. The main namespaces are `drive`, `gimbal`,
`camera`, `lights`, `status`, and `wifi`; `call()` reaches any daemon tool by
name. `wait()`, `every()` and `alongside()` provide pacing and concurrency.

```python
for tick in every(0.5, for_s=3):
    lights.set(255 if tick % 2 == 0 else 0)
lights.set(0)
```

Two actions can overlap when one is passed as a callable to `alongside`:

```python
def flashing():
    for tick in every(0.5):
        lights.set(255 if tick % 2 == 0 else 0)

with alongside(flashing):
    drive.turn(90)
lights.set(0)
```

A failed rover call raises `RoverError`. A stop noticed by `wait` or `every`
raises `Stopped`; a supplied deadline raises `Deadline`. Cleanup belongs in
`finally` blocks or after an `alongside` block.

## Limits

Use behavior-sized calls. Repeated raw motor or gimbal writes would compete with
the control loops already responsible for timing and safety. Pace polling with
`every`; an unbounded loop can consume a core even when each individual call is
valid.

Scripts cannot make the lidar see drops, steps or obstacles outside its scan
plane. They inherit each underlying tool's limits. A code endpoint also cannot
make unsafe source trustworthy merely because it arrived from a model.

Saved behavior catalogs, persistent script state, per-script capabilities and a
vision question primitive are not implemented. Add them only for a reproduced
use case rather than extending the surface speculatively.

## Verification

```bash
python rover_daemon/test_scripting.py
python rover_daemon/selftest.py
```

`voice_chat/mock_rover.py` implements the same wire protocol for testing programs
without hardware. Final verification for changes to deployed scripting code is a
call through TCP 8769 on the rover, followed by confirmation that the daemon and
movement stop behavior remain healthy.
