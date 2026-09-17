"""Hold the OCP CAD Viewer on a printed part, and draw it again after every save.

    python cad/watch.py                    the rail mount, redrawn on save
    python cad/watch.py --section -23      sliced, so the jaws can be seen
    python cad/watch.py --serve            and bring a viewer up first
    python cad/watch.py --list             what can be shown

Everything this does not recognise is handed to the part's own script unchanged,
so whatever `python cad/oak_rail_mount.py --show ...` draws, this draws and then
keeps current.

`watch.cmd` at the root of the repository is the one-command version: it finds
the interpreter that has build123d in it and runs this, so `watch.cmd
--section -23` is a sliced assembly that redraws itself while you edit.

It watches every .py in cad/ -- not just the one being shown -- and when one of
them changes it draws the view again in a FRESH interpreter. That is the whole
reason this is a subprocess and not a reload loop: drawing.py reads its
dimensions out of oak_rail_mount.py, so reloading one module in place would
leave the rest holding the old constants, which is a viewer that lies to you.

A build that fails does not stop it. The traceback prints and the watch carries
on, so the loop is edit, save, look, with no command in between.

WHICH PORT THE VIEWER IS ON IS NOT ASSUMED. 3939 is only where the VS Code
extension starts looking for a free one, so a standalone viewer or a second
window pushes the panel to 3940 and up -- and a watcher aimed at 3939 then sits
waiting with the viewer open on screen in front of it. Every viewer announces
itself in ~/.ocpvscode, so this asks that which viewers exist rather than
asserting where one ought to be, and hands the answer to the part's script in
OCP_PORT, which is what cad/oak_rail_mount.py reads.

THE VS CODE PANEL IS THE DEFAULT and a browser tab is the fallback, because a
part beside the code it is drawn from is the point of watching it at all.
Nothing here can open that panel -- it is a VS Code command, `OCP CAD Viewer:
Open viewer`, and the extension registers no URI handler -- so the wait for one
is a printed instruction rather than something this can do for you. `--serve` is
the other way: it starts the standalone viewer itself and stops it again when
you stop watching.

AND IT STILL PROVES NOTHING. Watching a part rebuild says nothing about whether
it fits: a jaw can close on air and look perfect from outside, which is what the
section view is for and still only to the eye. `python cad/oak_rail_mount.py`
runs the fit checks against a model of the rail and exits non-zero when one
fails, and `python cad/drawing.py` checks the sheets. Those are the gates. This
is for looking at the part while you work on it.
"""

from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

CAD = Path(__file__).resolve().parent
# Where every viewer announces itself, and the port the first one takes.
REGISTRY = Path.home() / ".ocpvscode"
DEFAULT_PORT = 3939
# A module is showable if it has a show() for the viewer to be handed.
SHOWS = re.compile(r"^def show\(", re.MULTILINE)


def targets() -> dict[str, Path]:
    """The parts that can be drawn, by the name you would ask for them by.

    Found rather than listed, so a second printed part becomes watchable by
    having a show() and nothing else.
    """
    found: dict[str, Path] = {}
    for path in sorted(CAD.glob("*.py")):
        if path.name == Path(__file__).name:
            continue
        try:
            if SHOWS.search(path.read_text(encoding="utf-8")):
                found[path.stem] = path
        except OSError:
            continue
    return found


def summary(path: Path) -> str:
    """The first line of a module's docstring, for --list."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return ""
    if not text.startswith(('"""', "'''")):
        return ""
    return text[3:].splitlines()[0].strip()


def sources() -> dict[Path, float]:
    """Every file whose change should redraw, with the time it last changed."""
    stamps: dict[Path, float] = {}
    for path in sorted(CAD.glob("*.py")):
        if path.name == Path(__file__).name:
            continue  # editing the watcher is not a reason to redraw
        try:
            stamps[path] = path.stat().st_mtime
        except OSError:
            continue  # deleted between the glob and the stat; the next pass sees it
    return stamps


def viewer_listening(port: int) -> bool:
    """Whether anything holds the viewer's port, which is all we can ask cheaply."""
    with socket.socket() as probe:
        probe.settimeout(0.25)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def registered_ports() -> list[int]:
    """Every viewer that has announced itself, whether or not it is still there.

    ~/.ocpvscode is the registry all of this shares: the VS Code extension
    writes it as its panel opens, the standalone viewer writes it too, and every
    Python client reads it to find one. A viewer removes its own line as it
    stops -- but one that is killed rather than stopped leaves it behind, so this
    is the candidate list and find_viewer() is the answer.
    """
    try:
        config = json.loads(REGISTRY.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    ports = []
    for name in config.get("services", {}):
        try:
            ports.append(int(name))
        except ValueError:
            continue  # a malformed line is somebody else's to clean up
    return sorted(ports)


def find_viewer(pinned: int | None = None) -> int | None:
    """The port of a viewer that is actually answering, or None."""
    if pinned is not None:
        return pinned if viewer_listening(pinned) else None
    live = [port for port in registered_ports() if viewer_listening(port)]
    if not live:
        # A viewer old enough not to register, or one whose line was lost.
        return DEFAULT_PORT if viewer_listening(DEFAULT_PORT) else None
    if DEFAULT_PORT in live:
        return DEFAULT_PORT  # the conventional one, when there is a choice
    return live[0]


def wait_for_viewer(
    pinned: int | None = None, server: subprocess.Popen | None = None
) -> int | None:
    """Block until a viewer answers, and return the port it answered on.

    The port is looked for again on every pass, because the viewer being waited
    for does not exist yet and will register whichever port it ends up on. With
    no viewer of our own there is nothing to do but wait for a person to open
    one, so the message is the instruction.
    """
    found = find_viewer(pinned)
    if found is not None:
        return found
    if server is None:
        where = f"port {pinned}" if pinned else "any port"
        print(f"no viewer is listening on {where}; waiting for one.")
        print('In VS Code: ctrl-shift-P, "OCP CAD Viewer: Open viewer".')
        print("In a browser: run this again with --serve (watch.cmd --browser).")
    while found is None:
        if server is not None and server.poll() is not None:
            print(f"the viewer exited with {server.returncode} before it listened")
            return None
        time.sleep(1.0)
        found = find_viewer(pinned)
    print(f"viewer is up on port {found}.")
    return found


def viewer_module() -> str | None:
    """Which package serves the standalone viewer, which moved in 4.1.

    The server and the client were one package until OCP CAD Viewer 4.1, and
    only one of them was renamed: what pushes geometry is still `ocp_vscode`,
    which is what cad/oak_rail_mount.py imports, while the standalone server
    moved out into `ocp_viewer`. `python -m ocp_vscode` does not fail once that
    has happened -- it prints where the viewer went and exits 0 -- so asking
    which package is here beats finding out from a viewer that never listened.
    """
    from importlib.util import find_spec

    for name in ("ocp_viewer", "ocp_vscode"):
        try:
            if find_spec(name) is not None:
                return name
        except (ImportError, ValueError):
            continue
    return None


def serve_viewer(port: int) -> subprocess.Popen | None:
    """Start the standalone viewer ourselves, unless something already answers.

    An open OCP CAD Viewer panel in VS Code is a viewer, and a second server on
    the same port would only fail to bind it, so a port that answers is left
    alone and there is then nothing of ours to stop afterwards.
    """
    if viewer_listening(port):
        print(f"a viewer already holds port {port}; using that one")
        return None
    module = viewer_module()
    if module is None:
        print("neither ocp_viewer nor ocp_vscode is installed; cannot serve one")
        return None
    print(f"starting the standalone viewer on port {port} ({module})")
    command = [sys.executable, "-m", module, "--port", str(port)]
    # Its own process group off Windows, so that stopping it later stops what it
    # started underneath it rather than orphaning the server.
    group = {} if sys.platform == "win32" else {"start_new_session": True}
    return subprocess.Popen(command, cwd=CAD.parent, **group)


def open_viewer_page(port: int, module: str | None) -> None:
    """Put a viewer we started on screen, if it does not do that itself.

    Up to OCP CAD Viewer 4.0 the standalone server opened a browser tab as it
    came up, and 4.0.1 is what this repository has. `ocp_viewer` serves the page
    and waits to be visited instead, which is a watcher listening, watching and
    redrawing with nothing on screen to redraw onto -- so the tab is ours to open
    only in that case.
    """
    if module != "ocp_viewer":
        return
    url = f"http://127.0.0.1:{port}"
    print(f"opening {url}")
    try:
        opened = webbrowser.open(url)
    except OSError as failure:  # no browser to open, on a headless machine
        opened = False
        print(f"  could not: {failure}")
    if not opened:
        print(f"  open {url} yourself to see the part")


def stop_viewer(server: subprocess.Popen | None) -> None:
    """Take a viewer we started down with us, children and all."""
    if server is None or server.poll() is not None:
        return
    print("stopping the viewer")
    if sys.platform == "win32":
        subprocess.run(
            ["taskkill", "/F", "/T", "/PID", str(server.pid)], capture_output=True
        )
    else:
        os.killpg(os.getpgid(server.pid), signal.SIGTERM)
    try:
        server.wait(timeout=10)
    except subprocess.TimeoutExpired:
        server.kill()


def draw(command: list[str], port: int) -> bool:
    """Build and push the view once, letting the part's own output through.

    The port goes in the environment rather than on the command line because
    OCP_PORT is what cad/oak_rail_mount.py reads, and a part added later gets
    the same treatment for free by reading it too.
    """
    env = {**os.environ, "OCP_PORT": str(port)}
    started = time.time()
    done = subprocess.run(command, cwd=CAD.parent, env=env)
    seconds = time.time() - started
    if done.returncode == 0:
        print(f"  drawn in {seconds:.1f}s")
        return True
    print(f"  FAILED after {seconds:.1f}s (exit {done.returncode}); still watching")
    return False


def changed_names(before: dict[Path, float], after: dict[Path, float]) -> str:
    """The files that moved, for the one line printed before each redraw."""
    names = sorted(
        {p.name for p, when in after.items() if before.get(p) != when}
        | {p.name for p in before if p not in after}
    )
    head = ", ".join(names[:3])
    return head if len(names) <= 3 else f"{head} and {len(names) - 3} more"


class Options:
    """What was asked for, and what goes through to the part's own script."""

    def __init__(self) -> None:
        self.port: int | None = None
        self.interval = 0.5
        self.serve = False
        self.browser = True
        self.listing = False
        self.target: str | None = None
        self.forwarded: list[str] = []


def parse(argv: list[str], names: dict[str, Path]) -> Options:
    """Take this script's own arguments out and leave the rest alone.

    Hand-written rather than argparse because of `--section -23`: argparse reads
    a bare negative number as a positional and would take it for the name of a
    part. So the only thing treated as a name here is a name that exists, and
    everything unrecognised is forwarded exactly as it arrived.
    """
    options = Options()
    rest = list(argv)
    while rest:
        arg = rest.pop(0)
        low = arg.lower()
        if low in ("-h", "--help"):
            print(__doc__)
            raise SystemExit(0)
        elif low in ("--serve", "--browser"):
            options.serve = True
        elif low in ("--no-browser", "--no_browser"):
            options.browser = False
        elif low == "--list":
            options.listing = True
        elif low in ("--port", "--interval"):
            if not rest:
                raise SystemExit(f"{arg} needs a value")
            value = rest.pop(0)
            try:
                if low == "--port":
                    options.port = int(value)
                else:
                    options.interval = float(value)
            except ValueError:
                raise SystemExit(f"{arg} {value}: not a number") from None
        elif arg in names or arg.removesuffix(".py") in names:
            options.target = arg.removesuffix(".py")
        else:
            options.forwarded.append(arg)
    return options


def main(argv: list[str]) -> int:
    names = targets()
    options = parse(argv, names)

    if options.listing or not names:
        for name, path in names.items():
            print(f"  {name:<20} {summary(path)}")
        if not names:
            print("nothing in cad/ has a show(); there is nothing to watch")
            return 1
        return 0

    if options.target is None:
        if len(names) > 1:
            print("name the part to watch; cad/ has more than one:")
            for name in names:
                print(f"  {name}")
            return 2
        options.target = next(iter(names))
    script = names[options.target]

    # The view runs as a child that writes to this same stdout, so a buffered
    # parent would print its own lines out of order behind the child's -- and a
    # watcher's whole job is to say what it is doing as it does it.
    sys.stdout.reconfigure(line_buffering=True)

    # A viewer we start is one we choose the port of; one we find is one we ask
    # the registry about. Either way the number is known before anything is
    # drawn, and the part's script is then told it rather than repeating the
    # search for itself.
    served = options.port or DEFAULT_PORT
    module = viewer_module() if options.serve else None
    server = serve_viewer(served) if options.serve else None
    pinned = served if options.serve else options.port

    try:
        port = wait_for_viewer(pinned, server)
        if port is None:
            return 1
        if server is not None and options.browser:
            open_viewer_page(port, module)
        command = [sys.executable, str(script), "--show", *options.forwarded]

        watched = sources()
        print(f"watching {len(watched)} files in cad/; ctrl-c to stop")
        print(f"{time.strftime('%H:%M:%S')} first view of {options.target}")

        while True:
            if not draw(command, port):
                # A closed panel recovers, and recovers onto whatever port it
                # comes back on, which is why the port is looked up again --
                # a failed draw is as often a viewer that went away as a part
                # that will not build.
                port = wait_for_viewer(pinned, server)
                if port is None:
                    return 1
            now = sources()
            while now == watched:
                time.sleep(options.interval)
                now = sources()
            while True:  # let the burst finish before building anything
                time.sleep(options.interval)
                settled = sources()
                if settled == now:
                    break
                now = settled
            print(f"{time.strftime('%H:%M:%S')} {changed_names(watched, now)}")
            watched = now
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        stop_viewer(server)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
