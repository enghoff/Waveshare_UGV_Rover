#!/usr/bin/env python3
"""Deploy committed rover code over SSH, then restart and verify affected services.

The repository remains the source of truth.  This program packages only files
tracked by git, copies them to the host described by deploy/manifest.json, runs
the component's existing restart/verification commands, and records a per-
component deployed commit on the remote host.

No third-party Python packages are required.  The workstation needs git, ssh and
scp; the Linux targets need python3, tar and rsync (rsync is used only on the
remote side for directory-shaped components).
"""

from __future__ import annotations

import argparse
import base64
import fnmatch
import io
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = Path(__file__).with_name("manifest.json")
STATE_VERSION = 1


class DeployError(RuntimeError):
    pass


def run(args: list[str], *, cwd: Path = ROOT, input_text: str | None = None,
        capture: bool = False, check: bool = True) -> subprocess.CompletedProcess[str]:
    if capture:
        return subprocess.run(args, cwd=cwd, input=input_text, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              check=check)
    return subprocess.run(args, cwd=cwd, input=input_text, text=True, check=check)


def git(*args: str) -> str:
    proc = run(["git", *args], capture=True)
    return proc.stdout.strip()


def require_tools() -> None:
    missing = [name for name in ("git", "ssh", "scp") if shutil.which(name) is None]
    if missing:
        raise DeployError("missing local command(s): " + ", ".join(missing))


def load_manifest() -> dict[str, Any]:
    with MANIFEST_PATH.open("r", encoding="utf-8") as f:
        manifest = json.load(f)
    if manifest.get("version") != 1:
        raise DeployError(f"unsupported manifest version: {manifest.get('version')!r}")
    return manifest


def assert_repo() -> None:
    actual = Path(git("rev-parse", "--show-toplevel")).resolve()
    if actual != ROOT.resolve():
        raise DeployError(f"run from this checkout; expected git root {ROOT}")


def assert_clean() -> None:
    # Untracked files are ignored because packaging is based on `git ls-files`.
    # Tracked changes would make the recorded SHA lie about what was deployed.
    dirty = git("status", "--porcelain", "--untracked-files=no")
    if dirty:
        raise DeployError(
            "tracked files are modified. Commit them first so the deployed SHA "
            "describes the bytes on the rover.\n" + dirty
        )


def tracked_files_with_modes() -> dict[str, int]:
    out = git("ls-files", "-s")
    result: dict[str, int] = {}
    for line in out.splitlines():
        if not line:
            continue
        meta, path = line.split("\t", 1)
        mode = int(meta.split()[0], 8)
        # Git uses 100755/100644. Tar wants only permission bits.
        result[path.replace("\\", "/")] = mode & 0o777
    return result


def matches(path: str, patterns: Iterable[str]) -> bool:
    return any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns)


def component_files(component: dict[str, Any], tracked: dict[str, int]) -> list[tuple[str, str, int]]:
    """Return (repo path, archive path, mode), detecting flatten collisions."""
    selected: dict[str, tuple[str, str, int]] = {}
    for source in component["sources"]:
        patterns = source["patterns"]
        strip = source.get("strip", "")
        for repo_path, mode in tracked.items():
            if not matches(repo_path, patterns):
                continue
            if strip:
                prefix = strip.rstrip("/") + "/"
                if not repo_path.startswith(prefix):
                    raise DeployError(f"{repo_path}: source strip {strip!r} does not match")
                archive_path = repo_path[len(prefix):]
            else:
                archive_path = repo_path
            archive_path = str(PurePosixPath(archive_path))
            previous = selected.get(archive_path)
            if previous and previous[0] != repo_path:
                raise DeployError(
                    f"component {component['name']}: {previous[0]} and {repo_path} "
                    f"both deploy as {archive_path}"
                )
            selected[archive_path] = (repo_path, archive_path, mode)
    return sorted(selected.values(), key=lambda item: item[1])


def diff_paths(base: str, head: str, patterns: Iterable[str], *, deleted_only: bool = False) -> list[str]:
    if base == head:
        return []
    args = ["git", "diff"]
    if deleted_only:
        args.append("--diff-filter=D")
    args.extend(["--name-only", f"{base}..{head}", "--"])
    proc = run(args, capture=True)
    paths = [line.strip().replace("\\", "/") for line in proc.stdout.splitlines() if line.strip()]
    return [path for path in paths if matches(path, patterns)]


def changed_paths(base: str, head: str, patterns: Iterable[str]) -> list[str]:
    return diff_paths(base, head, patterns)


def deleted_paths(base: str, head: str, patterns: Iterable[str]) -> list[str]:
    return diff_paths(base, head, patterns, deleted_only=True)


def all_patterns(component: dict[str, Any]) -> list[str]:
    patterns: list[str] = []
    for source in component["sources"]:
        patterns.extend(source["patterns"])
    patterns.extend(component.get("triggers", []))
    return patterns


def ssh(host: str, command: str, *, capture: bool = False,
        input_text: str | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
    return run(["ssh", host, command], capture=capture, input_text=input_text, check=check)


def state_cat_command(path: str) -> str:
    if path.startswith("~/"):
        safe = path[2:].replace(chr(34), "\\\"")
        return 'cat "$HOME/' + safe + '" 2>/dev/null || true'
    return f"cat {shlex.quote(path)} 2>/dev/null || true"


def read_remote_state(host_cfg: dict[str, Any]) -> dict[str, Any]:
    host = host_cfg["ssh"]
    path = host_cfg["state_file"]
    command = state_cat_command(path)
    proc = ssh(host, command, capture=True)
    text = proc.stdout.strip()
    if not text:
        return {"version": STATE_VERSION, "components": {}}
    try:
        state = json.loads(text)
    except json.JSONDecodeError as exc:
        raise DeployError(f"{host}: invalid deployment state in {path}: {exc}") from exc
    if state.get("version") != STATE_VERSION or not isinstance(state.get("components"), dict):
        raise DeployError(f"{host}: unsupported deployment state in {path}")
    return state


def write_remote_state(host_cfg: dict[str, Any], state: dict[str, Any]) -> None:
    host = host_cfg["ssh"]
    path = host_cfg["state_file"]
    payload = (json.dumps(state, indent=2, sort_keys=True) + "\n").encode("utf-8")
    encoded = base64.b64encode(payload).decode("ascii")
    # The state is source metadata, not a secret, but keep it under ~/.ugv with
    # the rest of the rover's non-source state. Write atomically, then sync: this
    # card is mounted commit=120 and has previously lost recent writes on reset.
    py = (
        "import base64,os,pathlib,tempfile; "
        f"p=pathlib.Path({path!r}).expanduser(); p.parent.mkdir(parents=True,exist_ok=True); "
        "fd,tmp=tempfile.mkstemp(prefix=p.name+'.',dir=p.parent); "
        f"os.write(fd,base64.b64decode({encoded!r})); os.fsync(fd); os.close(fd); "
        "os.replace(tmp,p)"
    )
    ssh(host, "python3 -c " + shlex.quote(py) + " && sync")


def make_archive(component: dict[str, Any], files: list[tuple[str, str, int]]) -> Path:
    fd, name = tempfile.mkstemp(prefix=f"ugv-{component['name']}-", suffix=".tar")
    os.close(fd)
    archive = Path(name)
    with tarfile.open(archive, "w") as tf:
        for repo_path, archive_path, mode in files:
            data = (ROOT / repo_path).read_bytes()
            info = tarfile.TarInfo(archive_path)
            info.size = len(data)
            info.mode = mode
            info.mtime = 0
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            tf.addfile(info, io.BytesIO(data))
    return archive


def archive_path_for(component: dict[str, Any], repo_path: str) -> str | None:
    for source in component["sources"]:
        if not matches(repo_path, source["patterns"]):
            continue
        strip = source.get("strip", "")
        if strip:
            prefix = strip.rstrip("/") + "/"
            if not repo_path.startswith(prefix):
                return None
            return str(PurePosixPath(repo_path[len(prefix):]))
        return str(PurePosixPath(repo_path))
    return None


def stage_component(host_cfg: dict[str, Any], component: dict[str, Any],
                    files: list[tuple[str, str, int]], deleted: list[str]) -> None:
    host = host_cfg["ssh"]
    archive = make_archive(component, files)
    remote_archive = f"/tmp/ugv-deploy-{os.getpid()}-{component['name']}.tar"
    try:
        run(["scp", str(archive), f"{host}:{remote_archive}"])
        destination = component["destination"]
        preserve = component.get("preserve", [])
        prune = bool(component.get("prune", False))

        # Extract to a temporary directory on the target. rsync then gives us a
        # true mirror without requiring rsync on the Windows workstation.
        #
        # --checksum is not optional here, and it cost a deploy to learn why.
        # make_archive() normalises every mtime to the epoch so that the same
        # commit always produces the same bytes, which means every file on the
        # target is dated 1970. rsync's default quick check skips a file whose
        # size and mtime both match -- and with the mtime pinned, that reduces
        # to size alone. On 2026-08-26 a one-character fix to restart.sh
        # ($5 -> $4, same length) was therefore never copied, while the deploy
        # reported success and recorded the new commit as installed. The state
        # file then claimed bytes the rover did not have, which is the one
        # thing this program exists to prevent. Compare contents instead; these
        # trees are a few megabytes and the read costs nothing worth measuring.
        rsync_flags = ["rsync", "-a", "--checksum"]
        if prune:
            rsync_flags.append("--delete")
        for item in preserve:
            rsync_flags.extend(["--exclude", item])
        rsync_cmd = " ".join(shlex.quote(part) for part in rsync_flags)
        delete_commands: list[str] = []
        for repo_path in deleted:
            arc = archive_path_for(component, repo_path)
            if arc is None:
                continue
            delete_commands.append(f"rm -f -- {destination.rstrip('/')}/{shlex.quote(arc)}")
        delete_text = "; ".join(delete_commands)
        if delete_text:
            delete_text += "; "

        command = (
            "set -eu; "
            "stage=$(mktemp -d /tmp/ugv-deploy.XXXXXX); "
            "trap 'rm -rf \"$stage\" " + shlex.quote(remote_archive) + "' EXIT; "
            f"tar -xf {shlex.quote(remote_archive)} -C \"$stage\"; "
            f"mkdir -p {destination}; "
            f"{delete_text}"
            f"{rsync_cmd} \"$stage/\" {destination.rstrip('/')}/"
        )
        ssh(host, command)
    finally:
        archive.unlink(missing_ok=True)


def run_commands(host_cfg: dict[str, Any], component: dict[str, Any],
                 changed: list[str]) -> None:
    host = host_cfg["ssh"]
    commands = component.get("commands", [])
    special = component.get("special_commands", [])
    for rule in special:
        if matches_any(changed, rule["when"]):
            commands = [cmd for cmd in commands if cmd not in rule.get("replaces", [])]
            commands.extend(rule["commands"])
    for command in commands:
        print(f"  $ ssh {host} {command}")
        ssh(host, command)


def matches_any(paths: Iterable[str], patterns: Iterable[str]) -> bool:
    return any(matches(path, patterns) for path in paths)


def run_system_install(host_cfg: dict[str, Any], component: dict[str, Any]) -> None:
    install = component.get("system_install")
    if not install:
        return
    secret_path = ROOT / install["sudo_password_file"]
    if not secret_path.is_file():
        raise DeployError(f"system install requires {secret_path.relative_to(ROOT)}")
    password = secret_path.read_text(encoding="utf-8").strip()
    if not password:
        raise DeployError(f"{secret_path.relative_to(ROOT)} is empty")
    host = host_cfg["ssh"]
    command = "sudo -S -p '' " + install["command"]
    print(f"  $ ssh {host} sudo ... {install['command']}")
    ssh(host, command, input_text=password + "\n")
    for verify in install.get("verify", []):
        print(f"  $ ssh {host} {verify}")
        ssh(host, verify)


def selected_components(manifest: dict[str, Any], names: list[str] | None,
                        host_filter: str | None) -> list[dict[str, Any]]:
    components = []
    requested = set(names or [])
    if host_filter is not None and host_filter not in manifest["hosts"]:
        raise DeployError("unknown host: %s (manifest knows %s)"
                          % (host_filter, ", ".join(sorted(manifest["hosts"]))))
    known = {item["name"] for item in manifest["components"]}
    unknown = requested - known
    if unknown:
        raise DeployError("unknown component(s): " + ", ".join(sorted(unknown)))
    for component in manifest["components"]:
        if requested and component["name"] not in requested:
            continue
        if not requested and not host_filter and component.get("default", True) is False:
            continue
        if host_filter and component["host"] != host_filter:
            continue
        components.append(component)
    return components


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", action="store_true", help="show what would deploy; do not copy/restart/write state")
    parser.add_argument("--full", action="store_true", help="deploy selected components even if no prior state exists")
    parser.add_argument("--adopt", action="store_true", help="record HEAD as deployed for selected components without copying anything")
    parser.add_argument("--system", action="store_true", help="allow privileged system installers (wifi_roam/netwatch)")
    parser.add_argument("--only", action="append", metavar="COMPONENT", help="limit to one component; repeatable")
    # Deliberately not `choices=`: those are fixed when the parser is built and
    # the manifest is not read until after that, so the list went stale -- it was
    # still offering `media`, a GPU host this repository stopped deploying to
    # some time ago. The manifest is the source of truth for what hosts exist,
    # so the name is checked against it in selected_components() instead.
    parser.add_argument("--host", metavar="HOST",
                        help="limit to one target host, as named in manifest.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        require_tools()
        assert_repo()
        manifest = load_manifest()
        if args.plan and args.adopt:
            raise DeployError("--plan and --adopt are mutually exclusive")
        if args.adopt and args.full:
            raise DeployError("--adopt and --full are mutually exclusive")
        assert_clean()

        head = git("rev-parse", "HEAD")
        tracked = tracked_files_with_modes()
        components = selected_components(manifest, args.only, args.host)
        if not components:
            raise DeployError("no components selected")

        states: dict[str, dict[str, Any]] = {}
        for host_name in sorted({c["host"] for c in components}):
            states[host_name] = read_remote_state(manifest["hosts"][host_name])

        if args.adopt:
            for component in components:
                states[component["host"]]["components"][component["name"]] = head
            for host_name in sorted(states):
                write_remote_state(manifest["hosts"][host_name], states[host_name])
            print(f"Adopted {head[:12]} for {len(components)} component(s); no files were copied.")
            return 0

        work: list[tuple[dict[str, Any], list[str], str | None]] = []
        unknown: list[str] = []
        for component in components:
            base = states[component["host"]]["components"].get(component["name"])
            if base is None:
                if args.full:
                    changes = [repo_path for repo_path, _, _ in component_files(component, tracked)]
                else:
                    changes = []
                    unknown.append(component["name"])
            else:
                try:
                    changes = changed_paths(base, head, all_patterns(component))
                except subprocess.CalledProcessError as exc:
                    raise DeployError(
                        f"cannot diff {component['name']} from remote state {base[:12]}; "
                        "the commit may no longer exist locally. Fetch history or use --full."
                    ) from exc
            if args.full or changes:
                work.append((component, changes, base))

        print(f"HEAD {head}")
        for component, changes, base in work:
            marker = "full" if base is None or args.full else f"{base[:12]}..HEAD"
            print(f"  {component['name']:<14} {component['host']:<5} {marker:<22} {len(changes)} changed path(s)")
        if not work:
            print("Nothing needs deployment.")

        if unknown and not args.full:
            print("\nNo recorded deployment state for: " + ", ".join(unknown), file=sys.stderr)
            print("Use --full to reconcile those components, or --adopt if the hosts are already known to match HEAD.", file=sys.stderr)
            if not work:
                return 2

        if args.plan:
            if unknown and not args.full:
                print("\n(plan is incomplete until unknown components are --full or --adopt)")
            return 0

        failures = 0
        pending_system = 0
        for component, changes, base in work:
            host_name = component["host"]
            host_cfg = manifest["hosts"][host_name]
            print(f"\n== {component['name']} -> {host_cfg['ssh']} ==")
            try:
                files = component_files(component, tracked)
                if not files:
                    raise DeployError(f"{component['name']}: source selection is empty")
                deleted = [] if base is None or args.full else deleted_paths(base, head, all_patterns(component))
                stage_component(host_cfg, component, files, deleted)
                run_commands(host_cfg, component, changes)
                if component.get("system_install"):
                    if not args.system:
                        pending_system += 1
                        print("  staged and verified, but the running system copy was not replaced.")
                        print("  rerun with --system after reviewing this network/system change.")
                        continue
                    run_system_install(host_cfg, component)
                states[host_name]["components"][component["name"]] = head
                write_remote_state(host_cfg, states[host_name])
                print(f"  OK {component['name']} @ {head[:12]}")
            except (DeployError, subprocess.CalledProcessError) as exc:
                failures += 1
                print(f"  FAILED {component['name']}: {exc}", file=sys.stderr)
                # Do not advance state. Later components can still be independent,
                # but a failed component will remain changed on the next run.

        if failures:
            print(f"\nDEPLOY FAILED: {failures} component(s) failed", file=sys.stderr)
            return 1
        if pending_system:
            print(f"\nDEPLOY INCOMPLETE: {pending_system} system component(s) need --system", file=sys.stderr)
            return 3
        print("\nDEPLOY SUCCESSFUL")
        return 0
    except (DeployError, subprocess.CalledProcessError) as exc:
        print(f"deploy: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
