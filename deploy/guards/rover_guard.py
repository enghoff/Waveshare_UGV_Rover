#!/usr/bin/env python3
"""Refuse two shell commands AGENTS.md forbids: editing the rover's deploy tree
in place, and putting a credential on a command line.

Both rules are ones a person can forget between one ssh and the next, so they
are checked here rather than only written down. Reads the PreToolUse payload on
stdin and answers with a permission decision; anything it cannot parse is
allowed, because a broken guard must not stop the work.
"""
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# The rover's deployed copy. ~/.ugv is deliberately outside it -- secrets, TLS
# material, deploy state and the chassis calibration live there and are written
# on the rover by design -- so the pattern must not match that.
TREE = r"(?:~|\$HOME|/home/jetson)/ugv(?![\w.])"

WRITES = [
    (r">>?\s*['\"]?" + TREE, "a redirect into the deploy tree"),
    (r"\btee\s+(?:-\S+\s+)*['\"]?" + TREE, "tee into the deploy tree"),
    (r"\bsed\b[^|;&]*\s-i\b[^|;&]*" + TREE, "sed -i on a deployed file"),
    (r"\b(?:cp|mv|install)\b[^|;&]*" + TREE, "cp/mv into the deploy tree"),
    (r"\brm\b[^|;&]*" + TREE, "rm inside the deploy tree"),
    (r"\b(?:vi|vim|nano|ed)\s+['\"]?" + TREE, "an editor on a deployed file"),
    (r"\btruncate\b[^|;&]*" + TREE, "truncate on a deployed file"),
    (r"\bpatch\b[^|;&]*" + TREE, "patch on a deployed file"),
]

ESCAPE = "deploy-guard: allow"


def targets_rover(cmd):
    return re.search(r"\bssh\s+(?:-\S+\s+)*(?:orin\b|jetson@)", cmd) is not None


def copy_to_rover(cmd):
    """scp/rsync whose *destination* is the rover -- pulling a log back is fine."""
    if not re.search(r"\b(?:scp|rsync)\b", cmd):
        return False
    tokens = cmd.split()
    return bool(tokens) and re.match(r"^(?:orin|jetson@[\w.-]+):", tokens[-1]) is not None


def in_place_edit(cmd):
    for pattern, what in WRITES:
        if re.search(pattern, cmd):
            return what
    return None


def leaked_secret(cmd):
    """Name the secrets/ file whose contents appear in the command, never the value."""
    directory = os.path.join(ROOT, "secrets")
    if not os.path.isdir(directory):
        return None
    for name in sorted(os.listdir(directory)):
        path = os.path.join(directory, name)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8", errors="ignore") as handle:
                value = handle.read().strip()
        except OSError:
            continue
        if len(value) >= 6 and value in cmd:
            return name
    return None


def objection(cmd):
    """The reason this command is refused, or None."""
    secret = leaked_secret(cmd)
    if secret:
        return ("This contains the contents of secrets/%s. A credential must not "
                "appear in a command, a transcript or a commit -- pass it on stdin "
                "instead (docs/deploy.md has the sudo -S mechanics)." % secret)

    if ESCAPE in cmd:
        return None

    if copy_to_rover(cmd):
        return ("This copies files onto the rover directly. The repository is the "
                "source of truth: commit here and run deploy/deploy.py, which "
                "copies the registered components and runs their own restart and "
                "verification checks.")

    if targets_rover(cmd):
        what = in_place_edit(cmd)
        if what:
            return ("This is %s (~/ugv on the rover). Tracked files are edited here "
                    "and deployed, never changed in place -- the recorded commit "
                    "must describe the bytes that were sent. Edit the repository "
                    "copy and run deploy/deploy.py. If this really is manual "
                    "recovery (docs/rover-unresponsive.md), add the comment "
                    "'# %s' to the command." % (what, ESCAPE))
    return None


def staged_secret():
    """The secrets/ file whose contents appear in what is staged for commit."""
    import subprocess
    try:
        diff = subprocess.run(["git", "diff", "--cached", "-U0"], cwd=ROOT,
                              capture_output=True, text=True, errors="ignore")
    except OSError:
        return None
    added = "\n".join(line for line in diff.stdout.splitlines()
                      if line.startswith("+") and not line.startswith("+++"))
    return leaked_secret(added)


def main():
    # Called by hand, by another agent's tooling, or by .githooks/pre-commit.
    if "--command" in sys.argv:
        cmd = sys.argv[sys.argv.index("--command") + 1]
        reason = objection(cmd)
        if reason:
            print(reason, file=sys.stderr)
            sys.exit(1)
        return
    if "--staged" in sys.argv:
        name = staged_secret()
        if name:
            print("Refusing the commit: it stages the contents of secrets/%s. "
                  "Credentials never go in a commit." % name, file=sys.stderr)
            sys.exit(1)
        return

    # Called as a Claude Code PreToolUse hook.
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return
    cmd = (payload.get("tool_input") or {}).get("command") or ""
    reason = objection(cmd) if cmd else None
    if reason:
        json.dump({"hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "permissionDecision": "deny",
            "permissionDecisionReason": reason,
        }}, sys.stdout)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        pass
