#!/usr/bin/env python3
"""Notice when a session edits a deployed component and then tries to finish
without deploying it.

AGENTS.md: a change is not done until it runs on the host that uses it. Which
files that covers is decided by deploy/manifest.json, so this reads the manifest
rather than keeping a list of its own. It runs twice: after a tool call it
records edits and clears them when deploy.py succeeds, and at the end of a turn
it reports what is still only in the repository.

Pending edits live in the system temp directory, keyed by session, so nothing
lands in the working tree.
"""
import json
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MANIFEST = os.path.join(ROOT, "deploy", "manifest.json")


def state_path(session):
    safe = re.sub(r"[^\w.-]", "_", session or "nosession")
    return os.path.join(tempfile.gettempdir(), "ugv-deploy-pending-%s.json" % safe)


def load(session):
    try:
        with open(state_path(session), encoding="utf-8") as handle:
            return json.load(handle)
    except Exception:
        return {"components": {}, "warned": False}


def save(session, state):
    try:
        with open(state_path(session), "w", encoding="utf-8") as handle:
            json.dump(state, handle)
    except OSError:
        pass


def to_regex(pattern):
    """deploy/manifest.json globs: ** spans directories, * stays within one."""
    out, i = [], 0
    while i < len(pattern):
        char = pattern[i]
        if char == "*":
            if pattern[i:i + 2] == "**":
                out.append(".*")
                i += 2
                continue
            out.append("[^/]*")
        elif char == "?":
            out.append("[^/]")
        else:
            out.append(re.escape(char))
        i += 1
    return re.compile("^" + "".join(out) + "$")


def components_for(relpath):
    """Every component whose sources or triggers cover this file."""
    try:
        with open(MANIFEST, encoding="utf-8") as handle:
            manifest = json.load(handle)
    except Exception:
        return []
    hits = []
    for component in manifest.get("components", []):
        patterns = list(component.get("triggers", []))
        for source in component.get("sources", []):
            patterns.extend(source.get("patterns", []))
        if any(to_regex(p).match(relpath) for p in patterns):
            hits.append(component["name"])
    return hits


def relative(path):
    if not path:
        return None
    try:
        rel = os.path.relpath(os.path.abspath(path), ROOT)
    except ValueError:
        return None
    if rel.startswith(".."):
        return None
    return rel.replace(os.sep, "/")


def post_tool_use(payload, session):
    name = payload.get("tool_name")
    tool_input = payload.get("tool_input") or {}
    state = load(session)

    if name == "Bash":
        cmd = tool_input.get("command") or ""
        # This event only fires on success, so a deploy.py that reaches it worked.
        if re.search(r"deploy[/\\]deploy\.py", cmd) and "--plan" not in cmd:
            save(session, {"components": {}, "warned": False})
        return

    if name not in ("Write", "Edit", "NotebookEdit"):
        return
    rel = relative(tool_input.get("file_path"))
    if not rel:
        return
    for component in components_for(rel):
        files = state["components"].setdefault(component, [])
        if rel not in files:
            files.append(rel)
    if state["components"]:
        save(session, state)


def stop(session):
    state = load(session)
    pending = state.get("components") or {}
    if not pending:
        return
    summary = "; ".join("%s (%s)" % (name, ", ".join(sorted(files)))
                        for name, files in sorted(pending.items()))

    if not state.get("warned"):
        state["warned"] = True
        save(session, state)
        json.dump({"decision": "block", "reason": (
            "Deployed components changed but nothing was deployed this session: "
            "%s. A change is not done until it runs on the host that uses it -- "
            "commit, run deploy/deploy.py, and verify the running service on the "
            "rover. If deploying is genuinely not the next step, say why and "
            "finish." % summary)}, sys.stdout)
        return

    json.dump({"systemMessage": "Still undeployed: %s" % summary}, sys.stdout)


def main():
    if "--pending" in sys.argv:
        # For any agent or script that wants the same answer without the hook.
        index = sys.argv.index("--pending")
        session = sys.argv[index + 1] if len(sys.argv) > index + 1 else None
        pending = load(session).get("components") or {}
        print(json.dumps(pending, indent=1) if pending else "nothing pending")
        return
    if "--components" in sys.argv:
        index = sys.argv.index("--components")
        for path in sys.argv[index + 1:]:
            rel = relative(path) or path
            print("%s -> %s" % (rel, ", ".join(components_for(rel)) or "not deployed"))
        return
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return
    session = payload.get("session_id")
    event = payload.get("hook_event_name") or ("PostToolUse" if payload.get("tool_name") else "Stop")
    if event == "Stop":
        stop(session)
    else:
        post_tool_use(payload, session)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        pass
