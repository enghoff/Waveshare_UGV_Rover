"""The system prompt and the tool schemas, read out of the source that owns them.

[server.py](server.py) is the one place the rover's prompt is written and
[rover_daemon.py](../rover_daemon/rover_daemon.py) is the one place its tools are
described. Every sentence in both was arrived at through six-sample runs, and the
position of one of them is worth nine points out of ninety, so a second copy here
would be a fossil the first time somebody improved the original.

Importing them is not an option: the service pulls in `torch` and the daemon
pulls in `serial`, and this runs on a desk that has neither. So the values are
parsed out with `ast`, which works because both files hold them as plain
literals. [omni_bench/schemas.py](../omni_bench/schemas.py) does the same thing
for the same reason and deliberately stays standalone, because it is uploaded to
a rented card where this directory does not exist.

The tools here are a fallback, not the usual path. A live daemon is asked what it
can do over the wire (see [rover_tools.py](rover_tools.py)), because it is the
authority and it may have been restarted with more tools since anyone last
looked. These are for the two cases where there is no daemon to ask: the mock in
[mock_rover.py](mock_rover.py), and printing what would be sent.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DAEMON = ROOT / "rover_daemon" / "rover_daemon.py"
VOICE = Path(__file__).resolve().parent / "server.py"


def _assignments(tree: ast.Module) -> dict[str, ast.expr]:
    """Every module-level `NAME = ...`, both the plain and the annotated form.

    `TOOLS: list[dict[str, Any]] = [...]` is an `AnnAssign` while
    `LIGHT_MAX = 255` is an `Assign`, and the schemas need both because one of
    them refers to the other.
    """
    found: dict[str, ast.expr] = {}
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.value is not None:
                found[node.target.id] = node.value
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    found[target.id] = node.value
    return found


def _evaluate(node: ast.expr, names: dict[str, ast.expr]) -> object:
    """`literal_eval`, plus the two shapes these particular files are written in.

    The daemon writes `"maximum": LIGHT_MAX` rather than 255, and the service
    wraps every prompt in `os.environ.get(NAME, "the actual text")`. Resolving
    the first keeps this honest -- change the headlight ceiling and the schema
    read here changes with it -- and unwrapping the second is what makes the
    prompts readable at all.
    """
    if isinstance(node, ast.Name):
        if node.id not in names:
            raise ValueError(f"cannot resolve {node.id}")
        return _evaluate(names[node.id], names)
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_evaluate(item, names) for item in node.elts]
    if isinstance(node, ast.Dict):
        return {
            _evaluate(key, names): _evaluate(value, names)
            for key, value in zip(node.keys, node.values)
            if key is not None
        }
    if isinstance(node, ast.Call):  # os.environ.get(KEY, default)
        if len(node.args) < 2:
            raise ValueError("a call with no default to read")
        return _evaluate(node.args[1], names)
    return ast.literal_eval(node)


def _literal(source: Path, name: str) -> object:
    """The value assigned to `name` at module level, without importing anything."""
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = _assignments(tree)
    if name not in names:
        raise ValueError(f"{name} not found in {source}")
    return _evaluate(names[name], names)


def system_prompt(*, vision: bool = True) -> str:
    """What the deployed service sends: spoken English, then tools, then vision.

    Assembled in the order `_generate` assembles it. That order is not cosmetic:
    the tool prompt's closing sentence about not saying "I will" is worth nine
    points out of ninety where it sits and costs twenty-four at the front.

    There is no `# Tools` block on the end, unlike the benchmark's version of
    this. The realtime API takes schemas as a field of its own and renders that
    block itself, so adding one here would put the ten tools in front of the
    model twice.
    """
    base = _literal(VOICE, "SYSTEM_PROMPT")
    tool = _literal(VOICE, "TOOL_PROMPT")
    look = _literal(VOICE, "VISION_PROMPT") if vision else ""
    return f"{base}{tool}{look}"


def tools(*, vision: bool = True, nav: bool = False) -> list[dict]:
    """The daemon's schemas, in the order it offers them.

    `look` comes after the fixed set and the driving tools after that, because that
    is the order the daemon appends them in, and order matters here -- the finding
    in [README.md](README.md) is that a tool is read against its neighbours, so a
    reordered list is a different experiment.

    `nav` is off by default even though the rover now has a lidar on it. Every
    measurement in that README was taken against ten tools, and quietly making the
    default fifteen would change what those numbers mean without changing the page
    they are written on. Ask for them.
    """
    found = list(_literal(DAEMON, "TOOLS"))
    if vision:
        found.append(_literal(DAEMON, "LOOK_TOOL"))
    if nav:
        found += list(_literal(DAEMON, "NAV_TOOLS"))
        if vision:
            found.append(_literal(DAEMON, "MAP_TOOL"))
    return found


def names(tool_list: list[dict] | None = None) -> list[str]:
    """Just the names, for printing and for checking what a service accepted."""
    return [t["function"]["name"] for t in (tool_list if tool_list is not None else tools())]


if __name__ == "__main__":
    schemas = tools()
    prompt = system_prompt()
    print(f"{len(schemas)} tools: {', '.join(names(schemas))}")
    print(f"\nsystem prompt: {len(prompt)} chars, ~{len(prompt) // 4} tokens\n")
    print(prompt)
