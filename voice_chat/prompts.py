"""The current spoken prompt and the rover tool schemas.

The Alibaba realtime session imports this module directly. Prompt text lives here
because this is the current code that sends it. Tool schemas do not: the daemon's
`tool_schemas.py` remains their source of truth and is parsed with `ast` so this
helper does not have to import the daemon (and therefore `serial`).

A live daemon is still preferred over these parsed fallback schemas; see
`rover_tools.py`. The parsed set is used by the mock and by offline checks where
there is no daemon to ask.
"""

from __future__ import annotations

import ast
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

SYSTEM_PROMPT = os.environ.get(
    "VOICE_SYSTEM_PROMPT",
    "You are the voice of a small tracked rover. You are speaking out loud, so "
    "reply in one to three short sentences of plain spoken English. Never use "
    "markdown, bullet points, headings, emoji or code. Write numbers and units "
    "as you would say them aloud. If you do not know something, say so briefly.",
)

TOOL_PROMPT = os.environ.get(
    "VOICE_TOOL_PROMPT",
    " You control this rover through the tools you have been given. Call a tool "
    "whenever you are asked to do something one of them covers, including when "
    "the request is phrased as a question such as 'can you turn the lights off'. "
    "You have done something only if you have called a tool for it: never say you "
    "have switched, moved, started or stopped anything unless the call was made "
    "and answered. Then say what you did in one short sentence, without reading "
    "the tool call or its result out loud. Do not say 'I will', 'I'll' or 'I am "
    "going to' about anything a tool does. Call the tool instead, and say what "
    "you did afterwards.",
)

VISION_PROMPT = os.environ.get(
    "VOICE_VISION_PROMPT",
    " You see by taking a picture with the tool that takes one, so if you are "
    "asked what you can see, or what something looks like, or to read or describe "
    "anything in front of you, take a picture first and answer from it. Describe "
    "only what is actually in the picture.",
)


def _source(name: str, *repository: str) -> Path:
    """Where a source file lives in the checkout or flattened rover deploy."""
    for candidate in (
        ROOT.joinpath(*repository) if repository else HERE / name,
        HERE / name,
        ROOT / name,
    ):
        if candidate.exists():
            return candidate
    return HERE / name


DAEMON = _source("tool_schemas.py", "rover_daemon", "tool_schemas.py")
#: The navigation half of the daemon, read for the map limits a mock has to
#: enforce if it is to stand in for the rover. `mock_rover` asks for these by
#: name, and without this line every console test that needs a mock fails on
#: the import rather than on anything it was testing.
ROVER_NAV = _source("rover_nav.py", "rover_daemon", "rover_nav.py")


def _assignments(tree: ast.Module) -> dict[str, ast.expr]:
    """Every module-level ``NAME = ...`` or annotated assignment."""
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
    """``literal_eval`` plus named constants used by the daemon schemas."""
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
    if isinstance(node, ast.Call) and len(node.args) >= 2:
        # Kept for schema constants that may themselves be written as
        # os.environ.get(KEY, default); only the deterministic default is useful
        # to an offline fallback.
        return _evaluate(node.args[1], names)
    return ast.literal_eval(node)


def _literal(source: Path, name: str) -> object:
    """Value assigned to ``name`` at module level without importing the file."""
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = _assignments(tree)
    if name not in names:
        raise ValueError(f"{name} not found in {source}")
    return _evaluate(names[name], names)


def system_prompt(*, vision: bool = True) -> str:
    """Prompt sent to the current realtime session.

    Order is deliberate: spoken-system rules, tool rules, then the vision rule.
    The realtime API receives tool schemas separately, so they are not rendered a
    second time into this string.
    """
    return f"{SYSTEM_PROMPT}{TOOL_PROMPT}{VISION_PROMPT if vision else ''}"


def tools(*, vision: bool = True, nav: bool = False) -> list[dict]:
    """Fallback copy of the daemon schemas, read from the daemon's source."""
    found = list(_literal(DAEMON, "TOOLS"))
    if vision:
        found.append(_literal(DAEMON, "LOOK_TOOL"))
    if nav:
        found += list(_literal(DAEMON, "NAV_TOOLS"))
        if vision:
            found.append(_literal(DAEMON, "MAP_TOOL"))
            found.append(_literal(DAEMON, "MAP_POINT_TOOL"))
    return found


def names(tool_list: list[dict] | None = None) -> list[str]:
    """Tool names for display/checking."""
    selected = tool_list if tool_list is not None else tools()
    return [tool["function"]["name"] for tool in selected]


if __name__ == "__main__":
    schemas = tools()
    prompt = system_prompt()
    print(f"{len(schemas)} tools: {', '.join(names(schemas))}")
    print(f"\nsystem prompt: {len(prompt)} chars, ~{len(prompt) // 4} tokens\n")
    print(prompt)
