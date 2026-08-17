"""The rover's ten tools and the prompt that asks for them, read from the source.

There is no second copy of a schema in this directory, and that is deliberate.
The daemon is already the one place a tool is described --
[rover_daemon.py](../rover_daemon/rover_daemon.py) says so in its own docstring,
and every word of those descriptions was arrived at through six-sample runs. A
benchmark that pasted them here would be measuring a fossil the first time
somebody improved one.

So this reads them out of the source with `ast` rather than importing it. The
daemon imports `serial` and the voice service imports `torch`, neither of which
belongs in a harness that has to run on a rented card, and both files hold their
schemas and prompts as plain literals, which parse perfectly well without being
run.

The prompt matters as much as the schemas. The 66/90 and 75/90 numbers this whole
exercise compares against were taken with a specific system prompt, and the
tool-prompt wording is worth nine points of it -- so the spoken run has to use the
same text, right down to the sentence about not saying "I will", which is only
worth anything where it currently sits.
"""

from __future__ import annotations

import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DAEMON = ROOT / "rover_daemon" / "rover_daemon.py"
VOICE = ROOT / "voice_chat" / "server.py"


def _assignments(tree: ast.Module) -> dict[str, ast.expr]:
    """Every module-level `NAME = ...`, including the annotated form.

    `TOOLS: list[dict[str, Any]] = [...]` is an `AnnAssign` and `LIGHT_MAX = 255`
    is an `Assign`; the schemas need both, since one of them refers to the other.
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
    """`ast.literal_eval` with one extension: a name that is itself a literal.

    The daemon writes `"maximum": LIGHT_MAX` rather than 255, which is the right
    way round for the daemon and fatal to a plain `literal_eval`. Resolving the
    name keeps the harness honest -- if somebody changes the headlight ceiling,
    the schema this measures changes with it.
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
            _evaluate(k, names): _evaluate(v, names)
            for k, v in zip(node.keys, node.values)
            if k is not None
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


def tools() -> list[dict]:
    """All ten, in the order the daemon offers them with `--vision`.

    `look` is last because that is where the daemon appends it, and order is not
    cosmetic here: the README's finding is that a tool is read against its
    neighbours, so reordering the list is a change to the experiment.
    """
    return list(_literal(DAEMON, "TOOLS")) + [_literal(DAEMON, "LOOK_TOOL")]


def system_prompt(*, vision: bool = True) -> str:
    """Exactly what the deployed service sends: spoken-English, tools, vision.

    Assembled the way `_generate` assembles it -- base, then the tool prompt,
    then the vision line -- because the position of the tool prompt's last
    sentence is worth nine points out of ninety and moving it to the front costs
    twenty-four.
    """
    base = _literal(VOICE, "SYSTEM_PROMPT")
    tool = _literal(VOICE, "TOOL_PROMPT")
    look = _literal(VOICE, "VISION_PROMPT") if vision else ""
    return f"{base}{tool}{look}"


def tools_block(tool_list: list[dict] | None = None) -> str:
    """The `# Tools` system section, rendered exactly as the chat template does.

    Both candidates carry the same Qwen-derived template -- MiniCPM-o 4.5 inherits
    it wholesale, tool tokens and all -- so this is not an approximation of what
    the model would see, it is the same string. Rendering it here rather than
    passing `tools=` to a template means the harness can put tools in front of a
    model whose own `chat()` entry point never accepted them, which is the case
    for MiniCPM-o's multimodal path.
    """
    tool_list = tool_list if tool_list is not None else tools()
    lines = [
        "# Tools",
        "",
        "You may call one or more functions to assist with the user query.",
        "",
        "You are provided with function signatures within <tools></tools> XML tags:",
        "<tools>",
    ]
    lines += [json.dumps(t) for t in tool_list]
    lines += [
        "</tools>",
        "",
        "For each function call, return a json object with function name and "
        "arguments within <tool_call></tool_call> XML tags:",
        "<tool_call>",
        '{"name": <function-name>, "arguments": <args-json-object>}',
        "</tool_call>",
    ]
    return "\n".join(lines)


def full_system(*, vision: bool = True) -> str:
    """What actually goes in the system turn: the rover's prompt, then the tools."""
    return f"{system_prompt(vision=vision)}\n\n{tools_block()}"


if __name__ == "__main__":
    schemas = tools()
    print(f"{len(schemas)} tools: {', '.join(t['function']['name'] for t in schemas)}")
    prompt = full_system()
    print(f"\nsystem prompt: {len(prompt)} chars, ~{len(prompt) // 4} tokens\n")
    print(prompt[:600] + "\n...")
