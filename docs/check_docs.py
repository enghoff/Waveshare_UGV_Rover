#!/usr/bin/env python3
"""Check the documentation framework described in docs/README.md.

Four things are verified:

  * every requirement record is well formed -- an identifier, a state, and the
    supporting field that state owes;
  * requirement identifiers are unique, sequential and in the right file;
  * every ``R-AREA-n`` mentioned anywhere in the repository resolves to a real
    requirement, so a plan cannot claim one that does not exist;
  * every relative link in the documentation and the component READMEs points at
    something on disk, including the heading anchors between requirements.

``--write`` regenerates the summary table in docs/README.md between its markers.

Exits non-zero if anything failed. Runs from the repository root or from docs/.
"""

from __future__ import annotations

import argparse
import re
import sys
import unicodedata
from pathlib import Path

# The area prefix each requirement file owns.
AREAS = {
    "SAFE": "safety.md",
    "NAV": "navigation.md",
    "WS": "world-state.md",
    "CTL": "control.md",
    "PLAT": "platform.md",
}

# Each state and the field a record in that state must carry.
STATES = {
    "settled": "Evidence",
    "open": "Blocked by",
    "proposed": "Proposed in",
    "failing": "Broken by",
    "retired": "Superseded by",
}

# Order the summary table reports states in.
STATE_ORDER = ["settled", "failing", "open", "proposed", "retired"]

REQ_HEADING = re.compile(r"^###\s+(R-([A-Z]+)-(\d+))\s+[-—]\s+(.+?)\s*$")
AREA_MARKER = re.compile(r"^<!--\s*requirement-area:\s*([A-Z]+)\s*-->\s*$")
FIELD = re.compile(r"^-\s+\*\*([^:*]+):\*\*\s*(.*)$")
REQ_MENTION = re.compile(r"\bR-([A-Z]+)-(\d+)\b")

# [text](target) but not ![image](target); target must not be a bare URL.
MD_LINK = re.compile(r"(?<!!)\[(?:[^\]]*)\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")

SUMMARY_BEGIN = "<!-- begin: requirement summary (python docs/check_docs.py --write) -->"
SUMMARY_END = "<!-- end: requirement summary -->"

# Directories that are not ours to validate.
SKIP_DIRS = {".git", ".venv", ".cache", "__pycache__", "node_modules"}

# Extensions searched for requirement mentions and stale document paths. A
# document path in a C header or a capture manifest rots just as quietly as one
# in a markdown link, which is how three of them survived here for weeks.
TEXT_SUFFIXES = {".md", ".py", ".sh", ".js", ".mjs", ".html", ".css", ".json",
                 ".txt", ".c", ".h", ".yaml", ".yml", ".cfg", ".toml"}

# Extensionless text files worth scanning.
TEXT_NAMES = {".gitattributes", ".gitignore", "requirements.txt", "Makefile"}


class Requirement:
    def __init__(self, ident: str, area: str, number: int, title: str,
                 path: Path, line: int) -> None:
        self.ident = ident
        self.area = area
        self.number = number
        self.title = title
        self.path = path
        self.line = line
        self.state: str | None = None
        self.fields: dict[str, str] = {}


def repo_root() -> Path:
    here = Path(__file__).resolve().parent
    return here.parent


def anchor(text: str) -> str:
    """GitHub's heading anchor: lowercase, strip punctuation, spaces to dashes."""
    text = unicodedata.normalize("NFKD", text)
    text = "".join(c for c in text if not unicodedata.combining(c))
    text = text.lower()
    text = re.sub(r"[^\w\s-]", "", text)
    return re.sub(r"\s+", "-", text.strip())


def walk(root: Path):
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        if path.suffix.lower() in TEXT_SUFFIXES or path.name in TEXT_NAMES:
            yield path


def read(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8", errors="replace").splitlines()


def parse_requirements(docs: Path, fail) -> list[Requirement]:
    found: list[Requirement] = []
    seen: dict[str, Requirement] = {}

    for area, name in sorted(AREAS.items()):
        path = docs / "requirements" / name
        if not path.exists():
            fail(path, 0, f"requirement file for area {area} is missing")
            continue

        lines = read(path)
        declared = None
        for line in lines[:10]:
            m = AREA_MARKER.match(line)
            if m:
                declared = m.group(1)
                break
        if declared is None:
            fail(path, 1, f"no '<!-- requirement-area: {area} -->' marker in the first 10 lines")
        elif declared != area:
            fail(path, 1, f"declares area {declared} but is the file for {area}")

        current: Requirement | None = None
        numbers: list[int] = []

        for n, line in enumerate(lines, 1):
            m = REQ_HEADING.match(line)
            if m:
                ident, ident_area, number, title = m.group(1), m.group(2), int(m.group(3)), m.group(4)
                current = Requirement(ident, ident_area, number, title, path, n)
                found.append(current)

                if ident_area != area:
                    fail(path, n, f"{ident} belongs in the {ident_area} area file, not this one")
                if ident in seen:
                    other = seen[ident]
                    fail(path, n, f"{ident} is already defined at {other.path.name}:{other.line}")
                seen[ident] = current
                numbers.append(number)
                continue

            if current is None:
                continue

            m = FIELD.match(line)
            if m:
                key, value = m.group(1).strip(), m.group(2).strip()
                current.fields[key] = value
                if key == "State":
                    if value not in STATES:
                        fail(path, n, f"{current.ident} has unknown state {value!r}; "
                                      f"expected one of {', '.join(sorted(STATES))}")
                    else:
                        current.state = value

        expected = list(range(1, len(numbers) + 1))
        if numbers != expected:
            fail(path, 1, f"identifiers are {numbers}, expected {expected} -- "
                          f"numbers are assigned once, in order, and never reused")

    for req in found:
        if req.state is None:
            if "State" not in req.fields:
                fail(req.path, req.line, f"{req.ident} has no '- **State:**' field")
            continue
        needed = STATES[req.state]
        value = req.fields.get(needed, "")
        if not value:
            fail(req.path, req.line,
                 f"{req.ident} is {req.state} and must carry '- **{needed}:**' with a value")

    return found


# A docs/ path named in a comment or a string, e.g. "See docs/reference/i2c.md".
DOC_PATH = re.compile(r"(?<![\w./-])(docs/[\w./-]+\.md)\b")


def check_doc_paths(root: Path, fail) -> None:
    """Every docs/... path named anywhere in the repository must exist.

    Markdown links are checked separately with proper relative resolution; this
    catches the bare paths that live in code comments, which is how three
    references to deleted documents survived for weeks.
    """
    for path in walk(root):
        for n, line in enumerate(read(path), 1):
            for m in DOC_PATH.finditer(line):
                target = m.group(1)
                if not (root / target).exists():
                    fail(path, n, f"names a document that does not exist: {target}")


def check_mentions(root: Path, known: set[str], fail) -> None:
    for path in walk(root):
        for n, line in enumerate(read(path), 1):
            # The heading is the definition of the identifier, not a mention of
            # it. Everything else -- including cross-references between the
            # requirement files, where a stale identifier is most likely -- is
            # checked.
            if REQ_HEADING.match(line) or EXPLICIT_ANCHOR.search(line):
                continue
            for m in REQ_MENTION.finditer(line):
                ident = f"R-{m.group(1)}-{m.group(2)}"
                if m.group(1) not in AREAS:
                    fail(path, n, f"{ident} names area {m.group(1)}, which is not a requirement area")
                elif ident not in known:
                    fail(path, n, f"{ident} does not exist")


EXPLICIT_ANCHOR = re.compile(r"""<a\s+(?:id|name)\s*=\s*["']([^"']+)["']""", re.I)


def collect_anchors(path: Path) -> set[str]:
    """Heading anchors plus the explicit <a id="..."> anchors requirements carry."""
    out = set()
    for line in read(path):
        if line.startswith("#"):
            out.add(anchor(line.lstrip("#").strip()))
        for m in EXPLICIT_ANCHOR.finditer(line):
            out.add(m.group(1))
    return out


def check_links(root: Path, fail) -> None:
    targets = [root / "docs", root / "AGENTS.md", root / "README.md"]
    targets += sorted(root.glob("*/README.md")) + sorted(root.glob("*/AGENTS.md"))

    files: list[Path] = []
    for t in targets:
        if t.is_dir():
            files += [p for p in walk(t) if p.suffix == ".md"]
        elif t.is_file() and t.suffix == ".md":
            files.append(t)

    anchors: dict[Path, set[str]] = {}

    for path in sorted(set(files)):
        for n, line in enumerate(read(path), 1):
            for m in MD_LINK.finditer(line):
                target = m.group(1)
                if target.startswith(("http://", "https://", "mailto:", "#")):
                    if target.startswith("#"):
                        frag = target[1:]
                        anchors.setdefault(path, collect_anchors(path))
                        if frag not in anchors[path]:
                            fail(path, n, f"link to '#{frag}' has no matching heading in this file")
                    continue

                file_part, _, frag = target.partition("#")
                if not file_part:
                    continue
                dest = (path.parent / file_part).resolve()
                if not dest.exists():
                    fail(path, n, f"link target does not exist: {target}")
                    continue
                if frag and dest.suffix == ".md":
                    anchors.setdefault(dest, collect_anchors(dest))
                    if frag not in anchors[dest]:
                        fail(path, n, f"link to {file_part} has no heading anchor '#{frag}'")


def summary_table(reqs: list[Requirement]) -> str:
    counts: dict[str, dict[str, int]] = {a: {} for a in AREAS}
    for r in reqs:
        if r.area in counts and r.state:
            counts[r.area][r.state] = counts[r.area].get(r.state, 0) + 1

    used = [s for s in STATE_ORDER if any(c.get(s) for c in counts.values())]
    titles = {
        "SAFE": "[Safety and authority](requirements/safety.md)",
        "NAV": "[Mapping and movement](requirements/navigation.md)",
        "WS": "[Visual memory](requirements/world-state.md)",
        "CTL": "[Control surface](requirements/control.md)",
        "PLAT": "[Host and deployment](requirements/platform.md)",
    }

    head = "| Area | " + " | ".join(f"`{s}`" for s in used) + " | Total |"
    rule = "|---|" + "---|" * (len(used) + 1)
    rows = [head, rule]
    totals = {s: 0 for s in used}

    for area in AREAS:
        c = counts[area]
        total = sum(c.values())
        if not total:
            continue
        cells = []
        for s in used:
            totals[s] += c.get(s, 0)
            cells.append(str(c[s]) if c.get(s) else "--")
        rows.append(f"| {titles[area]} | " + " | ".join(cells) + f" | {total} |")

    rows.append("| **All** | " + " | ".join(f"**{totals[s]}**" for s in used)
                + f" | **{sum(totals.values())}** |")

    failing = [r.ident for r in reqs if r.state == "failing"]
    note = ""
    if failing:
        note = ("\n\nCurrently failing: "
                + ", ".join(f"[{i}](requirements/{AREAS[i.split('-')[1]]}#{anchor(i)})"
                            for i in failing)
                + ".")
    return "\n".join(rows) + note


def write_summary(docs: Path, table: str, fail) -> bool:
    path = docs / "README.md"
    text = path.read_text(encoding="utf-8")
    if SUMMARY_BEGIN not in text or SUMMARY_END not in text:
        fail(path, 0, "summary markers are missing")
        return False
    head, _, rest = text.partition(SUMMARY_BEGIN)
    _, _, tail = rest.partition(SUMMARY_END)
    new = f"{head}{SUMMARY_BEGIN}\n\n{table}\n\n{SUMMARY_END}{tail}"
    if new == text:
        return False
    path.write_text(new, encoding="utf-8", newline="\n")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--write", action="store_true",
                    help="regenerate the summary table in docs/README.md")
    args = ap.parse_args()

    root = repo_root()
    docs = root / "docs"
    problems: list[str] = []

    def fail(path: Path, line: int, message: str) -> None:
        try:
            where = path.relative_to(root).as_posix()
        except ValueError:
            where = str(path)
        problems.append(f"{where}:{line}: {message}" if line else f"{where}: {message}")

    reqs = parse_requirements(docs, fail)
    known = {r.ident for r in reqs}
    check_mentions(root, known, fail)
    check_doc_paths(root, fail)
    check_links(root, fail)

    table = summary_table(reqs)
    if args.write:
        if write_summary(docs, table, fail):
            print("docs/README.md: summary table updated")
        else:
            print("docs/README.md: summary table already current")
    else:
        current = (docs / "README.md").read_text(encoding="utf-8")
        if SUMMARY_BEGIN in current and table not in current:
            fail(docs / "README.md", 0,
                 "summary table is out of date -- run: python docs/check_docs.py --write")

    if problems:
        for p in problems:
            print(p, file=sys.stderr)
        print(f"\n{len(problems)} problem(s)", file=sys.stderr)
        return 1

    states = ", ".join(
        f"{sum(1 for r in reqs if r.state == s)} {s}"
        for s in STATE_ORDER if any(r.state == s for r in reqs)
    )
    print(f"{len(reqs)} requirements ({states}); links and identifiers resolve")
    return 0


if __name__ == "__main__":
    sys.exit(main())
