"""The page itself: every pane its tabs offer, and what scrolls.

The console is one HTML file, so these read it as text. A tab with no pane behind
it and a popup that scrolls its whole body rather than its lists are both faults
a browser would show and no other check would.
"""
from __future__ import annotations

import os
import re

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import check


def test_the_page_draws_every_pane_its_tabs_offer() -> None:
    """A tab whose pane is never unhidden is a tab that does nothing.

    Cheap to get wrong when a tab is added, invisible until somebody clicks it,
    and there is no browser in this repository's test loop to catch it.
    """
    import os
    import re

    page = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "drive_web.html")
    with open(page, encoding="utf-8") as handle:
        html = handle.read()
    tabs = set(re.findall(r'data-wtab="([a-z]+)"', html))
    check("the popup offers four tabs", len(tabs), 4)
    for tab in sorted(tabs):
        pane = f'id="wPane{tab.capitalize()}"'
        check(f"the {tab} tab has a pane", pane in html, True)
        check(f"...that something unhides",
              f'$("wPane{tab.capitalize()}").hidden' in html, True)
    # Every element the script reaches for by name has to be in the markup, which
    # is the other half of the same mistake.
    for name in sorted(set(re.findall(r'\$\("(w[A-Za-z]+)"\)', html))):
        check(f"the page has an element called {name}",
              f'id="{name}"' in html, True)


def test_the_world_popup_scrolls_its_lists_not_its_body() -> None:
    """The map and headings stay put while the two entity lists move.

    These are layout rules rather than decoration: putting overflow back on the
    popup body makes the map leave the screen, and an unbounded list makes the
    popup body grow until that outer scrollbar returns.
    """
    page = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "drive_web.html")
    with open(page, encoding="utf-8") as handle:
        html = handle.read()

    def rule(selector: str) -> str:
        match = re.search(re.escape(selector) + r"\s*\{([^}]*)\}", html)
        return re.sub(r"\s+", " ", match.group(1)) if match else ""

    box = rule("#worldBox")
    body = rule("#worldBody")
    entities = rule("#wPaneEntities")
    entity_list = rule("#wList")
    observation_list = rule("#wDetail .wscroll")
    check("the world popup has a definite available height",
          re.search(r"(?<!-)height: 100%", box) is not None, True)
    check("the popup body cannot own a scrollbar", "overflow: hidden" in body,
          True)
    check("...and can shrink inside the popup", "min-height: 0" in body, True)
    check("the map and lists share one fixed-height row",
          "overflow: hidden" in entities and "align-items: stretch" in entities,
          True)
    check("the entity list scrolls by itself", "overflow-y: auto" in entity_list,
          True)
    check("the selected observations fill and scroll in the right pane",
          "flex: 1" in observation_list
          and "overflow-y: auto" in observation_list
          and "max-height: none" in observation_list, True)


TESTS = (
    test_the_page_draws_every_pane_its_tabs_offer,
    test_the_world_popup_scrolls_its_lists_not_its_body,
)
