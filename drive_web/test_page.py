"""The page itself: every pane its tabs offer, what scrolls, and what is served.

The console is four files -- the markup, its stylesheet, and the two scripts --
so these read them as text. A tab with no pane behind it and a popup that scrolls
its whole body rather than its lists are both faults a browser would show and no
other check would. So is a stylesheet the server does not know how to hand over,
which is why the last check here fetches all four over a real socket.
"""
from __future__ import annotations

import os
import re

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def _console(*parts: str) -> str:
    """The console's source, in whichever of its four files is wanted.

    Ask for them by suffix: "html", "css", "js" for drive_web.js, and "world.js"
    for drive_world.js. A check that spans more than one of them -- an element
    the script names and the markup has to declare -- reads them together.
    """
    import os

    here = os.path.dirname(os.path.abspath(__file__))
    out = []
    for part in parts:
        name = f"drive_{part}" if part.endswith(".js") else f"drive_web.{part}"
        with open(os.path.join(here, name), encoding="utf-8") as f:
            out.append(f.read())
    return "\n".join(out)


def test_the_page_draws_every_pane_its_tabs_offer() -> None:
    """A tab whose pane is never unhidden is a tab that does nothing.

    Cheap to get wrong when a tab is added, invisible until somebody clicks it,
    and there is no browser in this repository's test loop to catch it.
    """
    import re

    # drive_world.js too: most of the `$("wXxx")` lookups are the popup's.
    html = _console("html", "js", "world.js")
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
    html = _console("css")

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


def test_the_page_brings_its_stylesheet_and_script_with_it() -> None:
    """All four files, over a real socket, with the types a browser needs.

    The stylesheet and the scripts used to be inside the page and are beside it
    now, which moves them from something that cannot go missing to something that
    can. A console served without its script is a page of dead buttons, and it
    looks exactly like a rover that stopped answering. The order of the two
    scripts is checked too: drive_web.js calls `start()` as it loads, so the
    popup's half has to be there already.
    """
    try:
        import http.client
        import threading

        import drive_web
    except ImportError as exc:
        SKIP.append(f"the console's files ({type(exc).__name__})")
        return

    was, drive_web.Handler.session = (drive_web.Handler.session,
                                      drive_web.Session(None, 3.0, 480))
    server = drive_web.Console(("127.0.0.1", 0), drive_web.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    connection = http.client.HTTPConnection("127.0.0.1",
                                            server.server_address[1], timeout=5)
    try:
        wanted = (("/", "text/html", b"<html"),
                  ("/drive_web.css", "text/css", b":root"),
                  ("/drive_web.js", "javascript", b'"use strict"'),
                  ("/drive_world.js", "javascript", b"drawWorld"))
        for path, kind, inside in wanted:
            connection.request("GET", path)
            reply = connection.getresponse()
            body = reply.read()
            check(f"{path} is served", reply.status, 200)
            check(f"...as {kind}", kind in reply.getheader("Content-Type"), True)
            check("...and is the file itself", inside in body, True)

        # And the page has to ask for the other two, or serving them is no use.
        connection.request("GET", "/")
        page = connection.getresponse().read().decode()
        check("the page asks for its stylesheet",
              'href="/drive_web.css"' in page, True)
        check("...and for both of its scripts",
              page.index('src="/drive_world.js"') < page.index('src="/drive_web.js"'),
              True)
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        drive_web.Handler.session = was


TESTS = (
    test_the_page_draws_every_pane_its_tabs_offer,
    test_the_world_popup_scrolls_its_lists_not_its_body,
    test_the_page_brings_its_stylesheet_and_script_with_it,
)
