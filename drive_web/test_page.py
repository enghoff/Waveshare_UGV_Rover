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
    check("the popup offers three tabs", len(tabs), 3)
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


def test_the_observation_stream_is_tiled_and_opens_one_at_a_time() -> None:
    """Thumbnails in a grid, and the one look a click puts over them.

    Layout again rather than decoration, and the same kind of fault: a grid whose
    tiles are the tallest of their row, or a large view the popup body cannot
    position, are both things only a browser would show. The last check here is
    the one that matters most -- space and Escape stop the rover, and a large
    picture that took the keyboard to close itself would take the stop key with
    it.
    """
    css = _console("css")
    js = _console("world.js")

    def rule(selector: str) -> str:
        match = re.search(re.escape(selector) + r"\s*\{([^}]*)\}", css)
        return re.sub(r"\s+", " ", match.group(1)) if match else ""

    tiles, tile = rule(".wtiles"), rule(".wtile")
    check("the stream is a grid of thumbnails",
          "display: grid" in tiles and "auto-fill" in tiles, True)
    check("...whose tiles pack to their own height rather than their row's",
          "align-items: start" in tiles, True)
    check("...and are drawn by the script as tiles",
          'className = "wtile"' in js or '"wtile"' in js, True)
    check("a tile is a button, so the grid is reachable by keyboard",
          'createElement("button")' in js and "wtile" in js, True)
    check("...reset out of looking like one", "font: inherit" in tile, True)

    body, zoom, zoomed = rule("#worldBody"), rule("#wZoom"), rule("#wZoomBody")
    check("the popup body can position the large view",
          "position: relative" in body, True)
    check("the large view covers it", "position: absolute" in zoom
          and "inset: 0" in zoom, True)
    check("...and scrolls itself rather than the body it covers",
          "overflow-y: auto" in zoomed and "min-height: 0" in zoomed, True)
    check("clicking a tile opens it", "worldZoom = observation.id" in js, True)
    check("...and the close button shuts it",
          '$("wZoomClose").onclick' in js, True)
    check("...as does moving to another tab",
          js.count("worldZoom = null") >= 3, True)
    # The one thing that must not have happened. Stop is space and Escape, held
    # by drive_web.js, and nothing in the popup may listen for a key.
    check("nothing in the popup listens for a key", "keydown" in js, False)

    box = rule(".wbox")
    check("the measured box is red rather than the console's accent",
          "var(--box)" in box, True)
    check("...and that red is declared for both themes",
          css.count("--box:") >= 2, True)


def test_the_search_box_narrows_the_views_rather_than_owning_one() -> None:
    """One phrase, and every view of the store answers it.

    The search used to be a fourth tab holding a ranked list of crops beside
    three other views of the same store, so finding something meant reading the
    answer in one place and hunting for it in the others. It is a filter now, and
    what has to hold is that all of it moves together: an entity list narrowed to
    one thing beside a map still covered in everything is two answers to one
    question, and this popup exists to make disagreements like that visible
    rather than to produce them.
    """
    html = _console("html")
    js = _console("world.js")
    css = _console("css")

    check("the box sits above the views rather than inside one",
          html.index('id="wFilter"') < html.index('id="worldBody"'), True)
    check("...so nothing is left of the pane it used to answer in",
          "wPaneSearch" in html or "wSearchResults" in html, False)
    check("the list and the map are narrowed by the same reading of the answer",
          js.count("= wShown()"), 2)
    check("...and the observation grid by the matches themselves",
          "worldFilter.looks.values()" in js, True)
    # Paging is by where the drawn stream ends, and under a filter the grid is
    # not the stream -- so scrolling to the bottom of the matches must not go
    # asking the rover for the looks below whatever the stream happens to hold.
    check("scrolling the matches does not fetch more history",
          "if (worldFilter || pane.hidden" in js, True)
    check("the verdict is on the line under the box",
          "#wSearchNote.wverdict" in css, True)
    check("...and a refusal to find something does not merely score lower",
          "#wSearchNote.wfound" in css and "#wSearchNote.wmissing" in css, True)
    # A phrase has spaces in it, and space is the rover's stop everywhere else on
    # this page. Without the exemption the box silently refuses the space bar and
    # halts the rover instead, which is what a person typing here first sees.
    stop = _console("js")
    check("a space typed in the box goes into the box",
          'event.target.id === "wSearchBox"' in stop, True)
    check("...and Escape still stops the rover from inside it",
          'event.key === "Escape"' in stop
          and 'event.key === " " && !phrase' in stop, True)


def test_the_popup_can_be_read_while_the_rover_is_filling_it() -> None:
    """Nothing a person is inside is thrown away because the store moved.

    The rover records a look a second, and every one of them sends a new body to
    a popup that is open. Building the views again from nothing on each of those
    is what made the popup unreadable while the rover worked: measured in a
    browser on the rover's own store of sixty things, the entity list slid 52 px
    under the pointer per body and went on sliding, and one look arriving on the
    chosen thing threw the reader back to the top of its pictures with an opened
    raw block shut and all nine crops fetched again.

    So the three views a person scrolls keep their rows and move them, which is
    also what gives the browser something to hold the view still against. What is
    checked here is that each of them still does -- the drawing itself is in a
    browser and nothing in this repository has one.
    """
    js = _console("world.js")

    def body(name: str) -> str:
        """One function out of the script, brace to brace at column zero."""
        match = re.search(r"^function " + name + r"\(.*?\n\}", js,
                          re.S | re.M)
        check(f"the script still has {name}", bool(match), True)
        return match.group(0) if match else ""

    # Each view holds its rows by what the rover calls them, and moves the ones
    # it already has rather than making them again.
    for name, held in (("drawWorldList", "worldRows"),
                       ("drawWorldLooks", "worldDetailRows"),
                       ("drawWorldObservations", "worldTiles")):
        drawn = body(name)
        check(f"{name} keeps its rows by identifier", held in drawn, True)
        check("...and moves the ones it has", "insertBefore" in drawn, True)
        check("...rather than emptying the view first",
              "replaceChildren()" in drawn, False)

    # The chosen thing's pane is the one with something to lose: its rows carry
    # the pictures and any raw block opened under them. So a look is only drawn
    # again when what it says has changed, and the boxes it lives in are only
    # replaced when a *different* thing is chosen.
    looks = body("drawWorldLooks")
    check("a look is redrawn only when it has really changed",
          "row.drawn !== drawn" in looks, True)
    chosen = body("drawWorldDetail")
    check("the pane is replaced only for a different thing",
          "worldDetailFor !== entity.id" in chosen, True)
    check("...and the heading only when it has something new to say",
          "key === worldDetailHead" in body("drawWorldHead"), True)

    # The entity list is the other way round, and deliberately: one of the things
    # a row says is how long ago the thing was last seen, so a row held unchanged
    # would be a row whose age had quietly stopped counting.
    listed = body("drawWorldList")
    check("every row in the list is written afresh", "wRowFace(row" in listed, True)
    check("...which is what keeps its ages counting",
          "wAgo(" in body("wRowFace"), True)


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


def test_the_page_has_no_voice_token_control() -> None:
    """Starting Qwen is a direct button action, with no stored browser secret."""
    html = _console("html", "js")
    check("the talk control has no token field", "voiceToken" in html, False)
    check("the browser stores no Qwen token", "omniToken" in html, False)
    check("the audio socket carries no token query", "/audio?k=" in html, False)


TESTS = (
    test_the_page_draws_every_pane_its_tabs_offer,
    test_the_world_popup_scrolls_its_lists_not_its_body,
    test_the_observation_stream_is_tiled_and_opens_one_at_a_time,
    test_the_search_box_narrows_the_views_rather_than_owning_one,
    test_the_popup_can_be_read_while_the_rover_is_filling_it,
    test_the_page_has_no_voice_token_control,
    test_the_page_brings_its_stylesheet_and_script_with_it,
)
