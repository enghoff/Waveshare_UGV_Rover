"""The page itself: every pane its tabs offer, what scrolls, and what is served.

The console is markup, a stylesheet and four scripts, so these read them as text.
A tab with no pane behind it and a popup that scrolls
its whole body rather than its lists are both faults a browser would show and no
other check would. So is a stylesheet the server does not know how to hand over,
which is why the last check here fetches every asset over a real socket.
"""
from __future__ import annotations

import os
import re

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def _console(*parts: str) -> str:
    """The console source, joined by logical component.

    `world.js` joins all three world modules so checks do not depend on their
    file boundaries.
    """
    import os

    here = os.path.dirname(os.path.abspath(__file__))
    out = []
    for part in parts:
        names = (["drive_world.js", "drive_world_map.js",
                  "drive_world_observations.js"] if part == "world.js"
                 else [f"drive_{part}" if part.endswith(".js")
                       else f"drive_web.{part}"])
        for name in names:
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


def test_the_map_offers_two_acts_and_the_script_can_find_them_both() -> None:
    """The map card had five controls and has two, and both of those do something.

    Three went on 2026-09-05 and the argument was the popup's own: the map
    redraws itself every few seconds, so a "refresh" button could only ever fetch
    what the console had a moment ago; "describe surroundings" put into words
    what the picture above it was already showing; and turning the map to the
    rover's heading made the room swing under the reader on every turn. What is
    left is the two things that are acts rather than views -- putting the rover
    back on the map, and throwing the map away.

    The half of this worth automating is the other half: an element removed from
    the markup while the script still reaches for it is a page whose script dies
    on the first line that touches it, taking every panel with it, and there is
    no browser in this repository's test loop to notice.
    """
    import re

    html, js = _console("html"), _console("js")
    row = html[html.index('id="mapCard"'):html.index('id="cameraCard"')]
    buttons = re.findall(r'<button id="([A-Za-z]+)"', row)
    check("the map card offers exactly two buttons", len(buttons), 2)
    check("...one that fits the rover to the map", "refitPose" in buttons, True)
    check("...and one that throws the map away", "clearMap" in buttons, True)
    for gone in ("roverUp", "refreshMap", "describe"):
        check(f"nothing on the page is called {gone} any more",
              f'id="{gone}"' in html, False)
        check(f"...and the script does not reach for it either",
              f'$("{gone}")' in js, False)
    # And the general form of that mistake, for every element this script names.
    for name in sorted(set(re.findall(r'\$\("([A-Za-z][A-Za-z0-9_]*)"\)', js))):
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


def test_the_evidence_bar_narrows_both_views_and_says_what_it_hid() -> None:
    """How well placed a thing must be to be drawn, and the count that goes with it.

    The same rule the phrase above obeys: a list narrowed to the well-placed
    things beside a map still carrying the cloud would be two answers to one
    question. And a bar that hid things silently would read as a store that had
    lost them, so the line under the map says how many it is holding back --
    separately from the things that have no position at all, which is a different
    absence and already counted there.
    """
    html = _console("html")
    js = _console("world.js")

    check("the bar sits beside the phrase, above the views",
          html.index('id="wGrade"') < html.index('id="worldBody"'), True)
    check("...and offers three steps",
          html[html.index('id="wGrade"'):
               html.index("</select>", html.index('id="wGrade"'))
               ].count("<option"), 3)
    check("every step the markup offers is a step the script knows",
          sorted(re.findall(r'<option value="([a-z]+)"', html)),
          sorted(re.findall(r"^  ([a-z]+): \(", js, re.M)))
    check("the bar is applied where both views read their things",
          "WGRADES[worldGrade]" in js, True)
    check("...which is one function they share", js.count("= wShown()"), 2)
    check("changing it redraws both without asking the rover",
          '$("wGrade").onchange' in js
          and "drawWorldList();" in js and "drawWorldMap();" in js, True)
    check("the line under the map says how many it is holding back",
          "held} held back" in js, True)
    check("...counted against what is placed, not against the whole store",
          "one.placement && !keep(one)" in js, True)


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
    """Every page asset, over a real socket, with the types a browser needs.

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
                  ("/drive_world.js", "javascript", b"drawWorld"),
                  ("/drive_world_map.js", "javascript", b"drawWorldMap"),
                  ("/drive_world_observations.js", "javascript",
                   b"drawWorldObservations"))
        for path, kind, inside in wanted:
            connection.request("GET", path)
            reply = connection.getresponse()
            body = reply.read()
            check(f"{path} is served", reply.status, 200)
            check(f"...as {kind}", kind in reply.getheader("Content-Type"), True)
            check("...and is the file itself", inside in body, True)

        # The page must load every declaration before drive_web.js calls start().
        connection.request("GET", "/")
        page = connection.getresponse().read().decode()
        check("the page asks for its stylesheet",
              'href="/drive_web.css"' in page, True)
        scripts = [page.index(f'src="/{name}"') for name in
                   ("drive_world.js", "drive_world_map.js",
                    "drive_world_observations.js", "drive_web.js")]
        check("...and loads every world module before the main script",
              scripts, sorted(scripts))
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


#: What text that has been round-tripped through the wrong encoding looks like
#: afterwards, as byte patterns. A file read as Latin-1 and written back as UTF-8
#: turns each byte of a real character into a character of its own, and the
#: leading byte of the original is what gives it away: a middle dot or a degree
#: sign starts 0xC2 and comes back with a capital A-circumflex in front of it,
#: while a curly quote, an em dash or an ellipsis starts 0xE2 and comes back as
#: a-circumflex followed by a euro sign. Neither pair occurs in honest prose.
MANGLED = (
    (rb"\xc3\x82[\xc2-\xc3][\x80-\xbf]", "a middle dot or a degree sign"),
    (rb"\xc3\xa2\xe2\x82\xac", "a curly quote, an em dash or an ellipsis"),
)


def test_the_console_is_written_in_the_encoding_it_is_served_in() -> None:
    """**Mojibake is invisible to every other check here.**

    The console says things like "6 observations, at (-1.7, 0.4) m", with a
    middle dot between the parts and a degree sign after every bearing, and
    those are exactly the characters an editor that guessed Latin-1 mangles.
    Nothing else notices: the file is still valid UTF-8, the page still parses,
    and the tests above still find every string they look for, because they read
    the same mangled bytes the browser does. It reached the rover -- 26 of them
    were live on the console until 2026-09-05, two characters on screen wherever
    one was meant.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    for name in ("drive_web.html", "drive_web.css", "drive_web.js",
                 "drive_world.js", "drive_world_map.js",
                 "drive_world_observations.js"):
        with open(os.path.join(here, name), "rb") as handle:
            raw = handle.read()
        try:
            raw.decode("utf-8")
        except UnicodeDecodeError:
            check(f"{name} is UTF-8, which is what it is served as", False, True)
            continue
        for pattern, what in MANGLED:
            check(f"{name} has {what} the browser can read",
                  len(re.findall(pattern, raw)), 0)


TESTS = (
    test_the_page_draws_every_pane_its_tabs_offer,
    test_the_console_is_written_in_the_encoding_it_is_served_in,
    test_the_evidence_bar_narrows_both_views_and_says_what_it_hid,
    test_the_map_offers_two_acts_and_the_script_can_find_them_both,
    test_the_world_popup_scrolls_its_lists_not_its_body,
    test_the_observation_stream_is_tiled_and_opens_one_at_a_time,
    test_the_search_box_narrows_the_views_rather_than_owning_one,
    test_the_popup_can_be_read_while_the_rover_is_filling_it,
    test_the_page_has_no_voice_token_control,
    test_the_page_brings_its_stylesheet_and_script_with_it,
)
