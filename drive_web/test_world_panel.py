"""The world-state panel: finding a thing, and the switch that builds it.

The console is the only place a person reads the semantic world, so what is
checked is that a search reaches the store and comes back shaped the way the
panel draws it, and that the URLs it offers are the ones the server answers.
"""
from __future__ import annotations

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_the_world_state_popup() -> None:
    """The console's side of the semantic world, with no rover and no browser.

    What the page draws comes out of `Session` and out of two URLs, so those are
    what is checked here: the counts and the tag that rides in every pushed state,
    the payload the popup fetches when that tag moves, and the two failures the
    viewer has to survive -- a rover with no world-state component at all, and an
    observation whose stored frame is not there any more.

    The drawing itself is JavaScript in a browser and is not covered by anything
    here; the repository has no browser in its test loop. What that costs is
    written down in world_state/README.md rather than left to be discovered.
    """
    try:
        import json

        import drive_web
        from console_model import Reply
    except ImportError as exc:
        SKIP.append(f"the world-state popup ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)

    # Nothing has been asked yet, so the button says nothing about a rover it has
    # not spoken to. `available: None` is that third state, and it matters: a
    # console that assumed either answer would either hide a working feature or
    # offer a broken one.
    world = session.world_state()
    check("the world starts unknown rather than absent", world["available"], None)
    check("...and shut", world["open"], False)
    check("...with no picture to fetch yet", world["gen"], "")

    # An empty world. The popup opens on a rover that has never inspected
    # anything, and that has to read as "press the button", not as an error.
    session.world_handle("world_state_entities", {
        "ok": True, "entities": [], "recent": [], "unmatched": [],
        "summary": {"entities": 0, "observations": 0, "inspections": 0,
                    "map_session": 1}}, 0.1)
    check("an empty world is available rather than broken",
          session.world_state()["available"], True)
    check("...and says so in the counts", session.world_state()["entities"], 0)
    check("...and the popup has something to fetch",
          bool(session.world_state()["gen"]), True)

    # A populated one.
    observation = {"id": 1, "entity_id": "object:1", "observed_at": 1.0,
                   "source": "perception", "frame_id": "20260901-120000-abc123",
                   "label": "", "bbox": [0.1, 0.3, 0.5, 0.9],
                   "observer_pan_deg": 20.0, "observer_tilt_deg": -5.0,
                   "pose": {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0},
                   "map_session": 1, "model_id": "tensorrt",
                   "prompt_version": None,
                   "raw": {"region_score": 0.83, "area": 0.24}, "note": None}
    session.world_handle("world_state_entities", {
        "ok": True,
        "entities": [{"id": "object:1", "kind": "object", "label": "",
                      "canonical_description": "",
                      "created_at": 1.0, "last_seen_at": 2.0,
                      "observation_count": 2, "last_map_session": 1,
                      "last_frame_id": "20260901-120000-abc123",
                      "rays": [{"x_m": 1.0, "y_m": 2.0, "bearing_deg": 70.0,
                                "span_deg": 26.0, "length_m": 2.5}]}],
        "recent": [observation], "unmatched": [],
        "summary": {"entities": 1, "observations": 2, "inspections": 1,
                    "map_session": 1}}, 0.2)
    check("a populated world reaches the counts",
          (session.world_state()["entities"], session.world_state()["observations"]),
          (1, 2))
    check("...and the payload carries the ray the map is drawn from",
          session.world_payload["entities"][0]["rays"][0]["bearing_deg"], 70.0)
    check("...and the whole observation stream, so duplicates are visible",
          len(session.world_payload["recent"]), 1)

    # The frame behind an observation. The console fetches the newest few and
    # serves them at their own URL; one it has not fetched is a name, not a broken
    # picture, and the page is told which by the list of frames it holds.
    check("a frame that has not arrived is not offered to the page",
          session.world_payload["frames"], [])
    session.world_handle("world_state_frame", {
        "ok": True, "frame_id": "20260901-120000-abc123",
        "jpeg_base64": "/9j/4AAQSkZJRg=="}, 0.1)
    check("one that has is", session.world_payload["frames"],
          ["20260901-120000-abc123"])
    check("...and is held where the URL can find it",
          "20260901-120000-abc123" in session.world_frames, True)

    # An inspection, and the one line the popup's header gets out of it.
    session.world_handle("world_inspect", {
        "ok": True, "created": 2, "matched": 1, "rejected": 0,
        "detail": ""}, 61.0)
    check("an inspection says what it did, in a line",
          session.world_state()["note"], "2 new, 1 recognised -- 61 s")
    check("...and is no longer running", session.world_state()["busy"], False)

    session.world_handle("world_inspect", {
        "ok": True, "created": 0, "matched": 0, "rejected": 0}, 58.0)
    check("...and an inspection that found nothing says that rather than nothing",
          session.world_state()["note"],
          "nothing worth recording in view -- 58 s")

    # A failure. The popup has to be able to say the sidecar failed, because the
    # alternative reading of an unchanged world -- nothing was in view -- is fixed
    # somewhere else entirely.
    session.world_handle("world_inspect", {
        "ok": False, "error": "the perception sidecar at http://127.0.0.1:8776 is "
                              "not answering"}, 3.0)
    check("a failed inspection is reported as one",
          "not answering" in session.world_state()["error"], True)
    check("...and the button comes back", session.world_state()["busy"], False)

    # Clearing. One button, the map's, because everything the world state holds is
    # a position measured in the map's frame: what used to survive a map clear was
    # a list of things with nowhere to be, and the two were always cleared
    # together anyway.
    session.world_link = _Recorder()
    session.world_map_cleared()
    check("clearing the map clears the world with it",
          [name for name, _ in session.world_link.calls],
          ["world_state_clear", "world_map_session"])
    check("...and lets go of the frames it was holding",
          session.world_frames, {})
    check("...and of whatever was selected", session.world_selected, "")

    # The store says what went, and it is the map's clear that is being reported.
    session.world_handle("world_state_clear",
                         {"ok": True, "entities": 4, "observations": 96}, 0.1)
    note = session.world_state()["note"]
    check("what went is counted", "4 entities" in note and "96 " in note, True)
    check("...as part of clearing the map", note.startswith("cleared"), True)

    # A rover with no world-state component says so once. Anything else is a
    # popup that shows the same error every few seconds for the rest of the day.
    session.world_handle("world_state_summary", {
        "ok": False, "error": "this rover has no world_state component installed"},
        0.1)
    check("a rover without the component is remembered as such",
          session.world_state()["available"], False)
    check("...and the reason is kept for the popup to show",
          "no world_state component" in session.world_state()["error"], True)


class _Recorder:
    """A channel that writes down what it was asked for instead of asking."""

    def __init__(self) -> None:
        self.calls = []

    def submit(self, name, arguments=None) -> None:
        self.calls.append((name, arguments))


def test_finding_a_thing_from_the_console() -> None:
    """Typing a phrase, and the two answers it can get.

    The interesting case is the third one: the model takes several seconds, so an
    answer can arrive after the person has typed something else, and drawing it
    would read as the search having got the wrong thing. It is dropped instead.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the console search ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.world_link = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()

    session.world_act({"what": "search", "query": "  a spray bottle  "})
    check("the phrase is sent, trimmed", sent[-1][0], "world_state_search")
    check("...as the rover's own argument", sent[-1][1]["query"], "a spray bottle")
    check("...and the box says it is working",
          session.world_state()["searching"], True)

    session.world_handle("world_state_search", {
        "ok": True, "query": "a spray bottle", "confident": True,
        "best": 0.13, "considered": 31, "skipped": 0,
        "detail": "the best match scores 0.130 against that description",
        "matches": [{"score": 0.13, "observation_id": 4, "entity_id": "object:1",
                     "frame_id": "f1",
                     "placement": {"x_m": 2.0, "y_m": 1.0}}]}, 4.0)
    check("the answer stops the working state",
          session.world_state()["searching"], False)
    check("...and is kept for the popup to draw",
          session.world_payload["search"]["matches"][0]["observation_id"], 4)

    # An answer to the phrase before last, arriving after the box has moved on.
    session.world_act({"what": "search", "query": "a bicycle"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "a spray bottle", "confident": True,
        "matches": [{"score": 0.9, "observation_id": 99}]}, 4.0)
    check("a late answer to an older phrase is dropped",
          session.world_payload["search"]["matches"][0]["observation_id"], 4)

    # An empty box clears the pane rather than asking the rover about nothing.
    before = len(sent)
    session.world_act({"what": "search", "query": "   "})
    check("an empty phrase asks the rover nothing", len(sent), before)
    check("...and takes the last answer off the screen",
          "search" in session.world_payload, False)

    # A rover that cannot answer says so without leaving the box spinning.
    session.world_act({"what": "search", "query": "a mirror"})
    session.world_handle("world_state_search",
                         {"ok": False, "error": "no perception sidecar"}, 0.2)
    check("a refusal stops the working state",
          session.world_state()["searching"], False)
    check("...and is shown", session.world_state()["error"], "no perception sidecar")


def test_the_switch_for_building_the_world_state() -> None:
    """On by default on the rover, and this panel never guesses at it.

    What it shows comes back from the rover rather than from what this console
    last asked for, because the voice session, another console or a script can
    turn it off -- and a panel showing its own past would leave a rover that had
    quietly stopped recording still looking busy.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the world-building switch ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.watch = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()

    check("nothing is claimed before the rover has been asked",
          session.world_state()["building"], None)

    session.world_act({"what": "build", "on": False})
    check("the switch is sent to the rover", sent[-1][0], "world_building")
    check("...as what was asked for", sent[-1][1], {"on": False})
    check("...and the panel still does not guess",
          session.world_state()["building"], None)

    session.world_handle("world_building",
                         {"ok": True, "building": False, "looks": 7}, 0.0)
    check("the panel shows what the rover said",
          session.world_state()["building"], False)
    check("...and how much it has recorded",
          session.world_state()["built_looks"], 7)

    session.world_handle("world_building",
                         {"ok": True, "building": True, "looks": 8}, 0.0)
    check("and again when it is turned back on",
          session.world_state()["building"], True)

    # The loop's own complaint is the only place a rover that has stopped
    # recording would ever say so.
    session.world_handle("world_building",
                         {"ok": True, "building": True, "looks": 8,
                          "error": "the perception sidecar is not running"}, 0.0)
    check("a loop that is failing says why", session.world_state()["error"],
          "the perception sidecar is not running")

    # A rover with no world state at all stops being asked, rather than showing
    # an error every ten seconds for the rest of the session.
    session.world_handle("world_building",
                         {"ok": False, "error": "no world_state component"}, 0.0)
    check("a rover without it is marked absent",
          session.world_state()["available"], False)
    check("...and the poll is not left outstanding",
          session.world_build_outstanding, False)


def test_the_world_urls() -> None:
    """`/world.json` and `/world_frame.jpg`, over a real socket.

    Both exist for the same reason the network list and the map do: the payload is
    tens of kilobytes and the frames are whole JPEGs, and the state they would
    otherwise ride in goes out ten times a second.
    """
    try:
        import http.client
        import json
        import threading

        import drive_web
    except ImportError as exc:
        SKIP.append(f"the world URLs ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.world_payload = {"summary": {"entities": 1}, "frames": ["a-frame"]}
    session.world_frames["a-frame"] = b"\xff\xd8\xff\xe0not really a jpeg\xff\xd9"

    # The handler reaches its session through a class attribute rather than
    # through the server, which is how the console itself wires it up.
    was, drive_web.Handler.session = drive_web.Handler.session, session
    server = drive_web.Console(("127.0.0.1", 0), drive_web.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    try:
        connection.request("GET", "/world.json?gen=x")
        reply = connection.getresponse()
        body = json.loads(reply.read())
        check("the world payload is served", reply.status, 200)
        check("...as the popup's own copy", body["summary"]["entities"], 1)

        connection.request("GET", "/world_frame.jpg?id=a-frame")
        reply = connection.getresponse()
        frame = reply.read()
        check("a stored frame is served as a picture", reply.status, 200)
        check("...with the bytes the rover kept", frame.startswith(b"\xff\xd8"), True)

        # The row outlives the file. A viewer that fell over here would be hiding
        # exactly the observations it was opened to look at.
        connection.request("GET", "/world_frame.jpg?id=gone")
        reply = connection.getresponse()
        reply.read()
        check("a frame that is not held answers plainly rather than failing",
              reply.status, 404)

        connection.request("GET", "/world_frame.jpg")
        reply = connection.getresponse()
        reply.read()
        check("...and so does asking for no frame at all", reply.status, 404)
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        drive_web.Handler.session = was


TESTS = (
    test_the_world_state_popup,
    test_finding_a_thing_from_the_console,
    test_the_switch_for_building_the_world_state,
    test_the_world_urls,
)
