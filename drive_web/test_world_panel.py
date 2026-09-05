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
        "ok": True, "entities": [], "recent": [],
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
                      "placement": {"x_m": 3.0, "y_m": 4.0,
                                    "uncertainty_m": 0.4,
                                    "error_major_m": 0.4,
                                    "error_minor_m": 0.08,
                                    "error_major_deg": 30.0,
                                    "extent_m": 0.2,
                                    "rays_agreeing": 7, "viewpoints": 3,
                                    "refined_from": 7},
                      "placement_map_session": 1,
                      "rays": [{"id": 9, "x_m": 1.0, "y_m": 2.0,
                                "heading_deg": 90.0, "pan_deg": 20.0,
                                "bearing_deg": 70.0,
                                "span_deg": 26.0, "length_m": 2.5,
                                "relation": {"range_m": 2.83, "to_deg": 45.0,
                                             "off_deg": 25.0, "miss_m": 1.32,
                                             "tolerance_m": 0.68,
                                             "agrees": False}}]}],
        "recent": [observation],
        "summary": {"entities": 1, "observations": 2, "inspections": 1,
                    "map_session": 1}}, 0.2)
    check("a populated world reaches the counts",
          (session.world_state()["entities"], session.world_state()["observations"]),
          (1, 2))
    drawn = session.world_payload["entities"][0]["rays"][0]
    check("...and the payload carries the ray the map is drawn from",
          drawn["bearing_deg"], 70.0)
    # The map draws each look against the one position the thing settled on, and
    # the numbers behind that are the rover's -- the resolver's own arithmetic,
    # worked out where the geometry lives. A console that recomputed them could
    # draw a look as agreeing that the rover would refuse.
    check("...with where the rover was facing, which the camera angle hides",
          (drawn["heading_deg"], drawn["pan_deg"]), (90.0, 20.0))
    check("...and how that look stands to the settled position",
          (drawn["relation"]["miss_m"], drawn["relation"]["agrees"]),
          (1.32, False))
    check("...and the observation it came from, so a row can be joined to it",
          drawn["id"], 9)
    # The error is a shape rather than a radius, and the popup has both axes: a
    # crossing taken at a shallow angle is long down the sight line and short
    # across it, and one number is the long one.
    place = session.world_payload["entities"][0]["placement"]
    check("...and the placement keeps the shape of its error, not just a radius",
          (place["error_major_m"], place["error_minor_m"],
           place["error_major_deg"]), (0.4, 0.08, 30.0))
    # Seven rays from three places is different evidence from seven rays taken
    # from one doorway, and the observation count beside it cannot say which.
    check("...and how many separate places agreed with it, not just how many "
          "looks did", (place["viewpoints"], place["rays_agreeing"]), (3, 7))
    check("...and the whole observation stream, so duplicates are visible",
          len(session.world_payload["recent"]), 1)

    # The frame behind an observation, which every row in the popup now draws.
    # It is fetched off the rover the first time the page asks for it, so what
    # has to hold is that a picture already held is answered without going back,
    # and that a frame the rover has thrown away is a missing picture rather than
    # an exception on the thread serving the page.
    check("a frame nothing has asked for yet is not in memory",
          session.world_frames, {})
    held = bytes([0xFF, 0xD8]) + b"a stored picture"
    session.world_frames["20260901-120000-abc123"] = held
    check("one that is held is answered from memory",
          session.world_frame("20260901-120000-abc123"), held)
    check("...and a rover this console never connected to is a missing picture",
          session.world_frame("20260901-120000-nothing"), None)

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

    def submit(self, name, arguments=None, tag="") -> None:
        self.calls.append((name, arguments) if not tag
                          else (name, arguments, tag))


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
    # The pane counts the wait off, so the seconds are the rover's own rather
    # than each browser's guess from when it first saw the flag.
    session.world_search_since -= 3.2
    check("...for as long as it has been working",
          session.world_state()["searched_s"], 3)

    session.world_handle("world_state_search", {
        "ok": True, "query": "a spray bottle", "confident": True,
        "best": 0.13, "considered": 31, "skipped": 0,
        "detail": "the best match scores 0.130 against that description",
        "matches": [{"score": 0.13, "observation_id": 4, "entity_id": "object:1",
                     "frame_id": "f1",
                     "placement": {"x_m": 2.0, "y_m": 1.0}}]}, 4.0)
    check("the answer stops the working state",
          session.world_state()["searching"], False)
    check("...and stops the clock with it",
          session.world_state()["searched_s"], 0)
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


def test_the_best_thing_a_search_found_is_chosen_without_a_click() -> None:
    """A search asks where one thing is, so the answer is one thing.

    Three cases decide the rule: the top of the ranking is a look belonging to a
    thing, the top of it is a look belonging to nothing yet, and none of it
    belongs to anything. The second is the one worth having -- a thing seen once
    has no entity behind its look, and stopping at that row would leave the real,
    placed thing under it unselected on a screen that had just narrowed to it.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the search's own choice ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.world_link = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()

    session.world_act({"what": "search", "query": "the sofa"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "the sofa", "confident": True, "floor": 0.09,
        "matches": [{"score": 0.14, "observation_id": 4, "entity_id": "object:2"},
                    {"score": 0.11, "observation_id": 5, "entity_id": "object:7"}],
    }, 4.0)
    check("the best-scoring thing is chosen", session.world_selected, "object:2")
    check("...and its own history asked for", sent[-1],
          ("world_state_entity", {"id": "object:2"}))

    # The best *look* belongs to nothing, which is the ordinary state of anything
    # seen once. The choice falls through to the best one that does.
    session.world_act({"what": "search", "query": "the lamp"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "the lamp", "confident": True, "floor": 0.09,
        "matches": [{"score": 0.16, "observation_id": 8, "entity_id": None},
                    {"score": 0.12, "observation_id": 9, "entity_id": "object:7"}],
    }, 4.0)
    check("a look with nothing behind it is passed over",
          session.world_selected, "object:7")

    # Below the floor the answer is "nothing here matches", and the nearest thing
    # the rover has is still what it settled for. Hiding it would leave the person
    # who wanted to see that with nothing on the screen.
    session.world_act({"what": "search", "query": "a jet engine"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "a jet engine", "confident": False, "floor": 0.09,
        "matches": [{"score": 0.05, "observation_id": 11, "entity_id": "object:3"}],
    }, 4.0)
    check("what the rover settled for is shown even when it is not a match",
          session.world_selected, "object:3")

    # And a phrase that matched only looks nothing has been made of leaves the
    # detail pane describing something the narrowed list no longer shows.
    session.world_act({"what": "search", "query": "a kite"})
    session.world_handle("world_state_search", {
        "ok": True, "query": "a kite", "confident": True, "floor": 0.09,
        "matches": [{"score": 0.13, "observation_id": 12, "entity_id": None}],
    }, 4.0)
    check("nothing placed means nothing chosen", session.world_selected, "")


def test_a_looking_loop_that_has_failed_still_says_so() -> None:
    """There is no world-state line on the page any more, and none is wanted.

    The rover looks around for as long as it is switched on, so a panel reporting
    that could only ever report the same thing. What had to survive the panel
    going is the loop's own complaint: a rover that has quietly stopped recording
    is the one thing about its looking that a person cannot see for themselves,
    and it now lands on the error line of the popup that is showing the store it
    has stopped filling.
    """
    try:
        import drive_web
    except ImportError as exc:
        SKIP.append(f"the looking loop's complaint ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    sent = []
    session.watch = type("Link", (), {
        "submit": lambda _self, name, arguments=None: sent.append((name, arguments)),
    })()
    counts = {"entities": 2, "observations": 9}

    check("nothing is claimed before the rover has been asked",
          session.world_state()["available"], None)

    # Nothing asks the rover to build, or to stop: a cached page still posting
    # the switch that used to be here is ignored rather than obeyed.
    session.world_act({"what": "build", "on": False})
    check("there is nothing to send", sent, [])

    session.world_handle("world_state_summary",
                         {"ok": True, "summary": counts, "backend": "tensorrt",
                          "building_error": "the perception sidecar is not "
                                            "running"}, 0.0)
    check("a loop that is failing says why", session.world_state()["error"],
          "the perception sidecar is not running")

    session.world_handle("world_state_summary",
                         {"ok": True, "summary": counts,
                          "backend": "tensorrt"}, 0.0)
    check("...and the line clears once it recovers",
          session.world_state()["error"], "")

    # It is still the world's own call that decides whether this rover has a
    # world state at all, and the popup says so rather than showing an error.
    session.world_handle("world_state_summary",
                         {"ok": False,
                          "error": "this rover has no world_state component "
                                   "installed"}, 0.0)
    check("the world's own call is what marks a rover absent",
          session.world_state()["available"], False)


def test_an_open_popup_keeps_itself_current() -> None:
    """Nobody presses refresh, and nothing is re-sent that has not changed.

    The rover records a look a second and settles identities every ten, so a
    popup that only moved when it was asked to was a still photograph of a store
    that had gone on changing. What has to hold is both halves of the fix: that
    an open popup asks the rover on its own, and that asking every couple of
    seconds does not push 74 kB of unchanged payload at the browser each time --
    the tag it fetches under only moves when the body is really different.
    """
    try:
        import drive_web
        from console_model import WORLD_OPEN_POLL_S, WORLD_RETRY_S
    except ImportError as exc:
        SKIP.append(f"the open world popup ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.world_link = _Recorder()
    session.listeners = 1
    # No rover to find, so the pump would otherwise spend every tick looking for
    # one and throwing away the connections under this test.
    session.find_outstanding = True

    def asked():
        names = [name for name, _ in session.world_link.calls]
        session.world_link.calls.clear()
        return names

    # Shut, and the rover is left alone however long the console runs.
    session.world["available"] = True
    for _ in range(3):
        session.world_watched_at = 0.0
        session.pump()
    check("a shut popup asks the rover nothing", asked(), [])

    # Open. What goes out is the counts alone -- 7 kB against the 74 kB the
    # entity list costs, and they move whenever anything in the store does.
    session.world["open"] = True
    session.world_watched_at = 0.0
    session.pump()
    check("an open popup asks on its own, with nobody pressing anything",
          asked(), ["world_state_summary"])
    check("...and not again before the poll is due", (session.pump(), asked())[1],
          [])

    counts = {"entities": 1, "observations": 12, "inspections": 3,
              "map_session": 4}
    session.world_handle("world_state_summary",
                         {"ok": True, "summary": counts, "backend": "tensorrt"}, 0.0)
    check("counts that have moved fetch the body they describe",
          asked(), ["world_state_entities"])

    # ...and do not move the tag themselves. The body they sent for is a moment
    # away and moves it, and bumping here as well would send the browser back
    # for 74 kB twice on every change in the store.
    check("...without sending the browser for the payload twice",
          session.world_state()["gen"], "")
    session.world_handle("world_state_entities",
                         {"ok": True, "entities": [],
                          "recent": [], "summary": counts}, 0.0)
    first = session.world_state()["gen"]
    check("...and the body that arrives is what says there is something new",
          bool(first), True)

    # The same counts again, which is what a rover recording nothing looks like.
    session.world_outstanding = 0
    session.world_watched_at = 0.0
    session.pump()
    check("the counts are asked for again", asked(), ["world_state_summary"])
    session.world_handle("world_state_summary",
                         {"ok": True, "summary": counts, "backend": "tensorrt"}, 0.0)
    check("...and counts that have not moved fetch nothing", asked(), [])
    check("...and do not send the browser back for a payload it holds",
          session.world_state()["gen"], first)

    # A body that arrives identical -- the entity list can be unchanged even when
    # an inspection has been recorded -- must not move the tag either.
    body = {"ok": True, "entities": [], "recent": [],
            "summary": counts}
    session.world_handle("world_state_entities", body, 0.0)
    held = session.world_state()["gen"]
    session.world_handle("world_state_entities", dict(body), 0.0)
    check("an unchanged body leaves the tag alone",
          session.world_state()["gen"], held)
    session.world_handle("world_state_entities", dict(body, recent=[{"id": 9}]), 0.0)
    check("...and a changed one moves it",
          session.world_state()["gen"] != held, True)

    # Opening the popup asks for the body itself, so the counts that come back
    # beside it must not fetch it a second time -- which at a look a second they
    # would, because the store moves between those two calls.
    session.world_refresh()
    check("opening it asks for everything", asked(),
          ["world_state_entities", "world_state_summary"])
    session.world_handle("world_state_summary",
                         {"ok": True, "summary": dict(counts, observations=13),
                          "backend": "tensorrt"}, 0.0)
    check("...and its counts do not fetch the body a second time", asked(), [])

    # A rover that refuses is not a reason to re-send the payload every two
    # seconds for the rest of the session.
    moved = session.world_state()["gen"]
    session.world_handle("world_state_summary",
                         {"ok": False, "error": "the store could not be opened"},
                         0.0)
    check("a refusal leaves the tag where it was",
          session.world_state()["gen"], moved)
    check("...and says why, in the state that is pushed anyway",
          session.world_state()["error"], "the store could not be opened")

    # A rover that has refused is asked again, slowly, rather than never. There
    # is no refresh button to press any more, so a popup that gave up on the
    # first refusal would need closing and opening to find out that the rover
    # has since been given the component -- or that its store was only busy for
    # a moment while the daemon restarted.
    session.world["available"] = False
    session.world_outstanding = 0
    session.world_watched_at = 0.0
    session.pump()
    check("a refusal does not stop the asking for good", asked(),
          ["world_state_summary"])
    check("...but slows it right down", WORLD_RETRY_S >= 10 * WORLD_OPEN_POLL_S,
          True)
    session.world_handle("world_state_summary",
                         {"ok": True, "summary": counts, "backend": "tensorrt"},
                         0.0)
    check("...and an answer brings the panel back on its own",
          session.world_state()["available"], True)

    # Shutting it stops the asking, rather than leaving a console polling a
    # store nobody is looking at.
    asked()                       # the body that answer sent for
    session.world["open"] = False
    session.world_outstanding = 0
    session.world_watched_at = 0.0
    session.pump()
    check("shutting the popup stops the asking", asked(), [])
    check("the poll is paced, not per tick", WORLD_OPEN_POLL_S >= 1.0, True)


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

        # The rest of the observation history, which the payload deliberately
        # does not carry: it is re-sent every time the rover records, so it holds
        # the newest forty and the stream fetches what is under them as it is
        # scrolled. A page is asked for by the oldest row on the screen.
        asked = []
        session.address = "rover:8769"
        session._aside_client = type("Client", (), {
            "describe": lambda _self: "rover:8769",
            "call": lambda _self, name, arguments: (
                asked.append((name, arguments))
                or {"ok": True, "observations": [{"id": 6}], "more": False}),
        })()
        connection.request("GET", "/world_observations.json"
                                  "?before_at=1756900000.5&before_id=7")
        reply = connection.getresponse()
        page = json.loads(reply.read())
        check("a page of the older history is served", reply.status, 200)
        check("...off the rover, on the connection the pictures use",
              asked[-1][0], "world_state_observations")
        check("...starting at the row the browser named, not at a row count",
              (asked[-1][1]["before_at"], asked[-1][1]["before_id"]),
              (1756900000.5, 7))
        check("...and the looks under it are what comes back",
              [row["id"] for row in page["observations"]], [6])

        # A cursor the page could not have sent. Answered plainly rather than by
        # a traceback on the thread serving the browser.
        connection.request("GET", "/world_observations.json?before_at=soon")
        reply = connection.getresponse()
        reply.read()
        check("...while a place that is not a place is refused", reply.status, 404)
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        drive_web.Handler.session = was


def test_going_to_look_at_a_thing() -> None:
    """The "go to" button, from the press to what the row says afterwards.

    Two calls on two connections, and the join between them is the whole of what
    is checked here. Where to stand is arithmetic over the rover's map and goes
    out on the world channel; what comes back is a place on the map, and it
    reaches the wheels through the same path a click on the map takes -- so it
    stops what is running, waits for the wheels and is dropped if they never come
    free. The one thing it carries that a click does not is which way to be
    facing when it gets there, because the point of the drive is to be looking at
    something.

    None of the choosing is done here. The console never works out where to stand
    from the position in the row: that is the rover's map to read, and the row is
    as old as the last body the popup fetched.
    """
    try:
        import drive_web
        from console_model import Reply
    except ImportError as exc:
        SKIP.append(f"going to look at a thing ({type(exc).__name__})")
        return

    class Fake:
        def __init__(self):
            self.sent = []

        def submit(self, name, arguments=None):
            self.sent.append((name, arguments or {}))

    def console():
        session = drive_web.Session(None, 3.0, 480)
        session.world_link = _Recorder()
        session.moves, session.halt = Fake(), Fake()
        session.tools = ["drive_to", "drive", "stop_driving"]
        session.can_drive = True
        return session

    session = console()
    session.act({"do": "world", "what": "approach", "id": "object:7"})
    check("pressing go to asks the rover where to stand",
          session.world_link.calls, [("world_state_viewpoint", {"id": "object:7"})])
    check("...and nothing has been sent to the wheels on the strength of the row",
          session.moves.sent, [])
    check("...while the row says what is happening",
          session.world_state()["going"], "object:7")

    # The rover has read its own map and answered. That becomes a destination.
    session.world_handle("world_state_viewpoint", {
        "ok": True, "id": "object:7", "x_m": 3.2, "y_m": -1.0,
        "heading_deg": 45.0, "range_m": 0.8, "travel_m": 2.2,
        "placement": {"x_m": 3.8, "y_m": -0.4}}, 0.01)
    check("the answer is driven to", [name for name, _ in session.moves.sent],
          ["drive_to"])
    sent = session.moves.sent[0][1]
    check("...as a place on the map, with the way to face on arrival",
          sent, {"x_m": 3.2, "y_m": -1.0, "heading_deg": 45.0})
    check("...and not at the thing's own position, which is inside the thing",
          (sent["x_m"], sent["y_m"]) == (3.8, -0.4), False)
    check("...with the row still saying where the rover is going",
          session.world_state()["going"], "object:7")
    check("...and how far, in the popup's own note",
          "2.2 m away" in session.world_state()["note"], True)

    # And the move's verdict, in the popup -- which is over the notice line that
    # would otherwise be the only place it was said.
    session.handle(Reply("drive_to", sent,
                         {"ok": True, "reason": "arrived", "travelled_m": 2.3},
                         9.0))
    check("what became of the drive is said in the popup",
          session.world_state()["note"], "object:7: arrived")
    check("...and the row stops claiming the rover is on its way",
          session.world_state()["going"], "")

    # A thing the rover will not be sent to. The refusal is the rover's sentence,
    # because the three reasons -- no position, a position measured under a map
    # that has been cleared, and nowhere to see it from -- are all things only the
    # rover knows, and they are acted on differently.
    session = console()
    session.act({"do": "world", "what": "approach", "id": "object:9"})
    session.world_handle("world_state_viewpoint",
                         {"ok": False,
                          "error": "the floor within 2.5 m of it has not been "
                                   "mapped"}, 0.01)
    world = session.world_state()
    check("a refusal moves nothing", session.moves.sent, [])
    check("...and says why, in the rover's own words",
          "has not been mapped" in world["error"], True)
    check("...and the row goes back to offering the button", world["going"], "")

    # Interrupting: the popup's destination waits for the wheels exactly as a
    # click does, and a verdict belongs to the move it was asked of. The one in
    # flight answers first, and that outcome is not this drive's.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "world", "what": "approach", "id": "object:3"})
    session.world_handle("world_state_viewpoint", {
        "ok": True, "id": "object:3", "x_m": 1.0, "y_m": 2.0,
        "heading_deg": -90.0, "range_m": 0.9, "travel_m": 4.0}, 0.01)
    check("a rover already driving is stopped rather than told it is busy",
          [name for name, _ in session.halt.sent], ["stop_driving"])
    check("...and the destination waits for the wheels",
          (session.pending_target or {}).get("heading_deg"), -90.0)
    session.handle(Reply("drive_to", {"x_m": 9.0, "y_m": 9.0},
                         {"ok": True, "reason": "stopped"}, 1.0))
    check("the interrupted move's own verdict is not written under this thing",
          "stopped" in session.world_state()["note"], False)
    check("...and the row goes on saying where the rover is headed",
          session.world_state()["going"], "object:3")
    check("...because the waiting destination goes once the wheels are free",
          [name for name, _ in session.moves.sent], ["drive_to"])
    session.handle(Reply("drive_to", session.moves.sent[0][1],
                         {"ok": True, "reason": "arrived", "travelled_m": 4.1},
                         30.0))
    check("...and it is that move whose verdict the row shows",
          session.world_state()["note"], "object:3: arrived")

    # A destination that never gets the wheels is dropped, and the row has to stop
    # saying the rover is on its way to something it gave up on.
    session = console()
    session.busy_since, session.busy_name = 100.0, "drive_to"
    session.act({"do": "world", "what": "approach", "id": "object:4"})
    session.world_handle("world_state_viewpoint", {
        "ok": True, "id": "object:4", "x_m": 1.0, "y_m": 2.0,
        "heading_deg": 0.0, "range_m": 0.9, "travel_m": 4.0}, 0.01)
    session.forget_target("the move it interrupted did not let go of the wheels")
    check("a dropped destination clears the row", session.world_state()["going"], "")
    check("...and says so rather than going quiet",
          "was dropped" in session.world_state()["error"], True)

    # And a rover with no driving tools at all is refused here rather than by the
    # rover, which would answer a minute later with a sentence about a tool.
    session = console()
    session.tools, session.can_drive = ["look"], False
    session.act({"do": "world", "what": "approach", "id": "object:7"})
    check("a rover that cannot drive is not asked where to stand",
          session.world_link.calls, [])
    check("...and the popup says so", "no driving tools" in
          session.world_state()["error"], True)


def _a_room(pose, *placements):
    """A world payload with things placed round a rover standing at `pose`.

    Each placement is (x, y) in map metres, with one look taken from a metre
    short of it -- which is what the popup's map has to hold: the ring round the
    position, the arrowhead where the rover stood, and the line between them.
    """
    entities = []
    for index, (x, y) in enumerate(placements):
        entities.append({
            "id": f"object:{index}",
            "placement": {"x_m": x, "y_m": y, "error_major_m": 0.3,
                          "error_minor_m": 0.1, "extent_m": 0.2},
            "placement_map_session": 1,
            "rays": [{"id": index, "x_m": pose[0], "y_m": pose[1],
                      "bearing_deg": 0.0, "span_deg": 20.0, "length_m": 2.5,
                      "heading_deg": 0.0,
                      "relation": {"range_m": 1.0, "agrees": True}}],
        })
    return {"entities": entities, "summary": {"entities": len(entities),
                                              "map_session": 1}}


def test_the_popup_gets_a_map_wide_enough_for_what_it_draws() -> None:
    """The fault this fixes: the popup drew its bearings over the driving map.

    The card behind the popup is drawn a few metres around wherever the rover is
    standing, because that is what driving needs. The popup draws bearings taken
    from all over a flat. On a real store of 203 things read off this rover on
    2026-09-05, half of them were beyond 4.9 m and 29 were beyond 8 m, while the
    map underneath reached three -- so a thing perfectly well placed sat on black
    with "outside the drawn map" written under it. What is checked here is that
    the console now asks the rover for a second picture sized to hold what the
    panel is about to draw on it.
    """
    try:
        import drive_web
        from console_model import MAP_EXTENTS_M
        from drive_world import WORLD_MAP_GAP_S, WORLD_MAP_PX
    except ImportError as exc:
        SKIP.append(f"the popup's own map ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.picture = _Recorder()
    session.world["open"] = True
    # The driving map, as the console has it: six metres across, round a rover
    # standing at the origin.
    session.map_view = {"half_extent_m": 3.0, "scale": 4, "rover_up": False,
                        "pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}}

    # A thing seven metres off, which is an ordinary distance in a flat and four
    # metres outside the picture the popup used to draw it on.
    session.world_payload = _a_room((0.0, 0.0), (7.0, 0.5))
    wanted = session.world_map_extent()
    check("the driving map does not reach the thing the popup is drawing",
          session.map_view["half_extent_m"] >= 7.0, False)
    check("...so the popup asks for a map that does", wanted >= 7.5, True)
    check("...at a rung of the console's own zoom ladder rather than a raw number",
          wanted in MAP_EXTENTS_M, True)

    # Choosing one thing narrows it. All of the store at once is the overview and
    # a room-wide picture behind one thing's bearings throws away exactly the
    # resolution that makes a fork between a bearing and a position readable.
    session.world_payload = _a_room((0.0, 0.0), (7.0, 0.5), (1.2, -0.4))
    check("nothing chosen covers the whole store",
          session.world_map_extent() >= 7.5, True)
    session.world_selected = "object:1"
    session.world_payload["selected"] = session.world_payload["entities"][1]
    session.world_payload["selected_rays"] = \
        session.world_payload["entities"][1]["rays"]
    check("...and one thing chosen closes in on that thing",
          session.world_map_extent() <= 3.0, True)

    # A store with nothing in it yet asks for nothing: the popup goes on drawing
    # over the driving map, which is what it did before any of this existed.
    session.world_selected = ""
    session.world_payload = {"entities": []}
    check("an empty store buys no picture", session.world_map_extent(), None)
    # And so does a console that has never had a map, because there is then
    # nowhere to measure the extent from.
    session.world_payload = _a_room((0.0, 0.0), (7.0, 0.5))
    session.map_view = None
    check("...and so does a console with no map to measure from",
          session.world_map_extent(), None)
    session.map_view = {"half_extent_m": 3.0, "scale": 4, "rover_up": False,
                        "pose": {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}}

    # Only while somebody is looking. This is the most expensive thing the
    # console asks the rover for, and a shut popup draws nothing.
    session.world["open"] = False
    check("a shut popup asks for no map", session.world_map_due(1000.0), None)
    session.world["open"] = True
    half = session.world_map_due(1000.0)
    check("an open one does", half >= 7.5, True)

    session.world_map_refresh(half)
    check("...on the connection the driving map already uses",
          session.picture.calls[-1][0], "map_png")
    check("...tagged, so the two pictures can be told apart",
          session.picture.calls[-1][2], "world")
    check("...at the extent the popup needs",
          session.picture.calls[-1][1]["half_extent_m"], half)
    check("...and on a picture big enough to close in on",
          session.picture.calls[-1][1]["pixels"], WORLD_MAP_PX)
    check("nothing else is asked for while that one is in flight",
          session.world_map_due(1001.0), None)

    # The picture arrives. The page is told the geometry it was drawn at, not the
    # geometry that was asked for: whole cells at whole pixels cannot reach every
    # size, and a page that laid bearings over its own request would put them in
    # the wrong place.
    import base64

    png = b"\x89PNG\r\n\x1a\n" + b"\x00" * 8 + (962).to_bytes(4, "big")
    session.world_map_arrived({
        "ok": True, "half_extent_m": 8.0, "scale": 2, "pixels": 962,
        "pose": {"x_m": 0.2, "y_m": -0.1, "heading_deg": 12.0},
        "png_base64": base64.b64encode(png).decode()})
    state = session.world_state()
    check("the popup's map is published for the page to fetch",
          bool(state["map"]["gen"]), True)
    check("...with the size it really came out as", state["map"]["width"], 962)
    check("...and the pose it was really drawn from, not the pose now",
          state["map"]["view"]["pose"]["x_m"], 0.2)
    check("...and never turned under the reader",
          state["map"]["view"]["rover_up"], False)
    check("the bytes are held for the URL that serves them",
          session.world_map_png, png)

    # It is drawn again as it ages, and at once when the room it has to cover
    # stops matching the room the popup is drawing.
    session.world_map_done_at = 1000.0
    check("a picture that is still fresh is left alone",
          session.world_map_due(1000.0 + WORLD_MAP_GAP_S / 2), None)
    # And is left alone without walking the store to decide it. The pump runs ten
    # times a second beside SLAM, and one pass over the 203 things this rover was
    # holding is a couple of milliseconds on the Orin -- so an answer that cannot
    # have changed must not be worked out again.
    session.world_marks = lambda: (_ for _ in ()).throw(
        AssertionError("the extent was worked out again for nothing"))
    check("...and without counting the store again to say so",
          session.world_map_due(1000.0 + WORLD_MAP_GAP_S / 2), None)
    del session.world_marks
    check("...and one that has gone stale is drawn again",
          session.world_map_due(1000.0 + WORLD_MAP_GAP_S + 1) is not None, True)
    session.world_payload = _a_room((0.0, 0.0), (7.0, 0.5), (1.2, -0.4))
    session.world_selected = "object:1"
    session.world_payload["selected"] = session.world_payload["entities"][1]
    session.world_payload["selected_rays"] = \
        session.world_payload["entities"][1]["rays"]
    check("choosing a thing redraws it without waiting for the gap",
          session.world_map_due(1000.0) is not None, True)

    # A refusal says nothing on the page: the panel falls back to the driving map,
    # and a red line about a render would be the popup complaining about its own
    # backdrop rather than about the world it is there to show.
    before = session.world_state()["map"]["gen"]
    session.world_map_arrived({"ok": False, "error": "there is no map yet"})
    check("a refused picture keeps the last one",
          session.world_state()["map"]["gen"], before)
    check("...and says nothing about it", session.world_state()["error"], "")


def test_the_popup_map_is_served_at_its_own_url() -> None:
    """`/world_map.png`, over a real socket, beside the driving map.

    Two pictures of one room, and they have to stay two: a browser that fetched
    the driving map here would be back to laying bearings over six metres of a
    flat that is twenty across.
    """
    try:
        import http.client
        import threading

        import drive_web
    except ImportError as exc:
        SKIP.append(f"the popup's map URL ({type(exc).__name__})")
        return

    session = drive_web.Session(None, 3.0, 480)
    session.map_png = b"\x89PNG the one to drive by"
    session.world_map_png = b"\x89PNG the one the popup draws on"

    was, drive_web.Handler.session = drive_web.Handler.session, session
    server = drive_web.Console(("127.0.0.1", 0), drive_web.Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    connection = http.client.HTTPConnection(
        "127.0.0.1", server.server_address[1], timeout=5)
    try:
        connection.request("GET", "/world_map.png?gen=abc-3")
        reply = connection.getresponse()
        body = reply.read()
        check("the popup's map is served", reply.status, 200)
        check("...as a picture", reply.getheader("Content-Type"), "image/png")
        check("...and is the popup's own, not the one to drive by",
              body, session.world_map_png)
        check("...immutable, because the URL carries the generation",
              "immutable" in (reply.getheader("Cache-Control") or ""), True)

        # A console that has not been asked for one yet answers plainly. The page
        # falls back to the driving map, so this must not be an error the browser
        # has to survive as a broken image.
        session.world_map_png = b""
        connection.request("GET", "/world_map.png?gen=abc-4")
        reply = connection.getresponse()
        reply.read()
        check("...and there is a plain answer before the first one is drawn",
              reply.status, 404)
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        drive_web.Handler.session = was


TESTS = (
    test_the_world_state_popup,
    test_going_to_look_at_a_thing,
    test_finding_a_thing_from_the_console,
    test_the_best_thing_a_search_found_is_chosen_without_a_click,
    test_a_looking_loop_that_has_failed_still_says_so,
    test_an_open_popup_keeps_itself_current,
    test_the_world_urls,
    test_the_popup_gets_a_map_wide_enough_for_what_it_draws,
    test_the_popup_map_is_served_at_its_own_url,
)
