"""Perception: what counts as a region, and what the engines answer.

A frame too dark to see is not an empty room, and a crop with no picture in it is
not a region -- both were real faults. The engines are also checked for falling
back when a board has none, and for being exactly the ones the installer builds.
"""
from __future__ import annotations

import os
import tempfile

from test_harness import SKIP, check
from test_fakes import HERE


# --- the two perception backends --------------------------------------------
#
# Nothing here loads a model. What is worth checking offline is the part that
# decides *which* backend runs and whether the two are really interchangeable,
# because the failure mode is silent: a rover that quietly drops to the CPU
# keeps answering, and its vectors stop comparing with the ones already stored.


def test_a_board_with_no_engines_falls_back_and_says_why() -> None:
    """The ordinary state of a freshly deployed rover, and it must not be fatal."""
    from world_state import engines
    from world_state.perceive import Perception

    with tempfile.TemporaryDirectory() as empty:
        ready, why = engines.available(empty)
        check("an empty directory offers no GPU path", ready, False)
        check("...and the reason names the installer",
              "install_perception.sh" in why or "TensorRT" in why, True)
        backend, missing = Perception(empty).chosen()
        check("...so the CPU backend is the one chosen", backend, "onnxruntime")
        check("...with the reason kept rather than swallowed", bool(missing), True)


def test_the_cpu_backend_can_open_after_construction() -> None:
    """The lazy open must retain numpy instead of referring to a dead local."""
    from world_state.perceive import _CpuModels

    class Numpy:
        int64 = "int64"
        float32 = "float32"

        @staticmethod
        def zeros(shape, dtype):
            return shape, dtype

    model = object.__new__(_CpuModels)
    model._numpy = Numpy
    model._regions = object()
    model.open()
    check("the CPU backend retains numpy until lazy open", model._pad_ids,
          ((1, 64), "int64"))


def test_a_query_can_be_embedded_before_anything_has_been_looked_at() -> None:
    """The first thing a freshly booted sidecar is asked may be a search.

    Reproduces a fault seen on the rover: the query path reached for the numeric
    library the loader installs on the object *before* calling the loader, so the
    first search after a reboot died with an AttributeError instead of either
    answering or saying the models were missing.
    """
    from world_state.perceive import Perception, Unavailable

    with tempfile.TemporaryDirectory() as empty:
        perception = Perception(empty)
        check("a search before the first look has nothing loaded",
              perception._loaded, False)
        try:
            perception.embed_text(["a spray bottle"])
            raised = None
        except Exception as failure:                      # noqa: BLE001
            raised = failure
        check("...and asking it for a vector fails for a nameable reason",
              isinstance(raised, Unavailable), True)
        check("...not because the loader had not run yet",
              isinstance(raised, AttributeError), False)


def test_both_backends_answer_the_same_four_questions() -> None:
    """They are swapped at run time, so a method on one and not the other is a
    crash on whichever board has the wrong one. This is the whole of what
    `Perception` asks a backend for; giving the text tower back is not on the
    list, because only the GPU backend holds one and only its own `open` ever
    asks."""
    from world_state.perceive import _CpuModels, _GpuModels

    wanted = {"regions", "appearance", "image_vectors", "text_vectors", "open"}
    for backend in (_CpuModels, _GpuModels):
        have = {name for name in dir(backend) if not name.startswith("_")}
        check(f"{backend.name} answers all five",
              wanted - have, set())
    check("the two name themselves differently",
          _CpuModels.name != _GpuModels.name, True)


def test_the_tokenizer_is_built_once_and_not_once_a_search() -> None:
    """Reading it back from its 34 MB of JSON was measured on the rover at 2.3 s
    of a core, and it was being paid on every search for a thing that never
    changes -- more than half of what a four-second search cost."""
    from world_state.perceive import Perception

    with tempfile.TemporaryDirectory() as empty:
        perception = Perception(empty)
        already = object()
        perception._tokenizer = already
        # The directory is empty, so a second read would raise rather than
        # quietly cost the 2.3 s again.
        check("a tokenizer already built is the one a search uses",
              perception._words() is already, True)
        check("...and the one after it", perception._words() is already, True)


def test_the_text_tower_is_kept_until_a_look_needs_the_room() -> None:
    """What a search used to spend nearly all its time on, and the safety net
    that lets it stop.

    The tower is 1.1 GB and was opened and given back for every single search,
    which was 2.8 s of a four-second answer. It is kept now. What makes that
    safe is not that it fits -- measured, it does, with the language model that
    used to crowd it gone from this rover, at 1.8 GB held for good out of the
    3.6 that were free -- but that a look which cannot find room puts it down
    and tries again, so the thing given up is the search nobody is waiting for
    rather than the rover's eyes.
    """
    from world_state import engines as engines_module
    from world_state.perceive import _GpuModels

    opened, closed, crowded = [], [], []

    class Fake:
        def __init__(self, path, vendor=None):
            self.name = os.path.basename(path)
            if crowded and self.name != engines_module.SIGLIP_TEXT:
                raise engines_module.NoEngines("no room for an execution context")
            opened.append(self.name)

        def run(self, feed):
            return {"pooler_output": feed}

        def close(self):
            closed.append(self.name)
            crowded.clear()

    real = engines_module.Engine
    engines_module.Engine = Fake
    try:
        models = _GpuModels(os.path.join(HERE, "vendor"))
        models.text_vectors("ids")
        models.text_vectors("ids")
        check("two searches open the text tower once between them",
              opened.count(engines_module.SIGLIP_TEXT), 1)
        check("...and neither gives it back", closed, [])

        crowded.append("no room")
        models.open()
        check("a look that cannot find room gives the text tower back",
              closed, [engines_module.SIGLIP_TEXT])
        check("...and then opens its own three",
              [opened.count(name) for name in (engines_module.REGIONS,
                                               engines_module.DINO,
                                               engines_module.SIGLIP_VISION)],
              [1, 1, 1])
        check("...leaving the next search to open the tower again",
              models._text_engine, None)
    finally:
        engines_module.Engine = real


def test_the_installer_builds_exactly_the_engines_the_runtime_opens() -> None:
    """A renamed engine is a rover that silently runs on the CPU for ever."""
    from world_state import engines

    script = os.path.join(HERE, "install_perception.sh")
    with open(script, encoding="utf-8") as handle:
        built = {line.split()[1] for line in handle
                 if line.strip().startswith("build ") and ".plan" in line}
    check("every engine the runtime wants is built by the installer",
          set(engines.REQUIRED) - built, set())
    check("...and the installer builds nothing the runtime ignores",
          built - set(engines.REQUIRED), set())


def test_a_region_that_is_an_edge_is_not_a_thing() -> None:
    """The three-line filter, on the shapes it exists to reject."""
    from world_state.perceive import _worth_keeping

    check("a chair-sized box is kept", _worth_keeping([0.3, 0.3, 0.5, 0.6]), True)
    check("half the frame is the floor", _worth_keeping([0.0, 0.0, 0.9, 0.9]), False)
    check("a speck is a highlight", _worth_keeping([0.5, 0.5, 0.52, 0.52]), False)
    check("a skirting board is an edge",
          _worth_keeping([0.0, 0.5, 0.95, 0.56]), False)
    check("a box with no width is nothing",
          _worth_keeping([0.5, 0.5, 0.5, 0.7]), False)


def test_a_frame_too_dark_to_see_is_not_an_empty_room() -> None:
    """**The other end of the white-out, and the same trap.**

    On the driven run of 2026-09-02 two of seven frames came back 95% and 97%
    black: the rover drove out of a lit hallway into an unlit room and inspected
    before the camera's automatic exposure had caught up. Each yielded one junk
    region, and the diagnostics said "1 of 6 regions kept" -- which is also what
    a working rover says about a nearly bare room. Those two readings must not
    look alike, so a frame with no picture in it is refused whole and says which
    it was.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"refusing a frame too dark to see ({error})")
        return

    from world_state.perceive import _too_dark

    unlit = numpy.full((60, 80, 3), 6.0, dtype="float32")
    check("an unlit room is not a picture", _too_dark(numpy, unlit), True)

    # The frame that must keep working: a dim hallway with a lit end, which is
    # 29% black on the rover and kept eleven regions.
    hallway = numpy.full((60, 80, 3), 55.0, dtype="float32")
    hallway[:, :24] = 8.0
    check("...but a dim room with something lit in it is",
          _too_dark(numpy, hallway), False)

    # And the darkest frame the rover has kept anything from: 57% black, three
    # regions. The rule has to sit clear of it, and the gap is wide -- the next
    # frame up is 87% black and kept one.
    dim = numpy.full((60, 80, 3), 90.0, dtype="float32")
    dim[:, :46] = 7.0                    # 57% of it black
    check("...and so is the darkest frame this rover has used",
          _too_dark(numpy, dim), False)


def test_a_crop_with_no_picture_in_it_is_not_a_region() -> None:
    """**Reproduces the entity the rover built out of blown-out windows.**

    The filter above works on the shape of a box and nothing else, so a window
    the camera has burnt to white and a bare patch of wall both sail through it:
    right size, right aspect, nothing inside. On the drive of 2026-09-02, 58 of
    the 338 regions the rover stored -- 17% -- were one or the other, and they do
    real harm rather than merely wasting a slot, because two pictures of nothing
    look like each other. `object:14` was built almost entirely out of them and
    wandered four metres across the map.

    Two ways to have no picture, so two numbers: no contrast at all, and mostly
    burnt out. A window frame across a white sky has plenty of contrast and still
    says nothing about what is behind it, which is why the second test exists.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"rejecting a crop with no picture in it ({error})")
        return

    from world_state.perceive import _blank

    wall = numpy.full((40, 40, 3), 200.0, dtype="float32")
    check("a bare patch of wall is not a region", _blank(numpy, wall), True)

    window = numpy.zeros((40, 40, 3), dtype="float32")
    window[:, :34] = 255.0                      # burnt-out glass, frame at one edge
    check("...nor is a window the camera has blown out",
          _blank(numpy, window), True)

    chair = numpy.zeros((40, 40, 3), dtype="float32")
    chair[:20] = 40.0
    chair[20:] = 190.0                          # dark against a light floor
    check("but something with a picture in it is",
          _blank(numpy, chair), False)

    # A pale lampshade against a wall is the case this must not eat: bright, and
    # burnt out over part of itself, but with a shape in it. The line is drawn by
    # contrast and by how much of the crop is at full white, not by brightness.
    lamp = numpy.full((40, 40, 3), 210.0, dtype="float32")
    lamp[8:32, 8:32] = 255.0                    # half the crop, blown out
    lamp[32:] = 120.0                           # the table it stands on
    check("...and neither is a pale thing with a shape in it",
          _blank(numpy, lamp), False)


def test_a_cushion_inside_a_sofa_is_not_a_second_thing() -> None:
    """Reproduces what made the rover record one sofa several times over.

    A region finder proposes parts as readily as wholes, so a sofa comes back as
    a sofa *and* as its arm, its back and each of its cushions -- FastSAM did it
    to 57% of the regions it kept, and YOLOE, which replaced it, still does it to
    one in 237. Ordinary suppression
    cannot see that: it divides the overlap by the union of the pair, and a
    cushion inside a sofa scores about 0.15 that way -- below any threshold
    worth setting -- so both survived, both got a bearing, and both became
    entities. On ten of the rover's own frames 57% of everything it embedded
    was a piece of something else it embedded from the same picture.

    Dividing by the smaller box instead is the whole fix, and these are the two
    cases that have to come out differently: a part inside a whole, and two
    genuinely separate things that merely touch.
    """
    try:
        import numpy
    except ImportError as error:
        SKIP.append(f"suppressing a part inside a whole ({error})")
        return

    from world_state.perceive import YOLOE_OVERLAP, _suppress

    sofa = [0.10, 0.30, 0.80, 0.75]
    cushion = [0.30, 0.45, 0.55, 0.70]      # wholly inside it
    lamp = [0.78, 0.10, 0.95, 0.50]         # beside it, overlapping a corner
    boxes = numpy.array([sofa, cushion, lamp])
    scores = numpy.array([0.9, 0.8, 0.7])
    kept = sorted(int(index) for index in
                  _suppress(numpy, boxes, scores, YOLOE_OVERLAP))
    check("the sofa and the lamp are two things", kept, [0, 2])

    # And the other way round, because suppression keeps the higher score and
    # the part is often the more confident box. The whole must win on the merit
    # of containing the other, not on having scored better.
    scores = numpy.array([0.6, 0.95, 0.7])
    kept = sorted(int(index) for index in
                  _suppress(numpy, boxes, scores, YOLOE_OVERLAP))
    check("...whichever of the pair scored higher", len(kept), 2)

    # The case the old rule got right, which the new one must not break: two
    # rival guesses at the same object, near enough the same size.
    boxes = numpy.array([[0.1, 0.1, 0.5, 0.5], [0.12, 0.12, 0.52, 0.52]])
    kept = _suppress(numpy, boxes, numpy.array([0.9, 0.8]), YOLOE_OVERLAP)
    check("two guesses at one object are still one", len(kept), 1)

    # Two things that touch are two things. A quarter of the smaller box lies
    # inside the larger here, which is well under the threshold.
    boxes = numpy.array([[0.1, 0.1, 0.5, 0.5], [0.4, 0.4, 0.8, 0.8]])
    kept = _suppress(numpy, boxes, numpy.array([0.9, 0.8]), YOLOE_OVERLAP)
    check("...but two that only touch are two", len(kept), 2)


TESTS = (
    test_a_board_with_no_engines_falls_back_and_says_why,
    test_the_cpu_backend_can_open_after_construction,
    test_a_query_can_be_embedded_before_anything_has_been_looked_at,
    test_both_backends_answer_the_same_four_questions,
    test_the_tokenizer_is_built_once_and_not_once_a_search,
    test_the_text_tower_is_kept_until_a_look_needs_the_room,
    test_the_installer_builds_exactly_the_engines_the_runtime_opens,
    test_a_region_that_is_an_edge_is_not_a_thing,
    test_a_frame_too_dark_to_see_is_not_an_empty_room,
    test_a_crop_with_no_picture_in_it_is_not_a_region,
    test_a_cushion_inside_a_sofa_is_not_a_second_thing,
)
