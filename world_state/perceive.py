"""Regions and embeddings for one frame, from three models, none of which is
allowed to name anything.

This is the half of the world state that replaced asking a language model which
lasting thing it was looking at. Three questions, three mechanisms, and the point
is that they are separate:

    what regions are in this frame?    YOLOE, whose vocabulary is cut off at
                                       the graph so only "something is here"
                                       comes out of it
    is this the same instance?         DINOv2 similarity, gated by geometry
    which lasting thing is it?         **not answered here.** That is a
                                       triangulated map position, and it needs a
                                       second look from somewhere else.

Nothing in this file decides an identity, and nothing in it may. What it produces
is a measurement of one picture: boxes and two vectors per box.

**Nothing here names anything, and that is deliberate.** A region used to be
called the nearest phrase in a fixed word list to its SigLIP2 vector, and the
name was worth nothing: the scores sat between 0.08 and 0.12 whatever was in the
picture, so the ranking was all there was, and on the rover's own frames it put
"a computer monitor" on a sofa. What the vector is genuinely good for is the
other question -- "find me the spray bottle" -- where the phrase a person types
is embedded by the same tower and compared against the stored vectors, with a
floor that was measured against forty real queries. So the vector is stored and
the search asks the question; nothing in between guesses a word.

The models are unpacked into `vendor/` by `install_perception.sh`, which is the
only installer this component has: the language model that used to sit beside
them, and its own installer, were removed from the rover on 2026-09-02.

**There are two backends and they are not equivalent.** Where the board has
TensorRT engines built for it the work runs on the GPU, and where it has not it
runs on the CPU under onnxruntime. The GPU is not merely faster: measured
against a full-precision reference on the rover's own frame, the engines agree
with it to 1.000 while the int8 graphs the CPU path uses agree to 0.86. So every
look says which backend produced it, and a vector from one must never be
compared with a vector from the other.
"""
from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(HERE, "vendor")

#: The three graphs and the tokenizer, as `install_perception.sh` names them.
#:
#: **The region finder is not a stock export and cannot be downloaded.** YOLOE
#: ends in 4,585 class scores per anchor, and the rover wants one number from
#: that block -- whether anything is there at all. `export_regions.py` on a
#: workstation takes the maximum inside the graph, which leaves an output shaped
#: exactly like the FastSAM export this replaced: four box numbers, one score,
#: thirty-two mask coefficients. Everything below the model is unchanged by the
#: swap, and the vocabulary never reaches this machine.
YOLOE = "yoloe-11s-seg-objectness.onnx"
DINO = "dinov2-small-int8.onnx"
SIGLIP = "siglip2-base-patch32-256-int8.onnx"
TOKENIZER = "siglip2-tokenizer.json"

#: What the region finder is shown. **Fixed at the export rather than chosen
#: here**: FastSAM's graph took any size and this one is built for 512 alone, so
#: changing this number without re-exporting is an error the session raises
#: rather than a slower or coarser look. 512 is where it was measured, and where
#: the model it replaced was measured before it.
YOLOE_SIZE = 512
#: Below this the box is a texture rather than a thing. **Lower than the 0.4 the
#: previous region finder ran at, and that is the swap's one tuning.** Measured
#: over the 33 lit frames the rover had stored: 146 regions at 0.4, 188 at 0.25,
#: 237 at 0.15 and 270 at 0.08, against 258 from FastSAM at its own 0.4. At 0.15
#: the yield matches what the rover used to get and the extra boxes are still
#: things -- the chairs round the dining table, a flipchart, a second doorway --
#: while below it they stop matching anything FastSAM proposed and more crops
#: come back blank.
YOLOE_CONF = 0.15
#: How much two boxes may overlap before the lower-scoring one is dropped, as a
#: fraction of the **smaller** box -- see `_suppress`, where the choice of
#: denominator is argued and measured. For two boxes of the same size this is
#: about as strict as the 0.7 against the union it replaces; the difference is
#: entirely in what it does to a box sitting inside a bigger one. Anything from
#: 0.9 down to 0.5 removes the nesting; lower than 0.8 starts costing regions
#: that were not nested (95 at 0.8, 81 at 0.5, over the same ten frames), so
#: this sits at the top of the range that works rather than in the middle of it.
#:
#: Measured on FastSAM's boxes and kept unchanged through the swap, where it has
#: much less to do: of the 237 regions YOLOE keeps across the rover's 38 stored
#: frames, one is nested. It stays because the rule is about what a part is, not
#: about which model proposed it.
YOLOE_OVERLAP = 0.8

#: What DINOv2 and SigLIP2 are shown, from their own preprocessor configs.
DINO_SIZE = 224
SIGLIP_SIZE = 256
#: SigLIP's text side is padded to a fixed width, and the tokenizer file already
#: carries that. Stated here only so the number is somewhere readable.
SIGLIP_TOKENS = 64

#: The three-line filter from the plan, which takes a frame's regions from about
#: thirty to about eighteen: drop anything bigger than a third of the frame (the
#: floor, a wall), smaller than this (a highlight on a tile), or more than six
#: times longer than it is wide (an edge, a shadow, a skirting board).
MAX_AREA = 1 / 3
MIN_AREA = 0.004
MAX_ASPECT = 6.0

#: How many regions are embedded, largest first. **A cost ceiling rather than a
#: belief about rooms.** Every crop is a forward pass through two networks, so
#: this is the number that decides whether a look is under a second or over three,
#: and the ones it drops are the smallest -- which are also the ones a bearing is
#: least able to place. Raise it when the encoders get cheaper, not when a room
#: looks busy.
MAX_REGIONS = 12

#: A crop is taken slightly wider than the box. A tight crop of a chair is a
#: picture of upholstery; a little of what is behind it is what makes it a chair.
CROP_PAD = 0.02

#: **A crop with no picture in it is not a region, and it took a rover to find
#: that out.** The box filter above works on shape alone, so a window the camera
#: has blown to white and a bare patch of wall both pass it: they are the right
#: size and the right aspect, and there is nothing in them at all. Measured on
#: the 338 regions of the drive of 2026-09-02, 58 of them -- 17% -- were one or
#: the other, and every one that reached the resolver did damage, because two
#: pictures of nothing resemble each other. One entity was built almost entirely
#: out of blown-out windows and wandered four metres across the map.
#:
#: Two numbers, because there are two ways to have no picture. Contrast, as the
#: standard deviation of the crop's brightness, is below 12 for a flat wall and
#: above 27 for three quarters of the regions the rover keeps. And the fraction
#: of the crop that is at full white, which catches the other case: a window
#: frame across a white sky has plenty of contrast and still says nothing about
#: what is behind it.
MIN_CONTRAST = 12.0
MAX_BLOWN = 0.6

#: When the whole frame is too dark to be a picture of anything, and the two
#: numbers that say so: a pixel below `DARK_AT` out of 255 is black rather than
#: dim, and a frame with more than `DARK_FRACTION` of itself like that has
#: nothing in it.
#:
#: **The same lesson as the white-out of 2026-09-02, from the other end.** A
#: blank frame reads exactly like an empty room: "1 of 6 regions kept" and
#: "nothing to place" are both things the rover says when it is working
#: perfectly, so a whole drive was once recorded off white frames before anybody
#: looked at one. On the driven run of 2026-09-02 two of seven frames came back
#: 95% and 97% black, because the rover drove out of a lit hallway into an unlit
#: room and inspected before the camera's automatic exposure had caught up. They
#: yielded one region and two, both junk.
#:
#: Measured over all 52 frames the rover has stored: five are more than 80%
#: black and hold seven regions between them, a median of one each, while the
#: other 47 average nine. The gap in the middle is wide -- the next frame down is
#: 57% black and keeps three regions -- so this refuses nothing worth having.
#:
#: There is deliberately no matching test at the bright end. Nothing in the
#: stored frames is washed out enough to be useless (the worst is 43% at full
#: white and it kept ten regions), so there is no measurement to set one from,
#: and the per-crop `MAX_BLOWN` above already refuses the regions that are.
DARK_AT = 20.0
DARK_FRACTION = 0.8


class Unavailable(RuntimeError):
    """The models are not installed, or the runtime cannot be imported.

    Its own type because the sidecar answers it with a sentence rather than a
    traceback: a rover that has been deployed to but not yet installed on is an
    ordinary state, and it should read as "run install_perception.sh" rather than
    as a crash.
    """


def _vendored():
    """Import numpy and cv2, from vendor/ if that is where they are.

    An installed copy wins, so that a desk with these on the path is unaffected;
    the rover has none of them installed and all of them unpacked, because its
    Python is externally managed and sudo wants a password no deploy script has.

    Pictures only. Whichever backend runs the models imports its own runtime,
    because a board with built engines has no need of onnxruntime and should not
    be held back by its absence.
    """
    for path in (VENDOR, os.path.join(os.path.dirname(HERE), "vendor")):
        if os.path.isdir(path) and path not in sys.path:
            sys.path.append(path)
    try:
        import cv2
        import numpy
    except ImportError as error:
        raise Unavailable(
            f"the perception models are not installed on this host: {error}. "
            f"Run ~/ugv/world_state/install_perception.sh") from error
    return numpy, cv2


def _onnxruntime():
    """The CPU backend's runtime, imported only when that backend is chosen."""
    _vendored()
    try:
        import onnxruntime
    except ImportError as error:
        raise Unavailable(
            f"onnxruntime is not installed on this host: {error}. "
            f"Run ~/ugv/world_state/install_perception.sh") from error
    return onnxruntime


class _CpuModels:
    """The three graphs under onnxruntime, on the CPU, which is where this began.

    Kept because an engine is not a model file: it is compiled for one GPU and
    one TensorRT version, so a fresh install, a JetPack upgrade or a driver that
    has stopped answering all leave a rover that must still be able to see.

    It is slower and it is also less accurate, and the second matters more.
    Measured on the rover's own frame against a full-precision reference, these
    int8 graphs agree to 0.96 on DINOv2 but only 0.86 on SigLIP2, which is
    enough to move a search's ranking. Dynamic quantisation is what costs that:
    it recomputes its scales from each activation rather than from a calibration
    set.
    """

    name = "onnxruntime"

    def __init__(self, directory: str, threads: int) -> None:
        numpy = _vendored()[0]
        ort = _onnxruntime()
        options = ort.SessionOptions()
        if threads:
            options.intra_op_num_threads = threads
        options.graph_optimization_level = \
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # **The single most expensive line in this file, and it is a one-word
        # setting.** Three sessions run one after another on every look, and by
        # default an onnxruntime thread pool keeps spinning for a while after
        # its own work finishes -- so the region finder's threads are still
        # burning cores while DINOv2 runs, and DINOv2's while SigLIP2 does. Measured on the
        # rover's own frames: a look costs 2.64 s with spinning left on and
        # 0.80 s with it off, and the models themselves are identical. Each one
        # alone is as fast either way, which is exactly why this is easy to miss.
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")

        def session(name):
            path = os.path.join(directory, name)
            if not os.path.isfile(path):
                raise Unavailable(f"no {name} at {directory}; run "
                                  f"install_perception.sh")
            return ort.InferenceSession(path, options,
                                        providers=["CPUExecutionProvider"])

        self._session = session
        # SigLIP2 first and the other two on `open`, so that both backends load
        # in the same order and the GPU's memory peak stays where it can be
        # reasoned about. This one export carries both towers, so it is also what
        # a text search runs through.
        self._siglip = session(SIGLIP)
        self._regions = self._dino = None

    def open(self) -> None:
        """The two graphs a look needs beyond the one already open."""
        if self._regions is None:
            self._regions = self._session(YOLOE)
            self._dino = self._session(DINO)
        # This SigLIP2 export carries both towers in one graph, so the image
        # path has to be handed an `input_ids` whether it wants one or not. A
        # row of pad tokens is the cheapest thing the graph accepts and its text
        # output is thrown away; the blank picture is the same trick the other
        # way round.
        self._pad_ids = numpy.zeros((1, SIGLIP_TOKENS), dtype=numpy.int64)
        self._blank = numpy.zeros((1, 3, SIGLIP_SIZE, SIGLIP_SIZE),
                                  dtype=numpy.float32)

    def regions(self, blob):
        return self._regions.run(None, {"images": blob})[0]

    def appearance(self, batch):
        return self._dino.run(None, {"pixel_values": batch})[0][:, 0]

    def image_vectors(self, batch):
        return self._siglip.run(["image_embeds"],
                                {"pixel_values": batch,
                                 "input_ids": self._pad_ids})[0]

    def text_vectors(self, ids):
        return self._siglip.run(["text_embeds"],
                                {"pixel_values": self._blank,
                                 "input_ids": ids})[0]


class _GpuModels:
    """The same models as TensorRT engines, on the Orin's own GPU.

    Measured on the rover, one frame with twelve regions: the region finder
    16 ms, DINOv2 70 ms for all twelve crops in a single call, SigLIP2 42 ms for
    the same twelve. On the CPU those three are 520 ms, 1137 ms and 854 ms.
    Batching is most of the difference and it is why the engines carry an
    optimisation profile centred on twelve rather than on one.

    **The region finder costs 10 ms more than the FastSAM engine it replaced**,
    which is 16 ms against 5.7, and that is the price of the swap on a look that
    runs to about a fifth of a second. Two thirds of the difference is the
    maximum over 4,585 class scores, which the export folds into the graph: left
    outside it the same work is a 99 MB copy off the GPU and 32 ms of numpy, and
    the whole thing costs 51 ms rather than 16.

    Everything here runs in full precision except the region finder. fp16 was
    measured to break SigLIP2 outright -- fifty-seven phrases through the text
    tower collapsed into a 0.92 cone, so every phrase matched everything --
    while the region engine's boxes in fp16 match the CPU graph's to a mean
    overlap of 0.998 over 418 boxes from the rover's own frames, worst 0.894,
    which is the only thing a box is asked for.
    """

    name = "tensorrt"

    def __init__(self, directory: str) -> None:
        from .engines import DINO as DINO_ENGINE
        from .engines import REGIONS as REGIONS_ENGINE
        from .engines import SIGLIP_TEXT, SIGLIP_VISION, Engine

        self._directory = directory
        self._paths = {
            "regions": os.path.join(directory, REGIONS_ENGINE),
            "dino": os.path.join(directory, DINO_ENGINE),
            "vision": os.path.join(directory, SIGLIP_VISION),
        }
        self._text = os.path.join(directory, SIGLIP_TEXT)
        self._regions = self._dino = self._vision = self._text_engine = None
        # Nothing is loaded here, and the text tower is not loaded even by a
        # start-up that opens the other three: it is 1.1 GB against their 0.5,
        # only a search wants it, and a rover nobody searches should not be
        # carrying it. The first search opens it and every search after that
        # finds it already open.

    def open(self) -> None:
        """The three engines a look needs. Idempotent."""
        from .engines import NoEngines

        if None not in (self._regions, self._dino, self._vision):
            return
        try:
            self._open()
        except NoEngines:
            # Out of room, and the text tower is the one piece of this that a
            # look does not need. Give it back and try once more: a search is
            # something a person types and can wait for, and a look is the
            # rover's eyes.
            if self._text_engine is None:
                raise
            self.release_text()
            self._open()

    def _open(self) -> None:
        """Whichever of the three is not open, so that a retry after running out
        of room part-way through opens the rest rather than the lot again."""
        from .engines import Engine

        if self._regions is None:
            self._regions = Engine(self._paths["regions"], self._directory)
        if self._dino is None:
            self._dino = Engine(self._paths["dino"], self._directory)
        if self._vision is None:
            self._vision = Engine(self._paths["vision"], self._directory)

    def release_text(self) -> None:
        """Give the text tower back. Only a look short of room asks for this."""
        if self._text_engine is not None:
            self._text_engine.close()
            self._text_engine = None

    def regions(self, blob):
        return self._regions.run({"images": blob})["regions"]

    def appearance(self, batch):
        return self._dino.run({"pixel_values": batch})["last_hidden_state"][:, 0]

    def image_vectors(self, batch):
        return self._vision.run({"pixel_values": batch})["pooler_output"]

    def text_vectors(self, ids):
        """The text tower, opened by the first search and then kept.

        **It used to be loaded for the call and given back**, because with the
        local language model holding 3.2 GB of this board's 7.4 the text tower
        and a look's three engines did not both fit -- asking for it on top of
        them got a null execution context back, which is TensorRT's way of
        saying it has run out of room, and once it got the sidecar killed
        outright. That model has not been on the rover since it moved to the
        Orin, and measured again with it gone all four engines open in one
        process, come to 2.7 GB, run a look and a search either side of each
        other, and leave 1.3 GB spare.

        What it was costing was the whole of a search: 2.6 s to deserialise
        1.1 GB, 0.2 s to give it back, and a look's three engines to open again
        afterwards, around a forward pass that takes ten milliseconds. Held,
        the second search and every one after it costs the ten milliseconds.

        `open` is what makes this safe rather than a hope: a look that cannot
        find room puts the text tower down and tries again, so the thing that
        gets given up is the search nobody is waiting for.
        """
        from .engines import Engine

        if self._text_engine is None:
            self._text_engine = Engine(self._text, self._directory)
        return self._text_engine.run({"input_ids": ids})["pooler_output"]


class Perception:
    """The one interface: a JPEG in, regions with vectors out.

    Loaded lazily and once. Three sessions and a 34 MB tokenizer are the better
    part of a gigabyte of resident memory on a board that shares eight with SLAM,
    the navigator and a language model, so nothing is loaded until something asks
    for a look.

    One lock around the whole of `look`. onnxruntime is thread-safe, but two
    looks at once on six cores is two slow looks rather than one fast one, and
    the caller is a sidecar serving one client.
    """

    def __init__(self, directory: str = VENDOR, threads: int = 0) -> None:
        self.dir = directory
        self.threads = threads
        self._lock = threading.RLock()
        self._loaded = False
        self.load_s = 0.0
        #: Which backend actually ran, filled in by `load`. It travels with
        #: every look because the two do not produce comparable vectors.
        self.backend = ""
        #: Why the GPU was not used, when it was not. Empty when nothing was
        #: given up; a sentence when the rover is seeing less well than it could.
        self.fallback = ""
        #: The tokenizer, built on the first search and kept. Reading it back
        #: from its 34 MB of JSON costs 2.3 s of a core, measured on the rover,
        #: and it was being paid on every search for a thing that never changes.
        #: 34 MB held for the life of the process is the cheaper half of that
        #: trade by a wide margin.
        self._tokenizer = None

    # --- loading --------------------------------------------------------------

    def chosen(self) -> tuple[str, str]:
        """Which backend this host would use, and why not the other one.

        The GPU wins whenever its engines are there, because it is both faster
        and closer to full precision. Nothing here loads a model.
        """
        from . import engines

        ready, why_not = engines.available(self.dir)
        if ready:
            return "tensorrt", ""
        return "onnxruntime", why_not

    def available(self) -> tuple[bool, str]:
        """(ready, why not), without loading anything.

        Cheap on purpose: the daemon asks this before it opens the camera, and
        the answer on a host that has been deployed to but not installed on must
        cost nothing.
        """
        try:
            _vendored()
        except Unavailable as error:
            return False, str(error)
        backend, _ = self.chosen()
        # The tokenizer is wanted either way: it is what turns a search phrase
        # into the numbers the text tower reads, and neither backend has one of
        # its own.
        wanted = ((TOKENIZER,) if backend == "tensorrt"
                  else (YOLOE, DINO, SIGLIP, TOKENIZER))
        missing = [name for name in wanted
                   if not os.path.isfile(os.path.join(self.dir, name))]
        if missing:
            return False, (f"the perception models are missing from {self.dir}: "
                           f"{', '.join(missing)}. Run install_perception.sh")
        return True, ""

    def load(self) -> None:
        """Open the models a look needs. Idempotent.

        A GPU that has engines but cannot run them falls back rather than
        failing: a rover that sees a little worse is worth having and a rover
        that sees nothing is not. The fallback is reported, never silent.
        """
        with self._lock:
            if self._loaded:
                return
            np, cv2 = _vendored()
            self._np, self._cv2 = np, cv2
            began = time.monotonic()

            backend, why_not = self.chosen()
            self.fallback = ""
            if backend == "tensorrt":
                from .engines import NoEngines
                try:
                    self._models = _GpuModels(self.dir)
                except NoEngines as error:
                    self.fallback = str(error)
                    self._models = _CpuModels(self.dir, self.threads)
            else:
                self.fallback = why_not
                self._models = _CpuModels(self.dir, self.threads)

            self.backend = self._models.name
            # A look's own models and nothing else. The text tower is not opened
            # here and is not held: on the GPU it is over half a gigabyte, and a
            # search is the only thing that wants it. See `_GpuModels`.
            self._models.open()
            self.load_s = round(time.monotonic() - began, 2)
            self._loaded = True

    # --- looking --------------------------------------------------------------

    def look(self, jpeg: bytes) -> dict[str, Any]:
        """One frame, measured. Never raises for anything a rover does daily.

        The answer is a list of regions, each with a box in fractions of the
        frame and the two vectors behind it. Nothing is named: what a region is
        called was measured to be worthless, and the question a person actually
        asks -- "find me the spray bottle" -- is answered from the same vector by
        `search.py`. The timings come with it because the whole question about
        this pipeline is whether it fits in the time between two lidar scans.
        """
        with self._lock:
            self.load()
            # Idempotent, and here rather than only in `load()` because a search
            # puts a look's engines down to make room for the text tower. This is
            # what picks them back up, and on the usual path it does nothing.
            self._models.open()
            np, cv2 = self._np, self._cv2
            began = time.monotonic()
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("that was not a picture this could decode")

            # Before anything is asked of the models: is there a picture here at
            # all? See DARK_FRACTION. A frame the camera has not managed to
            # expose is not an empty room, and the two must not look alike.
            if _too_dark(np, image):
                return {"regions": [], "found": 0, "kept": 0, "blank": 0,
                        "dark": True, "backend": self.backend,
                        "timings": {"regions_ms": 0, "dino_ms": 0,
                                    "siglip_ms": 0},
                        "took_s": round(time.monotonic() - began, 2)}

            boxes, scores, region_s = self._regions(image)
            kept = [(box, score) for box, score in zip(boxes, scores)
                    if _worth_keeping(box)]
            # Largest first, then cut. The cut is a cost ceiling and the order is
            # what makes it defensible: what falls off the end is the smallest
            # thing in the room, which is also the hardest to place from a bearing.
            kept.sort(key=lambda pair: -_area(pair[0]))
            cropped = []
            blank = 0
            for box, score in kept[:MAX_REGIONS]:
                patch = self._crop(image, box)
                if patch is None:
                    continue
                # The one test that needs the pixels rather than the box. See
                # MIN_CONTRAST: a blown-out window and a bare wall pass every
                # filter above and carry no identity whatever.
                if _blank(np, patch):
                    blank += 1
                    continue
                cropped.append((box, score, patch))

            if not cropped:
                return {"regions": [], "found": len(boxes), "kept": len(kept),
                        "blank": blank, "dark": False, "backend": self.backend,
                        "timings": {"regions_ms": round(region_s * 1000),
                                    "dino_ms": 0, "siglip_ms": 0},
                        "took_s": round(time.monotonic() - began, 2)}

            patches = [patch for _, _, patch in cropped]
            dino, dino_s = self._appearance(patches)
            siglip, siglip_s = self._semantic(patches)

            regions = []
            for index, (box, score, _) in enumerate(cropped):
                regions.append({
                    "bbox": [round(float(value), 4) for value in box],
                    "region_score": round(float(score), 3),
                    "area": round(_area(box), 4),
                    "dino": dino[index].astype("float32").tobytes(),
                    "siglip": siglip[index].astype("float32").tobytes(),
                })
            return {
                "regions": regions,
                "found": len(boxes),
                "kept": len(kept),
                "blank": blank,
                "dark": False,
                "backend": self.backend,
                "timings": {"regions_ms": round(region_s * 1000),
                            "dino_ms": round(dino_s * 1000),
                            "siglip_ms": round(siglip_s * 1000)},
                "took_s": round(time.monotonic() - began, 2),
            }

    # --- the three models -----------------------------------------------------

    def _regions(self, image):
        """Class-agnostic boxes, as fractions of the frame.

        YOLOE is a YOLO11 segmentation head, so what comes back is one row per
        anchor of four box numbers, a score and thirty-two mask coefficients --
        the same shape FastSAM produced, because the export folds the 4,585 class
        scores down to their maximum before the graph ends. The masks are not
        decoded: the box is all that a bearing and a crop need, and the
        prototypes are the expensive half.

        **The score is therefore a class score and not an objectness.** It is how
        strongly the best of 4,585 tags fits, which is why the threshold sits
        lower than the one FastSAM ran at, and why a thing with no tag anywhere
        near it is a thing this model does not propose at all.
        """
        np = self._np
        canvas, scale, left, top = self._letterbox(image, YOLOE_SIZE)
        blob = (canvas[:, :, ::-1].transpose(2, 0, 1)[None]
                .astype(np.float32) / 255.0)
        began = time.monotonic()
        raw = self._models.regions(blob)
        took = time.monotonic() - began

        rows = raw[0].T
        scores = rows[:, 4]
        rows, scores = rows[scores >= YOLOE_CONF], scores[scores >= YOLOE_CONF]
        if not len(rows):
            return np.zeros((0, 4)), np.zeros(0), took
        cx, cy, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
        keep = _suppress(np, boxes, scores, YOLOE_OVERLAP)
        boxes, scores = boxes[keep], scores[keep]

        height, width = image.shape[:2]
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / scale / width
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / scale / height
        return np.clip(boxes, 0.0, 1.0), scores, took

    def _appearance(self, patches):
        """DINOv2's class token per crop: is this the same *instance*.

        **Weaker at that than the plan assumed, measured on this rover's own
        frames.** Two crops of the same chair from the same viewpoint score
        0.981, which is the number the plan recorded as evidence. Across a
        genuine change of viewpoint the same chair scores 0.696 -- and the *twin*
        chair across the room, seen from an angle more like the original, scores
        0.735. Higher. On this evidence appearance answers "does this look like
        that picture" and not "is this the same object", and the two come apart
        exactly where the rover needs them not to.

        That is not a reason to drop it; it is the reason geometry is the
        arbiter rather than the tiebreaker. Kept because it is nearly free beside
        a bearing, because it separates a chair from a bottle without effort
        (0.12), and because a resolver that has already narrowed to one place can
        use a weak signal safely. It must never be allowed to overrule a
        placement, which is what the plan's redundant-furniture test exists to
        check.
        """
        np = self._np
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        batch = np.stack([
            ((self._square(patch, DINO_SIZE)[:, :, ::-1].astype(np.float32) / 255.0)
             - mean) / std for patch in patches]).transpose(0, 3, 1, 2)
        began = time.monotonic()
        out = self._models.appearance(batch)
        return _unit(np, out), time.monotonic() - began

    def _semantic(self, patches):
        """SigLIP2's image embedding per crop: what a description would match.

        Stored rather than reduced to a word, because reducing it was measured to
        throw away everything it was worth: the nearest phrase in a fixed list
        scored between 0.08 and 0.12 whatever the crop held, while the same
        vector against a phrase somebody actually typed separates present from
        absent at a floor of 0.09. The vector is the record; the question is
        asked later.
        """
        np = self._np
        batch = np.stack([
            (self._square(patch, SIGLIP_SIZE)[:, :, ::-1].astype(np.float32) / 255.0
             - 0.5) / 0.5 for patch in patches]).transpose(0, 3, 1, 2)
        began = time.monotonic()
        out = self._models.image_vectors(batch)
        return _unit(np, out), time.monotonic() - began

    # --- pictures -------------------------------------------------------------

    def _letterbox(self, image, size):
        """Scale to fit a square and pad with grey, keeping the aspect ratio.

        The padding matters more than it looks: stretching a 640x480 frame to a
        square moves every box, and a box is the measurement everything
        downstream turns into a bearing.
        """
        np, cv2 = self._np, self._cv2
        height, width = image.shape[:2]
        scale = min(size / width, size / height)
        new_w, new_h = int(round(width * scale)), int(round(height * scale))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        top, left = (size - new_h) // 2, (size - new_w) // 2
        canvas[top:top + new_h, left:left + new_w] = resized
        return canvas, scale, left, top

    def _square(self, patch, size):
        return self._letterbox(patch, size)[0]

    def _crop(self, image, box):
        """The picture inside one box, or None if it is too small to be one."""
        height, width = image.shape[:2]
        left = int(max(0.0, box[0] - CROP_PAD) * width)
        top = int(max(0.0, box[1] - CROP_PAD) * height)
        right = int(min(1.0, box[2] + CROP_PAD) * width)
        bottom = int(min(1.0, box[3] + CROP_PAD) * height)
        if right - left < 8 or bottom - top < 8:
            return None
        return image[top:bottom, left:right]

    def embed_text(self, phrases: list[str]):
        """Text vectors for arbitrary phrases, which is what a search is made of.

        The only thing the text tower is loaded for. A phrase lands in the same
        space as every stored region vector, so the comparison is a dot product
        and `search.py` does the rest.
        """
        # Loading comes first, and everything else waits for it. `load()` is what
        # puts the vendored wheels on the path and the numeric library on this
        # object, so anything reached for beforehand -- the tokenizer import as
        # much as `self._np` -- fails on every search that arrives before the
        # first look, which after a reboot is every search. Measured on the
        # rover with the import above this line: ModuleNotFoundError, where a
        # host with no models installed should be told exactly that.
        self.load()

        np = self._np
        ids = np.array([e.ids for e in self._words().encode_batch(
            [phrase.lower() for phrase in phrases])], dtype=np.int64)
        return _unit(np, self._models.text_vectors(ids))

    def _words(self):
        """The tokenizer, built once.

        Under the same lock as everything else here, because two searches
        arriving together would otherwise build it twice and the second one
        would be paying the 2.3 s this exists to stop paying.
        """
        with self._lock:
            if self._tokenizer is None:
                from tokenizers import Tokenizer

                tokenizer = Tokenizer.from_file(os.path.join(self.dir, TOKENIZER))
                tokenizer.enable_padding(length=SIGLIP_TOKENS, pad_id=0,
                                         pad_token="<pad>")
                tokenizer.enable_truncation(max_length=SIGLIP_TOKENS)
                self._tokenizer = tokenizer
        return self._tokenizer


# --- arithmetic that needs no model -----------------------------------------

def _area(box) -> float:
    return float(max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1]))


def _worth_keeping(box) -> bool:
    """The plan's three-line filter, and it really is three lines.

    Bigger than a third of the frame is the floor or a wall; smaller than 0.4%
    is a highlight on a tile; longer than six times its width is an edge. What
    survives on the rover's own frames is the spray bottle, the air purifier, the
    framed pictures, the armchair and the doorway.
    """
    width, height = box[2] - box[0], box[3] - box[1]
    if width <= 0 or height <= 0:
        return False
    if not MIN_AREA <= width * height <= MAX_AREA:
        return False
    return max(width / height, height / width) <= MAX_ASPECT


def _too_dark(np, image) -> bool:
    """Whether this whole frame is too dark to be a picture of anything.

    Cheap enough to run before the models rather than after: one mean over the
    frame's own pixels, against a rule whose numbers are argued at
    `DARK_FRACTION`.
    """
    grey = image.mean(axis=2) if image.ndim == 3 else image
    return float((grey < DARK_AT).mean()) > DARK_FRACTION


def _blank(np, patch) -> bool:
    """Whether this crop is a picture of nothing.

    Two ways for it to be, and both were on the rover: a flat patch of wall or
    ceiling, which has no contrast, and a window the camera has blown to white,
    which has plenty at its frame and nothing inside it. Neither can be told
    apart from any other one of its kind by any encoder, so both are refused
    here rather than left for the resolver to be confused by.
    """
    grey = patch.mean(axis=2) if patch.ndim == 3 else patch
    if float(grey.std()) < MIN_CONTRAST:
        return True
    return float((grey > 250).mean()) > MAX_BLOWN


def _suppress(np, boxes, scores, threshold):
    """Greedy suppression, written out rather than imported.

    onnxruntime has an NMS operator and torchvision has a fast one, but this
    runs on a few dozen boxes once a look and the alternative is another
    dependency on a board that is deliberately short of them.

    **Overlap is measured against the smaller of the two boxes, not their
    union**, which is the one place this differs from ordinary NMS and the
    reason it is written out at all. Union is the right denominator when the
    boxes are rival guesses at the same object, and the wrong one when one box
    is a *part* of the other: a seat cushion inside a sofa scores about 0.15
    against the union, nowhere near any threshold anybody would set, so the
    cushion and the sofa both survive and the rover records two things where
    there is one. Measured on ten of the rover's own frames, 65 of the 114
    regions it embedded -- 57% -- were at least four fifths inside another
    region embedded from the same picture. Dividing by the smaller box instead
    takes that to none, and the freed slots refill from further down the list,
    so the count barely moves: 95 regions where there were 114, and all of them
    separate things.
    """
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        best = order[0]
        keep.append(best)
        if order.size == 1:
            break
        rest = order[1:]
        x1 = np.maximum(boxes[best, 0], boxes[rest, 0])
        y1 = np.maximum(boxes[best, 1], boxes[rest, 1])
        x2 = np.minimum(boxes[best, 2], boxes[rest, 2])
        y2 = np.minimum(boxes[best, 3], boxes[rest, 3])
        overlap = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        area_best = ((boxes[best, 2] - boxes[best, 0])
                     * (boxes[best, 3] - boxes[best, 1]))
        area_rest = ((boxes[rest, 2] - boxes[rest, 0])
                     * (boxes[rest, 3] - boxes[rest, 1]))
        smaller = np.minimum(area_best, area_rest)
        order = rest[overlap / np.maximum(smaller, 1e-9) <= threshold]
    return keep


def _unit(np, vectors):
    """L2-normalised rows, so every later comparison is a dot product."""
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    return vectors / np.maximum(norms, 1e-12)
