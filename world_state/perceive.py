"""Regions, embeddings and names for one frame, from three models that know no
categories between them.

This is the half of the world state that replaced asking a language model which
lasting thing it was looking at. Four questions, four mechanisms, and the point is
that they are separate:

    what regions are in this frame?    FastSAM, which has no vocabulary at all
    is this the same instance?         DINOv2 similarity, gated by geometry
    what is it called?                 the nearest phrase in vocabulary.txt to
                                       the region's SigLIP2 vector
    which lasting thing is it?         **not answered here.** That is a
                                       triangulated map position, and it needs a
                                       second look from somewhere else.

Nothing in this file decides an identity, and nothing in it may. What it produces
is a measurement of one picture: boxes, two vectors per box, and a name that is
derived rather than detected. The box is the measurement and the label is a hint
-- on two byte-identical frames the detectors redrew the same box to within a
quarter of a degree of bearing, while the language model's label for one chair
moved between "black leather recliner" and "blue leather recliner".

**The vectors are what is stored; the name is worked out again at display time.**
That is what makes the vocabulary a config file rather than a model: editing
`vocabulary.txt` re-labels every object the rover has ever seen without
reprocessing a frame.

The models are unpacked into `vendor/` by `install_perception.sh`, which is
deliberately separate from the Cosmos installer -- the language model is still
wanted for the conversational `look`, and the two have to break independently.
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
FASTSAM = "FastSAM-s.onnx"
DINO = "dinov2-small-int8.onnx"
SIGLIP = "siglip2-base-patch32-256-int8.onnx"
TOKENIZER = "siglip2-tokenizer.json"
VOCABULARY = os.path.join(HERE, "vocabulary.txt")

#: What FastSAM is shown, and the number was walked down until something broke.
#: Its own default is 1024, which on a 640x480 camera is upsampling before it is
#: anything else. Measured against objects read off the frames by hand: 640 and
#: 512 both find the spray bottle, the armchair, the framed picture and the air
#: purifier, and 512 costs 330 ms on the rover against 510. **448 is where it
#: breaks** -- the air purifier disappears entirely -- so this is one step above
#: the cliff rather than at the bottom of it.
FASTSAM_SIZE = 512
#: Below this the box is a texture rather than a thing. Measured: at the library's
#: own default of 0.25 the same living-room frame came back with 49 boxes of which
#: half were reflections on a tiled floor; at 0.4 it is 29, which is what the plan
#: recorded and what the eye agrees with.
FASTSAM_CONF = 0.4
FASTSAM_IOU = 0.7

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


class Unavailable(RuntimeError):
    """The models are not installed, or the runtime cannot be imported.

    Its own type because the sidecar answers it with a sentence rather than a
    traceback: a rover that has been deployed to but not yet installed on is an
    ordinary state, and it should read as "run install_perception.sh" rather than
    as a crash.
    """


def _vendored():
    """Import numpy, cv2 and onnxruntime, from vendor/ if that is where they are.

    An installed copy wins, so that a desk with these on the path is unaffected;
    the rover has none of them installed and all of them unpacked, because its
    Python is externally managed and sudo wants a password no deploy script has.
    """
    for path in (VENDOR, os.path.join(os.path.dirname(HERE), "vendor")):
        if os.path.isdir(path) and path not in sys.path:
            sys.path.append(path)
    try:
        import cv2
        import numpy
        import onnxruntime
    except ImportError as error:
        raise Unavailable(
            f"the perception models are not installed on this host: {error}. "
            f"Run ~/ugv/world_state/install_perception.sh") from error
    return numpy, cv2, onnxruntime


def read_vocabulary(path: str = VOCABULARY) -> list[str]:
    """The phrases a region can be called, in file order.

    Comments and blank lines are dropped. Order is kept because it is what the
    stored label index would mean if anything ever stored one -- nothing does,
    and it should stay that way: the vector is the record and the name is a view
    of it.
    """
    words = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line and not line.startswith("#"):
                words.append(line)
    return words


class Perception:
    """The one interface: a JPEG in, regions with vectors and names out.

    Loaded lazily and once. Three sessions and a 34 MB tokenizer are the better
    part of a gigabyte of resident memory on a board that shares eight with SLAM,
    the navigator and a language model, so nothing is loaded until something asks
    for a look.

    One lock around the whole of `look`. onnxruntime is thread-safe, but two
    looks at once on six cores is two slow looks rather than one fast one, and
    the caller is a sidecar serving one client.
    """

    def __init__(self, directory: str = VENDOR, threads: int = 0,
                 vocabulary: str = VOCABULARY) -> None:
        self.dir = directory
        self.threads = threads
        self.vocabulary_path = vocabulary
        self._lock = threading.RLock()
        self._loaded = False
        self.words: list[str] = []
        self.load_s = 0.0

    # --- loading --------------------------------------------------------------

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
        missing = [name for name in (FASTSAM, DINO, SIGLIP, TOKENIZER)
                   if not os.path.isfile(os.path.join(self.dir, name))]
        if missing:
            return False, (f"the perception models are missing from {self.dir}: "
                           f"{', '.join(missing)}. Run install_perception.sh")
        return True, ""

    def load(self) -> None:
        """Open the three sessions and embed the vocabulary. Idempotent."""
        with self._lock:
            if self._loaded:
                return
            np, cv2, ort = _vendored()
            self._np, self._cv2 = np, cv2
            began = time.monotonic()

            options = ort.SessionOptions()
            if self.threads:
                options.intra_op_num_threads = self.threads
            options.graph_optimization_level = \
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            # **The single most expensive line in this file, and it is a
            # one-word setting.** Three sessions run one after another on every
            # look, and by default an onnxruntime thread pool keeps spinning for
            # a while after its own work finishes -- so FastSAM's threads are
            # still burning cores while DINOv2 runs, and DINOv2's while SigLIP2
            # does. Measured on the rover's own frames: a look costs 2.64 s with
            # spinning left on and 0.80 s with it off, and the models themselves
            # are identical. Each one alone is as fast either way, which is
            # exactly why this is easy to miss.
            options.add_session_config_entry("session.intra_op.allow_spinning", "0")

            def session(name):
                path = os.path.join(self.dir, name)
                if not os.path.isfile(path):
                    raise Unavailable(f"no {name} at {self.dir}; run "
                                      f"install_perception.sh")
                return ort.InferenceSession(path, options,
                                            providers=["CPUExecutionProvider"])

            self._fastsam = session(FASTSAM)
            self._dino = session(DINO)
            self._siglip = session(SIGLIP)
            self._load_words()
            self.load_s = round(time.monotonic() - began, 2)
            self._loaded = True

    def _load_words(self) -> None:
        """Embed the vocabulary once, because it is the same every frame.

        Measured: thirty phrases through the text tower is 870 ms, which is more
        than every crop in a frame costs put together. Doing it per look would
        have made the vocabulary the most expensive thing in the pipeline, for an
        answer that cannot change between frames.
        """
        from tokenizers import Tokenizer

        np = self._np
        self.words = read_vocabulary(self.vocabulary_path)
        tokenizer = Tokenizer.from_file(os.path.join(self.dir, TOKENIZER))
        # The file already carries padding to 64 with pad id 0. Setting it here
        # with a different id is the mistake that costs a day: every text vector
        # comes back plausible and wrong, and the only symptom is that one phrase
        # wins on every region.
        tokenizer.enable_padding(length=SIGLIP_TOKENS, pad_id=0, pad_token="<pad>")
        tokenizer.enable_truncation(max_length=SIGLIP_TOKENS)
        ids = np.array([e.ids for e in tokenizer.encode_batch(
            [word.lower() for word in self.words])], dtype=np.int64)
        self._word_ids = ids
        blank = np.zeros((1, 3, SIGLIP_SIZE, SIGLIP_SIZE), dtype=np.float32)
        vectors = self._siglip.run(["text_embeds"],
                                   {"pixel_values": blank, "input_ids": ids})[0]
        self._word_vectors = _unit(np, vectors)
        # One token's worth of text is the cheapest input the graph accepts, and
        # the image path needs an `input_ids` whether it wants one or not: this
        # model carries both towers in one graph, which is also what makes the
        # text search in a later phase free of a second download.
        self._one_word = ids[:1]

    # --- looking --------------------------------------------------------------

    def look(self, jpeg: bytes) -> dict[str, Any]:
        """One frame, measured. Never raises for anything a rover does daily.

        The answer is a list of regions, each with a box in fractions of the
        frame, the two vectors, and the nearest phrase in the vocabulary. The
        timings come with it because the whole question about this pipeline is
        whether it fits in the time between two lidar scans.
        """
        with self._lock:
            self.load()
            np, cv2 = self._np, self._cv2
            began = time.monotonic()
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("that was not a picture this could decode")

            boxes, scores, region_s = self._regions(image)
            kept = [(box, score) for box, score in zip(boxes, scores)
                    if _worth_keeping(box)]
            # Largest first, then cut. The cut is a cost ceiling and the order is
            # what makes it defensible: what falls off the end is the smallest
            # thing in the room, which is also the hardest to place from a bearing.
            kept.sort(key=lambda pair: -_area(pair[0]))
            cropped = []
            for box, score in kept[:MAX_REGIONS]:
                patch = self._crop(image, box)
                if patch is not None:
                    cropped.append((box, score, patch))

            if not cropped:
                return {"regions": [], "found": len(boxes), "kept": len(kept),
                        "timings": {"regions_ms": round(region_s * 1000),
                                    "dino_ms": 0, "siglip_ms": 0},
                        "took_s": round(time.monotonic() - began, 2)}

            patches = [patch for _, _, patch in cropped]
            dino, dino_s = self._appearance(patches)
            siglip, siglip_s = self._semantic(patches)
            names = self._name(siglip)

            regions = []
            for index, (box, score, _) in enumerate(cropped):
                label, confidence = names[index]
                regions.append({
                    "bbox": [round(float(value), 4) for value in box],
                    "region_score": round(float(score), 3),
                    "area": round(_area(box), 4),
                    "label": label,
                    "label_score": round(float(confidence), 4),
                    "dino": dino[index].astype("float32").tobytes(),
                    "siglip": siglip[index].astype("float32").tobytes(),
                })
            return {
                "regions": regions,
                "found": len(boxes),
                "kept": len(kept),
                "timings": {"regions_ms": round(region_s * 1000),
                            "dino_ms": round(dino_s * 1000),
                            "siglip_ms": round(siglip_s * 1000)},
                "took_s": round(time.monotonic() - began, 2),
            }

    # --- the three models -----------------------------------------------------

    def _regions(self, image):
        """Class-agnostic boxes, as fractions of the frame.

        FastSAM is a YOLOv8 segmentation head, so what comes back is one row per
        anchor of four box numbers, one objectness and thirty-two mask
        coefficients. The masks are not decoded: the box is all that a bearing
        and a crop need, and the prototypes are the expensive half.
        """
        np = self._np
        canvas, scale, left, top = self._letterbox(image, FASTSAM_SIZE)
        blob = (canvas[:, :, ::-1].transpose(2, 0, 1)[None]
                .astype(np.float32) / 255.0)
        began = time.monotonic()
        raw = self._fastsam.run(None, {"images": blob})[0]
        took = time.monotonic() - began

        rows = raw[0].T
        scores = rows[:, 4]
        rows, scores = rows[scores >= FASTSAM_CONF], scores[scores >= FASTSAM_CONF]
        if not len(rows):
            return np.zeros((0, 4)), np.zeros(0), took
        cx, cy, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
        keep = _suppress(np, boxes, scores, FASTSAM_IOU)
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
        out = self._dino.run(None, {"pixel_values": batch})[0]
        return _unit(np, out[:, 0]), time.monotonic() - began

    def _semantic(self, patches):
        """SigLIP2's image embedding per crop: what this is, and what it matches.

        The same vector answers two questions in different phases -- the name
        now, and "find me the spray bottle" later -- which is why it is stored
        rather than reduced to a label.
        """
        np = self._np
        batch = np.stack([
            (self._square(patch, SIGLIP_SIZE)[:, :, ::-1].astype(np.float32) / 255.0
             - 0.5) / 0.5 for patch in patches]).transpose(0, 3, 1, 2)
        began = time.monotonic()
        out = self._siglip.run(["image_embeds"],
                               {"pixel_values": batch,
                                "input_ids": self._one_word})[0]
        return _unit(np, out), time.monotonic() - began

    def _name(self, vectors):
        """The nearest phrase in the vocabulary to each region's vector.

        A dot product against a few dozen stored vectors, which is the whole of
        it and the reason the plan refuses a vector database. The score comes
        back with the name and is **not** a confidence: these sit between about
        0.08 and 0.12 whatever is in the picture, and only the ordering means
        anything until somebody calibrates a floor against real crops.
        """
        scores = vectors @ self._word_vectors.T
        best = scores.argmax(axis=1)
        return [(self.words[index], scores[row, index])
                for row, index in enumerate(best)]

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
        """Text vectors for arbitrary phrases, for the search a later phase adds.

        Here rather than in that phase because the model that answers it is
        already loaded and the tokenizer is already open. It is not on the
        per-look path and nothing calls it yet.
        """
        from tokenizers import Tokenizer

        np = self._np
        self.load()
        tokenizer = Tokenizer.from_file(os.path.join(self.dir, TOKENIZER))
        tokenizer.enable_padding(length=SIGLIP_TOKENS, pad_id=0, pad_token="<pad>")
        tokenizer.enable_truncation(max_length=SIGLIP_TOKENS)
        ids = np.array([e.ids for e in tokenizer.encode_batch(
            [phrase.lower() for phrase in phrases])], dtype=np.int64)
        blank = np.zeros((1, 3, SIGLIP_SIZE, SIGLIP_SIZE), dtype=np.float32)
        out = self._siglip.run(["text_embeds"],
                               {"pixel_values": blank, "input_ids": ids})[0]
        return _unit(np, out)


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


def _suppress(np, boxes, scores, threshold):
    """Greedy non-maximum suppression, written out rather than imported.

    onnxruntime has an NMS operator and torchvision has a fast one, but this
    runs on a few dozen boxes once a look and the alternative is another
    dependency on a board that is deliberately short of them.
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
        union = area_best + area_rest - overlap
        order = rest[overlap / np.maximum(union, 1e-9) <= threshold]
    return keep


def _unit(np, vectors):
    """L2-normalised rows, so every later comparison is a dot product."""
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    return vectors / np.maximum(norms, 1e-12)
