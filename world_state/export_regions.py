#!/usr/bin/env python3
"""Make the region finder the rover runs. Not run on the rover, and not deployed.

    pip install ultralytics onnx onnxslim
    python world_state/export_regions.py
    scp yoloe-11s-seg-objectness.onnx orin:~/ugv/world_state/vendor/

**Every other model in vendor/ is downloaded and this one cannot be**, which is
the one cost of the swap away from FastSAM. The published YOLOE weights are
PyTorch, exporting them wants ultralytics and torch, and the rover has neither
and no room to gain them. So the file is built on a workstation and copied over,
and `install_perception.sh` checks for it rather than fetching it.

What the fold is for
--------------------

YOLOE finds regions the way FastSAM did and then, unlike FastSAM, tries to name
each one against a built-in vocabulary of 4,585 tags. The rover throws every one
of those names away: nothing in the world state records what a thing is called,
because nothing in it can measure that. But the scores are computed anyway, and
in the stock export they leave the graph -- 4,585 numbers for each of 5,376
anchors, 99 MB a frame, 6 ms to copy off the GPU and 32 ms for numpy to reduce.

Taking the maximum inside the graph costs nothing and leaves an output shaped
exactly like the FastSAM export it replaces: four box numbers, one score,
thirty-two mask coefficients. `perceive.py` reads it unchanged.
"""
from __future__ import annotations

import argparse
import os

WEIGHTS = "yoloe-11s-seg-pf.pt"
#: Small, to sit where FastSAM-s sat. The larger YOLOE weights were not measured
#: on the rover; a look has about a fifth of a second and the encoders own most
#: of it.
SIZE = 512
COEFFICIENTS = 32
OUTPUT = "yoloe-11s-seg-objectness.onnx"


def export(weights: str, size: int) -> str:
    """The stock ultralytics export, at the rover's input size."""
    from ultralytics import YOLOE

    model = YOLOE(weights)
    print(f"{weights}: {len(model.names)} tags in the built-in vocabulary, "
          f"none of which will reach the rover")
    return str(model.export(format="onnx", imgsz=size, opset=17, simplify=True))


def fold(source: str, target: str) -> None:
    """Replace the class block with its maximum, in the graph.

    Four slices and a concat: boxes through untouched, the class block reduced to
    one number, the mask coefficients through untouched. The result is named
    `regions` because `output0` no longer describes it.
    """
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    model = onnx.load(source)
    graph = model.graph
    head = graph.output[0]
    channels = head.type.tensor_type.shape.dim[1].dim_value
    anchors = head.type.tensor_type.shape.dim[2].dim_value
    classes = channels - 4 - COEFFICIENTS
    if classes < 2:
        raise SystemExit(f"{source} has {channels} channels; already folded?")
    print(f"{head.name}: {channels} channels = 4 box + {classes} class + "
          f"{COEFFICIENTS} mask, over {anchors} anchors")

    def const(name, values):
        graph.initializer.append(
            numpy_helper.from_array(np.array(values, dtype=np.int64), name))
        return name

    graph.node.extend([
        helper.make_node("Slice", [head.name, const("fold_box_from", [0]),
                                   const("fold_box_to", [4]),
                                   const("fold_axis", [1])], ["fold_boxes"]),
        helper.make_node("Slice", [head.name, const("fold_cls_from", [4]),
                                   const("fold_cls_to", [4 + classes]),
                                   "fold_axis"], ["fold_classes"]),
        helper.make_node("ReduceMax", ["fold_classes"], ["fold_score"],
                         axes=[1], keepdims=1),
        helper.make_node("Slice", [head.name, const("fold_msk_from", [4 + classes]),
                                   const("fold_msk_to", [channels]),
                                   "fold_axis"], ["fold_masks"]),
        helper.make_node("Concat", ["fold_boxes", "fold_score", "fold_masks"],
                         ["regions"], axis=1),
    ])

    folded = helper.make_tensor_value_info(
        "regions", TensorProto.FLOAT, [1, 4 + 1 + COEFFICIENTS, anchors])
    rest = [out for out in graph.output if out.name != head.name]
    del graph.output[:]
    graph.output.extend([folded] + rest)

    onnx.checker.check_model(model)
    onnx.save(model, target)
    print(f"wrote {target}: output [1, {4 + 1 + COEFFICIENTS}, {anchors}], "
          f"{os.path.getsize(target) / 1e6:.1f} MB")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default=WEIGHTS)
    parser.add_argument("--size", type=int, default=SIZE)
    parser.add_argument("--out", default=OUTPUT)
    parser.add_argument("--from-onnx", default="",
                        help="fold an export that already exists")
    args = parser.parse_args()

    source = args.from_onnx or export(args.weights, args.size)
    fold(source, args.out)
    print("copy it to the rover:\n"
          f"    scp {args.out} orin:~/ugv/world_state/vendor/\n"
          "    ssh orin '~/ugv/world_state/install_perception.sh --engines'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
