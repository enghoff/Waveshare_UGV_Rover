#!/bin/sh
# Put the perception models on the rover: regions, appearance and semantics.
#
#     ssh orin '~/ugv/world_state/install_perception.sh'            # fetch what is missing
#     ssh orin '~/ugv/world_state/install_perception.sh --force'    # fetch it all again
#     ssh orin '~/ugv/world_state/install_perception.sh --engines'  # rebuild the engines only
#
# This is the half of the world state that replaced asking a language model which
# lasting thing it was looking at. Three small networks, none of which knows any
# categories, and between them they answer three separate questions:
#
#     YOLOE-11s     what regions are in this frame, with its vocabulary cut off
#     DINOv2-small  is this the same *instance* as that one
#     SigLIP2       what is this called, and later: find me the thing I describe
#
# **The only installer this component has.** There were two: the other fetched a
# local language model and a llama.cpp server to run it, and both left the rover
# on 2026-09-02 when the encoders took over the per-look path and nothing was
# left calling the model.
#
# Under two gigabytes, fetched here rather than deployed: the deployer carries the
# bytes a commit describes, and neither a quantized ONNX graph nor a compiled
# engine is described by a commit.
# vendor/ is preserved across deploys by deploy/manifest.json.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
VENDOR="$HERE/vendor"
TRTEXEC=/usr/src/tensorrt/bin/trtexec
FORCE=""
ENGINES_ONLY=""
case "${1:-}" in
    --force) FORCE=1 ;;
    --engines) ENGINES_ONLY=1 ;;
esac

# --- the runtimes -----------------------------------------------------------
#
# There are two, and which one a look uses is decided by whether the engines
# below got built.
#
# **onnxruntime is the CPU fallback, and its CPU provider is the only one it can
# offer here.** That is measured rather than assumed and the reason is worth
# writing down: CUDA and cuDNN *are* available for this board from NVIDIA, and
# JetPack installs them, but no build of onnxruntime exists for JetPack 7. The
# community Jetson wheel index stops at JetPack 6, and the official aarch64
# wheel on PyPI carries kernels compiled for sm_70 through sm_121 -- every
# architecture except this Orin's sm_87 -- so it opens a session on the GPU and
# then fails at the first launch with "no kernel image is available". Installing
# more CUDA does not help, because the gap is inside the wheel.
#
# The onnxruntime wheel is built per CPython version rather than abi3, so this
# pin is to Python 3.12, which is what Ubuntu 24.04 ships here. A board that
# moves to 3.13 needs the URL below changed, and will say so by failing to
# import rather than by behaving oddly.
ORT_WHEEL=onnxruntime-1.29.0-cp312-cp312-manylinux_2_28_aarch64.whl
ORT_URL=https://files.pythonhosted.org/packages/30/12/4be0e345d38fe707a701ca07e8f63c05b152a2e6285d1e43a7faf63fedd2/$ORT_WHEEL
ORT_SHA=d2fb19e848f7c33ed8d3182b52504aaa11c5e8da438bbb47296f85b133cbcf6b

# The Gemma tokenizer SigLIP2 uses, as a Rust extension with an abi3 wheel.
TOK_WHEEL=tokenizers-0.23.1-cp310-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
TOK_URL=https://files.pythonhosted.org/packages/6c/36/e006edf031154cba92b8416057d92c3abe3635e4c4b0aa0b5b9bb39dde70/$TOK_WHEEL
TOK_SHA=1bf13402aff9bc533c89cb849ec3b412dc3fbeacc9744840e423d7bf3f7dc0e3

# TensorRT itself comes from JetPack and is installed system-wide; these two
# small wheels are only the CUDA memory calls Python needs to hand an engine its
# input and take its answer back.
CUDAB_WHEEL=cuda_bindings-13.3.1-cp312-cp312-manylinux_2_24_aarch64.manylinux_2_28_aarch64.whl
CUDAB_URL=https://files.pythonhosted.org/packages/ce/67/5e7dba1ba576dd73da5dee894ca076ca5e959450dfff66d6d510a255d1f7/$CUDAB_WHEEL
CUDAB_SHA=c7855c4868aabc0cfae28abbe83d56734bdfbd08f08fc234ac1912a12858bf49

PATHF_WHEEL=cuda_pathfinder-1.8.0-py3-none-any.whl
PATHF_URL=https://files.pythonhosted.org/packages/a1/b1/ef21259ec74fe0b265ed201379de1d0ef7c14178313ee03705952f1b7093/$PATHF_WHEEL
PATHF_SHA=c44e574dc997fae2814721d1ae97d0fd6db76db82decbe9b753bf75de53f515e

# --- the models -------------------------------------------------------------
#
# Pinned by size, which is what catches the failure this actually has: a wi-fi
# link that drops mid-fetch leaves a short file, and a truncated ONNX graph does
# not announce itself as truncated -- the parser reports a protobuf error that
# reads like a corrupt model rather than a partial download.
#
# Two sets, because the two backends cannot share one. The int8 graphs are what
# onnxruntime runs; they are *dynamically* quantized, computing their scales from
# each activation, which TensorRT cannot parse at all -- it rejects 122 nodes in
# DINOv2 and 254 in SigLIP2. The fp16 exports are what the engines are built
# from. Their weights are rounded but the engines compute in fp32, which is what
# onnxruntime was silently doing with them anyway.
# **The one model here that is not fetched, because there is nothing to fetch.**
# The published YOLOE export ends in 4,585 class scores per anchor and this rover
# wants their maximum and nothing else: taken in numpy that costs 32 ms and a
# 99 MB copy off the GPU on every look, and taken inside the graph it costs
# nothing at all. `export_regions.py`, run on a workstation that has ultralytics,
# does the export and the fold and leaves a file to copy into vendor/ here. So
# this one is checked for rather than downloaded, and the message says where it
# comes from.
REGIONS_FILE=yoloe-11s-seg-objectness.onnx
REGIONS_LEAST_BYTES=40000000

DINO_FILE=dinov2-small-int8.onnx
DINO_URL=https://huggingface.co/onnx-community/dinov2-small/resolve/main/onnx/model_int8.onnx
DINO_BYTES=24446700

# **patch32 at 256, not patch16 at 224, and that was measured rather than
# assumed.** The patch16 model is the obvious choice and it is the worse one
# here: 66 ms a crop against 24, and on the rover's own living-room frame it
# called the spray bottle a cardboard box and the armchair a sofa, where patch32
# named the spray bottle, the armchair and the framed picture correctly. Sixty-four
# patch tokens against a hundred and ninety-six, and the crops are small.
SIGLIP_FILE=siglip2-base-patch32-256-int8.onnx
SIGLIP_URL=https://huggingface.co/onnx-community/siglip2-base-patch32-256-ONNX/resolve/main/onnx/model_int8.onnx
SIGLIP_BYTES=379362771

TOKENIZER_FILE=siglip2-tokenizer.json
TOKENIZER_URL=https://huggingface.co/onnx-community/siglip2-base-patch32-256-ONNX/resolve/main/tokenizer.json
TOKENIZER_BYTES=34363039

# The engine sources. SigLIP2 is fetched as two towers rather than as the
# combined graph: only the vision half runs per look, and the text half runs
# only when somebody types a search and is let go again straight after.
DINO_FP16_FILE=dinov2-small-fp16.onnx
DINO_FP16_URL=https://huggingface.co/onnx-community/dinov2-small/resolve/main/onnx/model_fp16.onnx
DINO_FP16_BYTES=44420939

VISION_FP16_FILE=siglip2-vision-fp16.onnx
VISION_FP16_URL=https://huggingface.co/onnx-community/siglip2-base-patch32-256-ONNX/resolve/main/onnx/vision_model_fp16.onnx
VISION_FP16_BYTES=189374263

TEXT_FP16_FILE=siglip2-text-fp16.onnx
TEXT_FP16_URL=https://huggingface.co/onnx-community/siglip2-base-patch32-256-ONNX/resolve/main/onnx/text_model_fp16.onnx
TEXT_FP16_BYTES=564862230

mkdir -p "$VENDOR"

fetch() {
    # fetch <url> <path> <expected bytes>
    url=$1; path=$2; want=$3
    if [ -z "$FORCE" ] && [ -f "$path" ]; then
        have=$(wc -c < "$path" | tr -d ' ')
        if [ "$have" = "$want" ]; then
            echo "have $(basename "$path") already"
            return 0
        fi
        echo "$(basename "$path") is $have bytes, wanted $want -- fetching the rest"
    fi
    # -C - resumes a part-fetched file, which is most of why this is worth
    # re-running rather than starting again.
    curl -fL --retry 5 --retry-delay 5 -C - -o "$path" "$url"
    have=$(wc -c < "$path" | tr -d ' ')
    if [ "$have" != "$want" ]; then
        echo "$(basename "$path") came back $have bytes, expected $want" >&2
        exit 1
    fi
}

require_exported() {
    # require_exported <path> <least bytes> -- for the model no URL serves
    path=$1; least=$2
    have=0
    [ -f "$path" ] && have=$(wc -c < "$path" | tr -d ' ')
    if [ "$have" -lt "$least" ]; then
        echo "no $(basename "$path") in $VENDOR ($have bytes)." >&2
        echo "It is exported rather than downloaded. On a workstation with" >&2
        echo "ultralytics installed:" >&2
        echo "    python world_state/export_regions.py" >&2
        echo "    scp $(basename "$path") orin:~/ugv/world_state/vendor/" >&2
        exit 1
    fi
    echo "have $(basename "$path") already"
}

unpack_wheel() {
    # unpack_wheel <url> <name> <sha256> <import name>
    url=$1; name=$2; sha=$3; module=$4
    if [ -z "$FORCE" ] && PYTHONPATH="$VENDOR" python3 -c "import $module" 2>/dev/null; then
        echo "$module is already unpacked at $VENDOR"
        return 0
    fi
    echo "--- fetching $name"
    curl -fL --retry 5 --retry-delay 5 -o "/tmp/$name" "$url"
    got=$(sha256sum "/tmp/$name" | cut -d' ' -f1)
    if [ "$got" != "$sha" ]; then
        echo "$name hashed $got, expected $sha" >&2
        rm -f "/tmp/$name"
        exit 1
    fi
    # A wheel is a zip. Unpacking it rather than installing it is the same
    # arrangement OpenCV has here and for the same reason: this host's Python is
    # externally managed, and sudo wants a password no deploy script has.
    python3 -c "import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" \
        "/tmp/$name" "$VENDOR"
    rm -f "/tmp/$name"
}

if [ -z "$ENGINES_ONLY" ]; then
    unpack_wheel "$ORT_URL"   "$ORT_WHEEL"   "$ORT_SHA"   onnxruntime
    unpack_wheel "$TOK_URL"   "$TOK_WHEEL"   "$TOK_SHA"   tokenizers
    unpack_wheel "$PATHF_URL" "$PATHF_WHEEL" "$PATHF_SHA" cuda.pathfinder
    unpack_wheel "$CUDAB_URL" "$CUDAB_WHEEL" "$CUDAB_SHA" cuda.bindings.runtime

    require_exported "$VENDOR/$REGIONS_FILE" "$REGIONS_LEAST_BYTES"
    fetch "$DINO_URL"        "$VENDOR/$DINO_FILE"        "$DINO_BYTES"
    fetch "$SIGLIP_URL"      "$VENDOR/$SIGLIP_FILE"      "$SIGLIP_BYTES"
    fetch "$TOKENIZER_URL"   "$VENDOR/$TOKENIZER_FILE"   "$TOKENIZER_BYTES"
    fetch "$DINO_FP16_URL"   "$VENDOR/$DINO_FP16_FILE"   "$DINO_FP16_BYTES"
    fetch "$VISION_FP16_URL" "$VENDOR/$VISION_FP16_FILE" "$VISION_FP16_BYTES"
    fetch "$TEXT_FP16_URL"   "$VENDOR/$TEXT_FP16_FILE"   "$TEXT_FP16_BYTES"
fi

# --- the engines ------------------------------------------------------------
#
# **This is the slow part and it needs the board largely to itself.** Building an
# engine means trying many kernel implementations and keeping their workspaces,
# and the first attempt at it here ended with the kernel's out-of-memory killer
# taking the build script: a language model was holding 3.1 GB of the board's
# 7.5 at the time. That model has since been removed from the rover altogether,
# which is where the headroom now comes from -- so nothing is stopped here any
# more, and anything else large that is running is the operator's to consider.
#
# About ten minutes, once. An engine is compiled for this GPU, this TensorRT
# version and these batch sizes, so it cannot be shipped and it does not survive
# a JetPack upgrade; when it stops loading, run this again with --engines.
build_engines() {
    if [ ! -x "$TRTEXEC" ]; then
        echo "no trtexec at $TRTEXEC -- no GPU path on this host, the CPU one" \
             "will be used instead. Install it with: sudo apt install nvidia-jetpack"
        return 0
    fi

    build() {
        # build <engine> <onnx> <extra trtexec arguments...>
        engine=$1; onnx=$2; shift 2
        if [ -z "$FORCE" ] && [ -s "$VENDOR/$engine" ]; then
            echo "have $engine already"
            return 0
        fi
        echo "--- building $engine (minutes)"
        "$TRTEXEC" --onnx="$VENDOR/$onnx" --saveEngine="$VENDOR/$engine.part" \
            --memPoolSize=workspace:1024 --skipInference "$@" > "/tmp/$engine.log" 2>&1 \
            || { echo "$engine failed to build; see /tmp/$engine.log" >&2
                 grep -iE "error|failed" "/tmp/$engine.log" | head -5 >&2
                 rm -f "$VENDOR/$engine.part"
                 return 1; }
        mv "$VENDOR/$engine.part" "$VENDOR/$engine"
        grep -E "Engine built" "/tmp/$engine.log" || true
    }

    # fp16 for the region finder only. Its boxes in fp16 match the CPU's to a
    # mean overlap of 0.998, which is all a box is asked for, and it is the one
    # model that runs on the whole frame rather than on crops.
    #
    # **Full precision for the other three, and that is not caution.** In genuine
    # fp16 SigLIP2 collapses: measured here, fifty-seven phrases through the text
    # tower came back within 0.92 of one another, so every phrase matched
    # everything. onnxruntime hides this because it has no fp16 kernels and
    # upcasts; a GPU does not.
    # No --shapes: this export is built for one input size, and trtexec
    # refuses explicit shapes for a graph that already states its own.
    build yoloe.plan "$REGIONS_FILE" --fp16 || return 1
    build dinov2.plan "$DINO_FP16_FILE" \
        --minShapes=pixel_values:1x3x224x224 \
        --optShapes=pixel_values:12x3x224x224 \
        --maxShapes=pixel_values:16x3x224x224 || return 1
    build siglip-vision.plan "$VISION_FP16_FILE" \
        --minShapes=pixel_values:1x3x256x256 \
        --optShapes=pixel_values:12x3x256x256 \
        --maxShapes=pixel_values:16x3x256x256 || return 1
    # One phrase is the whole of it now. This tower used to embed a word list
    # of fifty-seven at start-up; nothing names a region any more, so the only
    # thing that loads it is somebody typing a search, and that is one phrase.
    # The ceiling stays where it was because /embed accepts up to sixty-four.
    build siglip-text.plan "$TEXT_FP16_FILE" \
        --minShapes=input_ids:1x64 \
        --optShapes=input_ids:1x64 \
        --maxShapes=input_ids:128x64 || return 1
}

engine_status=ok
build_engines || engine_status="failed -- the CPU backend will be used"

# The sidecar comes back after a reboot the way every other service on this rover
# does: a crontab entry for the rover user.
LINE="@reboot $HERE/run_perception.sh"
current=$(crontab -l 2>/dev/null || true)
if printf '%s\n' "$current" | grep -q 'world_state/run_perception.sh'; then
    echo "crontab already starts the perception sidecar"
else
    if [ -n "$current" ]; then
        printf '%s\n%s\n' "$current" "$LINE" | crontab -
    else
        printf '%s\n' "$LINE" | crontab -
    fi
    echo "added: $LINE"
fi
sync

echo "--- what is installed"
echo "engines: $engine_status"
PYTHONPATH="$VENDOR" python3 - <<'PY'
try:
    import onnxruntime
    print("onnxruntime", onnxruntime.__version__, onnxruntime.get_available_providers())
except Exception as error:
    print("onnxruntime is not usable:", error)
try:
    import tokenizers
    print("tokenizers ", tokenizers.__version__)
except Exception as error:
    print("tokenizers is not usable:", error)
try:
    import tensorrt
    from cuda.bindings import runtime
    error, count = runtime.cudaGetDeviceCount()
    error, props = runtime.cudaGetDeviceProperties(0)
    print("tensorrt  ", tensorrt.__version__,
          f"on {props.name.decode()} sm_{props.major}{props.minor}")
except Exception as error:
    print("no GPU path:", error)
PY
ls -l "$VENDOR"/*.plan 2>/dev/null || echo "no engines built"
