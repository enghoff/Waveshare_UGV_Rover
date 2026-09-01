#!/bin/sh
# Put the perception models on the rover: regions, appearance and semantics.
#
#     ssh orin '~/ugv/world_state/install_perception.sh'          # fetch what is missing
#     ssh orin '~/ugv/world_state/install_perception.sh --force'  # fetch it all again
#
# This is the half of the world state that replaced asking a language model which
# lasting thing it was looking at. Three small networks, none of which knows any
# categories, and between them they answer three separate questions:
#
#     FastSAM-s     what regions are in this frame, with no vocabulary at all
#     DINOv2-small  is this the same *instance* as that one
#     SigLIP2       what is this called, and later: find me the thing I describe
#
# **Separate from install.sh on purpose.** That one fetches Cosmos Reason 2, which
# is still wanted for the conversational `look` where a person is waiting for
# prose. This one is the per-look path, and the two have to be installable and
# breakable independently.
#
# Half a gigabyte, fetched here rather than deployed, for the reason the GGUFs
# are: the deployer carries the bytes a commit describes, and a quantized ONNX
# graph is neither described by a commit nor worth sending on every change.
# vendor/ is preserved across deploys by deploy/manifest.json.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
VENDOR="$HERE/vendor"
FORCE=""
[ "${1:-}" = "--force" ] && FORCE=1

# --- the runtime ------------------------------------------------------------
#
# ONNX Runtime rather than PyTorch: twenty megabytes against two or three
# gigabytes, for the same three forward passes. **The CPU provider is what runs
# here**, and that is a measured limitation rather than a preference -- this
# JetPack has the GPU driver but no CUDA toolkit and no cuDNN, so onnxruntime has
# no GPU provider to offer. See world_state/README.md for what that costs.
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

# --- the models -------------------------------------------------------------
#
# Pinned by size, which is what catches the failure this actually has: a wi-fi
# link that drops mid-fetch leaves a short file, and a truncated ONNX graph does
# not announce itself as truncated -- onnxruntime reports a protobuf parse error
# that reads like a corrupt model rather than a partial download.
#
# **int8 for the two big ones, and it is not a compromise.** Measured on the
# rover's own frames on a desktop: int8 SigLIP2 ranks the same words as fp32 in
# the same order at half the cost, and int8 DINOv2 is 1.7x faster than fp32. The
# doc's own warning that SigLIP's raw cosines are uncalibrated applies equally to
# both, and only the ranking is used.
FASTSAM_FILE=FastSAM-s.onnx
FASTSAM_URL=https://huggingface.co/SpatialHub/fastsam-onnx/resolve/main/FastSAM-s.onnx
FASTSAM_BYTES=47251207

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

unpack_wheel "$ORT_URL" "$ORT_WHEEL" "$ORT_SHA" onnxruntime
unpack_wheel "$TOK_URL" "$TOK_WHEEL" "$TOK_SHA" tokenizers

fetch "$FASTSAM_URL"   "$VENDOR/$FASTSAM_FILE"   "$FASTSAM_BYTES"
fetch "$DINO_URL"      "$VENDOR/$DINO_FILE"      "$DINO_BYTES"
fetch "$SIGLIP_URL"    "$VENDOR/$SIGLIP_FILE"    "$SIGLIP_BYTES"
fetch "$TOKENIZER_URL" "$VENDOR/$TOKENIZER_FILE" "$TOKENIZER_BYTES"

# The sidecar comes back after a reboot the way every other service on this rover
# does: a crontab entry for the rover user. Its own line, separate from the
# language model's, so that stopping one does not stop the other.
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
PYTHONPATH="$VENDOR" python3 - <<'PY'
import onnxruntime, tokenizers
print("onnxruntime", onnxruntime.__version__, onnxruntime.get_available_providers())
print("tokenizers ", tokenizers.__version__)
PY
ls -l "$VENDOR/$FASTSAM_FILE" "$VENDOR/$DINO_FILE" "$VENDOR/$SIGLIP_FILE" \
      "$VENDOR/$TOKENIZER_FILE"
