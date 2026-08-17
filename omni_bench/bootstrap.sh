#!/usr/bin/env bash
# Everything the rented card needs, in one script, run once on arrival.
#
# It is written to be re-runnable and to log loudly, because the expensive
# failure here is not an error -- it is twenty minutes of a paid GPU spent
# watching a download that was never going to work. So the download goes first,
# in the background, and the packages install while it runs.
set -euo pipefail

MODEL="${1:-openbmb/MiniCPM-o-4_5}"
DEST="/workspace/$(basename "$MODEL")"

echo "=== $(date -Is) bootstrap for $MODEL"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
df -h /workspace | tail -1

export HF_HUB_ENABLE_HF_TRANSFER=1
export PIP_DISABLE_PIP_VERSION_CHECK=1

pip install -q hf_transfer "huggingface_hub[cli]" 2>&1 | tail -2

# The weights are the long pole: 20 GB for MiniCPM-o, 70 GB for Qwen3-Omni. Start
# it now and install packages underneath it.
mkdir -p "$DEST"
( hf download "$MODEL" --local-dir "$DEST" > /workspace/download.log 2>&1
  echo "DOWNLOAD_DONE $?" >> /workspace/download.log ) &
DOWNLOAD=$!

# MiniCPM-o's remote code imports more than transformers brings with it, and each
# missing one costs a model load to discover.
pip install -q \
    "transformers>=4.56" accelerate sentencepiece \
    librosa soundfile "numpy<2.3" \
    vocos vector_quantize_pytorch timm decord moviepy \
    2>&1 | tail -5

python -c "import torch, transformers; print('torch', torch.__version__, 'transformers', transformers.__version__, 'cuda', torch.cuda.is_available())"

echo "=== waiting on the download"
wait $DOWNLOAD || true
tail -2 /workspace/download.log
du -sh "$DEST"
echo "=== $(date -Is) bootstrap done"
