# DINOv2 feeding back into the region mask

Whether letting DINOv2's patch tokens and YOLOE's mask settle on one answer,
instead of passing one way, makes the appearance vector better at identity.
**Rejected on 2026-10-19**: see
[the progress entry](../../docs/progress/2026-10-19-dino-feeding-back-into-the-mask.md)
and [the decision](../../docs/decisions/dino-does-not-feed-back-into-the-mask.md).
The bar was committed before scoring, in [PREREGISTRATION.md](PREREGISTRATION.md).

| File | What it is |
|---|---|
| [settle.py](settle.py) | the loop: object and background fitted in patch-token space, mask as prior |
| [replay.py](replay.py) | re-runs YOLOE and DINOv2 over the 412 labelled looks, keeping all 257 tokens |
| [score.py](score.py) | the two tests and the bar; prints the verdict |
| [draw.py](draw.py) | the mask and the settled assignment drawn over their crops |
| [results.txt](results.txt) | what `score.py` printed |

Everything runs on this desktop from copied data; nothing touches the rover.

```powershell
python experiments/dino_feedback/replay.py --out <scratch>/cache_looks.npz   # about 10 minutes on the CPU
python experiments/dino_feedback/score.py <scratch>/cache_looks.npz
python experiments/dino_feedback/draw.py <scratch>/cache_looks.npz <scratch>/sheet.jpg [look ids]
```

`replay.py` reads the frozen benchmark's manifest, the store
`captures/2026-10-03-merge-review/world_now.db`, the frames under `tmp/` and
`captures/`, the YOLOE export in `captures/2026-10-05-depth-abstention/` and
the fp16 DINOv2 export in `.cache/entity-reader-groups/`. It checks itself
against what the rover stored: summary vector 0.9999, masked vector 0.9987,
mask share 0.000 in the median.
