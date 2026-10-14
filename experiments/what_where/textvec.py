"""A phrase as SigLIP2's text vector, the way the rover's search makes it
(world_state/perceive.py: lower case, 64 tokens padded with 0), from the text
tower's fp16 ONNX export on the CPU.

    python textvec.py MODEL_DIR OUT.npy "a photo of a person" ...
"""
import os
import sys

import numpy as np


def vectors(model_dir, phrases):
    import onnxruntime
    from tokenizers import Tokenizer
    tok = Tokenizer.from_file(os.path.join(model_dir, "siglip2-tokenizer.json"))
    tok.enable_padding(length=64, pad_id=0, pad_token="<pad>")
    tok.enable_truncation(max_length=64)
    ids = np.array([e.ids for e in tok.encode_batch([p.lower() for p in phrases])], dtype=np.int64)
    sess = onnxruntime.InferenceSession(os.path.join(model_dir, "siglip2-text-fp16.onnx"),
                                        providers=["CPUExecutionProvider"])
    names = [o.name for o in sess.get_outputs()]
    out = sess.run(None, {"input_ids": ids})
    pooled = out[names.index("pooler_output")] if "pooler_output" in names else out[-1]
    pooled = pooled.astype(np.float64)
    return pooled / np.linalg.norm(pooled, axis=1, keepdims=True)


if __name__ == "__main__":
    v = vectors(sys.argv[1], sys.argv[3:])
    np.save(sys.argv[2], v)
    print(v.shape)
