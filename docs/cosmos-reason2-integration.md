# Cosmos integration decision

Status: closed on 2026-09-02. Nothing described here runs on the rover.

Cosmos Reason 2 2B was tested locally as a vision-language sidecar for semantic
inspection and persistent identity. It was removed, including its weights and
supervisor, after real-frame tests showed that it was unsuitable for this role.

The decisive results were:

- an inspection took about 60 seconds rather than a fraction of a second;
- names drifted on byte-identical frames;
- the model failed to recognize previously seen objects reliably;
- a larger Cosmos variant recognized objects that were not present;
- the unused sidecar continued to reserve several gigabytes of shared memory.

The replacement is the current `world_state` pipeline: YOLOE regions, DINOv2 and
SigLIP2 vectors, identity constrained by pose, bearings, map visibility,
elevation and optional OAK range. Alibaba Qwen Omni remains the conversational
model and uses the rover's controlled tool boundary.

See [`world_state/README.md`](../world_state/README.md) for current operation and
[`task-semantic-world-state.md`](task-semantic-world-state.md) for the remaining
acceptance work. Detailed Cosmos experiments remain in Git history; reviving a
local VLM should start from new evidence that addresses the failures above.
