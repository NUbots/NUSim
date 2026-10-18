#!/usr/bin/env python3
"""Export an mjlab/AMP K1 velocity-policy ONNX (MLP) to the JSON the sim loads.

The sim runs the policy itself (module::Locomotion, WalkPolicy.hpp) with no ONNX
runtime dependency, so the graph must be a plain MLP: [Sub/Div obs normalizer] ->
Gemm (+Elu|Relu|Tanh)* -> Gemm. Anything else is refused rather than guessed at.

Observation (75): base_ang_vel(3) projected_gravity(3) joint_pos-default(22)
joint_vel(22) last_action(22) command[vx,vy,vyaw](3). Metadata (default pose, action
scale, kp/kd) is read from the ONNX metadata_props.

    python scripts/export_walk_policy.py model.onnx mujoco/models/policies/amp_walk.json
"""
import json
import os
import sys

import numpy as np
import onnx
from onnx import numpy_helper

ACTS = {"Elu": "elu", "Relu": "relu", "Tanh": "tanh"}


def csv(meta, key):
    return [float(x) for x in meta[key].split(",")]


def main(src, dst):
    m = onnx.load(src)
    meta = {p.key: p.value for p in m.metadata_props}
    init = {i.name: numpy_helper.to_array(i).astype(np.float64) for i in m.graph.initializer}
    consts = {}
    for n in m.graph.node:
        if n.op_type == "Constant":
            consts[n.output[0]] = numpy_helper.to_array(n.attribute[0].t).astype(np.float64)
    init.update(consts)

    mean = np.zeros(75)
    std = np.ones(75)
    layers = []
    cur = m.graph.input[0].name
    pending = None  # (W, b) of a Gemm waiting for its activation
    for n in m.graph.node:
        if n.op_type == "Sub" and n.input[0] == cur:
            mean = init[n.input[1]].reshape(-1)
            cur = n.output[0]
        elif n.op_type == "Div" and n.input[0] == cur:
            std = init[n.input[1]].reshape(-1)
            cur = n.output[0]
        elif n.op_type == "Gemm":
            assert n.input[0] == cur, "non-sequential graph"
            attrs = {a.name: onnx.helper.get_attribute_value(a) for a in n.attribute}
            W = init[n.input[1]]
            if attrs.get("transB", 0) == 0:
                W = W.T  # store as [out, in]
            assert attrs.get("alpha", 1.0) == 1.0 and attrs.get("beta", 1.0) == 1.0
            b = init[n.input[2]].reshape(-1)
            if pending:
                layers.append(dict(pending, act="none"))
            pending = dict(w=W.reshape(-1).tolist(), b=b.tolist(), n_in=W.shape[1], n_out=W.shape[0])
            cur = n.output[0]
        elif n.op_type in ACTS:
            assert n.input[0] == cur and pending
            layers.append(dict(pending, act=ACTS[n.op_type]))
            pending = None
            cur = n.output[0]
        else:
            sys.exit(f"unsupported op {n.op_type!r}: sim only runs plain MLPs")
    if pending:
        layers.append(dict(pending, act="none"))
    assert layers[0]["n_in"] == 75 and layers[-1]["n_out"] == 22, "expected 75-in/22-out"

    out = dict(
        source=os.path.basename(src),
        obs_mean=mean.tolist(),
        obs_std=std.tolist(),
        layers=layers,
        default_joint_pos=csv(meta, "default_joint_pos"),
        action_scale=csv(meta, "action_scale"),
        kp=csv(meta, "joint_stiffness"),
        kd=csv(meta, "joint_damping"),
    )
    with open(dst, "w") as f:
        json.dump(out, f, separators=(",", ":"))
    print(f"wrote {dst}: layers {[(l['n_in'], l['n_out'], l['act']) for l in layers]}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
