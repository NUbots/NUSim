#!/usr/bin/env python3
"""Export a trained brax-PPO actor to the deploy ONNX (obs[1,47] → action[1,12]).

    python training/export_onnx.py --checkpoint checkpoints/walk.pkl --out models/walk.onnx

Reconstructs the policy MLP from the saved brax params and emits a plain
Gemm/activation ONNX graph matching training/OBS_ACTION_CONTRACT.md, so the C++
PolicyBackend loads it directly. Drop the .onnx in, set config/locomotion.yaml
`backend: policy` + `policy.path`, and build the sim with -DK1_WITH_ONNX=ON.
"""
from __future__ import annotations

import argparse
import os
import pickle

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

# brax PPO squashes the actor output through tanh; the network emits mean+logstd,
# so the policy head width is 2*action_dim and we take the first half as the mean.
ACTION_DIM = 12
OBS_DIM = 47


def _dense_params(params):
    """Pull ordered (kernel, bias) pairs out of the brax policy network params."""
    # brax stores params as a FrozenDict-like: params[1] is the policy network,
    # with layers under 'params'/'hidden_0','hidden_1',.... We walk it generically.
    import jax

    policy_params = params[1] if isinstance(params, (tuple, list)) else params
    flat = {"/".join(map(str, k)): v for k, v in _flatten(policy_params)}
    # collect Dense_i kernel/bias in order
    layers = {}
    for key, val in flat.items():
        if key.endswith("kernel") or key.endswith("bias"):
            name = key.rsplit("/", 1)[0]
            layers.setdefault(name, {})[key.rsplit("/", 1)[1]] = np.asarray(val)
    ordered = [layers[n] for n in sorted(layers, key=lambda s: (len(s), s))]
    return [(l["kernel"], l["bias"]) for l in ordered if "kernel" in l and "bias" in l]


def _flatten(tree, prefix=()):
    import jax

    if hasattr(tree, "items"):
        for k, v in tree.items():
            yield from _flatten(v, prefix + (k,))
    else:
        yield prefix, tree


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default="checkpoints/walk.pkl")
    ap.add_argument("--out", default="models/walk.onnx")
    args = ap.parse_args()

    import onnx
    from onnx import TensorProto, helper

    ckpt = args.checkpoint if os.path.isabs(args.checkpoint) else os.path.join(_HERE, args.checkpoint)
    with open(ckpt, "rb") as f:
        blob = pickle.load(f)
    params = blob["params"]

    dense = _dense_params(params)
    if not dense:
        raise SystemExit("could not extract MLP layers from checkpoint params")

    # Build an ONNX MLP: Gemm/Tanh per hidden layer, final Gemm → slice mean → Tanh.
    nodes, inits = [], []
    x = "obs"
    for i, (kernel, bias) in enumerate(dense):
        wname, bname = f"W{i}", f"B{i}"
        inits.append(helper.make_tensor(wname, TensorProto.FLOAT, kernel.shape, kernel.astype(np.float32).flatten()))
        inits.append(helper.make_tensor(bname, TensorProto.FLOAT, bias.shape, bias.astype(np.float32).flatten()))
        y = f"h{i}"
        nodes.append(helper.make_node("Gemm", [x, wname, bname], [y], name=f"gemm{i}"))
        if i < len(dense) - 1:
            a = f"a{i}"
            nodes.append(helper.make_node("Tanh", [y], [a], name=f"act{i}"))  # brax uses tanh hidden acts
            x = a
        else:
            x = y

    # final layer emits [mean(12), logstd(12)] → take mean, tanh-squash to [-1,1]
    nodes.append(helper.make_node("Slice", [x, "s_start", "s_end", "s_axis"], ["mean"], name="take_mean"))
    inits += [
        helper.make_tensor("s_start", TensorProto.INT64, [1], [0]),
        helper.make_tensor("s_end", TensorProto.INT64, [1], [ACTION_DIM]),
        helper.make_tensor("s_axis", TensorProto.INT64, [1], [1]),
    ]
    nodes.append(helper.make_node("Tanh", ["mean"], ["action"], name="squash"))

    graph = helper.make_graph(
        nodes, "k1_walk_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, OBS_DIM])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, ACTION_DIM])],
        initializer=inits,
    )
    model = helper.make_model(graph, producer_name="k1sim-train", opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    out = args.out if os.path.isabs(args.out) else os.path.join(_HERE, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    onnx.save(model, out)

    # sanity: run it
    import onnxruntime as ort

    sess = ort.InferenceSession(out)
    y = sess.run(["action"], {"obs": np.zeros((1, OBS_DIM), np.float32)})[0]
    assert y.shape == (1, ACTION_DIM) and np.isfinite(y).all()
    print(f"[export] wrote {out}  obs[1,{OBS_DIM}] → action[1,{ACTION_DIM}]  (verified)")


if __name__ == "__main__":
    main()
