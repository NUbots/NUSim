"""Regenerate random_policy.onnx: a tiny deterministic *untrained* MLP matching the
PolicyBackend graph contract (docs/OBS_ACTION_CONTRACT.md) — obs[1,82] -> Gemm(82,64)
-> Relu -> Gemm(64,22) -> Tanh -> continuous_actions[1,22]. Smoke-tests the ONNX
plumbing only (test_locomotion.cpp's K1_WITH_ONNX case); not a locomotion policy.

Run from this directory with any python that has `onnx` and `numpy`:
    python make_random_policy.py
"""
import numpy as np
import onnx
from onnx import TensorProto, helper

OBS_DIM = 82
ACT_DIM = 22
HIDDEN = 64

rng = np.random.default_rng(1234)
w1 = rng.normal(0.0, 0.1, size=(OBS_DIM, HIDDEN)).astype(np.float32)
b1 = np.zeros(HIDDEN, dtype=np.float32)
w2 = rng.normal(0.0, 0.1, size=(HIDDEN, ACT_DIM)).astype(np.float32)
b2 = np.zeros(ACT_DIM, dtype=np.float32)

graph = helper.make_graph(
    nodes=[
        helper.make_node("Gemm", ["obs", "w1", "b1"], ["h1"]),
        helper.make_node("Relu", ["h1"], ["h1_relu"]),
        helper.make_node("Gemm", ["h1_relu", "w2", "b2"], ["logits"]),
        helper.make_node("Tanh", ["logits"], ["continuous_actions"]),
    ],
    name="random_policy",
    inputs=[helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, OBS_DIM])],
    outputs=[helper.make_tensor_value_info("continuous_actions", TensorProto.FLOAT, [1, ACT_DIM])],
    initializer=[
        helper.make_tensor("w1", TensorProto.FLOAT, w1.shape, w1.tobytes(), raw=True),
        helper.make_tensor("b1", TensorProto.FLOAT, b1.shape, b1.tobytes(), raw=True),
        helper.make_tensor("w2", TensorProto.FLOAT, w2.shape, w2.tobytes(), raw=True),
        helper.make_tensor("b2", TensorProto.FLOAT, b2.shape, b2.tobytes(), raw=True),
    ],
)

model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
model.ir_version = 8
onnx.checker.check_model(model)
onnx.save(model, "random_policy.onnx")
print("wrote random_policy.onnx")
