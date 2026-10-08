# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate tiny deterministic test models (requires Python onnx).

These are test functions, not learned robot policies. No Python dependency is
needed to run the C++ tests against the checked-in ONNX files.
"""
from pathlib import Path

import onnx
from onnx import helper as h
from onnx import TensorProto as T

HERE = Path(__file__).parent


def tensor(name, values):
    return h.make_tensor(name, T.INT64, [len(values)], values)


def save(name, inputs, outputs, nodes, initializers=()):
    model = h.make_model(
        h.make_graph(nodes, name, inputs, outputs, list(initializers)),
        opset_imports=[h.make_opsetid('', 13)], ir_version=8)
    onnx.checker.check_model(model)
    onnx.save(model, HERE / (name + '.onnx'))


obs = h.make_tensor_value_info('obs', T.FLOAT, [1, 21])
action = h.make_tensor_value_info('continuous_actions', T.FLOAT, [1, 2])
save('specialist', [obs], [action],
     [h.make_node('Slice', ['obs', 'start', 'end', 'axis'], ['continuous_actions'])],
     [tensor('start', [14]), tensor('end', [16]), tensor('axis', [1])])
save('adapter', [obs, h.make_tensor_value_info('history', T.FLOAT, [1, 12, 4])],
     [action], [h.make_node('Reshape', ['history', 'shape'], ['flat']),
                h.make_node('Gather', ['flat', 'indices'], ['continuous_actions'], axis=1)],
     [tensor('shape', [1, 48]), tensor('indices', [43, 47])])
save('rejected', [obs], [action],
     [h.make_node('Constant', [], ['continuous_actions'],
                  value=h.make_tensor('invalid', T.FLOAT, [1, 2], [100, 100]))])
save('velocity', [h.make_tensor_value_info('obs', T.FLOAT, [1, 132]),
                  h.make_tensor_value_info('velocity_history', T.FLOAT, [1, 20, 75])],
     [h.make_tensor_value_info('estimated_velocity', T.FLOAT, [1, 3]),
      h.make_tensor_value_info('continuous_actions', T.FLOAT, [1, 23])],
     [h.make_node('Constant', [], ['estimated_velocity'],
                  value=h.make_tensor('diagnostic', T.FLOAT, [1, 3], [99, 98, 97])),
      h.make_node('Constant', [], ['continuous_actions'],
                  value=h.make_tensor('actions', T.FLOAT, [1, 23], [0.125] * 23))])
