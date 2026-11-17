# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regenerate the tiny deterministic policy runtime fixture (requires onnx)."""
from pathlib import Path

import onnx
from onnx import helper, TensorProto

# Read the newest joint observation from two-frame term-major history.
weights = [0.0] * (21 * 2)
weights[2 * 2] = 1.0
weights[3 * 2 + 1] = 1.0
graph = helper.make_graph(
    [helper.make_node('MatMul', ['obs', 'weights'], ['action'])],
    'action_fault_test',
    [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 21])],
    [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 2])],
    [helper.make_tensor('weights', TensorProto.FLOAT, [21, 2], weights)],
)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
model.ir_version = 8
onnx.checker.check_model(model)
onnx.save(model, Path(__file__).with_name('policy.onnx'))
