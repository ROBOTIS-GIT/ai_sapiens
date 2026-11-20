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

"""Generate a policy whose output depends on all four observation frames."""
from pathlib import Path

import onnx
from onnx import helper, TensorProto

weights = [1.0 if i % 2 == j else 0.0 for i in range(8) for j in range(2)]
graph = helper.make_graph(
    [helper.make_node('MatMul', ['obs', 'weights'], ['action'])],
    'four_frame_history_test',
    [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 8])],
    [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 2])],
    [helper.make_tensor('weights', TensorProto.FLOAT, [8, 2], weights)],
)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
model.ir_version = 8
onnx.checker.check_model(model)
onnx.save(model, Path(__file__).with_name('policy.onnx'))
