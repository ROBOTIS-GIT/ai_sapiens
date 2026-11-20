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

"""Generate a tiny policy with deterministic runtime failures (requires onnx)."""
from pathlib import Path

import onnx
from onnx import helper, TensorProto

# Index 0 produces a normal action, index 1 an out-of-limit action, and
# index 2 throws an ONNX Gather error. The live observation selects the index.
graph = helper.make_graph(
    [helper.make_node('Cast', ['obs'], ['indices'], to=TensorProto.INT64),
     helper.make_node('Gather', ['values', 'indices'], ['action'], axis=0)],
    'inference_error_test',
    [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 2])],
    [helper.make_tensor_value_info('action', TensorProto.FLOAT, [1, 2])],
    [helper.make_tensor('values', TensorProto.FLOAT, [2], [0.4, 20.0])],
)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 13)])
model.ir_version = 8
onnx.checker.check_model(model)
onnx.save(model, Path(__file__).with_name('policy.onnx'))
