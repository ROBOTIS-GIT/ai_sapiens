# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Regenerate tiny Identity graphs for metadata tests (requires Python onnx)."""

from pathlib import Path

import onnx
from onnx import helper, TensorProto

directory = Path(__file__).resolve().parent
model = helper.make_model(helper.make_graph(
    [helper.make_node('Identity', ['obs'], ['actions'])], 'metadata_test',
    [helper.make_tensor_value_info('obs', TensorProto.FLOAT, [1, 4])],
    [helper.make_tensor_value_info('actions', TensorProto.FLOAT, [1, 4])]),
    ir_version=8, opset_imports=[helper.make_opsetid('', 13)])
onnx.save(model, directory / 'no_metadata.onnx')
helper.set_model_props(model, {
    'joint_names': 'hip,waist',
    'observation_names': 'robot_root_position_xy_w,reference_root_position_xy_w',
})
onnx.save(model, directory / 'position_metadata.onnx')
