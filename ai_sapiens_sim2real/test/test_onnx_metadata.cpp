// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

namespace ai_sapiens_sim2real
{
namespace
{

const std::vector<std::string> kJoints{"hip", "waist"};
const std::vector<std::string> kObservations{
  "robot_root_position_xy_w", "reference_root_position_xy_w"};

TEST(OnnxMetadata, AcceptsMatchingSemanticOrder)
{
  OnnxInference inference(std::string(TEST_DATA_DIR) + "/position_metadata.onnx");
  EXPECT_NO_THROW(inference.validate_policy_metadata(kJoints, kObservations, true));
}

TEST(OnnxMetadata, RejectsSameSizeVelocitySchemaAndReorderedTerms)
{
  OnnxInference inference(std::string(TEST_DATA_DIR) + "/position_metadata.onnx");
  EXPECT_THROW(inference.validate_policy_metadata(kJoints,
    {"robot_root_velocity_xy_h", "reference_root_velocity_xy_h"}, true), std::runtime_error);
  EXPECT_THROW(inference.validate_policy_metadata(kJoints,
    {kObservations[1], kObservations[0]}, true), std::runtime_error);
  EXPECT_THROW(inference.validate_policy_metadata({"waist", "hip"}, kObservations, true),
    std::runtime_error);
}

TEST(OnnxMetadata, LegacyMayOmitMetadataButLocalizedMimicRequiresIt)
{
  OnnxInference inference(std::string(TEST_DATA_DIR) + "/no_metadata.onnx");
  EXPECT_NO_THROW(inference.validate_policy_metadata(kJoints, kObservations, false));
  EXPECT_THROW(inference.validate_policy_metadata(kJoints, kObservations, true),
        std::runtime_error);
}

}  // namespace
}  // namespace ai_sapiens_sim2real
