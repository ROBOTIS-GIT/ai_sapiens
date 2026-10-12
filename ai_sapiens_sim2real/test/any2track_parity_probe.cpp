// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <fstream>
#include <iostream>
#include <cstring>
#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"
using namespace ai_sapiens_sim2real;  // NOLINT

int main(int argc, char ** argv)
{
  try {
    if (argc != 3) {throw std::runtime_error("Usage: any2track_parity_probe BUNDLE ORACLE.bin");}
    const std::filesystem::path b = argv[1];
    Sim2RealConfig cfg(b / "params/sim2real.yaml");
    MimicBehavior mimic;
    mimic.motion_file = b / "params/redred_k1.csv";
    Any2TrackPolicy core(cfg, mimic, b / "exported/policy.onnx");
    OnnxInference network((b / "exported/policy.onnx").string(), true);
    std::ifstream in(argv[2], std::ios::binary);
    char magic[8]; uint32_t count;
    in.read(magic, 8); in.read(reinterpret_cast<char *>(&count), 4);
    if (!in || std::memcmp(magic, "A2TPAR01", 8)) {throw std::runtime_error("Invalid oracle");}
    float max_input = 0, max_action = 0;
    for (uint32_t step = 0; step < count; ++step) {
      uint32_t reset, align;
      std::vector<float> row(7660);
      in.read(reinterpret_cast<char *>(&reset), 4); in.read(reinterpret_cast<char *>(&align), 4);
      in.read(reinterpret_cast<char *>(row.data()), row.size() * sizeof(float));
      if (!in) {throw std::runtime_error("Truncated oracle");}
      Any2TrackSample s;
      s.xy = Eigen::Vector2f(row[0], row[1]);
      s.orientation = Eigen::Quaternionf(row[2], row[3], row[4], row[5]);
      s.gyro = Eigen::Vector3f(row[6], row[7], row[8]);
      for (int j = 0; j < 23; ++j) {
        s.position[j] = row[9 + j]; s.velocity[j] = row[32 + j];
      }
      if (reset) {core.reset(s, align);}
      if (!core.observe(s)) {throw std::runtime_error("Premature motion completion");}
      size_t offset = 55;
      const auto compare = [&](const std::vector<float> & actual, float tolerance, float & worst,
        const std::string & label) {
          for (size_t j = 0; j < actual.size(); ++j) {
            const float error = std::abs(actual[j] - row[offset + j]);
            worst = std::max(worst, error);
            if (!std::isfinite(error) || error > tolerance) {
              throw std::runtime_error(label + " mismatch at step " + std::to_string(step) +
                " index " + std::to_string(j) + " error " + std::to_string(error));
            }
          }
          offset += actual.size();
        };
      for (const auto * name : {"obs", "velocity_history", "history"}) {
        compare(core.inputs().at(name), 2e-5f, max_input, name);
      }
      const auto action = network.run(core.inputs());
      compare(action, 2e-5f, max_action, "action");
      const auto target = core.target(action);
      compare(target, 2e-5f, max_action, "target");
      core.accept(target); core.advance();
    }
    std::cout << "PARITY steps=" << count << " max_input_error=" << max_input
              << " max_action_error=" << max_action << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
