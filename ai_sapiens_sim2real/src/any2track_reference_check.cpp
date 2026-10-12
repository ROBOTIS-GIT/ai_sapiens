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

// Offline ground-truth reference check, not the odometry-based sim2sim entrypoint.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>
#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"
#include "ai_sapiens_sim2real/policy/onnx_inference.hpp"

using namespace ai_sapiens_sim2real;  // NOLINT

int main(int argc, char ** argv)
{
  try {
    if (argc < 3 || argc > 5) {
      std::cerr << "Usage: any2track_reference_check BUNDLE SCENE.xml [STEPS=full] [ROLLOUT.csv]\n";
      return 2;
    }
    std::cout << "OFFLINE REFERENCE CHECK: XY comes from MuJoCo ground truth. "
              << "For pelvis-odometry sim2sim use redred_any2track.launch.py.\n";
    const std::filesystem::path bundle = std::filesystem::absolute(argv[1]);
    Sim2RealConfig cfg(bundle / "params/sim2real.yaml");
    MimicBehavior mimic;
    mimic.motion_file = bundle / "params/redred_k1.csv";
    Any2TrackPolicy core(cfg, mimic, bundle / "exported/policy.onnx");
    OnnxInference network((bundle / "exported/policy.onnx").string(), true);
    char error[2048]{};
    std::unique_ptr<mjModel, decltype(& mj_deleteModel)> model(
      mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {throw std::runtime_error(error);}
    if (model->nq != 30 || model->nv != 29 || model->nu != 23 ||
      model->jnt_type[0] != mjJNT_FREE)
    {
      throw std::runtime_error("Expected free-base K1 with 23 torque actuators");
    }
    // Match this training task's nominal physics settings, without domain randomization.
    model->opt.timestep = 0.005;
    model->opt.integrator = mjINT_EULER;
    model->opt.solver = mjSOL_NEWTON;
    model->opt.iterations = 10;
    model->opt.cone = mjCONE_PYRAMIDAL;
    std::unique_ptr<mjData, decltype(& mj_deleteData)> data(mj_makeData(model.get()),
      mj_deleteData);
    std::array<int, 23> qadr{}, vadr{}, actuator{};
    for (int j = 0; j < 23; ++j) {
      const int id = mj_name2id(model.get(), mjOBJ_JOINT, cfg.policy_joints()[j].c_str());
      if (id < 1) {throw std::runtime_error("Missing policy joint");}
      qadr[j] = model->jnt_qposadr[id]; vadr[j] = model->jnt_dofadr[id];
      actuator[j] = -1;
      for (int a = 0; a < 23; ++a) {
        if (model->actuator_trntype[a] == mjTRN_JOINT && model->actuator_trnid[2 * a] == id) {
          actuator[j] = a;
        }
      }
      if (actuator[j] < 0 || model->actuator_gaintype[actuator[j]] != mjGAIN_FIXED ||
        model->actuator_biastype[actuator[j]] != mjBIAS_NONE ||
        model->actuator_gainprm[actuator[j] * mjNGAIN] != 1 ||
        model->actuator_gear[actuator[j] * 6] != 1)
      {
        throw std::runtime_error("Expected direct unit-gear torque motor for every joint");
      }
    }
    const auto & initial = core.frame(core.start_frame());
    for (int j = 0; j < 7; ++j) {
      data->qpos[j] = initial[j];
    }
    for (int j = 0; j < 6; ++j) {
      data->qvel[j] = initial[30 + j];
    }
    for (int j = 0; j < 23; ++j) {
      data->qpos[qadr[j]] = initial[7 + j]; data->qvel[vadr[j]] = initial[36 + j];
    }
    mj_forward(model.get(), data.get());
    const auto sample = [&]() {
        Any2TrackSample s;
        s.xy = Eigen::Vector2f(data->qpos[0], data->qpos[1]);
        s.orientation = Eigen::Quaternionf(data->qpos[3], data->qpos[4], data->qpos[5],
        data->qpos[6]);
        for (int j = 0; j < 3; ++j) {
          s.gyro[j] = data->qvel[3 + j];
        }
        for (int j = 0; j < 23; ++j) {
          s.position[j] = data->qpos[qadr[j]]; s.velocity[j] = data->qvel[vadr[j]];
        }
        return s;
      };
    core.reset(sample(), false);
    const auto raw_config = YAML::LoadFile((bundle / "params/sim2real.yaml").string());
    const auto limits = raw_config["torque_limits"].as<std::vector<double>>();
    if (limits.size() != 23 || !std::all_of(limits.begin(), limits.end(),
      [](double v) {return std::isfinite(v) && v > 0;}))
    {throw std::runtime_error("Invalid torque limits");}
    const auto & gains = cfg.joint_properties();
    const size_t count = argc >= 4 ? std::stoul(argv[3]) : core.end_frame() - core.start_frame();
    std::ofstream rollout;
    if (argc >= 5) {
      rollout.open(argv[4]);
      if (!rollout) {throw std::runtime_error("Cannot create rollout CSV");}
      rollout << std::setprecision(10);
      rollout << "time";
      for (int j = 0; j < model->nq; ++j) {
        rollout << ",qpos_" << j;
      }
      rollout << '\n';
    }
    double max_error = 0, sum_sq_error = 0, min_height = data->qpos[2];
    size_t steps = 0;
    while (steps < count && core.observe(sample())) {
      const auto action = network.run(core.inputs());
      const auto target = core.target(action);
      for (int substep = 0; substep < 4; ++substep) {
        for (int j = 0; j < 23; ++j) {
          const double torque = gains.stiffness[j] * (target[j] - data->qpos[qadr[j]]) -
            gains.damping[j] * data->qvel[vadr[j]];
          data->ctrl[actuator[j]] = std::clamp(torque, -limits[j], limits[j]);
        }
        mj_step(model.get(), data.get());
      }
      core.accept(target);
      const auto & f = core.frame(core.cursor());
      const double distance = std::hypot(data->qpos[0] - f[0], data->qpos[1] - f[1]);
      max_error = std::max(max_error, distance); sum_sq_error += distance * distance;
      min_height = std::min(min_height, data->qpos[2]);
      if (rollout.is_open()) {
        rollout << data->time;
        for (int j = 0; j < model->nq; ++j) {
          rollout << ',' << data->qpos[j];
        }
        rollout << '\n';
      }
      if (!std::all_of(data->qpos, data->qpos + model->nq, [](double v) {
          return std::isfinite(v);
        }) ||
        data->qpos[2] < 0.3)
      {
        throw std::runtime_error("Rollout fell/nonfinite at step " + std::to_string(steps));
      }
      core.advance(); ++steps;
    }
    std::cout << "ROLLOUT steps=" << steps << " seconds=" << steps * cfg.step_dt()
              << " xy_rmse=" << std::sqrt(sum_sq_error / std::max<size_t>(1, steps))
              << " xy_max=" << max_error << " min_pelvis_height=" << min_height << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Any2Track reference check: " << e.what() << '\n';
    return 1;
  }
}
