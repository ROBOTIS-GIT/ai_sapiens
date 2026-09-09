// Copyright 2026 ROBOTIS CO., LTD. Licensed under Apache-2.0.
#include "ai_sapiens_mujoco/mujoco_simulation.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
namespace ai_sapiens_mujoco {
void MujocoSimulation::cache_physics_settings() {
  for (const char * name : {"pelvis", "base_link", "base", "torso_link"}) {
    int id = mj_name2id(model_, mjOBJ_BODY, name);
    if (id > 0 && model_->body_mass[id] > 0) { payload_body_ = id; break; }
  }
  if (payload_body_ < 0) {
    for (int id=1; id<model_->nbody; ++id) {
      if (model_->body_parentid[id] == 0 && model_->body_mass[id] > 0) {
        payload_body_ = id; break;
      }
    }
  }
  if (payload_body_ > 0) {
    physics_.payload_available = true;
    physics_.nominal_mass = physics_.current_mass = model_->body_mass[payload_body_];
    const char * name = mj_id2name(model_, mjOBJ_BODY, payload_body_);
    physics_.base_body = name ? name : "Unnamed body";
  }
  floor_id_ = mj_name2id(model_, mjOBJ_GEOM, "floor");
  if (floor_id_ < 0) {
    for (int id=0; id<model_->ngeom; ++id) {
      if (model_->geom_bodyid[id] == 0 &&
          (model_->geom_type[id] == mjGEOM_PLANE || model_->geom_type[id] == mjGEOM_HFIELD)) {
        floor_id_ = id; break;
      }
    }
  }
  if (floor_id_ < 0) return;
  physics_.floor_available = true;
  std::copy_n(model_->geom_friction + 3*floor_id_, 3, nominal_floor_.begin());
  for (int id=0; id<model_->npair; ++id) {
    if (model_->pair_geom1[id] != floor_id_ && model_->pair_geom2[id] != floor_id_) continue;
    floor_pairs_.push_back(id);
    std::array<mjtNum,5> value;
    std::copy_n(model_->pair_friction + 5*id,5,value.begin()); nominal_pairs_.push_back(value);
  }
  if (floor_pairs_.empty()) {
    for (int id=0; id<model_->ngeom; ++id) {
      if (id == floor_id_) continue;
      auto is_foot = [](const char * name) {
        if (!name) return false;
        std::string lower(name);
        std::transform(lower.begin(), lower.end(), lower.begin(),
          [](unsigned char c){return std::tolower(c);});
        return lower.find("foot") != std::string::npos ||
          lower.find("sole") != std::string::npos ||
          lower.find("ankle_roll") != std::string::npos;
      };
      bool foot = is_foot(mj_id2name(model_,mjOBJ_GEOM,id));
      // Current K1 MJCF uses unnamed foot geoms; identify their owning link too.
      for (int body=model_->geom_bodyid[id]; !foot && body>0; body=model_->body_parentid[body])
        foot = is_foot(mj_id2name(model_,mjOBJ_BODY,body));
      if (!foot) continue;
      if (!(model_->geom_contype[floor_id_] & model_->geom_conaffinity[id]) &&
          !(model_->geom_contype[id] & model_->geom_conaffinity[floor_id_])) continue;
      floor_feet_.push_back(id);
      std::array<mjtNum,3> value;
      std::copy_n(model_->geom_friction+3*id,3,value.begin()); nominal_feet_.push_back(value);
    }
  }
  physics_.friction = physics_.nominal_friction = !nominal_pairs_.empty() ? nominal_pairs_[0][0] :
    (!nominal_feet_.empty() ? nominal_feet_[0][0] : nominal_floor_[0]);
}
PhysicsSettings MujocoSimulation::physics_settings() const {
  std::lock_guard<std::mutex> lock(mutex_); return physics_;
}
void MujocoSimulation::set_floor_friction(double coefficient) {
  if (!std::isfinite(coefficient)) throw std::invalid_argument("Friction must be finite");
  std::lock_guard<std::mutex> lock(mutex_);
  if (floor_id_ < 0) return;
  coefficient = std::clamp(coefficient,0.05,2.0);
  model_->geom_friction[3*floor_id_] = coefficient;
  for (int id:floor_pairs_) model_->pair_friction[5*id] = model_->pair_friction[5*id+1] = coefficient;
  for (int id:floor_feet_) model_->geom_friction[3*id] = coefficient;
  physics_.friction = coefficient;
  mj_forward(model_,data_);
}
void MujocoSimulation::restore_floor_friction() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (floor_id_ < 0) return;
  std::copy(nominal_floor_.begin(),nominal_floor_.end(),model_->geom_friction+3*floor_id_);
  for (size_t i=0;i<floor_pairs_.size();++i)
    std::copy(nominal_pairs_[i].begin(),nominal_pairs_[i].end(),model_->pair_friction+5*floor_pairs_[i]);
  for (size_t i=0;i<floor_feet_.size();++i)
    std::copy(nominal_feet_[i].begin(),nominal_feet_[i].end(),model_->geom_friction+3*floor_feet_[i]);
  physics_.friction = physics_.nominal_friction;
  mj_forward(model_,data_);
}
void MujocoSimulation::set_payload(double additional_mass) {
  if (!std::isfinite(additional_mass)) throw std::invalid_argument("Payload must be finite");
  std::lock_guard<std::mutex> lock(mutex_);
  if (payload_body_ < 0) return;
  // Point mass at the existing body CoM: its rotational inertia stays unchanged.
  // mj_setConst needs scratch data; never let it change live integration state.
  std::unique_ptr<mjData,decltype(&mj_deleteData)> scratch(mj_makeData(model_),&mj_deleteData);
  if (!scratch) throw std::runtime_error("Cannot allocate payload update data");
  model_->body_mass[payload_body_] = physics_.nominal_mass + std::clamp(additional_mass,0.0,100.0);
  mj_setConst(model_,scratch.get());
  mj_forward(model_,data_);
  physics_.current_mass = model_->body_mass[payload_body_];
}
void MujocoSimulation::restore_payload() { set_payload(0); }
}  // namespace ai_sapiens_mujoco
