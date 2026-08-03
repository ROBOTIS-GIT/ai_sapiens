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
// Author: Kiwoong Park

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <memory>

#include "ai_sapiens_mujoco/center_of_mass_visualizer.hpp"

namespace ai_sapiens_mujoco
{
namespace
{

constexpr char kCenterOfMassModel[] =
  R"(
<mujoco>
  <worldbody>
    <body name="robot" pos="0 0 1">
      <freejoint/>
      <geom type="box" size=".1 .1 .1" density="1000"/>
      <body name="link" pos=".4 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" size=".08" density="500"/>
      </body>
    </body>
    <body name="gantry" mocap="true" pos="0 0 2">
      <geom type="box" size=".1 .1 .1" density="1000"/>
    </body>
  </worldbody>
</mujoco>
)";

struct ModelDeleter
{
  void operator()(mjModel * model) const
  {
    mj_deleteModel(model);
  }
};

struct DataDeleter
{
  void operator()(mjData * data) const
  {
    mj_deleteData(data);
  }
};

class CenterOfMassVisualizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[1024]{};
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    ASSERT_EQ(
      mj_addBufferVFS(
        &vfs, "center_of_mass_visualizer_test.xml",
        kCenterOfMassModel, sizeof(kCenterOfMassModel) - 1),
      0);
    model_.reset(
      mj_loadXML(
        "center_of_mass_visualizer_test.xml", &vfs, error, sizeof(error)));
    mj_deleteVFS(&vfs);
    ASSERT_NE(model_, nullptr) << error;

    data_.reset(mj_makeData(model_.get()));
    ASSERT_NE(data_, nullptr);
    mj_forward(model_.get(), data_.get());

    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjv_makeScene(model_.get(), &scene_, 16);
  }

  void TearDown() override
  {
    mjv_freeScene(&scene_);
  }

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  mjvOption option_{};
  mjvScene scene_{};
};

}  // namespace

TEST_F(CenterOfMassVisualizerTest, AppendsRobotAndLinkMarkers)
{
  option_.flags[mjVIS_COM] = 1;
  const int robot_id = mj_name2id(model_.get(), mjOBJ_BODY, "robot");
  const int link_id = mj_name2id(model_.get(), mjOBJ_BODY, "link");

  EXPECT_EQ(
    append_center_of_mass_markers(
      model_.get(), data_.get(), option_, &scene_),
    3);
  ASSERT_EQ(scene_.ngeom, 3);

  const mjvGeom & robot_com = scene_.geoms[0];
  EXPECT_EQ(robot_com.objtype, mjOBJ_UNKNOWN);
  EXPECT_EQ(robot_com.category, mjCAT_DECOR);
  EXPECT_FLOAT_EQ(
    robot_com.size[0], center_of_mass_marker_radius(model_.get()));
  EXPECT_EQ(robot_com.transparent, 0);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
      robot_com.pos[axis], data_->subtree_com[3 * robot_id + axis], 1.0e-6);
  }
  for (int channel = 0; channel < 4; ++channel) {
    EXPECT_FLOAT_EQ(
      robot_com.rgba[channel], model_->vis.rgba.com[channel]);
  }

  const mjvGeom & robot_link_com = scene_.geoms[1];
  const mjvGeom & child_link_com = scene_.geoms[2];
  EXPECT_EQ(robot_link_com.objtype, mjOBJ_BODY);
  EXPECT_EQ(robot_link_com.objid, robot_id);
  EXPECT_EQ(child_link_com.objtype, mjOBJ_BODY);
  EXPECT_EQ(child_link_com.objid, link_id);
  EXPECT_FLOAT_EQ(
    robot_link_com.size[0],
    center_of_mass_marker_radius(model_.get()));
  EXPECT_FLOAT_EQ(
    child_link_com.size[0],
    center_of_mass_marker_radius(model_.get()));
  EXPECT_FLOAT_EQ(robot_com.size[0], robot_link_com.size[0]);
  EXPECT_FLOAT_EQ(robot_link_com.size[0], child_link_com.size[0]);
  EXPECT_EQ(child_link_com.transparent, 1);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
      child_link_com.pos[axis], data_->xipos[3 * link_id + axis], 1.0e-6);
  }
  for (int channel = 0; channel < 4; ++channel) {
    EXPECT_FLOAT_EQ(
      child_link_com.rgba[channel], kLinkCenterOfMassColor[channel]);
  }
}

TEST_F(CenterOfMassVisualizerTest, ExcludesBodiesOutsideRobotSubtree)
{
  option_.flags[mjVIS_COM] = 1;
  const int gantry_id = mj_name2id(model_.get(), mjOBJ_BODY, "gantry");

  append_center_of_mass_markers(
    model_.get(), data_.get(), option_, &scene_);

  for (int index = 0; index < scene_.ngeom; ++index) {
    EXPECT_FALSE(
      scene_.geoms[index].objtype == mjOBJ_BODY &&
      scene_.geoms[index].objid == gantry_id);
  }
}

TEST_F(CenterOfMassVisualizerTest, AppendsNothingWhenDisabled)
{
  EXPECT_EQ(
    append_center_of_mass_markers(
      model_.get(), data_.get(), option_, &scene_),
    0);
  EXPECT_EQ(scene_.ngeom, 0);
}

TEST(CenterOfMassVisualizer, UsesNativeMujocoCenterOfMassScale)
{
  EXPECT_FLOAT_EQ(center_of_mass_marker_radius(nullptr), 0.0F);

  mjModel scale_model{};
  scale_model.stat.meansize = 0.25;
  scale_model.vis.scale.com = 0.12F;
  EXPECT_FLOAT_EQ(center_of_mass_marker_radius(&scale_model), 0.03F);
}

}  // namespace ai_sapiens_mujoco
