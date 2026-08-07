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

#include "ai_sapiens_mujoco/contact_force_visualizer.hpp"

namespace
{

constexpr char kContactModel[] =
  R"(
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="ball" pos="0 0 0.2">
      <freejoint/>
      <geom name="ball_geom" type="sphere" size="0.1" mass="1"/>
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

class ContactForceVisualizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[1024]{};
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    const int add_result = mj_addBufferVFS(
      &vfs, "contact_force_visualizer_test.xml",
      kContactModel, sizeof(kContactModel) - 1);
    ASSERT_EQ(add_result, 0);
    model_.reset(
      mj_loadXML(
        "contact_force_visualizer_test.xml", &vfs, error, sizeof(error)));
    mj_deleteVFS(&vfs);
    ASSERT_NE(model_, nullptr) << error;
    data_.reset(mj_makeData(model_.get()));
    ASSERT_NE(data_, nullptr);

    for (int step = 0; step < 1000; ++step) {
      mj_step(model_.get(), data_.get());
    }
    ASSERT_GT(data_->ncon, 0);
  }

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
};

TEST_F(ContactForceVisualizerTest, ComputesUpwardNormalForceLineAtEachGroundContact)
{
  ai_sapiens_mujoco::ContactNormalForceLine line;
  ASSERT_TRUE(ai_sapiens_mujoco::compute_contact_normal_force_line(
      model_.get(), data_.get(), 0, &line));

  EXPECT_GT(line.magnitude, 0.0);
  EXPECT_NEAR(line.end[0], line.start[0], 1.0e-9);
  EXPECT_NEAR(line.end[1], line.start[1], 1.0e-9);
  EXPECT_GT(line.end[2], line.start[2]);
}

TEST_F(ContactForceVisualizerTest, AppendsOneLimeLinePerActiveContact)
{
  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(model_.get(), &scene, 32);

  const int initial_geoms = scene.ngeom;
  const int appended = ai_sapiens_mujoco::append_contact_normal_force_lines(
    model_.get(), data_.get(), &scene);

  ASSERT_EQ(appended, data_->ncon);
  ASSERT_EQ(scene.ngeom, initial_geoms + appended);
  for (int geom_id = initial_geoms; geom_id < scene.ngeom; ++geom_id) {
    EXPECT_EQ(scene.geoms[geom_id].type, mjGEOM_LINE);
    EXPECT_EQ(scene.geoms[geom_id].category, mjCAT_DECOR);
    EXPECT_FLOAT_EQ(scene.geoms[geom_id].rgba[0], 0.55f);
    EXPECT_FLOAT_EQ(scene.geoms[geom_id].rgba[1], 1.0f);
    EXPECT_FLOAT_EQ(scene.geoms[geom_id].rgba[2], 0.15f);
    EXPECT_FLOAT_EQ(scene.geoms[geom_id].rgba[3], 1.0f);
  }

  mjv_freeScene(&scene);
}

TEST(ContactForceVisualizer, RejectsInvalidInput)
{
  ai_sapiens_mujoco::ContactNormalForceLine line;
  EXPECT_FALSE(ai_sapiens_mujoco::compute_contact_normal_force_line(
      nullptr, nullptr, 0, &line));
  EXPECT_EQ(ai_sapiens_mujoco::append_contact_normal_force_lines(
      nullptr, nullptr, nullptr), 0);
}

}  // namespace
