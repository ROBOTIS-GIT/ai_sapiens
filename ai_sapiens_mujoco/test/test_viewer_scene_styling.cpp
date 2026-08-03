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

#include "ai_sapiens_mujoco/viewer_scene_styling.hpp"

namespace ai_sapiens_mujoco
{
namespace
{

constexpr char kViewerModel[] =
  R"(
<mujoco>
  <worldbody>
    <body>
      <freejoint/>
      <geom name="visual" type="box" size=".1 .1 .1"
            group="2" rgba=".7 .7 .7 1"/>
      <geom name="collision" type="sphere" size=".05"
            group="3" rgba=".2 .5 .8 .35"/>
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

class ViewerSceneStylingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char error[1024]{};
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    ASSERT_EQ(
      mj_addBufferVFS(
        &vfs, "viewer_scene_styling_test.xml",
        kViewerModel, sizeof(kViewerModel) - 1),
      0);
    model_.reset(
      mj_loadXML(
        "viewer_scene_styling_test.xml", &vfs, error, sizeof(error)));
    mj_deleteVFS(&vfs);
    ASSERT_NE(model_, nullptr) << error;

    data_.reset(mj_makeData(model_.get()));
    ASSERT_NE(data_, nullptr);
    mj_forward(model_.get(), data_.get());

    mjv_defaultOption(&option_);
    mjv_defaultCamera(&camera_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjv_makeScene(model_.get(), &scene_, 32);
  }

  void TearDown() override
  {
    mjv_freeScene(&scene_);
  }

  void update_scene()
  {
    mjvOption scene_option = option_;
    scene_option.flags[mjVIS_COM] = 0;
    mjv_updateScene(
      model_.get(), data_.get(), &scene_option, &perturb_, &camera_,
      mjCAT_ALL, &scene_);
  }

  const mjvGeom * find_model_geom(const char * name) const
  {
    const int geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, name);
    for (int index = 0; index < scene_.ngeom; ++index) {
      const mjvGeom & geom = scene_.geoms[index];
      if (geom.objtype == mjOBJ_GEOM && geom.objid == geom_id) {
        return &geom;
      }
    }
    return nullptr;
  }

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  mjvOption option_{};
  mjvCamera camera_{};
  mjvPerturb perturb_{};
  mjvScene scene_{};
};

}  // namespace

TEST_F(ViewerSceneStylingTest, MakesOnlyVisualGroupTranslucentWhenComIsActive)
{
  option_.geomgroup[kCollisionGeomGroup] = 1;
  option_.flags[mjVIS_COM] = 1;
  update_scene();

  const mjvGeom * visual = find_model_geom("visual");
  const mjvGeom * collision = find_model_geom("collision");
  ASSERT_NE(visual, nullptr);
  ASSERT_NE(collision, nullptr);
  EXPECT_FLOAT_EQ(visual->rgba[3], 1.0F);
  EXPECT_FLOAT_EQ(collision->rgba[3], 0.35F);

  apply_center_of_mass_visual_style(model_.get(), option_, &scene_);

  EXPECT_FLOAT_EQ(visual->rgba[3], kCenterOfMassVisualAlpha);
  EXPECT_FLOAT_EQ(collision->rgba[3], 0.35F);
}

TEST_F(ViewerSceneStylingTest, LeavesVisualGroupOpaqueWhenComIsHidden)
{
  update_scene();
  const mjvGeom * visual = find_model_geom("visual");
  ASSERT_NE(visual, nullptr);

  apply_center_of_mass_visual_style(model_.get(), option_, &scene_);

  EXPECT_FLOAT_EQ(visual->rgba[3], 1.0F);
}

}  // namespace ai_sapiens_mujoco
