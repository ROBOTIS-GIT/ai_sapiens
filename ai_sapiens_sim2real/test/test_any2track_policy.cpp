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

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <openssl/evp.h>
#include "ai_sapiens_sim2real/policy/any2track_policy.hpp"
#include "ai_sapiens_sim2real/config/any2track_config.hpp"

using namespace ai_sapiens_sim2real;  // NOLINT
namespace
{
std::string digest(const std::filesystem::path & p)
{
  std::ifstream in(p, std::ios::binary);
  std::string bytes((std::istreambuf_iterator<char>(in)), {});
  unsigned char hash[32]; unsigned int size;
  EVP_Digest(bytes.data(), bytes.size(), hash, &size, EVP_sha256(), nullptr);
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i) {
    out << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(hash[i]);
  }
  return out.str();
}
class Any2TrackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char name[] = "/tmp/a2t-test-XXXXXX";
    dir = mkdtemp(name);
    std::filesystem::copy_file(std::filesystem::path(TEST_DATA_DIR) / "any2track_sim2real.yaml",
        dir / "sim2real.yaml");
    std::ofstream(dir / "policy.onnx") << "fixture";
    std::ofstream(dir / "motion.csv") << "fixture";
    std::ofstream data(dir / "reference.bin", std::ios::binary);
    data.write("A2TREF01", 8);
    uint32_t count = 4;
    data.write(reinterpret_cast<char *>(&count), 4);
    for (int i = 0; i < 4; ++i) {
      Any2TrackPolicy::Frame f{};
      f[0] = .1f * i; f[1] = -.2f * i; f[2] = .75f; f[3] = 1;
      for (int j = 0; j < 23; ++j) {f[7 + j] = .01f * j + .1f * i; f[36 + j] = .2f * j;}
      for (int j = 0; j < 4; ++j) {f[59 + j] = .03f * j;}
      data.write(reinterpret_cast<const char *>(f.data()), sizeof(f));
    }
    data.close();
    YAML::Node manifest;
    manifest["version"] = 1; manifest["columns"] = 63; manifest["frames"] = 4;
    manifest["step_dt"] = .02; manifest["reference_file"] = "reference.bin";
    manifest["joint_names"] = YAML::LoadFile((dir / "sim2real.yaml").string())["policy_joints"];
    for (const auto & item : std::vector<std::pair<std::string, std::string>>{
        {"reference_sha256", "reference.bin"}, {"motion_csv_sha256", "motion.csv"},
        {"policy_sha256", "policy.onnx"}, {"sim2real_sha256", "sim2real.yaml"}})
    {manifest[item.first] = digest(dir / item.second);}
    std::ofstream(dir / "any2track_reference.yaml") << manifest;
    mimic.motion_file = dir / "motion.csv";
  }
  void TearDown() override {std::filesystem::remove_all(dir);}
  std::unique_ptr<Any2TrackPolicy> make()
  {
    return std::make_unique<Any2TrackPolicy>(Sim2RealConfig(dir / "sim2real.yaml"), mimic,
        dir / "policy.onnx");
  }
  std::filesystem::path dir;
  MimicBehavior mimic;
};

TEST_F(Any2TrackTest, UpcomingReferenceAndDistinctHistoryOrders)
{
  auto p = make(); Any2TrackSample s;
  s.gyro = Eigen::Vector3f(2, 4, 6); s.position.setConstant(.2f); s.velocity.setConstant(.3f);
  p->reset(s, false); ASSERT_TRUE(p->observe(s));
  const auto & o = p->observation();
  EXPECT_NEAR(o[0], -.1f, 1e-7);
  EXPECT_NEAR(o[23], -.015f, 1e-7);
  EXPECT_FLOAT_EQ(o[98], .2f);
  EXPECT_FLOAT_EQ(o[127], .1f); EXPECT_FLOAT_EQ(o[128], -.2f);
  EXPECT_FLOAT_EQ(o[132], .09f); EXPECT_FLOAT_EQ(o[133], .75f);
  const auto & h = p->inputs().at("history");
  EXPECT_FLOAT_EQ(h[0], .1f); EXPECT_FLOAT_EQ(h[5 * 79], -1);
  EXPECT_FLOAT_EQ(h[52 * 79], .2f);
  const auto & v = p->inputs().at("velocity_history");
  EXPECT_FLOAT_EQ(v[0], 0); EXPECT_FLOAT_EQ(v[2], -1); EXPECT_FLOAT_EQ(v[3], .1f);
  EXPECT_FLOAT_EQ(v[74], .2f); EXPECT_FLOAT_EQ(v[19 * 75 + 3], .1f);
  const auto target = p->target(std::vector<float>(23, .5f));
  EXPECT_NEAR(target[0], .6f, 1e-7);
  p->accept(target); p->advance(); s.gyro.setZero(); s.position.setZero();
  ASSERT_TRUE(p->observe(s));
  EXPECT_FLOAT_EQ(p->observation()[98], target[0]);
  EXPECT_FLOAT_EQ(h[78], .1f);  // preceding sensor sample, not this tick's zero gyro
  EXPECT_FLOAT_EQ(h[52 * 79 + 78], target[0]);
  EXPECT_FLOAT_EQ(v[19 * 75 + 3], 0); EXPECT_FLOAT_EQ(v[18 * 75 + 3], .1f);
}

TEST_F(Any2TrackTest, ReentryClearsHistoryAndCapturesNewOrigin)
{
  auto p = make(); Any2TrackSample s;
  p->reset(s, true); p->observe(s); p->accept(p->target(std::vector<float>(23, .8f))); p->advance();
  s.xy = Eigen::Vector2f(8, -5); s.position.setConstant(.4f);
  s.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(1.2f, Eigen::Vector3f::UnitZ()));
  p->reset(s, true); ASSERT_TRUE(p->observe(s));
  EXPECT_EQ(p->cursor(), 1U);
  EXPECT_NEAR(p->observation()[127], .1f, 1e-6); EXPECT_NEAR(p->observation()[128], -.2f, 1e-6);
  EXPECT_FLOAT_EQ(p->observation()[98], .4f);
  EXPECT_FLOAT_EQ(p->inputs().at("history")[52 * 79], .4f);
  EXPECT_FLOAT_EQ(p->inputs().at("history")[53 * 79 - 1], .4f);
}

TEST_F(Any2TrackTest, ClipEndsWithoutWrapAndRejectsBadInputs)
{
  auto p = make(); Any2TrackSample s;
  p->reset(s, false);
  for (int i = 1; i <= 3; ++i) {
    ASSERT_TRUE(p->observe(s)); EXPECT_EQ(p->cursor(), i); p->advance();
                                                                                                    }
  EXPECT_FALSE(p->observe(s));
  s.orientation.coeffs().setZero(); EXPECT_THROW(p->reset(s, true), std::runtime_error);
  s.orientation.setIdentity(); p->reset(s, false); p->observe(s);
  EXPECT_THROW(p->target(std::vector<float>(23, 2)), std::runtime_error);
  EXPECT_THROW(p->target(std::vector<float>(22, 0)), std::runtime_error);
}

TEST_F(Any2TrackTest, MixedAssetsAndInvalidWindowsFailClosed)
{
  mimic.time_start = .01f; EXPECT_THROW(make(), std::runtime_error);
  mimic.time_start = 0; mimic.on_complete = "stay"; EXPECT_THROW(make(), std::runtime_error);
  mimic.on_complete = "Velocity";
  std::ofstream(dir / "motion.csv", std::ios::app) << "changed";
  EXPECT_THROW(make(), std::runtime_error);
}

TEST_F(Any2TrackTest, SchemaRejectsWrongObservationOrderAndHistory)
{
  auto n = YAML::LoadFile((dir / "sim2real.yaml").string());
  EXPECT_NO_THROW(validate_any2track_config(n));
  n["history_length"] = 78; EXPECT_THROW(validate_any2track_config(n), std::runtime_error);
  n["history_length"] = 79; n["path_tracking"]["reference_time_offset_steps"] = 0;
  EXPECT_THROW(validate_any2track_config(n), std::runtime_error);
  n["path_tracking"]["reference_time_offset_steps"] = 1;
  n["observations"]["gyro_pelvis"]["scale"][0] = .05;
  EXPECT_THROW(validate_any2track_config(n), std::runtime_error);
}
}  // namespace
