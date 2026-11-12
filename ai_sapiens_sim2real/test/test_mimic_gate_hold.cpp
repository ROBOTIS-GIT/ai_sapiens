#include <gtest/gtest.h>

#include "ai_sapiens_sim2real/policy/mimic_gate.hpp"

namespace
{

k1_mimic_gate::Gate make_gate(bool singleton_loop)
{
  k1_mimic_gate::Parameters params;
  params.quiet_seconds = 0.f;
  params.pending_timeout_seconds = 0.f;
  params.ramp_seconds = 0.02f;
  params.bridge_min_seconds = 0.02f;
  params.bridge_max_seconds = 0.02f;
  params.abort_on_release = true;
  params.hold_frames = {2};
  if (singleton_loop) {params.loops = {{0}};}

  k1_mimic_gate::Clip clip;
  for (int i = 0; i < 5; ++i) {
    clip.q.push_back(k1_mimic_gate::Vec::Zero());
    clip.dq.push_back(k1_mimic_gate::Vec::Zero());
    clip.root.push_back(Eigen::Quaternionf::Identity());
    clip.torso_tilt.push_back(0.f);
  }
  return {params, {clip}};
}

void reach_hold(k1_mimic_gate::Gate & gate, k1_mimic_gate::Inputs & input)
{
  gate.step(input);  // establish the initial switch positions
  input.sd = true;
  for (int i = 0; i < 100 && !gate.holding; ++i) {
    gate.step(input);
  }
  ASSERT_TRUE(gate.holding);
  ASSERT_EQ(gate.state, k1_mimic_gate::CLIP);
  ASSERT_EQ(gate.frame, 2);
}

TEST(MimicGateHold, SaReleaseResumesTheRemainingFrames)
{
  auto gate = make_gate(false);
  k1_mimic_gate::Inputs input;
  input.dial = 0;
  reach_hold(gate, input);
  for (int i = 0; i < 5; ++i) {
    gate.step(input);
    EXPECT_EQ(gate.frame, 2);
  }
  input.sd = false;
  gate.step(input);
  EXPECT_EQ(gate.state, k1_mimic_gate::CLIP);
  EXPECT_EQ(gate.frame, 3);
  for (int i = 0; i < 100 && gate.state != k1_mimic_gate::WALK; ++i) {
    gate.step(input);
  }
  EXPECT_EQ(gate.state, k1_mimic_gate::WALK);
}

TEST(MimicGateHold, ConfirmResumesSingletonSlotWithoutRepeating)
{
  auto gate = make_gate(true);
  k1_mimic_gate::Inputs input;
  input.loop = 0;
  reach_hold(gate, input);
  EXPECT_TRUE(gate.loop_stop);
  input.confirm = true;
  gate.step(input);
  EXPECT_EQ(gate.frame, 3);
  for (int i = 0; i < 100 && gate.state != k1_mimic_gate::WALK; ++i) {
    gate.step(input);
  }
  EXPECT_EQ(gate.state, k1_mimic_gate::WALK);
  for (int i = 0; i < 5; ++i) {
    gate.step(input);
    EXPECT_EQ(gate.state, k1_mimic_gate::WALK);
  }
}

TEST(MimicGateHold, TriggerAndConfirmCanShareTheSdSwitch)
{
  auto gate = make_gate(false);
  gate.clips.push_back(gate.clips.front());
  gate.params.hold_frames = {-1, -1};
  gate.params.inter_clip = 0;
  gate.params.inter_hold_frame = 2;
  gate.params.inter_low_clip = 1;
  gate.params.inter_low_frame = 3;
  gate.params.confirm_is_trigger = true;

  k1_mimic_gate::Inputs input;
  input.dial = 0;
  input.select = 1;
  gate.step(input);
  input.sd = true;
  input.confirm = true;
  gate.step(input);
  EXPECT_EQ(gate.choice, 0);  // Starting the clip must not confirm the handshake.
  for (int i = 0; i < 100 && !gate.holding; ++i) {
    gate.step(input);
  }
  ASSERT_TRUE(gate.holding);
  ASSERT_EQ(gate.clip, 0);
  input.sd = false;
  input.confirm = false;
  gate.step(input);
  EXPECT_EQ(gate.state, k1_mimic_gate::BRIDGE_IN);
  EXPECT_EQ(gate.clip, 1);
  EXPECT_EQ(gate.frame, 3);
  EXPECT_FALSE(gate.abort_requested);
}

}  // namespace
