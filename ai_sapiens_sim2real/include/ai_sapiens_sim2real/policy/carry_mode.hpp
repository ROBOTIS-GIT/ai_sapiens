#pragma once
// Stage-mode consumer for ai_sapiens. No actuator, network or ROS dependencies.
// Local deployment: velocity commands bypass the reference acceleration limiter.
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace k1_carry {
enum State {WALK, ENTER, RAISE, CARRY, LOWER, EXIT, BOW_IN, BOW, BOW_OUT};
struct Parameters {
  float dt{.02f}, blend_seconds{.5f}, arm_seconds{1.f};
  float bow_in_seconds{.5f}, bow_seconds{3.5f}, bow_out_seconds{.5f};
  float quiet_seconds{.7f};
  std::array<float,3> walk_limits{1.5f,1.f,1.57f};
  std::array<float,3> carry_limits{.8f,.25f,.8f};
};
inline float quintic(float x) {
  x=std::clamp(x,0.f,1.f);
  return x*x*x*(10.f+x*(-15.f+6.f*x));
}
class ModeMachine {
public:
  Parameters params;
  int state{WALK};
  float w{},h{},w_phase{},h_phase{},w_rate{},h_rate{},quiet_time{},bow_time{};
  bool se_previous{},bow_pending{};
  std::array<float,3> command{},speed_limits;
  explicit ModeMachine(Parameters p={}):params(p),speed_limits(p.walk_limits) {}
  void reset() {
    state=WALK; w=h=w_phase=h_phase=w_rate=h_rate=quiet_time=bow_time=0.f;
    se_previous=bow_pending=false; command={}; speed_limits=params.walk_limits;
  }
  std::array<float,6> mode() const {
    const float active=state>=BOW_IN?1.f:0.f;
    const float duration=params.bow_in_seconds+params.bow_seconds+params.bow_out_seconds;
    const float angle=6.283185307179586f*std::clamp(bow_time/duration,0.f,1.f);
    return {w,h,active,std::sin(angle)*active,std::cos(angle)*active,0.f};
  }
  std::array<float,6> step(bool sa,bool se,bool quiet,const std::array<float,3>& requested) {
    for(float v:requested) if(!std::isfinite(v)) throw std::invalid_argument("Nonfinite carry command");
    quiet_time=quiet?quiet_time+params.dt:0.f;
    const bool rising_se=se&&!se_previous;
    se_previous=se;
    const int old=state;
    // SE while moving/lowering is remembered; the command is held at zero until the bow starts. SA cancels.
    if(rising_se&&old<BOW_IN) bow_pending=true;
    if(sa) bow_pending=false;
    if(old==WALK) {
      if(sa) state=ENTER;
      else if(bow_pending&&quiet_time>=params.quiet_seconds) {state=BOW_IN;bow_time=0;bow_pending=false;}
    }
    if(old==ENTER) {
      if(!sa) {state=EXIT;if(ramp(false,false,params.blend_seconds)) state=WALK;}
      else if(ramp(false,true,params.blend_seconds)) state=RAISE;
    } else if(old==RAISE) {
      if(!sa) {state=LOWER;if(ramp(true,false,params.arm_seconds)) state=EXIT;}
      else if(ramp(true,true,params.arm_seconds)) state=CARRY;
    } else if(old==LOWER) {
      if(sa) {state=RAISE;if(ramp(true,true,params.arm_seconds)) state=CARRY;}
      else if(ramp(true,false,params.arm_seconds)) state=EXIT;
    } else if(old==EXIT) {
      if(sa) {state=ENTER;if(ramp(false,true,params.blend_seconds)) state=RAISE;}
      else if(ramp(false,false,params.blend_seconds)) state=WALK;
    } else if(old==CARRY&&!sa) state=LOWER;
    if(old>=BOW_IN) bow_time+=params.dt;
    if(old==BOW_IN&&ramp(false,true,params.bow_in_seconds)) state=BOW;
    if(old==BOW&&bow_time>=params.bow_in_seconds+params.bow_seconds) state=BOW_OUT;
    if(old==BOW_OUT&&ramp(false,false,params.bow_out_seconds)) {state=WALK;bow_time=0;}
    const bool carrying=state>=ENTER&&state<=EXIT;
    for(int i=0;i<3;++i) {
      const float limit=carrying?params.carry_limits[i]:params.walk_limits[i];
      speed_limits[i]=limit;
      const float target=(state>=BOW_IN||bow_pending)?0.f:std::clamp(requested[i],-speed_limits[i],speed_limits[i]);
      command[i]=target;
    }
    return mode();
  }
private:
  bool ramp(bool arm,bool target,float duration) {
    float &p=arm?h_phase:w_phase, &v=arm?h_rate:w_rate, &output=arm?h:w;
    const float desired=(target?1.f:-1.f)/duration;
    const bool rest=(p==0.f||p==1.f)&&v==0.f;
    const float bound=4.f*params.dt/(duration*duration);
    v=rest?desired:v+std::clamp(desired-v,-bound,bound);
    p=std::clamp(p+v*params.dt,0.f,1.f);
    const bool done=target?p>=1.f-1e-6f:p<=1e-6f;
    if(done) {p=target?1.f:0.f;v=0;}
    output=quintic(p);
    return done;
  }
};
} // namespace k1_carry
