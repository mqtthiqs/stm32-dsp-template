#pragma once

#include "dsp.h"
#include "maths.h"

namespace grm {

struct AgitationOscillator {
  f process() {
    u0_32 oldPhase_ = phase_;
    phase_ += actualFrequency_;

    if (phase_ < oldPhase_) {
      f periodic = state_ ? 1_f : -1_f;
      state_ = !state_;
      random2_ = random1_;
      // random wave not full-scale to match perceived range of periodic oscillations
      f random = Random::Float11() * 1.414_f;
      random1_ = DspUtil::crossfade(random, periodic, periodicity_);
      jitterFactor_ = 1_f / (1_f + jitter_ * Random::Float11());
    }

    slantFactor_ = random1_ < random2_ ? slantFactorUp_ : slantFactorDown_;
    actualFrequency_ = u0_32(frequency_ * slantFactor_ * jitterFactor_);

    auto phase = f(phase_);
    phase = Math::pow(phase, step_);
    phase = Math::fast_raised_cosine(phase);

    f sample = DspUtil::crossfade(random2_, random1_, phase);
    return sample * amplitude_;
  }

  void setFrequency(f frequency) { frequency_ = frequency; }
  void setAmplitude(f amplitude) { amplitude_ = amplitude; }
  void setStep(f step) {
    step = 1_f - step;
    step_ = step.square();
  }

  void setPeriodicity(f periodicity) { periodicity_ = periodicity; }
  void setSlant(f slant) {
    f epsilon = 0.001_f;  // to avoid div by zero
    slantFactorUp_ = 1_f / (slant + epsilon);
    slantFactorDown_ = 1_f / (1_f + epsilon - slant);
  }

  void setJitter(f jitter) { jitter_ = jitter; }

 private:
  f amplitude_ = 0_f;
  f step_ = 1_f;  // 1..inf
  f periodicity_ = 0.9_f;
  f slantFactorUp_ = 2_f, slantFactorDown_ = 2_f;
  f frequency_ = 0_f;
  f jitter_ = 1_f;

  u0_32 phase_ = 0_u0_32;
  u0_32 actualFrequency_ = 0_u0_32;
  f slantFactor_ = 1_f, jitterFactor_ = 1_f;
  bool state_ = false;
  f random1_ = 0_f, random2_ = 0_f;  // -1..1
};
}  // namespace grm
