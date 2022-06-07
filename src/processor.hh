#pragma once

#include "accelerometer.hh"
#include "grm.h"

using namespace grm;

struct Oscillator {

  f process() {
    phase_ += freq_;
    if (phase_ > 1_f)
      phase_--;
    
    return Math::sin(phase_ * 2_f * Math::pi);
  }

  void setFreq(f freq) { freq_ = freq; }

private:
  f phase_, freq_=0_f;
};

struct Processor {
  f process(Accelerometer::AccelData &accel) {
    f freq = 0.005_f;

    osc_.setFreq(freq);
    return osc_.process();
  }

private:
  Oscillator osc_;
};