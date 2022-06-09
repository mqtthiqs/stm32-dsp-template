#pragma once

#include "drivers/accelerometer.h"
#include "parameters.h"

#include "grm.h"

using namespace grm;

struct Oscillator {

  f process() {
    phase_ += freq_;
    if (phase_ > 1_f)
      phase_--;

    return Math::sine(phase_);
  }

  void setFreq(f freq) { freq_ = freq; }

private:
  f phase_, freq_ = 0_f;
};

struct Processor {
  f process(AccelData &accel) {

    f freq = f(accel.y);
    freq = freq.scale({-1_f, 1_f}, {0_f, 1_f}).cube();
    freq = freq.scale({0_f, 1_f}, {0.0001_f, 0.2_f});

    f amp = f(accel.x).square();

    f sum = 0_f;
    for (auto [n, osc] : enumerate(osc_)) {
      osc.setFreq(freq / f(n+1));
      sum += osc.process() * amp;
    }
    return sum / f(osc_.size());
  }

private:
  Array<Oscillator, 3> osc_;
};