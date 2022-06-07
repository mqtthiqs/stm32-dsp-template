#pragma once

#include "accelerometer.hh"
#include "grm.h"
#include "parameters.hh"

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
  f phase_, freq_ = 0_f;
};

struct Processor {
  f process(Accelerometer::AccelData &accel) {

    f note = f(accel.y);
    note = note.scale({-1_f, 1_f}, {0_f, 1_f}).cube();
    note = note.scale({0_f, 1_f}, {0.0001_f, 0.1_f});

    f amp = f(accel.x).square();

    f sum = 0_f;
    for (auto [n, osc] : enumerate(osc_)) {
      osc.setFreq(note / f(n+1));
      sum += osc.process() * amp;
    }
    return sum / f(osc_.size());
  }

private:
  Array<Oscillator, 10> osc_;
};