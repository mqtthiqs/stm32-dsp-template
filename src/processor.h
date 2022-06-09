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
    freq = freq.scale({0_f, 1_f}, {0.00001_f, 0.2_f});
    smoothFreq += (freq - smoothFreq) * (freq - smoothFreq).abs() * 0.5_f;

    f amp = Math::fast_raised_cosine(f(accel.x));
    smoothAmp += (amp - smoothAmp) * (amp - smoothAmp).abs() * 0.5_f;

    amp = smoothAmp;
    freq = smoothFreq;

    f sum = 0_f;
    for (auto [n, osc] : enumerate(osc_)) {
      osc.setFreq(freq * f(n+1));
      sum += osc.process() * amp / f(n+1);
    }
    return sum / f(osc_.size());
  }

 private:
  f smoothFreq = 0_f, smoothAmp=0_f;
  Array<Oscillator, 1> osc_;
};