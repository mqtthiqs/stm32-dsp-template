#include "debug_pins.hh"
#include "leds.hh"
#include "button.hh"
#include "dac_and_mic.hh"
#include "system.hh"
#include "accelerometer.hh"

#include "numtypes.h"

using namespace grm;

struct Oscillator {

  f process() {
    phase += 0.0001_f;
    return phase * 2_f - 1_f;
  }
  private:
  f phase = 0_f;
};

struct Main :
  System::SysTickCallback,
  DacAndMic::ProcessCallback {

  int led;

  System system_ {this};
  Leds leds_;
  Button button_;
  DacAndMic dam_ {I2S_FREQ_48000, this};
  Accelerometer accel_;

  Accelerometer::AccelData d;

  void Process(short *in, ShortFrame *out, size_t size) {
    debug.on(1);
    accel_.ReadAccelData(&d);

    for (size_t i=0; i<size; i++) {
      out[i].l = in[i];
      out[i].r = in[i];
    }
    debug.off(1);
  }

  void onSysTick() {
    if (system_.milliseconds() % 64 == 0) {
      leds_.set((LED)(led/2), !(led&1));
      led = (led+1) % 8;
    }
  }

  Main() {
    dam_.set_output_volume(200);
    dam_.Start();
    while(1) { __WFI(); }
  }
} _;
