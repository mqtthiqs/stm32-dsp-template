#include "debug_pins.hh"
#include "internal_dac.hh"
#include "leds.hh"
#include "button.hh"
#include "dac_and_mic.hh"
#include "system.hh"
#include "accelerometer.hh"

#include "dsp.hh"

struct Main :
  System::SysTickCallback,
  DacAndMic::ProcessCallback {

  System system_ {this};
  Leds leds_;
  Button button_;
  DacAndMic dam_ {I2S_FREQ_48000, this};
  Accelerometer accel_;

  int led = 0;
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
    while(1) { }
  }
} _;
