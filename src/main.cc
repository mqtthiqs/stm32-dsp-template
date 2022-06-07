#include "accelerometer.hh"
#include "button.hh"
#include "dac_and_mic.hh"
#include "debug_pins.hh"
#include "grm.h"
#include "leds.hh"
#include "system.hh"

#include "numtypes.h"
#include "processor.hh"

struct Main : System::SysTickCallback, Dac::ProcessCallback {
  System system_{this};
  Leds leds_;
  Button button_;
  Dac dac_{I2S_FREQ_48000, this};
  Accelerometer accel_;

  Accelerometer::AccelData d;
  Processor processor_;

  void Process(Array<Pair<s1_15>, kBlockSize> &out) {
    accel_.ReadAccelData(&d);

    for (auto &[x, y] : out) {
      x = y = s1_15::inclusive(processor_.process(d));
    }
  }

  void onSysTick() {
    static int led;
    if (system_.milliseconds() % 64 == 0) {
      leds_.set((LED)(led / 2), !(led & 1));
      led = (led + 1) % 8;
    }
  }

  Main() {
    dac_.set_volume(200);
    dac_.Start();
    while (1) {
      __WFI();
    }
  }
} _;
