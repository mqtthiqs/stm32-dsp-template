#include "grm.h"

#include "drivers/accelerometer.h"
#include "drivers/button.h"
#include "drivers/dac.h"
#include "drivers/debug_pins.h"
#include "drivers/leds.h"
#include "drivers/system.h"

#include "processor.h"

struct Main : System::SysTickCallback, Dac::ProcessCallback {
  System system_{this};
  Leds leds_;
  Button button_;
  Dac dac_{I2S_FREQ_48000, this};
  Accelerometer accel_;

  AccelData d;
  Processor processor_;

  void Process(Array<Pair<s1_15>, kBlockSize> &out) {

    for (auto &[x, y] : out) {
      x = y = s1_15::inclusive(processor_.process(d).clip());
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
    dac_.set_volume(210);
    dac_.Start();
    while (1) {
      accel_.ReadAccelData(&d);
      __WFI();
    }
  }
} _;
