#include "microphone.hh"
#include "dac.hh"

class DacAndMic :
  Dac,
  Microphone,
  Dac::ProcessCallback,
  Microphone::ProcessCallback {
public:

  struct ProcessCallback {
    virtual void Process(short *in, ShortFrame* out, size_t size) = 0;
  };

private:
  ProcessCallback *callback_;
  short *input_block_ = (short*)42;

  // Mic callback
  void Process(short *in, size_t size) {
    input_block_ = in;
  }

  // DAC callback
  void Process(ShortFrame* out, size_t size) {
    callback_->Process(input_block_, out, size);
  }

public:

  DacAndMic(I2S_Freq freq, ProcessCallback *callback) :
    Dac(freq, this),
    Microphone(freq, this),
    callback_(callback) {}

  void Start() {
    Dac::Start();
    Microphone::Start();
  }

  void set_output_volume(uint8_t vol) {
    Dac::set_volume(vol);
  }

};
