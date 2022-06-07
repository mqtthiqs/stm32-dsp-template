#pragma once

#include <cassert>   // for assert
#include <cstdint>   // for int16_t, uint32_t, UINT32_MAX, INT32_MAX
#include <optional>  // for optional
#include <utility>   // for pair

#include "buffer.h"     // for Buffer, DynBuffer, RingBuffer
#include "filter.h"     // for OnePoleHpFromCoef, NonLinearOnePoleLp
#include "iter.h"       // for zip
#include "maths.h"      // for Math, Math::pi
#include "numtypes.h"   // for f, operator""_f, Float, s1_15, Fixed, u0_32
#include "util.h"       // for ipow, lpow, Nocopy, Pair

namespace grm {

struct Random {
  static uint32_t state() noexcept { return state_; }
  static uint32_t Word() noexcept {
    state_ = state_ * 1664525L + 1013904223L;
    return state();
  }
  static int16_t Int16() noexcept { return int16_t(Word() >> 16); }
  // float between 0 and 1
  static f Float01() noexcept { return f(Word()) / f(UINT32_MAX); }
  // float between -1 and 1
  static f Float11() noexcept { return f(int32_t(Word())) / f(INT32_MAX); }
  static bool Bool() noexcept { return Word() & 1; }

  static constexpr uint32_t rand_max = UINT32_MAX;

 private:
  inline static uint32_t state_;
};

template <int MAX_STAGES>
struct CorrelatedRandom {
  Array<f, MAX_STAGES> values = {0_f};
  int idx = 0;
  f sum = 0_f;
  // returns random values in [-stages..stages]
  f process(int stages) noexcept {
    idx = (idx + 1) % stages;
    sum -= values[idx];
    values[idx] = Random::Float11();
    sum += values[idx];
    return sum;
  }
};

// STAGES number of stages, RATE downsample rate
// careful: gain must be <= 2^16
template <class T, int STAGES, int RATE>
class CicDownSampler {
  using Wider = typename T::Wider;
  Array<Wider, STAGES> hi;
  Array<Wider, STAGES> hc;
  static constexpr long gain = lpow(RATE, STAGES);
  static_assert(gain <= (1L << T::WIDTH), "gain too high");

 public:
  CicDownSampler() : hi(Wider(0_f)), hc(Wider(0_f)) {}

  // reads [R*size] input samples, writes [size] output samples:
  void process(T const *in, T *out, int size) noexcept {
    while (size--) {
      // N integrators
      for (int i = 0; i < RATE; i++) {
        hi[0] += Wider(*in++);
        for (int n = 1; n < STAGES; n++) { hi[n] += hi[n - 1]; }
      }
      // N combs
      Wider v = hi[STAGES - 1];
      for (int n = 0; n < STAGES; n++) {
        Wider inp = v;
        v -= hc[n];
        hc[n] = inp;
      }
      *out++ = T::wrap(v / gain);
    }
  }

  void process(DynBuffer<T> const input, DynBuffer<T> output) noexcept {
    assert(input.size() / RATE == output.size());
    process(input.data(), output.data(), output.size());
  }

  // reads [R*size] input samples, writes [size] output samples:
  template <int block_size>
  void process(Buffer<T, block_size> const &input, Buffer<T, block_size / RATE> &output) noexcept {
    process(input.begin().repr(), output.begin().repr(), block_size / RATE);
  }
};

// STAGES number of stages, RATE interpolation rate
// careful: gain must be <= 2^16
template <class T, int STAGES, int RATE>
class CicUpSampler {
  using Wider = typename T::Wider;
  Array<Wider, STAGES> hi;
  Array<Wider, STAGES> hc;
  static constexpr int gain = ipow(RATE, STAGES - 1);
  static_assert(gain <= (1 << T::width()), "gain too high");

 public:
  CicUpSampler() {
    hi.fill(Wider(0_f));
    hc.fill(Wider(0_f));
  }

  // reads [size] input samples, writes [R*size] output samples:
  void process(T *in, T *out, int size) noexcept {
    while (size--) {
      // N combs
      Wider v = Wider(*in++);
      for (int n = 0; n < STAGES; n++) {
        Wider inp = v;
        v -= hc[n];
        hc[n] = inp;
      }
      // N integrators
      for (int i = 0; i < RATE; i++) {
        hi[0] += i == 0 ? v : Wider(0_f);
        for (int n = 1; n < STAGES; n++) { hi[n] += hi[n - 1]; }
        *out++ = T::wrap(hi[STAGES - 1] / gain);
      }
    }
  }

  void process(DynBuffer<T> const input, DynBuffer<T> output) noexcept {
    assert(output.size() == input.size() * RATE);
    process(input.begin(), output.begin(), output.size());
  }

  template <int block_size>
  void process(Buffer<T, block_size> &input, Buffer<T, block_size * RATE> &output) noexcept {
    process(input.begin(), output.begin(), block_size);
  }
};

template <int N, int R>
class PdmFilter : CicDownSampler<s1_15, N, R> {
  // setbits[i] = number of bits set in integer i
  static constexpr Array<int16_t, 256> setbits = {
      short(0), short(1), short(1), short(2), short(1), short(2), short(2), short(3), short(1),
      short(2), short(2), short(3), short(2), short(3), short(3), short(4), short(1), short(2),
      short(2), short(3), short(2), short(3), short(3), short(4), short(2), short(3), short(3),
      short(4), short(3), short(4), short(4), short(5), short(1), short(2), short(2), short(3),
      short(2), short(3), short(3), short(4), short(2), short(3), short(3), short(4), short(3),
      short(4), short(4), short(5), short(2), short(3), short(3), short(4), short(3), short(4),
      short(4), short(5), short(3), short(4), short(4), short(5), short(4), short(5), short(5),
      short(6), short(1), short(2), short(2), short(3), short(2), short(3), short(3), short(4),
      short(2), short(3), short(3), short(4), short(3), short(4), short(4), short(5), short(2),
      short(3), short(3), short(4), short(3), short(4), short(4), short(5), short(3), short(4),
      short(4), short(5), short(4), short(5), short(5), short(6), short(2), short(3), short(3),
      short(4), short(3), short(4), short(4), short(5), short(3), short(4), short(4), short(5),
      short(4), short(5), short(5), short(6), short(3), short(4), short(4), short(5), short(4),
      short(5), short(5), short(6), short(4), short(5), short(5), short(6), short(5), short(6),
      short(6), short(7), short(1), short(2), short(2), short(3), short(2), short(3), short(3),
      short(4), short(2), short(3), short(3), short(4), short(3), short(4), short(4), short(5),
      short(2), short(3), short(3), short(4), short(3), short(4), short(4), short(5), short(3),
      short(4), short(4), short(5), short(4), short(5), short(5), short(6), short(2), short(3),
      short(3), short(4), short(3), short(4), short(4), short(5), short(3), short(4), short(4),
      short(5), short(4), short(5), short(5), short(6), short(3), short(4), short(4), short(5),
      short(4), short(5), short(5), short(6), short(4), short(5), short(5), short(6), short(5),
      short(6), short(6), short(7), short(2), short(3), short(3), short(4), short(3), short(4),
      short(4), short(5), short(3), short(4), short(4), short(5), short(4), short(5), short(5),
      short(6), short(3), short(4), short(4), short(5), short(4), short(5), short(5), short(6),
      short(4), short(5), short(5), short(6), short(5), short(6), short(6), short(7), short(3),
      short(4), short(4), short(5), short(4), short(5), short(5), short(6), short(4), short(5),
      short(5), short(6), short(5), short(6), short(6), short(7), short(4), short(5), short(5),
      short(6), short(5), short(6), short(6), short(7), short(5), short(6), short(6), short(7),
      short(6), short(7), short(7), short(8)};

 public:
  // reads [8*R*size] binary input sample, outputs [size] samples
  void process(uint8_t *input, int16_t *output, int size) noexcept {
    Array<int16_t, R * size> temp;
    for (int i = 0; i < size * R; i++) {
      temp[i] = setbits[*input];
      input++;
    }
    CicDownSampler<s1_15, N, R>::process(temp, output, size);
  }
};

// Magic circle algorithm
class MagicSine {
  f sinz_ = 0_f;
  f cosz_ = 1_f;
  f freq_ = 2_f * Math::pi * 0.001_f;

 public:
  explicit MagicSine(f freq) noexcept : freq_(2_f * Math::pi * freq) {}

  std::pair<f, f> process() noexcept {
    sinz_ += freq_ * cosz_;
    cosz_ -= freq_ * sinz_;
    return std::pair(cosz_, sinz_);
  }

  void set_frequency(f freq) noexcept {  // freq = f_n / f_s
    freq_ = 2_f * Math::pi * freq;
    // this is an approximation, ok for small frequencies. The actual
    // value is f = 2sin(pi f_n T) (T sampling period, f_n freq)
  }
};

class Hysteresis {
  bool armed = true;
  f low, high;

 public:
  Hysteresis(f l, f h) : low(l), high(h) {}
  bool process(f x) noexcept {
    if (armed && x > high) {
      armed = false;
      return true;
    } else if (x < low) {
      armed = true;
    }
    return false;
  }
};

class Derivator {
  f state_;

 public:
  explicit Derivator(f s) noexcept : state_(s) {}
  Derivator() noexcept : state_(0_f) {}
  f process(f x) noexcept {
    f d = x - state_;
    state_ = x;
    return d;
  }
  f previous() noexcept { return state_; }
};

class Integrator {
  f state_;

 public:
  f process(f x) noexcept {
    state_ += x;
    return state_;
  }
};

class LeakyIntegrator {
  f state_;

 public:
  f process(f x, f c) noexcept {
    state_ = state_ * c + x;
    return state_;
  }
};

// if a peak occured in the input signal (i.e. if derivative changed sign),
// return [value of peak, subsample delay when happened]; otherwise return [0,
// 0].
class PeakPicker {
  f y_1 = 10000000_f;
  f y_2 = 10000000_f;

 public:
  template <int i>
  std::pair<f, f> process(f x_0) noexcept;
};

// 1st order: returns the closest peak sample occurence, with no subsample info
template <>
inline std::pair<f, f> PeakPicker::process<0>(f y_0) noexcept {
  auto res = std::pair(y_0 <= y_1 && y_1 > y_2 ? y_1 : 0_f, 0_f);
  y_2 = y_1;
  y_1 = y_0;
  return res;
}

// 2nd order: fits a parabola through the last 3 points and returns its vertex
template <>
inline std::pair<f, f> PeakPicker::process<1>(f y_0) noexcept {
  std::pair<f, f> res;
  if (y_0 <= y_1 && y_1 > y_2) {
    f a = -y_1 + 0.5_f * (y_2 + y_0);
    f b = -2_f * y_1 + 0.5_f * y_2 + 1.5_f * y_0;
    f c = y_0;
    f y_max = c - b * b / (4_f * a);
    f x_max = -b / (2_f * a);
    assert(x_max <= 0_f && x_max >= -2_f);
    res = std::pair(y_max, x_max);
  } else {
    res = std::pair(0_f, 0_f);
  }
  y_2 = y_1;
  y_1 = y_0;
  return res;
}

template <int DELAY>
class ChangeDetector {
  Derivator d1 {0_f}, d2 {0_f};
  Hysteresis hy_pos, hy_neg;
  bool armed_pos = true, armed_neg = true;
  int delay = 0;

 public:
  ChangeDetector(f lo, f hi) noexcept : hy_pos(lo, hi), hy_neg(lo, hi) {}
  bool process(f input) noexcept {
    f speed = d1.process(input);
    f accel = d2.process(speed);
    bool pos = hy_pos.process(accel);
    bool neg = hy_neg.process(-accel);
    if (delay && delay-- == 1) { return true; }
    if (pos) { armed_pos = true; }
    if (neg) { armed_neg = true; }
    if ((neg && armed_pos) || (pos && armed_neg)) {
      armed_pos = armed_neg = false;
      delay = DELAY;
    }
    return false;
  }
};

template <class T>
class Sampler {
  T buffer;
  bool hold_ = false;

 public:
  T process(T x) noexcept { return hold_ ? buffer : buffer = x; }
  void hold() noexcept { hold_ = true; }
  void release() noexcept { hold_ = false; }
};

struct RampOscillator {
  explicit RampOscillator(u0_32 freq) noexcept : freq_(freq) {}
  RampOscillator() noexcept : freq_(0_u0_32) {}
  u0_32 process() noexcept {
    phase_ += freq_;
    return phase_;
  }

  void process(uint32_t i) noexcept { phase_ += freq_.load() * i; }

  void setFrequency(u0_32 freq) noexcept { freq_ = freq; }
  void setPhase(u0_32 phase) noexcept { phase_ = phase; }
  u0_32 getFrequency() const noexcept { return freq_; }
  u0_32 getPhase() const noexcept { return phase_; }

 private:
  u0_32 phase_ = 0_u0_32;
  std::atomic<u0_32> freq_;
};

struct TriangleOscillator : RampOscillator {
  u0_32 process() noexcept {
    u0_32 sample = RampOscillator::process() * 2;
    if (getPhase() > 0.5_u0_32) sample = 1_u0_32 - sample;
    return sample;
  }
};

struct SineOscillator : RampOscillator {
  f process() noexcept { return Math::sine(f(RampOscillator::process())); }
};

struct FastSineOscillator : RampOscillator {
  f process() noexcept { return Math::fast_sine(f(RampOscillator::process())); }
};

struct FasterSineOscillator : RampOscillator {
  f process() noexcept { return Math::faster_sine(f(RampOscillator::process())); }
};

// return true when oscillator resets
struct TriggerOscillator : RampOscillator {
  std::optional<u32_32> process(uint32_t steps) noexcept {
    u0_32 oldPhase = getPhase();
    RampOscillator::process(steps);
    if (oldPhase > getPhase() || getFrequency() >= max_val<u0_32> / steps)
      return getPhase() / getFrequency();
    else return {};
  }
};

struct PolyBlep : SineOscillator {
  PolyBlep() noexcept = default;

  f processImpulse() {
    u0_32 duty = getFrequency();
    f naiveImpulse = RampOscillator::process() > duty ? -2_f : 2_f;

    u0_32 phase = getPhase();
    u0_32 dephase = phase - duty;
    u0_32 freq = getFrequency();

    f blep = 2_f * processBlep(phase, freq);
    f blep2 = 2_f * processBlep(dephase, freq);

    return 2_f + naiveImpulse + blep - blep2 - 4_f * f(getFrequency());
  }

  f processRamp() {
    f naiveRamp = f(RampOscillator::process()) * 2_f - 1_f;
    f blep = processBlep(getPhase(), getFrequency());
    return naiveRamp - blep;
  }

  f processSquare() {
    f naiveSqr = RampOscillator::process() > 0.5_u0_32 ? 1_f : -1_f;

    u0_32 phase = getPhase();
    u0_32 dephase = phase + 0.5_u0_32;
    u0_32 freq = getFrequency();

    f blep = processBlep(phase, freq);
    f blep2 = processBlep(dephase, freq);

    return naiveSqr - blep + blep2;
  }

 private:
  f processBlep(u0_32 phase, u0_32 freq) noexcept {
    if (phase < freq) {
      f p = f(phase / freq);
      return 2_f * p - p * p - 1_f;
    } else if (freq > 0_u0_32 && phase > 0_u0_32 - freq) {
      f p = f((0_u0_32 - phase) / freq);
      return -(2_f * p - p * p - 1_f);
    } else {
      return 0_f;
    }
  }
};

struct MultiOscillator : PolyBlep {
  f processWhite() noexcept { return Random::Float11(); }
  f processSine() noexcept { return SineOscillator::process(); }
  f processImpulse() noexcept { return PolyBlep::processImpulse(); }
  f processRamp() noexcept { return PolyBlep::processRamp(); }
  f processSquare() noexcept { return PolyBlep::processSquare(); }
};

template <int WSIZE, int SIZE, bool type2, bool sdf>
class Correlator : Nocopy {
  RingBuffer<s1_15, WSIZE + SIZE> x_;
  RingBuffer<s1_15, WSIZE + SIZE> y_;
  Array<s17_15, SIZE> correlation_;

 public:
  Correlator() noexcept {
    correlation_.fill(0_s17_15);
    x_.fill(0_s1_15);
    y_.fill(0_s1_15);
  }

  static inline s17_15 compare(s1_15 x, s1_15 y) noexcept {
    return sdf ? s17_15::narrow((x - y) * (x - y)) : s17_15::narrow(x * y);
  }

  template <int block_size>
  void process(Buffer<s1_15, block_size> &x, Buffer<s1_15, block_size> &y) noexcept {
    for (auto [x, y] : zip(x, y)) process(x, y);
  }

  void process(s1_15 x, s1_15 y) noexcept {
    x_ << x;
    y_ << y;
    for (int tau = 0; tau < SIZE; tau++) {
      if (!type2) {
        correlation_[tau] += compare(x_[0], y_[tau]) - compare(x_[WSIZE], y_[WSIZE + tau]);
      } else {  // SIZE <= WSIZE
        correlation_[tau] += compare(x_[0], y_[tau]) - compare(x_[WSIZE - tau], y_[WSIZE]);
      }
    }
  }

  s17_15 value(int tau) noexcept { return correlation_[tau]; }

  int match(int beg, int end) noexcept {
    int best_index = beg;
    s17_15 best = value(best_index);
    for (int i = beg; i < end; i++) {
      if (value(i) > best) {
        best = value(i);
        best_index = i;
      }
    }
    return best_index;
  }
};

template <class T, int WSIZE, int SIZE, bool type2, bool sdf>
class Autocorrelator : Nocopy {
  using Wider = typename T::Wider;
  RingBuffer<T, WSIZE + SIZE> x_ {T(0_f)};

  Array<Wider, SIZE> correlation_;

  static_assert(!type2 || SIZE <= WSIZE);

 public:
  Array<Wider, SIZE> &result() { return correlation_; }

  Autocorrelator() noexcept : correlation_(Wider(0)) {}

  static inline Wider compare(T x, T y) noexcept {
    return sdf ? Wider::narrow((x - y) * (x - y)) : Wider::narrow(x * y);
  }

  void process(T x) noexcept {
    x_ << x;
    for (int tau = 0; tau < SIZE; tau++) {
      if (!type2) {
        correlation_[tau] += compare(x, x_[tau]) - compare(x_[WSIZE], x_[WSIZE + tau]);
      } else {  // SIZE <= WSIZE
        correlation_[tau] += compare(x, x_[tau]) - compare(x_[WSIZE - tau], x_[WSIZE]);
      }
    }
  }
};

class EnvelopeFollower {
  Average<u0_16, 128, 2> filter1_;

 protected:
  struct Transfer {
    static f process(f x) noexcept { return x < 0_f ? x * x.abs() * 0.1_f : x; }
  };

 private:
  NonLinearOnePoleLp<f, Transfer> filter2_;

 public:
  f process(s1_15 x) noexcept {
    u0_16 y = filter1_.process(u0_16::wrap(x.abs()));
    return filter2_.process(f(y));
  }
};

class TransientDetector : EnvelopeFollower {
  OnePoleHpFromCoef hp1_, hp2_;
  NonLinearOnePoleLp<f, Transfer> filter_;

 public:
  f process(s1_15 x) noexcept {
    /* TODO optimize: do first HP in s1_15 */
    f y = f(x);
    hp1_.process(0.05_f, y, y);
    f z = EnvelopeFollower::process(s1_15(y));
    hp2_.process(0.001_f, z, z);
    z = z * z * 10_f;
    return z;
  }
};

template <class P>
struct FloatIO : P {
  s1_15 process(s1_15 x) noexcept { return s1_15::inclusive(P::process(f(x))); }
};

template <class P>
struct Block : P {
  template <class T, int block_size>
  void process(Buffer<T, block_size> &in, Buffer<T, block_size> &out) noexcept {
    for (auto [i, o] : zip(in, out)) { o = P::process(i); }
  }
};

template <int rate, class P>
class Oversampled : public P {
  CicDownSampler<s1_15, 2, rate> down_;
  CicUpSampler<s1_15, 2, rate> up_;

 public:
  template <int block_size>
  void process(Buffer<f, block_size> &in, Buffer<f, block_size> &out) noexcept {
    Buffer<s1_15, block_size> in_s, out_s;
    for (auto [x, y] : zip(in, in_s)) y = s1_15::inclusive(x);
    process(in_s, out_s);
    for (auto [x, y] : zip(out_s, out)) y = f::inclusive(x);
  }

  template <int block_size>
  void process(Buffer<s1_15, block_size> &in, Buffer<s1_15, block_size> &out) noexcept {
    Buffer<s1_15, block_size * rate> in_up, out_up;
    up_.process(in, in_up);
    P::process(in_up, out_up);
    down_.process(out_up, out);
  }
};

template <class P>
struct Stereo {
  P left, right;

  template <class T>
  Pair<T> process(Pair<T> &in) noexcept {
    return {left.process(in.first), right.process(in.second)};
  }

  template <class T, int block_size>
  void process(PairBuffer<T, block_size> &in, PairBuffer<T, block_size> &out) noexcept {
    Buffer<T, block_size> in_left, in_right, out_left, out_right;
    in.destruct(in_left, in_right);
    left.process(in_left, out_left);
    right.process(in_right, out_right);
    out.recompose(out_left, out_right);
  }
};

struct ZeroCrossingDetector {
  explicit ZeroCrossingDetector(f hysteresis) : hysteresis_(hysteresis) {
    assert(hysteresis_ >= 0_f);
  }

  std::optional<f> process(f y) {
    f y0 = last_;
    last_ = y;
    if (y0 < -hysteresis_ && y >= hysteresis_) return y0 / (y - y0);
    return {};
  }

 private:
  f last_ = 0_f;
  f const hysteresis_;
};

}  // namespace grm
