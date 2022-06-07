#pragma once

#include "buffer.h"
#include "iter.h"
#include "maths.h"
#include "util.h"

namespace grm {

struct OnePoleLpFromCoef {
  template <int approx>  // 0=perfect, >0 for cruder
  static f cutoffToCoef(f freq);

  f process(f coef, f input) noexcept {
    state_ += (input - state_) * coef;
    return state_;
  }

  void process(f coef, f input, f& output) noexcept { output = process(coef, input); }

  void process(DynBuffer<f> const input, DynBuffer<f> output, f coef) {
    for (auto [x, y] : zip(input, output)) process(coef, x, y);
  }

  f state() noexcept { return state_; }

 private:
  f state_ = 0_f;
};

// mathematically correct, but with some quantization noise towards smaller
// frequencies:
template <>
inline f OnePoleLpFromCoef::cutoffToCoef<0>(f freq) {
  assert(freq >= 0_f && freq <= 0.5_f);
  f c = Math::cosine(freq);
  return c - 1_f + (c * c - 4_f * c + 3_f).sqrt();
}

// approximation from dspguide
template <>
inline f OnePoleLpFromCoef::cutoffToCoef<1>(f freq) {
  assert(freq >= 0_f);
  return 1_f - Math::fast_exp(-2_f * Math::pi * freq);
}

// very crude linear approximation, for frequencies < 0.15 only:
template <>
inline f OnePoleLpFromCoef::cutoffToCoef<2>(f freq) {
  assert(freq >= 0_f);
  return 6.284_f * freq.min(0.15_f);
}

struct OnePoleHpFromCoef : OnePoleLpFromCoef {
  f process(f coef, f in) {
    f out;
    process(coef, in, out);
    return out;
  }
  void process(f coef, f input, f& output) noexcept {
    OnePoleLpFromCoef::process(coef, input, output);
    output = input - output;
  }
};

template <typename Filter>
struct WithCoef : Filter {
  WithCoef() noexcept = default;
  explicit WithCoef(f coef) noexcept : coef_(coef) {}

  f process(f input) noexcept { return Filter::process(coef_, input); }
  void process(f input, f& output) noexcept { Filter::process(coef_, input, output); }

  f getCoef() { return coef_; }
  void setCoef(f coef) { coef_ = coef; }

  template <int approx>
  void setFreq(f freq) {
    coef_ = Filter::template cutoffToCoef<approx>(freq);
  }

  void process(DynBuffer<f> const input, DynBuffer<f> output) {
    assert(input.size() == output.size());

    for (auto [x, y] : zip(input, output)) process(x, y);
  }

 private:
  f coef_ = 0_f;
};

template <typename Filter, int STAGES>
struct MultiStage : Array<Filter, STAGES> {
  f process(f coef, f input) noexcept {
    for (auto& filter : *this) input = filter.process(coef, input);
    return input;
  }

  void process(f coef, f input, f& output) noexcept { output = process(coef, input); }

  void process(DynBuffer<f> const input, DynBuffer<f> output, f coef) {
    for (auto [x, y] : zip(input, output)) process(coef, x, y);
  }
};

struct OnePoleLp : WithCoef<OnePoleLpFromCoef> {
  f getGainAt(f freq) {
    f coef = 1_f - getCoef();
    return (1_f - coef) /
           (1_f + coef.square() - 2_f * coef * Math::cos(2_f * Math::pi * freq)).sqrt();
  }
};

struct OnePoleHp : WithCoef<OnePoleHpFromCoef> {
  f getGainAt(f freq) {
    f coef = getCoef();
    return (1_f - coef) /
           (1_f + coef.square() + 2_f * coef * Math::cos(2_f * Math::pi * freq)).sqrt();
  }
};

struct AsymmetricOnePoleLpFromCoef {
  f process(f attack, f release, f input) noexcept {
    f error = input - state_;
    f coef = error > 0_f ? attack : release;
    state_ += error * coef;
    return state_;
  }

  void reset(f state) { state_ = state; }

 private:
  f state_ = 0_f;
};

template <class T, int SHIFT>
struct IOnePoleLp {
  T state() noexcept { return state_; }
  void process(T input) noexcept {
    state_ += input.template div2<SHIFT>() - state_.template div2<SHIFT>();
  }
  void process(T input, T& output) noexcept {
    Procss(input);
    output = state_;
  }

 private:
  T state_ = T(0_f);
};

// complexity:
// size=size * (stages+1) * 4 + 4,
// time=O(stages)
template <class T, int SIZE, int STAGES>
class Average {
  static constexpr long long int gain = ipow(SIZE - 1, STAGES);
  static_assert(gain < max_val<T>.repr(), "Error: gain must be less than 2^N");
  using Wider = decltype(std::declval<T>().widen());
  Array<Wider, STAGES> state_i;
  Array<RingBuffer<Wider, SIZE>, STAGES> state_c;

 public:
  Average() { fill(T(0_f)); }
  explicit Average(T x) { fill(x); }
  void fill(T x) {
    Wider y = x.widen();
    for (auto& b : state_c) b.fill(y);
    state_i.fill(y);
  }
  T last() noexcept { return T::wrap(state_i[STAGES - 1] / gain); }
  T process(T x) noexcept {
    auto y = Wider(x);
    for (int i = 0; i < STAGES; i++) {
      state_c[i] << y;
      y -= state_c[i].last();
    }

    state_i[0] += y;
    for (int i = 1; i < STAGES; i++) { state_i[i] += state_i[i - 1]; }
    Wider z = state_i[STAGES - 1];
    return T::wrap(z / gain);
  }

  int irSize() { return SIZE * STAGES; }
};

template <class T, class transfer>
class NonLinearOnePoleLp {
  T state_ = T(0_f);

 public:
  T process(T input) noexcept {
    state_ += transfer::process(input - state_);
    return state_;
  }

  T last() noexcept { return state_; }
};

template <int divisor>
struct ITransferLinear {
  static s1_15 process(s1_15 x) noexcept { return x / divisor; }
};

template <int divisor>
struct ITransferQuadratic {
  static s1_15 process(s1_15 x) noexcept { return s1_15::narrow(x.abs() * x) / divisor; }
};

template <int divisor>
struct ITransferCubic {
  static s1_15 process(s1_15 x) noexcept {
    return s1_15::narrow(s1_15::narrow(x * x) * x) / divisor;
  }
};

template <int divisor>
struct FTransferQuadratic {
  static f process(f x) noexcept {
    constexpr f const fact = 1_f / f(divisor);
    return x.abs() * x * fact;
  }
};

template <int divisor>
struct FTransferCubic {
  static f process(f x) noexcept {
    constexpr f const fact = 1_f / f(divisor);
    return ((x * x) * x) * fact;
  }
};

// piecwise linear function: f(x) = x if x>0 , x/divisor otherwise
template <int divisor>
struct FTransferAsymmetric {
  static f process(f x) noexcept {
    constexpr f const fact = 1_f / f(divisor);
    constexpr f const inv = 1_f - fact;
    return (inv * x).max(0_f) + fact * x;
  }
};

template <int divisor>
using IQuadraticOnePoleLp = NonLinearOnePoleLp<s1_15, ITransferQuadratic<divisor>>;
template <int divisor>
using ICubicOnePoleLp = NonLinearOnePoleLp<s1_15, ITransferCubic<divisor>>;

template <int divisor>
using QuadraticOnePoleLp = NonLinearOnePoleLp<f, FTransferQuadratic<divisor>>;
template <int divisor>
using CubicOnePoleLp = NonLinearOnePoleLp<f, FTransferCubic<divisor>>;

// LP that rises with coefficient 1 but falls with coef 1/divisor
template <int divisor>
using AsymmetricOnePoleLp = NonLinearOnePoleLp<f, FTransferAsymmetric<divisor>>;

struct FourPoleLadderLp {
  Array<OnePoleLpFromCoef, 4> lps_;
  f fb = 0_f;
  void process(f in, f& out, f coef, f resonance) noexcept {
    in -= fb * resonance;
    for (auto& lp : lps_) lp.process(coef, in, in);
    out = fb = Math::fast_tanh(in);
  }
};

enum class FilterMode { LP, BP, NORM_BP, HP };

class Svf {
  f g_ = 0_f;
  f r_ = 0_f;
  f h_ = 0_f;

  f state_1_ = 0_f;
  f state_2_ = 0_f;

 public:
  Svf() noexcept { set<Math::TanApprox::DIRTY>(0.01_f, 1_f); }

  // Set frequency and resonance from true units. Various approximations
  // are available to avoid the cost of tanf.
  template <Math::TanApprox approx = Math::TanApprox::DIRTY>
  void set(f freq, f resonance) noexcept {
    g_ = Math::fast_tan<approx>(freq);
    setResonance(resonance);
  }

  void setResonance(f resonance) {
    r_ = 1_f / resonance;
    h_ = 1_f / (1_f + g_ * (r_ + g_));
  }

  template <Math::TanApprox approx = Math::TanApprox::DIRTY>
  void setFrequency(f freq) {
    g_ = Math::fast_tan<approx>(freq);
    h_ = 1_f / (1_f + g_ * (r_ + g_));
  }

  Svf& operator=(Svf const& that) noexcept {
    if (this != &that) {
      g_ = that.g_;
      r_ = that.r_;
      h_ = that.h_;
    }
    return *this;
  }

  template <FilterMode mode>
  f process(f in) noexcept {
    f hp, bp, lp;
    hp = (in - r_ * state_1_ - g_ * state_1_ - state_2_) * h_;
    bp = g_ * hp + state_1_;
    state_1_ = g_ * hp + bp;
    lp = g_ * bp + state_2_;
    state_2_ = g_ * bp + lp;

    if constexpr (mode == FilterMode::LP) {
      return lp;
    } else if constexpr (mode == FilterMode::BP) {
      return bp;
    } else if constexpr (mode == FilterMode::NORM_BP) {
      return bp * r_;
    } else if constexpr (mode == FilterMode::HP) {
      return hp;
    }
  }

  template <FilterMode mode>
  void process(f* in, f* out, int size) noexcept {
    while (size--) { *out++ = process<mode>(*in++); }
  }

  template <FilterMode mode>
  void process(DynBuffer<f> const in, DynBuffer<f> out) noexcept {
    assert(in.size() == out.size());
    int size = in.size();
    process<mode>(in.begin(), out.begin(), size);
  }

  template <FilterMode mode, int size>
  void process(Buffer<f, size>& in, Buffer<f, size>& out) noexcept {
    process<mode>(in.begin(), out.begin(), size);
  }
};

template <int ORDER>
class AntialiasingFilter {
  Array<Svf, ORDER> lps_;

 public:
  void set_freq(f freq) noexcept {
    lps_[0].set(freq, 0.8_f);
    for (int i = 1; i < ORDER; i++) { lps_[i] = lps_[0]; }
  }

  void set_freq(AntialiasingFilter<ORDER> that) noexcept {
    for (auto& lp : lps_) { lp.set(that.lps_[0]); }
  }

  f process(f x) noexcept {
    for (auto& lp : lps_) { x = lp.process<FilterMode::LP>(x); }
    return x;
  }
};

// simple DC blocker from https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// [coef] in 0..1. 0.995 is a reasonable at 44.1 Hz
// normalized so that gain <= 1 at all frequencies
struct DCBlocker {
  DCBlocker() : DCBlocker(0.995_f) {}
  DCBlocker(f coef) : coef_(coef) {}
  f process(f x) noexcept {
    f y = gain_ * (x - xm1_) + coef_ * ym1_;
    xm1_ = x;
    ym1_ = y;
    return y;
  }

 private:
  const f coef_;
  const f gain_ = (1_f + coef_) * 0.5_f;
  f xm1_ = 0_f, ym1_ = 0_f;
};

template <typename T>
class SlewLimiter {
  T state_ = T(0_f);
  T slew_up_, slew_down_;

 public:
  SlewLimiter(T slew_up, T slew_down) noexcept : slew_up_(slew_up), slew_down_(-slew_down) {}
  void process(T input, T& output) noexcept {
    T error = input - state_;
    if (error > slew_up_) error = slew_up_;
    if (error < slew_down_) error = slew_down_;
    state_ += error;
    output = state_;
  }

  void process(T* input, T* output, int size) noexcept {
    while (size--) process(*input++, output++);
  }
};

// FIR low pass, unit gain at 0
class TwoZerosLp {
 public:
  void set(f coef) {
    assert(coef_ >= 0_f && coef <= 1_f);
    coef_ = coef;
  }

  f process(f x) {
    f h0 = (1_f + coef_) * 0.5_f;   // 0.5..1
    f h1 = (1_f - coef_) * 0.25_f;  // 0.25..0
    f y = h0 * x0_ + h1 * (x + x1_);
    x1_ = x0_;
    x0_ = x;
    return y;
  }

 private:
  f x0_ = 0_f;
  f x1_ = 0_f;
  f coef_ = 0_f;
};

template <int maxDelay>
struct Allpass {
  Allpass(f delay, f gain) : gain_(gain), delay_(delay) {}
  Allpass(f gain) : Allpass(0_f, gain) {}
  Allpass() : Allpass(0_f) {}

  template <Interpolation::Type type = Interpolation::ZOH>
  f process(f x) {
    f r = history_.template interpolate<type>(delay_);
    f w = x - r * gain_;
    history_ << w;
    return gain_ * w + r;
  }

  void setGain(f gain) { gain_ = gain; }
  void setDelay(f delay) {
    assert(delay >= 0_f);
    delay_ = delay;
  }

 private:
  f gain_ = 0_f;
  f delay_ = 0_f;
  RingBuffer<f, maxDelay> history_;
};

// Second-order allpass filter, from https://thewolfsound.com/allpass-filter/
struct Allpass2 {
  Allpass2() = default;
  Allpass2(f bw) { setBandwidth(bw); }
  Allpass2(f freq, f bw) {
    setFreq(freq);
    setBandwidth(bw);
  }

  f process(f x) {
    f a = (1_f - c) * vm1;
    f v = x - d * a + c * vm2;
    f y = -c * v + d * a + vm2;
    vm2 = vm1;
    vm1 = v;
    return y;
  }

  void setBandwidth(f bw) { c = Math::tan(Math::pi * bw) - 1_f; }
  void setFreq(f freq) { d = -Math::cos(2_f * Math::pi * freq); }

 private:
  f vm1 = 0_f, vm2 = 0_f;
  f c = 0_f, d = 0_f;
};

class SimpleFloat {
  f value_;

 public:
  explicit SimpleFloat(f value) noexcept : value_(value) {}
  f next() noexcept { return value_; }
  void set(f value, int) noexcept { value_ = value; }
  void jump(f value) noexcept { value_ = value; }
};

template <class FLOAT>
class InterpolatedFloat {
  f value_;
  FLOAT increment_;

 public:
  explicit InterpolatedFloat(f value) noexcept : value_(value), increment_(0_f) {}
  InterpolatedFloat() noexcept : value_(0_f), increment_(0_f) {}

  /** Returns next interpolated value. Warning: do not call more than
   * [time] times between two calls to [set]
   */
  f next() noexcept {
    value_ += increment_.next();
    return value_;
  }

  /** Initializes a ramp from current value to [value] in [time]
   * steps (i.e. calls to [next]) */
  void set(f value, int time) noexcept { increment_.set((value - value_) * (1_f / f(time)), time); }

  /** Instantly sets the value */
  void jump(f value) noexcept {
    value_ = value;
    increment_.jump(0_f);
  }
};

template <class FLOAT>
class ProtectedInterpolatedFloat {
  f value_;
  f target_;
  FLOAT increment_;

 public:
  explicit ProtectedInterpolatedFloat(f value) noexcept : value_(value), increment_(0_f) {}
  ProtectedInterpolatedFloat() noexcept : value_(0_f), increment_(0_f) {}

  /** Returns next interpolated value. Will stale to the target value after reaching it. */
  f next() noexcept {
    f increment = increment_.next();
    value_ += increment;
    if (increment > 0_f) value_ = value_.min(target_);
    else value_ = value_.max(target_);
    return value_;
  }

  /** Initializes a ramp from current value to [value] in [time]
   * steps (i.e. calls to [next]) */
  void set(f target, int time) noexcept {
    target_ = target;
    increment_.set((target - value_) / f(time), time);
  }

  /** Instantly sets the value */
  void jump(f value) noexcept {
    value_ = target_ = value;
    increment_.jump(0_f);
  }
};

using PIFloat = ProtectedInterpolatedFloat<SimpleFloat>;
using IFloat = InterpolatedFloat<SimpleFloat>;
using IIFloat = InterpolatedFloat<InterpolatedFloat<SimpleFloat>>;
}  // namespace grm
