#pragma once

// #include <cassert>  // for assert
#include <cmath>    // for exp2, tanf
#include <cstdint>  // for uint32_t

#include "buffer.h"    // for DiffBuffer
#include "numtypes.h"  // for f, operator""_f, Float, s1_15, Float::BinRepr
#include "util.h"      // for bit_cast, ilog2, is_power_of_2

namespace grm {

struct Math {
  Math() {
    cosine_table = CosineTable();
    sine_table = SineTable();
    exp2_table = Exp2Table();
    log2_table = Log2Table();
    xfade_table = XFadeTable();
  }

  static constexpr f pi = 3.14159265358979323846264338327950288_f;
  static constexpr f pi_pow_2 = Math::pi * Math::pi;
  static constexpr f pi_pow_3 = pi_pow_2 * Math::pi;
  static constexpr f pi_pow_5 = pi_pow_3 * pi_pow_2;
  static constexpr f pi_pow_7 = pi_pow_5 * pi_pow_2;
  static constexpr f pi_pow_9 = pi_pow_7 * pi_pow_2;
  static constexpr f pi_pow_11 = pi_pow_9 * pi_pow_2;

  static constexpr f ln_2 = 0.69314718055994530941_f;
  static constexpr f log2_e = 1.44269504088896340736_f;
  static constexpr f log2_10 = 3.32192809488736234789_f;

  // math.h wrappers
  static f sin(f x) noexcept { return f(std::sin(x.repr())); }
  static f cos(f x) noexcept { return f(std::cos(x.repr())); }
  static f tan(f x) noexcept { return f(std::tan(x.repr())); }
  static f exp2(f x) noexcept { return f(std::exp2(x.repr())); }
  static f pow(f x, f y) noexcept { return f(std::pow(x.repr(), y.repr())); }

  // [0..1] --> [0..1], f(0)=0, f(1)=1, f'(0)=0, f'(1)=0
  static constexpr f fast_raised_cosine(f x) noexcept { return x * x * (3_f - x * 2_f); }

  // [0..1] --> [-1..1]
  static constexpr f faster_sine(f x) noexcept {
    x = (x * 2_f) - 1_f;
    return 4_f * (x - x * x.abs());
  }

  // WARNING untested:
  // [0..1] --> [-1..1]
  static constexpr s1_15 faster_sine(u0_32 x) noexcept {
    s1_31 y = x.to_signed_scale();
    s1_15 z = s1_15::narrow(y * 2);
    z -= s1_15::narrow(z * z.abs());
    z *= 2;
    z = z.add_sat(z);
    return z;
  }

  // [0..1] --> [-1..1]
  static constexpr f fast_sine(f x) noexcept {
    f y = faster_sine(x);
    y = 0.225_f * (y * y.abs() - y) + y;
    return y;
  }

  static f faster_inverse_sqrt(f x) noexcept {
    float number = x.repr();
    float const y = bit_cast<float>(0x5f3759df - (bit_cast<std::uint32_t>(number) >> 1));
    return f(y * (1.5f - (number * 0.5f * y * y)));
  }

  static f fast_inverse_sqrt(f x) noexcept {
    f y = faster_inverse_sqrt(x);
    y = y * (1.5_f - (x * 0.5_f * y * y));  // 2nd iteration, this can be removed
    return y;
  }

  // [0..1] --> [-1..1]
  static constexpr f sine(f x) noexcept { return sine_table.interpolateNormalized(x); }

  // [0..1] --> [-1..1]
  static constexpr f cosine(f x) noexcept { return cosine_table.interpolateNormalized(x); }

  static constexpr f faster_tanh(f x) noexcept { return x * (27_f + x * x) / (27_f + 9_f * x * x); }

  static constexpr f fast_tanh(f x) noexcept { return faster_tanh(x).min(1_f).max(-1_f); }

  static f fast_exp2(f x) noexcept {
    static_assert(is_power_of_2(exp2_size));
    constexpr int const BITS = ilog2(exp2_size);

    constexpr f const fact = f(1 << 23);
    u32 i = u32(((x + 127_f) * fact));
    auto c = f::BinRepr(i);
    c.setMantisa(exp2_table[int(c.mantisa()) >> (23 - BITS)]);
    return c.get();
  }

  static f fast_exp(f x) noexcept { return fast_exp2(x * log2_e); }

  static f fast_exp10(f x) noexcept { return fast_exp2(x * log2_10); }

  // the bigger [shift] is, the closer to zero this function is defined
  // fast_log2 : [1/(1<<shift) .. inf] -> [-inf .. inf]
  template <int shift = 32>
  static s9_23 fast_log2(f x) {
    // assert(x >= 1_f / f(1ULL << shift));
    static_assert(is_power_of_2(log2_size));
    constexpr int const BITS = ilog2(log2_size);

    x *= f(1ULL << shift);

    auto c = f::BinRepr(x);
    c.setExponent(c.exponent() - 127);

    c.setMantisa(log2_table[int(c.mantisa() >> (23 - BITS))]);

    return s9_23::of_repr(int(c.getInt())) - s9_23(shift);
  }

  template <int shift = 32>
  static constexpr f fast_ln(f x) {
    return f(fast_log2<shift>(x)) / ln_2;
  }

  template <int shift = 32>
  static constexpr f fast_log10(f x) {
    return f(fast_log2<shift>(x)) / log2_10;
  }

  static constexpr f softclip1(f x) noexcept {
    x *= 0.6666_f;
    return (x * (3_f - x * x)) * 0.5_f;
  }

  static constexpr f softclip2(f x) noexcept {
    x *= 0.536_f;
    f s = x * x;
    return x * (1.875_f + s * (-1.25_f + 0.375_f * s));
  }

  // quarter-turn cosine curve, based on LUT
  static constexpr f equal_power_crossfade(f x, f y, f c) noexcept {
    return xfade_table.interpolateNormalized(1_f - c) * x +
           xfade_table.interpolateNormalized(c) * y;
  }

  // polynomial approximation of quarter-turn cosine (faster_sine)
  static constexpr f faster_equal_power_crossfade(f x, f y, f c) noexcept {
    f a = 1_f - c.square();
    f b = 1_f - (1_f - c).square();
    return a * x + b * y;
  }

  // -inf..inf -> -1..1
  static constexpr s1_15 triangle(s17_15 x) noexcept {
    s1_15 y = s1_15::wrap(x);
    /* fixes bug when inverting minimum value (since s1_15 are not symmetrical)
     */
    if (y == -1_s1_15) y = y.succ();
    auto p = short(-(((x + 1_s17_15).div2<1>()).even()) * 2 + 1);
    return s1_15(y * p);
  }

  // -2..2 -> -1..1
  static constexpr f triangle(f x) noexcept { return x < -1_f ? -x - 2_f : x > 1_f ? 2_f - x : x; }

  enum class TanApprox { EXACT, ACCURATE, FAST, DIRTY };

  template <TanApprox approximation>
  static constexpr f fast_tan(f x) noexcept {
    if constexpr (approximation == TanApprox::EXACT) {
      // Clip coefficient to about 100.
      x = x.min(0.497_f);
      return Math::tan(Math::pi * x);
    } else if constexpr (approximation == TanApprox::DIRTY) {
      // Optimized for frequencies below 8kHz.
      const f a = 3.736e-01_f * pi_pow_3;
      return x * (Math::pi + a * x * x);
    } else if constexpr (approximation == TanApprox::FAST) {
      // The usual tangent approximation uses 3.1755e-01 and 2.033e-01, but
      // the coefficients used here are optimized to minimize error for the
      // 16Hz to 16kHz range, with a sample rate of 48kHz.
      const f a = 3.260e-01_f * pi_pow_3;
      const f b = 1.823e-01_f * pi_pow_5;
      f x2 = x * x;
      return x * (Math::pi + x2 * (a + b * x2));
    } else if constexpr (approximation == TanApprox::ACCURATE) {
      // These coefficients don't need to be tweaked for the audio range.
      const f a = 3.333314036e-01_f * pi_pow_3;
      const f b = 1.333923995e-01_f * pi_pow_5;
      const f c = 5.33740603e-02_f * pi_pow_7;
      const f d = 2.900525e-03_f * pi_pow_9;
      const f e = 9.5168091e-03_f * pi_pow_11;
      f x2 = x * x;
      return x * (Math::pi + x2 * (a + x2 * (b + x2 * (c + x2 * (d + x2 * e)))));
    }
  }

  static constexpr int gauss_sum(int n) {
    // assert(n > 0);
    return n * (n - 1) / 2;
  }

  static f hypot(f x, f y) { return f(std::hypot(x.repr(), y.repr())); }

 private:
  static constexpr int exp2_size = 8192;
  static constexpr double exp2_increment = 1.000084616272694313202633;  // 2 ^ (1/exp2_size)
  struct Exp2Table : Array<uint32_t, exp2_size> {
    Exp2Table() noexcept;
  } static exp2_table;

  static constexpr int log2_size = 1024;
  struct Log2Table : Array<uint32_t, log2_size> {
    Log2Table() noexcept;
  } static log2_table;

  static constexpr int xfade_size = 32;
  struct XFadeTable : DiffBuffer<f, xfade_size + 1> {
    XFadeTable() noexcept;
  } static xfade_table;

  static constexpr int sine_size = 1024;
  struct SineTable : DiffBuffer<f, sine_size + 1> {
    SineTable() noexcept;
  } static sine_table;

  struct CosineTable : DiffBuffer<f, sine_size + 1> {
    CosineTable() noexcept;
  } static cosine_table;
};

// overload for std::complex
inline f hypot(f x, f y) { return Math::hypot(x, y); }

}  // namespace grm
