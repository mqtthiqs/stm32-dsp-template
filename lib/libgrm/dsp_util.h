#pragma once

#include "numtypes.h"

namespace grm {

// vector of two values of type T
template <typename T>
struct Vec2 : Pair<T> {
  using Pair<T>::Pair;

  // initialize with 1 argument -> copy
  template <typename U>
  explicit constexpr Vec2(U x) : Vec2(T(x), T(x)) {}

  constexpr Vec2 operator+(Vec2 const& that) const {
    return {this->first + that.first, this->second + that.second};
  }
  constexpr Vec2 operator-(Vec2 const& that) const {
    return {this->first - that.first, this->second - that.second};
  }
  constexpr Vec2 operator*(Vec2 const& that) const {
    return {this->first * that.first, this->second * that.second};
  }
  constexpr Vec2 operator/(Vec2 const& that) const {
    return {this->first / that.first, this->second / that.second};
  }
};

struct Interval : Vec2<f> {
  constexpr Interval(Vec2<f>&& v) noexcept : Vec2<f>(std::move(v)) {}
  constexpr Interval(Vec2<f>& v) noexcept : Vec2<f>(v) {}
  constexpr Interval(f min, f max) noexcept : Vec2(min, max) { assert(min <= max); }

  template <typename U>
  constexpr Interval(U minmax) noexcept : Vec2(minmax) {}
  constexpr Interval() noexcept : Interval(0_f) {}

  f min() { return first; }
  f max() { return second; }
  f length() { return max() - min(); }

  Interval withMinimumLength(f minLength) {
    f newMax = length() < minLength ? min() + minLength : max();
    return {min(), newMax};
  }

  Interval enlarged(f x) {
    return x > max() ? Interval(min(), x) : x < min() ? Interval(x, max()) : *this;
  }
};

struct IntervalJoiner {
  IntervalJoiner(Interval start) : previous_(std::move(start)) {}

  // returns one of the smallest intervals that:
  // 1. contains [x],
  // 2. has an intersection with the previous processed interval of at least [thickness]
  Interval process(Interval x, f thickness) {
    auto newMin = x.min().min(previous_.max() - thickness);
    auto newMax = x.max().max(previous_.min() + thickness);
    return previous_ = Interval(newMin, newMax);
  }

 private:
  Interval previous_;
};

struct DspUtil {
  static constexpr f crossfade(f const x, f const y, f const phase) noexcept {
    return x + (y - x) * phase;
  }

  template <int FRAC>
  static constexpr f crossfade(f const x, f const y,
                               Fixed<UNSIGNED, 0, FRAC> const phase) noexcept {
    return crossfade(x, y, f(phase));
  }

  static constexpr s1_15 crossfade(s1_15 const x, s1_15 const y, f const phase) noexcept {
    return x + s1_15::narrow((y - x) * s1_15::inclusive(phase));
  }

  static constexpr s1_15 crossfade(s1_15 const x, s1_15 const y, u0_32 const phase) noexcept {
    return x + s1_15::narrow((y - x) * u0_16::narrow(phase).to_signed());
  }

  template <int INT, int FRAC>
  static constexpr Fixed<SIGNED, INT, FRAC> crossfade(Fixed<SIGNED, INT, FRAC> x,
                                                      Fixed<SIGNED, INT, FRAC> y,
                                                      u0_32 phase) noexcept {
    // TODO: fix and optimize
    return Fixed<SIGNED, INT, FRAC>(f(x) + (f(y) - f(x)) * f(phase));
  }

  static constexpr s1_15 crossfade(s1_15 const x, s1_15 const y, u0_16 const phase) noexcept {
    return x + s1_15::narrow((y - x) * phase.to_signed());
  }

  static constexpr u0_8 crossfade(u0_8 const x, u0_8 const y, u0_8 const phase) noexcept {
    return x + u0_8::narrow(((y.to_signed() - x.to_signed()) * phase.to_signed()).to_unsigned());
  }

  template <typename T, typename Phase>
  static constexpr Vec2<T> crossfade(Vec2<T> const x, Vec2<T> const y, Phase const phase) noexcept {
    return {crossfade(x.first, y.first, phase), crossfade(x.second, y.second, phase)};
  }

  static constexpr s1_15 crossfade_with_diff(s1_15 const a, s1_15 const d,
                                             u0_32 const fractional) noexcept {
    return a + s1_15::narrow(d * u0_16::narrow(fractional).to_signed());
  }

  static constexpr f crossfade_with_diff(f const a, f const d, f const fractional) noexcept {
    return a + d * fractional;
  }

  static constexpr f crossfade_with_diff(f const a, f const d, u0_32 const fractional) noexcept {
    return a + d * f(fractional);
  }

  // (p..1 -> 0..1)
  static constexpr f crop_down(f const p, f const x) noexcept {
    return ((x - p) / (1_f - p)).max(0_f);
  }

  // 0..(1-p) -> 0..1
  static constexpr f crop_up(f const p, f const x) noexcept { return (x / (1_f - p)).min(1_f); }

  // p..(1-p) -> 0..1
  static constexpr f crop(f const p, f const x) noexcept {
    return ((x - p) / (1_f - 2_f * p)).min(1_f).max(0_f);
  }
};
}  // namespace grm
