#pragma once

#include <cassert>
#include <cmath>
#include <limits>

#include "util.h"

namespace grm {

struct Float;
struct Float16;

enum sign { UNSIGNED, SIGNED };
template <sign SIGN, int INT, int FRAC>
struct Fixed;

// Boundary values:
template <typename T>
static constexpr T min_val;
template <typename T>
static constexpr T max_val;
template <typename T>
static constexpr T increment;

/**************
 * 32-bits Floating Point
 **************/

struct Float {
  using T = Float;
  Float() = default;
  constexpr explicit Float(float v) noexcept : val_(v) {}
  constexpr explicit Float(int i) noexcept : val_(float(i)) {}
  constexpr explicit Float(unsigned int i) noexcept : val_(float(i)) {}
  constexpr explicit Float(long i) noexcept : val_(float(i)) {}
  constexpr explicit Float(unsigned long i) noexcept : val_(float(i)) {}
  constexpr explicit Float(long long i) noexcept : val_(float(i)) {}
  constexpr explicit Float(unsigned long long i) noexcept : val_(float(i)) {}

  constexpr explicit Float(Float16) noexcept;

  template <sign SIGN, int INT, int FRAC>
  constexpr explicit Float(Fixed<SIGN, INT, FRAC>) noexcept;

  template <sign SIGN, int INT, int FRAC>
  static constexpr Float inclusive(Fixed<SIGN, INT, FRAC>) noexcept;

  constexpr float repr() const noexcept { return val_; }
  constexpr T operator+(T const y) const noexcept { return T(repr() + y.repr()); }
  constexpr T operator-(T const y) const noexcept { return T(repr() - y.repr()); }
  constexpr T operator-() const noexcept { return T(-repr()); }
  constexpr T operator*(T const y) const noexcept { return T((repr() * y.repr())); }
  constexpr T operator/(T const y) const noexcept { return T((repr()) / y.repr()); }

  constexpr bool operator<(T const y) const noexcept { return repr() < y.repr(); }
  constexpr bool operator>(T const y) const noexcept { return repr() > y.repr(); }
  constexpr bool operator<=(T const y) const noexcept { return repr() <= y.repr(); }
  constexpr bool operator>=(T const y) const noexcept { return repr() >= y.repr(); }
  constexpr bool operator==(T const y) const noexcept { return repr() == y.repr(); }
  constexpr bool operator!=(T const y) const noexcept { return repr() != y.repr(); }

  constexpr void operator+=(T const y) noexcept { val_ += y.repr(); }
  constexpr void operator-=(T const y) noexcept { val_ -= y.repr(); }
  constexpr void operator*=(T const y) noexcept { val_ = val_ * y.repr(); }
  constexpr void operator/=(T const y) noexcept { val_ = val_ / y.repr(); }
  constexpr void operator++() noexcept { ++val_; }
  constexpr void operator--() noexcept { --val_; }
  constexpr void operator++(int const) noexcept { val_++; }
  constexpr void operator--(int const) noexcept { val_--; }

  // Signum function
  constexpr T sgn() const noexcept { return T((T(0) < *this) - (*this < T(0))); }

  // compiles to vabs.f32 on arm
  constexpr T abs() const noexcept { return T(val_ > 0 ? val_ : -val_); }

  T sqrt() const noexcept {
    return T(std::sqrt(val_));
  }

  constexpr T min(const T y) const noexcept {
    return *this < y ? *this : y;
  }

  constexpr T max(const T y) const noexcept {
    return *this < y ? y : *this;
  }

  constexpr T clip(T const x, T const y) const noexcept { return max(x).min(y); }
  constexpr T clip() const noexcept { return clip(T(-1.0f), T(1.0f)); }

  constexpr T floor() const noexcept {
    return T(static_cast<float>(integral()));
  }

  constexpr T ceil() const noexcept {
    T y = floor();
    return y + T(y != *this);
  }

  constexpr T round() const noexcept { return (*this + T(0.5f)).floor(); }

  constexpr int integral() const noexcept { return static_cast<int>(val_); }

  // if x>=0, 0<=fractional<1; if x<0, -1<fractional<=0
  constexpr T fractional() const noexcept { return *this - floor(); }

  constexpr std::pair<int, T> integral_fractional() const noexcept {
    int i = static_cast<int>(val_);
    T f = *this - T(static_cast<float>(i));
    return {i, f};
  }

  constexpr T square() const noexcept { return *this * *this; }

  constexpr T cube() const noexcept { return *this * *this * *this; }

  constexpr T scaleFrom01(T min, T max) const noexcept { return min + *this * (max - min); }

  constexpr T scaleFrom01(Pair<T> minmax) const noexcept {
    return minmax.first + *this * (minmax.second - minmax.first);
  }

  constexpr T scale(Pair<T> const sourceRange, Pair<T> const targetRange) const noexcept {
    // assert(sourceRange.second !=
    //        sourceRange.first);  // mapping from a range of zero will produce NaN!
    return targetRange.first + (targetRange.second - targetRange.first) *
                                   (*this - sourceRange.first) /
                                   (sourceRange.second - sourceRange.first);
  }

  bool isFinite() const noexcept { return std::isfinite(val_); }

  // float representation conversion
  struct BinRepr {
    explicit BinRepr(T x) : repr_(*reinterpret_cast<Repr*>(&x)) {}                        // NOLINT
    explicit BinRepr(Fixed<UNSIGNED, 32, 0>& x) : repr_(*reinterpret_cast<Repr*>(&x)) {}  // NOLINT

    T get() { return *reinterpret_cast<T*>(&repr_); }                   // NOLINT
    uint32_t getInt() { return *reinterpret_cast<uint32_t*>(&repr_); }  // NOLINT

    // false for >=0, true for <0
    bool sign() { return repr_.sign; }
    uint32_t mantisa() { return repr_.mantisa; }
    uint8_t exponent() { return repr_.exponent; }
    void setMantisa(uint32_t m) { repr_.mantisa = m; }
    void setExponent(uint8_t e) { repr_.exponent = e; }
    void setSign(bool s) { repr_.sign = s; }

   private:
    struct Repr {
      uint32_t mantisa : 23;
      uint32_t exponent : 8;
      uint32_t sign : 1;
    } repr_;
  };

  BinRepr bin() { return BinRepr(*this); }

 private:
  float val_;
};

// Abbreviation aliases

using f32 = Float;
using f = f32;

// Boundary values

static constexpr f infinity = f(std::numeric_limits<float>::infinity());
static constexpr f minus_infinity = f(-std::numeric_limits<float>::infinity());

// Parser and printer

constexpr f operator"" _f(long double const x) noexcept { return f(float(x)); }
constexpr f operator"" _f(unsigned long long int const x) noexcept { return f(float(x)); }

/***************
 * Fixed-Point
 ***************/

template <int WIDTH, sign SIGN>
struct Basetype;

template <>
struct Basetype<8, SIGNED> {
  using T = int8_t;
};
template <>
struct Basetype<8, UNSIGNED> {
  using T = uint8_t;
};
template <>
struct Basetype<16, SIGNED> {
  using T = int16_t;
};
template <>
struct Basetype<16, UNSIGNED> {
  using T = uint16_t;
};
template <>
struct Basetype<32, SIGNED> {
  using T = int32_t;
};
template <>
struct Basetype<32, UNSIGNED> {
  using T = uint32_t;
};
template <>
struct Basetype<64, SIGNED> {
  using T = int64_t;
};
template <>
struct Basetype<64, UNSIGNED> {
  using T = uint64_t;
};

template <sign SIGN, int INT, int FRAC>
struct Fixed {
  static constexpr int INTEGRAL = INT;
  static constexpr int FRACTIONAL = FRAC;
  static constexpr int WIDTH = INT + FRAC;

  template <sign, int, int>
  friend struct Fixed;

  using Wider = Fixed<SIGN, (WIDTH * 2) - FRAC, FRAC>;

 private:
  using T = Fixed<SIGN, INT, FRAC>;
  using Base = typename Basetype<WIDTH, SIGN>::T;

  // Actual value
  Base val_;

  // 0b00000001
  static constexpr Base kEpsilon = static_cast<Base>(1);
  // 0b00010000
  static constexpr Base kOne = kEpsilon << FRAC;
  // 0b00001111
  static constexpr Base kFractionalMask = kOne - kEpsilon;
  // 0b11110000
  static constexpr Base kIntegralMask = ~kFractionalMask;

  template <typename A, int BITS>
  static constexpr A saturate_integer(A const x) noexcept {
    if constexpr (SIGN == SIGNED) {
      A vmin = -(1LL << (BITS - 1));
      A vmax = (1LL << (BITS - 1)) - 1;
      return x < vmin ? vmin : x > vmax ? vmax : x;
    } else {
      A vmin = 0;
      A vmax = (1ULL << BITS) - 1;
      return x < vmin ? vmin : x > vmax ? vmax : x;
    }
  }

  template <int BITS>
  constexpr T saturate() const noexcept {
    static_assert(BITS > 0 && BITS < WIDTH, "Invalid bit count");
    return T::of_repr(saturate_integer<Base, BITS>(val_));
  }

  enum struct dangerous { DANGER };
  constexpr explicit Fixed(dangerous, Base x) noexcept : val_(x) {}

 public:
  // CONSTRUCTORS

  // default constructor for arrays
  Fixed() noexcept = default;

  // unsafe constructor from representation
  static constexpr T of_repr(Base const x) noexcept { return T(dangerous::DANGER, x); }

  // from base type:
  constexpr explicit Fixed(Base const x) noexcept : val_(x * kOne) {
    // assert(x >= min_val<T>.repr() >> FRAC);
    // assert(x <= max_val<T>.repr() >> FRAC);
  }

  // from base types, for literal notations (see below)
  static constexpr T of_long_long(unsigned long long int const x) noexcept {
    return T::of_repr(static_cast<Base>(x * (1ULL << FRAC)));
  }

  static constexpr T of_double(long double const x) noexcept {
    return T(dangerous::DANGER, static_cast<Base>((x * (long double)(1ULL << FRAC))));
  }

  // from Float:
  constexpr explicit Fixed(f const x) noexcept
      : val_(static_cast<Base>((x * Float(1ULL << FRAC)).repr())) {}

  static constexpr T inclusive(f const x) noexcept {
    return T::of_repr(static_cast<Base>((x * Float((1ULL << FRAC) - 1)).repr()));
  }

  // from Fixed:
  template <int INT2, int FRAC2>
  constexpr explicit Fixed(Fixed<SIGN, INT2, FRAC2> const that) noexcept
      : val_(Fixed<SIGN, INT, FRAC>::of_repr(Base(that.repr()) * (kEpsilon << (FRAC - FRAC2)))
                 .repr()) {
    static_assert(FRAC2 <= FRAC, "Conversion with possible loss of precision");
    static_assert(INT2 <= INT, "Conversion with possible wrapover");
  }

  // from Fixed with truncation of fractional part (allows loss of precision)
  template <int INT2, int FRAC2>
  static constexpr T narrow(Fixed<SIGN, INT2, FRAC2> const that) noexcept {
    static_assert(FRAC2 > FRAC, "This is not a narrowing: use default constructor");
    static_assert(INT2 <= INT, "Conversion with possible wrapover");
    return T::of_repr(that.repr() >> (FRAC2 - FRAC));
  }

  // from Fixed with truncation of integer & fractional part (allows
  // wrapping & loss of precision); also allows changing sign!
  // TODO test
  template <sign SIGN2, int INT2, int FRAC2>
  static constexpr T wrap(Fixed<SIGN2, INT2, FRAC2> const that) noexcept {
    static_assert(INT2 > INT, "This does not wrap: use default constructor");
    if constexpr (FRAC2 >= FRAC) {
      return T::of_repr(typename T::Base(that.repr() >> (FRAC2 - FRAC)));
    } else return T::of_repr(typename T::Base(that.repr() << (FRAC - FRAC2)));
  }

  // CONVERSIONS

  // unsafe getter for representation
  constexpr Base repr() const noexcept { return val_; }

  // TODO optimize for ARM: SAT/USAT instructions have built-in shift
  template <int INT2, int FRAC2>
  constexpr Fixed<SIGN, INT2, FRAC2> const to_sat() const noexcept {
    static_assert(INT2 < INT, "will not saturate if destination is wider than source");
    Base x = saturate<WIDTH - (INT - INT2)>().repr();

    if constexpr (FRAC2 >= FRAC) {
      return Fixed<SIGN, INT2, FRAC2>::of_repr(x * (kEpsilon << (FRAC2 - FRAC)));
    } else {
      return Fixed<SIGN, INT2, FRAC2>::of_repr(x >> (FRAC - FRAC2));
    }
  }

  // preserves the represented value
  constexpr auto to_signed() const noexcept {
    static_assert(SIGN == UNSIGNED, "Only signed-to-unsigned conversion supported");
    using S = Fixed<SIGNED, INT + 1, FRAC - 1>;
    return S::of_repr(typename S::Base(val_ >> 1));
  }

  // remaps MIN..MAX to MIN..MAX (e.g. 0..1 --> -1..1)
  constexpr auto to_signed_scale() const noexcept {
    static_assert(SIGN == UNSIGNED, "Only signed-to-unsigned conversion supported");
    using S = Fixed<SIGNED, INT + 1, FRAC - 1>;
    return S::of_repr(typename S::Base(val_ - (1U << (WIDTH - 1))));
  }

  // preserves the represented value if it is positive
  constexpr auto to_unsigned() const noexcept {
    static_assert(SIGN == SIGNED, "Only unsigned-to-signed conversion supported");
    using S = Fixed<UNSIGNED, INT - 1, FRAC + 1>;
    return S::of_repr(typename S::Base(val_ * 2));
  }

  // remaps MIN..MAX to MIN..MAX (e.g. -1..1 --> 0..1)
  constexpr auto to_unsigned_scale() const noexcept {
    static_assert(SIGN == SIGNED, "Only unsigned-to-signed conversion supported");
    using S = Fixed<UNSIGNED, INT - 1, FRAC + 1>;
    return S::of_repr(typename S::Base(val_ + (kEpsilon << (WIDTH - 1))));
  }

  template <int SHIFT>
  constexpr Fixed<SIGN, INT + SHIFT, FRAC - SHIFT> const movr() const noexcept {
    return Fixed<SIGN, INT + SHIFT, FRAC - SHIFT>::of_repr(val_);
  }

  template <int SHIFT>
  constexpr Fixed<SIGN, INT - SHIFT, FRAC + SHIFT> const movl() const noexcept {
    return Fixed<SIGN, INT - SHIFT, FRAC + SHIFT>::of_repr(val_);
  }

  template <int SHIFT>
  constexpr Fixed<SIGN, INT + SHIFT, FRAC - SHIFT> const shiftr() const noexcept {
    return Fixed<SIGN, INT + SHIFT, FRAC - SHIFT>::of_repr(val_ >> SHIFT);
  }

  template <int SHIFT>
  constexpr Fixed<SIGN, INT - SHIFT, FRAC + SHIFT> const shiftl() const noexcept {
    return Fixed<SIGN, INT - SHIFT, FRAC + SHIFT>::of_repr(val_ << SHIFT);
  }

  constexpr Wider widen() const noexcept { return Wider::of_repr(repr()); }

  // Operations:

  // in/decrement by the smallest amount possible in the representation
  constexpr T succ() const noexcept { return T::of_repr(repr() + 1L); }
  constexpr T pred() const noexcept { return T::of_repr(repr() - 1L); }

  constexpr T floor() const noexcept { return T::of_repr(repr() & kIntegralMask); }
  constexpr T ceil() const noexcept {
    T y = floor();
    return y + T(y != *this);
  }

  constexpr T frac() const noexcept { return T::of_repr(repr() & kFractionalMask); }

  constexpr T sgn() const noexcept { return T::of_repr((val_ > 0) - (val_ < 0)); }
  constexpr T abs() const noexcept { return T::of_repr(val_ >= 0 ? val_ : -val_); }

  constexpr bool even() const noexcept { return val_ & T(1).repr(); }

  constexpr Base integral() const noexcept { return Fixed<SIGN, WIDTH, 0>::narrow(*this).repr(); }

  constexpr Fixed<SIGN, 0, WIDTH> fractional() const noexcept {
    return Fixed<SIGN, 0, WIDTH>::wrap(*this);
  }

  constexpr std::pair<Base, Fixed<SIGN, 0, WIDTH>> integral_fractional() const noexcept {
    return {integral(), fractional()};
  }

  constexpr T operator-() const noexcept {
    static_assert(SIGN == SIGNED, "Prefix negation is invalid on unsigned data");
    return T::of_repr(-repr());
  }

  template <int INT2, int FRAC2>
  constexpr auto operator+(Fixed<SIGN, INT2, FRAC2> const y) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T::of_repr(T(*this).repr() + T(y).repr());
  }

  template <int INT2, int FRAC2>
  constexpr auto operator-(Fixed<SIGN, INT2, FRAC2> const y) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T::of_repr(T(*this).repr() - T(y).repr());
  }

  template <int INT2, int FRAC2>
  constexpr auto operator*(Fixed<SIGNED, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGNED, INT + INT2 - 1, FRAC + FRAC2 + 1>;
    using B = typename T::Base;
    // TODO understand this * 2!
    return T::of_repr((B(repr()) * B(that.repr())) * B(2));
  }

  template <int INT2, int FRAC2>
  constexpr auto operator*(Fixed<UNSIGNED, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<UNSIGNED, INT + INT2, FRAC + FRAC2>;
    using B = typename T::Base;
    return T::of_repr((B(repr()) * B(that.repr())));
  }

  // TODO: understand the division case!
  template <int INT2, int FRAC2>
  constexpr auto operator/(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using Wider = typename Basetype<WIDTH + INT2 + FRAC2, SIGN>::T;
    Wider x = repr();
    x <<= INT2 + FRAC2;
    return Fixed<SIGN, INT + FRAC2, FRAC + INT2>::of_repr(x / that.repr());
  }

  // Degraded basic operators TODO: clean!
  constexpr T operator*(Base const y) const noexcept { return T::of_repr(repr() * y); }

  constexpr T operator/(Base const y) const noexcept { return T::of_repr(repr() / y); }

  template <int BITS>
  constexpr T div2() const noexcept {
    static_assert(BITS >= 0 && BITS < WIDTH, "Invalid bit count");
    return this->template shiftr<BITS>().template movl<BITS>();
  }

  constexpr auto div2(int bits) const noexcept {
    return Fixed<SIGN, INT, FRAC>::of_repr(val_ >> bits);
  }

  constexpr void operator+=(T const that) noexcept { *this = *this + that; }
  constexpr void operator-=(T const that) noexcept { *this = *this - that; }
  constexpr void operator*=(T const that) noexcept { *this = T::narrow(*this * that); }
  constexpr void operator/=(T const that) noexcept { *this = T::narrow(*this / that); }
  constexpr void operator*=(Base const that) noexcept { val_ = (*this * that).repr(); }
  constexpr void operator/=(Base const that) noexcept { val_ = (*this / that).repr(); }

  constexpr T operator++() noexcept {
    T temp = *this;
    *this += T(1_f);
    return temp;
  }
  constexpr T operator--() noexcept {
    T temp = *this;
    *this -= T(1_f);
    return temp;
  }
  constexpr T const operator++(int) noexcept {
    *this += T(1_f);
    return *this;
  }
  constexpr T const operator--(int) noexcept {
    *this -= T(1_f);
    return *this;
  }

  constexpr T operator%(T const that) const noexcept {
    static_assert(SIGN == UNSIGNED, "modulo undefined on unsigned types");
    return T::of_repr(val_ % that.val_);
  }

  template <int INT2, int FRAC2>
  constexpr bool operator<(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() < T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr bool operator>(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() > T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr bool operator<=(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() <= T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr bool operator>=(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() >= T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr bool operator==(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() == T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr bool operator!=(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    using T = Fixed<SIGN, grm::max(INT, INT2), grm::max(FRAC, FRAC2)>;
    return T(*this).repr() != T(that).repr();
  }

  template <int INT2, int FRAC2>
  constexpr auto min(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    return *this < that ? *this : that;
  }

  template <int INT2, int FRAC2>
  constexpr auto max(Fixed<SIGN, INT2, FRAC2> const that) const noexcept {
    return *this < that ? that : *this;
  }

  constexpr T clip(T const x, T const y) const noexcept { return max(x).min(y); }

  // saturates between -1 and 1 (signed) or 0 and 1 (unsigned)
  constexpr T clip() const { return SIGN == SIGNED ? saturate<FRAC + 1>() : saturate<FRAC>(); }

  // saturating add/sub
  constexpr T add_sat(const T y) const noexcept {
    using Wider = typename Basetype<WIDTH * 2, SIGN>::T;
    Wider r = static_cast<Wider>(val_) + static_cast<Wider>(y.val_);
    r = saturate_integer<Wider, WIDTH>(r);
    return T::of_repr(static_cast<Base>(r));
  }

  constexpr T sub_sat(const T y) const noexcept {
    using Wider = typename Basetype<WIDTH * 2, SIGN>::T;
    Wider r = static_cast<Wider>(val_) - static_cast<Wider>(y.val_);
    r = saturate_integer<Wider, WIDTH>(r);
    return T::of_repr(static_cast<Base>(r));
  }
};

// Boundary values:
template <sign SIGN, int INT, int FRAC>
static constexpr auto min_val<Fixed<SIGN, INT, FRAC>> = Fixed<SIGN, INT, FRAC>::of_repr(
    std::numeric_limits<typename Basetype<INT + FRAC, SIGN>::T>::min());

template <sign SIGN, int INT, int FRAC>
static constexpr auto max_val<Fixed<SIGN, INT, FRAC>> = Fixed<SIGN, INT, FRAC>::of_repr(
    std::numeric_limits<typename Basetype<INT + FRAC, SIGN>::T>::max());

template <sign SIGN, int INT, int FRAC>
static constexpr auto increment<Fixed<SIGN, INT, FRAC>> = Fixed<SIGN, INT, FRAC>::of_repr(1);

// Conversions Fixed -> Float

template <int FRAC>
static constexpr float kFixedConversionFactor = fpow2(-FRAC);

// TODO: fix when FRAC=64 (conversion from 64-bit Fixed)
template <int FRAC>
static constexpr float kInclusiveFixedConversionFactor = 1.f /
                                                         static_cast<float>((1ULL << FRAC) - 1);

template <sign SIGN, int INT, int FRAC>
constexpr f::Float(Fixed<SIGN, INT, FRAC> that) noexcept
    : val_(static_cast<float>(that.repr()) * kFixedConversionFactor<FRAC>) {}

template <sign SIGN, int INT, int FRAC>
constexpr f f::inclusive(Fixed<SIGN, INT, FRAC> that) noexcept {
  return f(static_cast<float>(that.repr()) * kInclusiveFixedConversionFactor<FRAC>);
}

// Abbreviation aliases

// 8 bits
using u8_0 = Fixed<UNSIGNED, 8, 0>;
using u7_1 = Fixed<UNSIGNED, 7, 1>;
using u6_2 = Fixed<UNSIGNED, 6, 2>;
using u5_3 = Fixed<UNSIGNED, 5, 3>;
using u4_4 = Fixed<UNSIGNED, 4, 4>;
using u3_5 = Fixed<UNSIGNED, 3, 5>;
using u2_6 = Fixed<UNSIGNED, 2, 6>;
using u1_7 = Fixed<UNSIGNED, 1, 7>;
using u0_8 = Fixed<UNSIGNED, 0, 8>;

using s8_0 = Fixed<SIGNED, 8, 0>;
using s7_1 = Fixed<SIGNED, 7, 1>;
using s6_2 = Fixed<SIGNED, 6, 2>;
using s5_3 = Fixed<SIGNED, 5, 3>;
using s4_4 = Fixed<SIGNED, 4, 4>;
using s3_5 = Fixed<SIGNED, 3, 5>;
using s2_6 = Fixed<SIGNED, 2, 6>;
using s1_7 = Fixed<SIGNED, 1, 7>;
using s0_8 = Fixed<SIGNED, 0, 8>;

// 16 bits
using u0_16 = Fixed<UNSIGNED, 0, 16>;
using u1_15 = Fixed<UNSIGNED, 1, 15>;
using u2_14 = Fixed<UNSIGNED, 2, 14>;
using u3_13 = Fixed<UNSIGNED, 3, 13>;
using u4_12 = Fixed<UNSIGNED, 4, 12>;
using u5_11 = Fixed<UNSIGNED, 5, 11>;
using u6_10 = Fixed<UNSIGNED, 6, 10>;
using u7_9 = Fixed<UNSIGNED, 7, 9>;
using u8_8 = Fixed<UNSIGNED, 8, 8>;
using u9_7 = Fixed<UNSIGNED, 9, 7>;
using u10_6 = Fixed<UNSIGNED, 10, 6>;
using u11_5 = Fixed<UNSIGNED, 11, 5>;
using u12_4 = Fixed<UNSIGNED, 12, 4>;
using u13_3 = Fixed<UNSIGNED, 13, 3>;
using u14_2 = Fixed<UNSIGNED, 14, 2>;
using u15_1 = Fixed<UNSIGNED, 15, 1>;
using u16_0 = Fixed<UNSIGNED, 16, 0>;

using s0_16 = Fixed<SIGNED, 0, 16>;
using s1_15 = Fixed<SIGNED, 1, 15>;
using s2_14 = Fixed<SIGNED, 2, 14>;
using s3_13 = Fixed<SIGNED, 3, 13>;
using s4_12 = Fixed<SIGNED, 4, 12>;
using s5_11 = Fixed<SIGNED, 5, 11>;
using s6_10 = Fixed<SIGNED, 6, 10>;
using s7_9 = Fixed<SIGNED, 7, 9>;
using s8_8 = Fixed<SIGNED, 8, 8>;
using s9_7 = Fixed<SIGNED, 9, 7>;
using s10_6 = Fixed<SIGNED, 10, 6>;
using s11_5 = Fixed<SIGNED, 11, 5>;
using s12_4 = Fixed<SIGNED, 12, 4>;
using s13_3 = Fixed<SIGNED, 13, 3>;
using s14_2 = Fixed<SIGNED, 14, 2>;
using s15_1 = Fixed<SIGNED, 15, 1>;
using s16_0 = Fixed<SIGNED, 16, 0>;

// 32 bits
using u0_32 = Fixed<UNSIGNED, 0, 32>;
using u1_31 = Fixed<UNSIGNED, 1, 31>;
using u2_30 = Fixed<UNSIGNED, 2, 30>;
using u3_29 = Fixed<UNSIGNED, 3, 29>;
using u4_28 = Fixed<UNSIGNED, 4, 28>;
using u5_27 = Fixed<UNSIGNED, 5, 27>;
using u6_26 = Fixed<UNSIGNED, 6, 26>;
using u7_25 = Fixed<UNSIGNED, 7, 25>;
using u8_24 = Fixed<UNSIGNED, 8, 24>;
using u9_23 = Fixed<UNSIGNED, 9, 23>;
using u10_22 = Fixed<UNSIGNED, 10, 22>;
using u11_21 = Fixed<UNSIGNED, 11, 21>;
using u12_20 = Fixed<UNSIGNED, 12, 20>;
using u13_19 = Fixed<UNSIGNED, 13, 19>;
using u14_18 = Fixed<UNSIGNED, 14, 18>;
using u15_17 = Fixed<UNSIGNED, 15, 17>;
using u16_16 = Fixed<UNSIGNED, 16, 16>;
using u17_15 = Fixed<UNSIGNED, 17, 15>;
using u18_14 = Fixed<UNSIGNED, 18, 14>;
using u19_13 = Fixed<UNSIGNED, 19, 13>;
using u20_12 = Fixed<UNSIGNED, 20, 12>;
using u21_11 = Fixed<UNSIGNED, 21, 11>;
using u22_10 = Fixed<UNSIGNED, 22, 10>;
using u23_9 = Fixed<UNSIGNED, 23, 9>;
using u24_8 = Fixed<UNSIGNED, 24, 8>;
using u25_7 = Fixed<UNSIGNED, 25, 7>;
using u26_6 = Fixed<UNSIGNED, 26, 6>;
using u27_5 = Fixed<UNSIGNED, 27, 5>;
using u28_4 = Fixed<UNSIGNED, 28, 4>;
using u29_3 = Fixed<UNSIGNED, 29, 3>;
using u30_2 = Fixed<UNSIGNED, 30, 2>;
using u31_1 = Fixed<UNSIGNED, 31, 1>;
using u32_0 = Fixed<UNSIGNED, 32, 0>;

using s0_32 = Fixed<SIGNED, 0, 32>;
using s1_31 = Fixed<SIGNED, 1, 31>;
using s2_30 = Fixed<SIGNED, 2, 30>;
using s3_29 = Fixed<SIGNED, 3, 29>;
using s4_28 = Fixed<SIGNED, 4, 28>;
using s5_27 = Fixed<SIGNED, 5, 27>;
using s6_26 = Fixed<SIGNED, 6, 26>;
using s7_25 = Fixed<SIGNED, 7, 25>;
using s8_24 = Fixed<SIGNED, 8, 24>;
using s9_23 = Fixed<SIGNED, 9, 23>;
using s10_22 = Fixed<SIGNED, 10, 22>;
using s11_21 = Fixed<SIGNED, 11, 21>;
using s12_20 = Fixed<SIGNED, 12, 20>;
using s13_19 = Fixed<SIGNED, 13, 19>;
using s14_18 = Fixed<SIGNED, 14, 18>;
using s15_17 = Fixed<SIGNED, 15, 17>;
using s16_16 = Fixed<SIGNED, 16, 16>;
using s17_15 = Fixed<SIGNED, 17, 15>;
using s18_14 = Fixed<SIGNED, 18, 14>;
using s19_13 = Fixed<SIGNED, 19, 13>;
using s20_12 = Fixed<SIGNED, 20, 12>;
using s21_11 = Fixed<SIGNED, 21, 11>;
using s22_10 = Fixed<SIGNED, 22, 10>;
using s23_9 = Fixed<SIGNED, 23, 9>;
using s24_8 = Fixed<SIGNED, 24, 8>;
using s25_7 = Fixed<SIGNED, 25, 7>;
using s26_6 = Fixed<SIGNED, 26, 6>;
using s27_5 = Fixed<SIGNED, 27, 5>;
using s28_4 = Fixed<SIGNED, 28, 4>;
using s29_3 = Fixed<SIGNED, 29, 3>;
using s30_2 = Fixed<SIGNED, 30, 2>;
using s31_1 = Fixed<SIGNED, 31, 1>;
using s32_0 = Fixed<SIGNED, 32, 0>;

// 64 bits
using u0_64 = Fixed<UNSIGNED, 0, 64>;
using u16_48 = Fixed<UNSIGNED, 16, 48>;
using u32_32 = Fixed<UNSIGNED, 32, 32>;
using u48_16 = Fixed<UNSIGNED, 48, 16>;
using u64_0 = Fixed<UNSIGNED, 64, 0>;

using s0_64 = Fixed<SIGNED, 0, 64>;
using s16_48 = Fixed<SIGNED, 16, 48>;
using s32_32 = Fixed<SIGNED, 32, 32>;
using s48_16 = Fixed<SIGNED, 48, 16>;
using s64_0 = Fixed<SIGNED, 64, 0>;

// shortcuts
using s8 = s8_0;
using u8 = u8_0;

using s16 = s16_0;
using u16 = u16_0;

using s32 = s32_0;
using u32 = u32_0;

using s64 = s64_0;
using u64 = u64_0;

using s = s32;
using u = u32;

// User-defined literals

// Generated with:

/*
#include <iostream>

void pr_literal(int i, int j) {
  for (char c : {'u', 's'}) {
    std::cout << "constexpr " << c << i << "_" << j
              << " operator \"\" _" << c << i << "_" << j
              << "(long double x) { return " << c << i << "_" << j
              << "::of_double(x); }" << std::endl;
    std::cout << "constexpr " << c << i << "_" << j
              << " operator \"\" _" << c << i << "_" << j
              << "(unsigned long long int x) { return " << c << i << "_" << j
              << "::of_long_long(x); }" << std::endl;
  }
}

int main()
{
  int bits = 8;

  for (int bits=8; bits <= 32; bits *= 2) {
    std::cout << "// " << bits << " bits" << std::endl;
    for (int i=0; i<bits+1; i++) {
      pr_literal(i, bits-i);
    }
    std::cout << std::endl;
  }
}
*/

// 8 bits
constexpr u0_8 operator"" _u0_8(long double x) { return u0_8::of_double(x); }
constexpr s0_8 operator"" _s0_8(long double x) { return s0_8::of_double(x); }
constexpr u0_8 operator"" _u0_8(unsigned long long int x) { return u0_8::of_long_long(x); }
constexpr s0_8 operator"" _s0_8(unsigned long long int x) { return s0_8::of_long_long(x); }
constexpr u1_7 operator"" _u1_7(long double x) { return u1_7::of_double(x); }
constexpr s1_7 operator"" _s1_7(long double x) { return s1_7::of_double(x); }
constexpr u1_7 operator"" _u1_7(unsigned long long int x) { return u1_7::of_long_long(x); }
constexpr s1_7 operator"" _s1_7(unsigned long long int x) { return s1_7::of_long_long(x); }
constexpr u2_6 operator"" _u2_6(long double x) { return u2_6::of_double(x); }
constexpr s2_6 operator"" _s2_6(long double x) { return s2_6::of_double(x); }
constexpr u2_6 operator"" _u2_6(unsigned long long int x) { return u2_6::of_long_long(x); }
constexpr s2_6 operator"" _s2_6(unsigned long long int x) { return s2_6::of_long_long(x); }
constexpr u3_5 operator"" _u3_5(long double x) { return u3_5::of_double(x); }
constexpr s3_5 operator"" _s3_5(long double x) { return s3_5::of_double(x); }
constexpr u3_5 operator"" _u3_5(unsigned long long int x) { return u3_5::of_long_long(x); }
constexpr s3_5 operator"" _s3_5(unsigned long long int x) { return s3_5::of_long_long(x); }
constexpr u4_4 operator"" _u4_4(long double x) { return u4_4::of_double(x); }
constexpr s4_4 operator"" _s4_4(long double x) { return s4_4::of_double(x); }
constexpr u4_4 operator"" _u4_4(unsigned long long int x) { return u4_4::of_long_long(x); }
constexpr s4_4 operator"" _s4_4(unsigned long long int x) { return s4_4::of_long_long(x); }
constexpr u5_3 operator"" _u5_3(long double x) { return u5_3::of_double(x); }
constexpr s5_3 operator"" _s5_3(long double x) { return s5_3::of_double(x); }
constexpr u5_3 operator"" _u5_3(unsigned long long int x) { return u5_3::of_long_long(x); }
constexpr s5_3 operator"" _s5_3(unsigned long long int x) { return s5_3::of_long_long(x); }
constexpr u6_2 operator"" _u6_2(long double x) { return u6_2::of_double(x); }
constexpr s6_2 operator"" _s6_2(long double x) { return s6_2::of_double(x); }
constexpr u6_2 operator"" _u6_2(unsigned long long int x) { return u6_2::of_long_long(x); }
constexpr s6_2 operator"" _s6_2(unsigned long long int x) { return s6_2::of_long_long(x); }
constexpr u7_1 operator"" _u7_1(long double x) { return u7_1::of_double(x); }
constexpr s7_1 operator"" _s7_1(long double x) { return s7_1::of_double(x); }
constexpr u7_1 operator"" _u7_1(unsigned long long int x) { return u7_1::of_long_long(x); }
constexpr s7_1 operator"" _s7_1(unsigned long long int x) { return s7_1::of_long_long(x); }
constexpr u8_0 operator"" _u8_0(long double x) { return u8_0::of_double(x); }
constexpr s8_0 operator"" _s8_0(long double x) { return s8_0::of_double(x); }
constexpr u8_0 operator"" _u8_0(unsigned long long int x) { return u8_0::of_long_long(x); }
constexpr s8_0 operator"" _s8_0(unsigned long long int x) { return s8_0::of_long_long(x); }

// 16 bits
constexpr u0_16 operator"" _u0_16(long double x) { return u0_16::of_double(x); }
constexpr s0_16 operator"" _s0_16(long double x) { return s0_16::of_double(x); }
constexpr u0_16 operator"" _u0_16(unsigned long long int x) { return u0_16::of_long_long(x); }
constexpr s0_16 operator"" _s0_16(unsigned long long int x) { return s0_16::of_long_long(x); }
constexpr u1_15 operator"" _u1_15(long double x) { return u1_15::of_double(x); }
constexpr s1_15 operator"" _s1_15(long double x) { return s1_15::of_double(x); }
constexpr u1_15 operator"" _u1_15(unsigned long long int x) { return u1_15::of_long_long(x); }
constexpr s1_15 operator"" _s1_15(unsigned long long int x) { return s1_15::of_long_long(x); }
constexpr u2_14 operator"" _u2_14(long double x) { return u2_14::of_double(x); }
constexpr s2_14 operator"" _s2_14(long double x) { return s2_14::of_double(x); }
constexpr u2_14 operator"" _u2_14(unsigned long long int x) { return u2_14::of_long_long(x); }
constexpr s2_14 operator"" _s2_14(unsigned long long int x) { return s2_14::of_long_long(x); }
constexpr u3_13 operator"" _u3_13(long double x) { return u3_13::of_double(x); }
constexpr s3_13 operator"" _s3_13(long double x) { return s3_13::of_double(x); }
constexpr u3_13 operator"" _u3_13(unsigned long long int x) { return u3_13::of_long_long(x); }
constexpr s3_13 operator"" _s3_13(unsigned long long int x) { return s3_13::of_long_long(x); }
constexpr u4_12 operator"" _u4_12(long double x) { return u4_12::of_double(x); }
constexpr s4_12 operator"" _s4_12(long double x) { return s4_12::of_double(x); }
constexpr u4_12 operator"" _u4_12(unsigned long long int x) { return u4_12::of_long_long(x); }
constexpr s4_12 operator"" _s4_12(unsigned long long int x) { return s4_12::of_long_long(x); }
constexpr u5_11 operator"" _u5_11(long double x) { return u5_11::of_double(x); }
constexpr s5_11 operator"" _s5_11(long double x) { return s5_11::of_double(x); }
constexpr u5_11 operator"" _u5_11(unsigned long long int x) { return u5_11::of_long_long(x); }
constexpr s5_11 operator"" _s5_11(unsigned long long int x) { return s5_11::of_long_long(x); }
constexpr u6_10 operator"" _u6_10(long double x) { return u6_10::of_double(x); }
constexpr s6_10 operator"" _s6_10(long double x) { return s6_10::of_double(x); }
constexpr u6_10 operator"" _u6_10(unsigned long long int x) { return u6_10::of_long_long(x); }
constexpr s6_10 operator"" _s6_10(unsigned long long int x) { return s6_10::of_long_long(x); }
constexpr u7_9 operator"" _u7_9(long double x) { return u7_9::of_double(x); }
constexpr s7_9 operator"" _s7_9(long double x) { return s7_9::of_double(x); }
constexpr u7_9 operator"" _u7_9(unsigned long long int x) { return u7_9::of_long_long(x); }
constexpr s7_9 operator"" _s7_9(unsigned long long int x) { return s7_9::of_long_long(x); }
constexpr u8_8 operator"" _u8_8(long double x) { return u8_8::of_double(x); }
constexpr s8_8 operator"" _s8_8(long double x) { return s8_8::of_double(x); }
constexpr u8_8 operator"" _u8_8(unsigned long long int x) { return u8_8::of_long_long(x); }
constexpr s8_8 operator"" _s8_8(unsigned long long int x) { return s8_8::of_long_long(x); }
constexpr u9_7 operator"" _u9_7(long double x) { return u9_7::of_double(x); }
constexpr s9_7 operator"" _s9_7(long double x) { return s9_7::of_double(x); }
constexpr u9_7 operator"" _u9_7(unsigned long long int x) { return u9_7::of_long_long(x); }
constexpr s9_7 operator"" _s9_7(unsigned long long int x) { return s9_7::of_long_long(x); }
constexpr u10_6 operator"" _u10_6(long double x) { return u10_6::of_double(x); }
constexpr s10_6 operator"" _s10_6(long double x) { return s10_6::of_double(x); }
constexpr u10_6 operator"" _u10_6(unsigned long long int x) { return u10_6::of_long_long(x); }
constexpr s10_6 operator"" _s10_6(unsigned long long int x) { return s10_6::of_long_long(x); }
constexpr u11_5 operator"" _u11_5(long double x) { return u11_5::of_double(x); }
constexpr s11_5 operator"" _s11_5(long double x) { return s11_5::of_double(x); }
constexpr u11_5 operator"" _u11_5(unsigned long long int x) { return u11_5::of_long_long(x); }
constexpr s11_5 operator"" _s11_5(unsigned long long int x) { return s11_5::of_long_long(x); }
constexpr u12_4 operator"" _u12_4(long double x) { return u12_4::of_double(x); }
constexpr s12_4 operator"" _s12_4(long double x) { return s12_4::of_double(x); }
constexpr u12_4 operator"" _u12_4(unsigned long long int x) { return u12_4::of_long_long(x); }
constexpr s12_4 operator"" _s12_4(unsigned long long int x) { return s12_4::of_long_long(x); }
constexpr u13_3 operator"" _u13_3(long double x) { return u13_3::of_double(x); }
constexpr s13_3 operator"" _s13_3(long double x) { return s13_3::of_double(x); }
constexpr u13_3 operator"" _u13_3(unsigned long long int x) { return u13_3::of_long_long(x); }
constexpr s13_3 operator"" _s13_3(unsigned long long int x) { return s13_3::of_long_long(x); }
constexpr u14_2 operator"" _u14_2(long double x) { return u14_2::of_double(x); }
constexpr s14_2 operator"" _s14_2(long double x) { return s14_2::of_double(x); }
constexpr u14_2 operator"" _u14_2(unsigned long long int x) { return u14_2::of_long_long(x); }
constexpr s14_2 operator"" _s14_2(unsigned long long int x) { return s14_2::of_long_long(x); }
constexpr u15_1 operator"" _u15_1(long double x) { return u15_1::of_double(x); }
constexpr s15_1 operator"" _s15_1(long double x) { return s15_1::of_double(x); }
constexpr u15_1 operator"" _u15_1(unsigned long long int x) { return u15_1::of_long_long(x); }
constexpr s15_1 operator"" _s15_1(unsigned long long int x) { return s15_1::of_long_long(x); }
constexpr u16_0 operator"" _u16_0(long double x) { return u16_0::of_double(x); }
constexpr s16_0 operator"" _s16_0(long double x) { return s16_0::of_double(x); }
constexpr u16_0 operator"" _u16_0(unsigned long long int x) { return u16_0::of_long_long(x); }
constexpr s16_0 operator"" _s16_0(unsigned long long int x) { return s16_0::of_long_long(x); }

// 32 bits
constexpr u0_32 operator"" _u0_32(long double x) { return u0_32::of_double(x); }
constexpr s0_32 operator"" _s0_32(long double x) { return s0_32::of_double(x); }
constexpr u0_32 operator"" _u0_32(unsigned long long int x) { return u0_32::of_long_long(x); }
constexpr s0_32 operator"" _s0_32(unsigned long long int x) { return s0_32::of_long_long(x); }
constexpr u1_31 operator"" _u1_31(long double x) { return u1_31::of_double(x); }
constexpr s1_31 operator"" _s1_31(long double x) { return s1_31::of_double(x); }
constexpr u1_31 operator"" _u1_31(unsigned long long int x) { return u1_31::of_long_long(x); }
constexpr s1_31 operator"" _s1_31(unsigned long long int x) { return s1_31::of_long_long(x); }
constexpr u2_30 operator"" _u2_30(long double x) { return u2_30::of_double(x); }
constexpr s2_30 operator"" _s2_30(long double x) { return s2_30::of_double(x); }
constexpr u2_30 operator"" _u2_30(unsigned long long int x) { return u2_30::of_long_long(x); }
constexpr s2_30 operator"" _s2_30(unsigned long long int x) { return s2_30::of_long_long(x); }
constexpr u3_29 operator"" _u3_29(long double x) { return u3_29::of_double(x); }
constexpr s3_29 operator"" _s3_29(long double x) { return s3_29::of_double(x); }
constexpr u3_29 operator"" _u3_29(unsigned long long int x) { return u3_29::of_long_long(x); }
constexpr s3_29 operator"" _s3_29(unsigned long long int x) { return s3_29::of_long_long(x); }
constexpr u4_28 operator"" _u4_28(long double x) { return u4_28::of_double(x); }
constexpr s4_28 operator"" _s4_28(long double x) { return s4_28::of_double(x); }
constexpr u4_28 operator"" _u4_28(unsigned long long int x) { return u4_28::of_long_long(x); }
constexpr s4_28 operator"" _s4_28(unsigned long long int x) { return s4_28::of_long_long(x); }
constexpr u5_27 operator"" _u5_27(long double x) { return u5_27::of_double(x); }
constexpr s5_27 operator"" _s5_27(long double x) { return s5_27::of_double(x); }
constexpr u5_27 operator"" _u5_27(unsigned long long int x) { return u5_27::of_long_long(x); }
constexpr s5_27 operator"" _s5_27(unsigned long long int x) { return s5_27::of_long_long(x); }
constexpr u6_26 operator"" _u6_26(long double x) { return u6_26::of_double(x); }
constexpr s6_26 operator"" _s6_26(long double x) { return s6_26::of_double(x); }
constexpr u6_26 operator"" _u6_26(unsigned long long int x) { return u6_26::of_long_long(x); }
constexpr s6_26 operator"" _s6_26(unsigned long long int x) { return s6_26::of_long_long(x); }
constexpr u7_25 operator"" _u7_25(long double x) { return u7_25::of_double(x); }
constexpr s7_25 operator"" _s7_25(long double x) { return s7_25::of_double(x); }
constexpr u7_25 operator"" _u7_25(unsigned long long int x) { return u7_25::of_long_long(x); }
constexpr s7_25 operator"" _s7_25(unsigned long long int x) { return s7_25::of_long_long(x); }
constexpr u8_24 operator"" _u8_24(long double x) { return u8_24::of_double(x); }
constexpr s8_24 operator"" _s8_24(long double x) { return s8_24::of_double(x); }
constexpr u8_24 operator"" _u8_24(unsigned long long int x) { return u8_24::of_long_long(x); }
constexpr s8_24 operator"" _s8_24(unsigned long long int x) { return s8_24::of_long_long(x); }
constexpr u9_23 operator"" _u9_23(long double x) { return u9_23::of_double(x); }
constexpr s9_23 operator"" _s9_23(long double x) { return s9_23::of_double(x); }
constexpr u9_23 operator"" _u9_23(unsigned long long int x) { return u9_23::of_long_long(x); }
constexpr s9_23 operator"" _s9_23(unsigned long long int x) { return s9_23::of_long_long(x); }
constexpr u10_22 operator"" _u10_22(long double x) { return u10_22::of_double(x); }
constexpr s10_22 operator"" _s10_22(long double x) { return s10_22::of_double(x); }
constexpr u10_22 operator"" _u10_22(unsigned long long int x) { return u10_22::of_long_long(x); }
constexpr s10_22 operator"" _s10_22(unsigned long long int x) { return s10_22::of_long_long(x); }
constexpr u11_21 operator"" _u11_21(long double x) { return u11_21::of_double(x); }
constexpr s11_21 operator"" _s11_21(long double x) { return s11_21::of_double(x); }
constexpr u11_21 operator"" _u11_21(unsigned long long int x) { return u11_21::of_long_long(x); }
constexpr s11_21 operator"" _s11_21(unsigned long long int x) { return s11_21::of_long_long(x); }
constexpr u12_20 operator"" _u12_20(long double x) { return u12_20::of_double(x); }
constexpr s12_20 operator"" _s12_20(long double x) { return s12_20::of_double(x); }
constexpr u12_20 operator"" _u12_20(unsigned long long int x) { return u12_20::of_long_long(x); }
constexpr s12_20 operator"" _s12_20(unsigned long long int x) { return s12_20::of_long_long(x); }
constexpr u13_19 operator"" _u13_19(long double x) { return u13_19::of_double(x); }
constexpr s13_19 operator"" _s13_19(long double x) { return s13_19::of_double(x); }
constexpr u13_19 operator"" _u13_19(unsigned long long int x) { return u13_19::of_long_long(x); }
constexpr s13_19 operator"" _s13_19(unsigned long long int x) { return s13_19::of_long_long(x); }
constexpr u14_18 operator"" _u14_18(long double x) { return u14_18::of_double(x); }
constexpr s14_18 operator"" _s14_18(long double x) { return s14_18::of_double(x); }
constexpr u14_18 operator"" _u14_18(unsigned long long int x) { return u14_18::of_long_long(x); }
constexpr s14_18 operator"" _s14_18(unsigned long long int x) { return s14_18::of_long_long(x); }
constexpr u15_17 operator"" _u15_17(long double x) { return u15_17::of_double(x); }
constexpr s15_17 operator"" _s15_17(long double x) { return s15_17::of_double(x); }
constexpr u15_17 operator"" _u15_17(unsigned long long int x) { return u15_17::of_long_long(x); }
constexpr s15_17 operator"" _s15_17(unsigned long long int x) { return s15_17::of_long_long(x); }
constexpr u16_16 operator"" _u16_16(long double x) { return u16_16::of_double(x); }
constexpr s16_16 operator"" _s16_16(long double x) { return s16_16::of_double(x); }
constexpr u16_16 operator"" _u16_16(unsigned long long int x) { return u16_16::of_long_long(x); }
constexpr s16_16 operator"" _s16_16(unsigned long long int x) { return s16_16::of_long_long(x); }
constexpr u17_15 operator"" _u17_15(long double x) { return u17_15::of_double(x); }
constexpr s17_15 operator"" _s17_15(long double x) { return s17_15::of_double(x); }
constexpr u17_15 operator"" _u17_15(unsigned long long int x) { return u17_15::of_long_long(x); }
constexpr s17_15 operator"" _s17_15(unsigned long long int x) { return s17_15::of_long_long(x); }
constexpr u18_14 operator"" _u18_14(long double x) { return u18_14::of_double(x); }
constexpr s18_14 operator"" _s18_14(long double x) { return s18_14::of_double(x); }
constexpr u18_14 operator"" _u18_14(unsigned long long int x) { return u18_14::of_long_long(x); }
constexpr s18_14 operator"" _s18_14(unsigned long long int x) { return s18_14::of_long_long(x); }
constexpr u19_13 operator"" _u19_13(long double x) { return u19_13::of_double(x); }
constexpr s19_13 operator"" _s19_13(long double x) { return s19_13::of_double(x); }
constexpr u19_13 operator"" _u19_13(unsigned long long int x) { return u19_13::of_long_long(x); }
constexpr s19_13 operator"" _s19_13(unsigned long long int x) { return s19_13::of_long_long(x); }
constexpr u20_12 operator"" _u20_12(long double x) { return u20_12::of_double(x); }
constexpr s20_12 operator"" _s20_12(long double x) { return s20_12::of_double(x); }
constexpr u20_12 operator"" _u20_12(unsigned long long int x) { return u20_12::of_long_long(x); }
constexpr s20_12 operator"" _s20_12(unsigned long long int x) { return s20_12::of_long_long(x); }
constexpr u21_11 operator"" _u21_11(long double x) { return u21_11::of_double(x); }
constexpr s21_11 operator"" _s21_11(long double x) { return s21_11::of_double(x); }
constexpr u21_11 operator"" _u21_11(unsigned long long int x) { return u21_11::of_long_long(x); }
constexpr s21_11 operator"" _s21_11(unsigned long long int x) { return s21_11::of_long_long(x); }
constexpr u22_10 operator"" _u22_10(long double x) { return u22_10::of_double(x); }
constexpr s22_10 operator"" _s22_10(long double x) { return s22_10::of_double(x); }
constexpr u22_10 operator"" _u22_10(unsigned long long int x) { return u22_10::of_long_long(x); }
constexpr s22_10 operator"" _s22_10(unsigned long long int x) { return s22_10::of_long_long(x); }
constexpr u23_9 operator"" _u23_9(long double x) { return u23_9::of_double(x); }
constexpr s23_9 operator"" _s23_9(long double x) { return s23_9::of_double(x); }
constexpr u23_9 operator"" _u23_9(unsigned long long int x) { return u23_9::of_long_long(x); }
constexpr s23_9 operator"" _s23_9(unsigned long long int x) { return s23_9::of_long_long(x); }
constexpr u24_8 operator"" _u24_8(long double x) { return u24_8::of_double(x); }
constexpr s24_8 operator"" _s24_8(long double x) { return s24_8::of_double(x); }
constexpr u24_8 operator"" _u24_8(unsigned long long int x) { return u24_8::of_long_long(x); }
constexpr s24_8 operator"" _s24_8(unsigned long long int x) { return s24_8::of_long_long(x); }
constexpr u25_7 operator"" _u25_7(long double x) { return u25_7::of_double(x); }
constexpr s25_7 operator"" _s25_7(long double x) { return s25_7::of_double(x); }
constexpr u25_7 operator"" _u25_7(unsigned long long int x) { return u25_7::of_long_long(x); }
constexpr s25_7 operator"" _s25_7(unsigned long long int x) { return s25_7::of_long_long(x); }
constexpr u26_6 operator"" _u26_6(long double x) { return u26_6::of_double(x); }
constexpr s26_6 operator"" _s26_6(long double x) { return s26_6::of_double(x); }
constexpr u26_6 operator"" _u26_6(unsigned long long int x) { return u26_6::of_long_long(x); }
constexpr s26_6 operator"" _s26_6(unsigned long long int x) { return s26_6::of_long_long(x); }
constexpr u27_5 operator"" _u27_5(long double x) { return u27_5::of_double(x); }
constexpr s27_5 operator"" _s27_5(long double x) { return s27_5::of_double(x); }
constexpr u27_5 operator"" _u27_5(unsigned long long int x) { return u27_5::of_long_long(x); }
constexpr s27_5 operator"" _s27_5(unsigned long long int x) { return s27_5::of_long_long(x); }
constexpr u28_4 operator"" _u28_4(long double x) { return u28_4::of_double(x); }
constexpr s28_4 operator"" _s28_4(long double x) { return s28_4::of_double(x); }
constexpr u28_4 operator"" _u28_4(unsigned long long int x) { return u28_4::of_long_long(x); }
constexpr s28_4 operator"" _s28_4(unsigned long long int x) { return s28_4::of_long_long(x); }
constexpr u29_3 operator"" _u29_3(long double x) { return u29_3::of_double(x); }
constexpr s29_3 operator"" _s29_3(long double x) { return s29_3::of_double(x); }
constexpr u29_3 operator"" _u29_3(unsigned long long int x) { return u29_3::of_long_long(x); }
constexpr s29_3 operator"" _s29_3(unsigned long long int x) { return s29_3::of_long_long(x); }
constexpr u30_2 operator"" _u30_2(long double x) { return u30_2::of_double(x); }
constexpr s30_2 operator"" _s30_2(long double x) { return s30_2::of_double(x); }
constexpr u30_2 operator"" _u30_2(unsigned long long int x) { return u30_2::of_long_long(x); }
constexpr s30_2 operator"" _s30_2(unsigned long long int x) { return s30_2::of_long_long(x); }
constexpr u31_1 operator"" _u31_1(long double x) { return u31_1::of_double(x); }
constexpr s31_1 operator"" _s31_1(long double x) { return s31_1::of_double(x); }
constexpr u31_1 operator"" _u31_1(unsigned long long int x) { return u31_1::of_long_long(x); }
constexpr s31_1 operator"" _s31_1(unsigned long long int x) { return s31_1::of_long_long(x); }
constexpr u32_0 operator"" _u32_0(long double x) { return u32_0::of_double(x); }
constexpr s32_0 operator"" _s32_0(long double x) { return s32_0::of_double(x); }
constexpr u32_0 operator"" _u32_0(unsigned long long int x) { return u32_0::of_long_long(x); }
constexpr s32_0 operator"" _s32_0(unsigned long long int x) { return s32_0::of_long_long(x); }

// shortcuts
constexpr s operator"" _s(unsigned long long int x) { return s::of_long_long(x); }
constexpr u operator"" _u(unsigned long long int x) { return u::of_long_long(x); }
constexpr s16 operator"" _s16(const unsigned long long int x) { return s16::of_long_long(x); }
constexpr u16 operator"" _u16(unsigned long long int x) { return u16::of_long_long(x); }
constexpr s32 operator"" _s32(unsigned long long int x) { return s32::of_long_long(x); }
constexpr u32 operator"" _u32(unsigned long long int x) { return u32::of_long_long(x); }

}  // namespace grm
