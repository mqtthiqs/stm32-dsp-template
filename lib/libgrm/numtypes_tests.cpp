#include <catch2/catch_all.hpp>

#include "numtypes.h"

using namespace grm;

// STATIC TESTS:

TEST_CASE("narrow") {
  STATIC_REQUIRE(s1_15::narrow(0.5_s1_31) == 0.5_s1_15);
  STATIC_REQUIRE(s1_15::narrow(-0.125_s1_31) == -0.125_s1_15);
  STATIC_REQUIRE(f(Fixed<SIGNED, 1, 63>(0.125_f)) == 0.125_f);
  STATIC_REQUIRE(s1_31::narrow(Fixed<SIGNED, 1, 63>(0.125_f)) == 0.125_s1_31);

  constexpr f x = 12_f;
  STATIC_REQUIRE(Fixed<SIGNED, 36, 28>::narrow(Fixed<SIGNED, 7, 57>(x)) ==
                 Fixed<SIGNED, 36, 28>(x));
}

TEST_CASE("float floor") {
  STATIC_REQUIRE((0.2_f).floor() == 0_f);
  STATIC_REQUIRE((0_f).floor() == 0_f);
  STATIC_REQUIRE((42_f).floor() == 42_f);
  STATIC_REQUIRE((42.9_f).floor() == 42_f);
  // TODO:
  // STATIC_REQUIRE((-42.9_f).floor() == -43_f);
  // STATIC_REQUIRE((-0.2_f).floor() == -1_f);
}

TEST_CASE("float ceil") {
  STATIC_REQUIRE((0.2_f).ceil() == 1_f);
  STATIC_REQUIRE((0_f).ceil() == 0_f);
  STATIC_REQUIRE((42_f).ceil() == 42_f);
  STATIC_REQUIRE((42.9_f).ceil() == 43_f);
  // TODO:
  // STATIC_REQUIRE((-42.9_f).ceil() == -42_f);
  // STATIC_REQUIRE((-0.2_f).ceil() == 0_f);
}

TEST_CASE("float integral/fractional") {
  auto val = GENERATE(42_f, 1.23_f, 0.00357375930274014_f, 423.39482_f);
  auto range = GENERATE(0.0001_f, -1_f, -7.32_f, 13_f, 180249_f);
  auto x = val * range;
  INFO("x = " << x);
  auto [integ, frac] = x.integral_fractional();

  SECTION("integral/fractional and integral_fractional agree") {
    REQUIRE(x.integral() == integ);
    REQUIRE(x.fractional() == frac);
  }

  SECTION("fractional range") {
    REQUIRE((x < 0_f || (frac < 1_f && frac >= 0_f)));
    REQUIRE((x >= 0_f || (frac > -1_f && frac <= 0_f)));
  }
  REQUIRE(f(integ) + frac == x);
}

TEST_CASE("float bin representation") {
  auto val = GENERATE(42_f, 1.23_f, 0.00357375930274014_f, 423.39482_f);
  auto range = GENERATE(0.0001_f, -1_f, -7.32_f, 13_f, 180249_f);
  auto x = val * range;
  INFO("x = " << x);

  SECTION("round trip f") { REQUIRE(x.bin().get() == x); }

  SECTION("round trip int") {
    auto y = u32(x);
    REQUIRE(f::BinRepr(y).getInt() == y.repr());
  }

  SECTION("sign") { REQUIRE(x.bin().sign() == (x < 0_f)); }

  SECTION("mantisa/exponent") {
    auto b = x.bin();
    auto m = b.mantisa();
    auto e = b.exponent();
    float y = std::exp2(float(e - 127)) * (1.f + float(m) / float(1 << 23));
    if (b.sign()) y = -y;
    REQUIRE(x.repr() == y);
  }

  SECTION("increment exponent") {
    auto b = x.bin();
    b.setExponent(b.exponent() + 1);
    REQUIRE(b.get() == x * 2_f);
  }

  SECTION("invert sign") {
    auto y = x.bin();
    y.setSign(!y.sign());
    REQUIRE(y.get() == -x);
  }
}

TEST_CASE("fixed wrap") {
  STATIC_REQUIRE(1_s1_15 == -1_s1_15);
  STATIC_REQUIRE(65536_u16 == 0_u16);
  STATIC_REQUIRE(1_u0_16 == 0_u0_16);
  STATIC_REQUIRE(1_u0_32 == 0_u0_32);
  STATIC_REQUIRE(1024_u10_22 == 0_u10_22);

  STATIC_REQUIRE(min_val<s1_15>.pred() == max_val<s1_15>);
  STATIC_REQUIRE(max_val<s1_15>.succ() == min_val<s1_15>);
  STATIC_REQUIRE(min_val<u0_16>.pred() == max_val<u0_16>);
  STATIC_REQUIRE(max_val<u0_16>.succ() == min_val<u0_16>);

  STATIC_REQUIRE(min_val<s1_31>.pred() == max_val<s1_31>);
  STATIC_REQUIRE(max_val<s1_31>.succ() == min_val<s1_31>);
  STATIC_REQUIRE(min_val<u0_32>.pred() == max_val<u0_32>);
  STATIC_REQUIRE(max_val<u0_32>.succ() == min_val<u0_32>);

  STATIC_REQUIRE(-1_s1_15 == min_val<s1_15>);
  STATIC_REQUIRE((1_s1_15).pred() == max_val<s1_15>);
}

TEST_CASE("float fixed conversion") {
  STATIC_REQUIRE(f(10_u).repr() == 10.f);
  STATIC_REQUIRE(f(12495_u) == 12495_f);
  STATIC_REQUIRE(f(-0.5_s1_15) == -0.5_f);
  STATIC_REQUIRE(f(0.45_u0_32) == 0.45_f);
  STATIC_REQUIRE(f(0.345_u0_32) == 0.345_f);
  STATIC_REQUIRE(f(-0.5_s1_31) == -0.5_f);
}

TEST_CASE("fixed min/max/clip") {
  STATIC_REQUIRE((32_u).min(42_u) == 32_u);
  STATIC_REQUIRE((32_u).max(42_u) == 42_u);
  STATIC_REQUIRE((42_u).min(32_u) == 32_u);
  STATIC_REQUIRE((42_u).max(32_u) == 42_u);

  STATIC_REQUIRE((742.625_u10_22).min(700_u10_22) == 700_u10_22);
  STATIC_REQUIRE((742.625_u10_22).max(700_u10_22) == 742.625_u10_22);

  STATIC_REQUIRE((742.625_u10_22).clip(700_u10_22, 750_u10_22) == 742.625_u10_22);
  STATIC_REQUIRE((642.625_u10_22).clip(700_u10_22, 750_u10_22) == 700_u10_22);
  STATIC_REQUIRE((842.625_u10_22).clip(700_u10_22, 750_u10_22) == 750_u10_22);

  STATIC_REQUIRE((-242.625_s10_22).min(-200_s10_22) == -242.625_s10_22);
  STATIC_REQUIRE((-242.625_s10_22).max(-200_s10_22) == -200_s10_22);
  STATIC_REQUIRE((-242.625_s10_22).clip(-250_s10_22, -200_s10_22) == -242.625_s10_22);
  STATIC_REQUIRE((-142.625_s10_22).clip(-250_s10_22, -200_s10_22) == -200_s10_22);
  STATIC_REQUIRE((-342.625_s10_22).clip(-250_s10_22, -200_s10_22) == -250_s10_22);

  STATIC_REQUIRE((0.9_u10_22).clip() == 0.9_u10_22);
  STATIC_REQUIRE((1.2_u10_22).clip() == (1_u10_22).pred());
  STATIC_REQUIRE((-0.9_s10_22).clip() == -0.9_s10_22);
  STATIC_REQUIRE((-1.2_s10_22).clip() == -1_s10_22);
}

TEST_CASE("fixed arithmetics") {
  STATIC_REQUIRE(742.625_u10_22 == 742.625_u10_22);

  STATIC_REQUIRE(f(-0.125_s1_15 - 0.5_s1_15) == -0.625_f);
  STATIC_REQUIRE(f(0.5_u0_16 + 0.125_u0_16) == 0.625_f);
  STATIC_REQUIRE(f(0.25_u0_16 - 0.125_u0_16) == 0.125_f);
  STATIC_REQUIRE(f(0.25_u0_16 - 0.5_u0_16) == 0.75_f);
  STATIC_REQUIRE(0.625_u0_32 + 0.125_u0_32 == 0.75_u0_32);
  STATIC_REQUIRE(f(-0.5_s1_31 + 0.75_s1_31) == 0.25_f);
}

TEST_CASE("fixed floor/integral/fractional") {
  STATIC_REQUIRE((-0.625_s1_15).floor() == -1_s1_15);
  STATIC_REQUIRE((742.625_u10_22).floor() == 742_u10_22);
  STATIC_REQUIRE((742.625_u10_22).integral() == 742);
  STATIC_REQUIRE((742.625_u10_22).frac() == 0.625_u10_22);
  STATIC_REQUIRE((-4.625_s10_22).frac() == 0.375_s10_22);
  STATIC_REQUIRE((742.625_u10_22).floor() + (742.625_u10_22).frac() == 742.625_u10_22);
  STATIC_REQUIRE((-0.25_s1_15).floor() + (-0.25_s1_15).frac() == -0.25_s1_15);
  STATIC_REQUIRE(Fixed<UNSIGNED, 28, 36>(1.25_f).floor() == Fixed<UNSIGNED, 28, 36>(1_f));
}

TEST_CASE("fixed ceil") {
  STATIC_REQUIRE((0.2_s17_15).ceil() == 1_s17_15);
  STATIC_REQUIRE((0_s17_15).ceil() == 0_s17_15);
  STATIC_REQUIRE((42_s17_15).ceil() == 42_s17_15);
  STATIC_REQUIRE((42.9_s17_15).ceil() == 43_s17_15);
  STATIC_REQUIRE((-42.9_s17_15).ceil() == -42_s17_15);
  STATIC_REQUIRE((-0.2_s17_15).ceil() == 0_s17_15);
}

TEST_CASE("fixed saturating arithmetics") {
  STATIC_REQUIRE((0.25_s10_22).to_sat<1, 31>() == 0.25_s1_31);
  STATIC_REQUIRE((-0.25_s10_22).to_sat<1, 31>() == -0.25_s1_31);
  STATIC_REQUIRE((0.75_s10_22).to_sat<1, 31>() == 0.75_s1_31);
  STATIC_REQUIRE((-0.75_s10_22).to_sat<1, 31>() == -0.75_s1_31);
  // TODO
  // STATIC_REQUIRE((1.25_s10_22).to_sat<1, 31>() == max_val<s1_31>);
  STATIC_REQUIRE((-1.25_s10_22).to_sat<1, 31>() == min_val<s1_31>);
  STATIC_REQUIRE((0.25_u10_22).to_sat<0, 32>() == 0.25_u0_32);
  STATIC_REQUIRE((0.75_u10_22).to_sat<0, 32>() == 0.75_u0_32);
  // TODO
  // STATIC_REQUIRE((1.25_u10_22).to_sat<0, 32>() == s0_32::max_val);

  STATIC_REQUIRE((0.25_s1_15).add_sat(0.5_s1_15) == 0.75_s1_15);
  STATIC_REQUIRE((0.75_s1_15).add_sat(0.5_s1_15) == (1_s1_15).pred());
  STATIC_REQUIRE((-0.75_s1_15).add_sat(-0.5_s1_15) == -1_s1_15);
  STATIC_REQUIRE((-0.75_s1_15).add_sat(-0.5_s1_15) == -1_s1_15);

  STATIC_REQUIRE((0.25_s1_15).sub_sat(0.5_s1_15) == -0.25_s1_15);
  STATIC_REQUIRE((0.75_s1_15).sub_sat(0.5_s1_15) == 0.25_s1_15);
  STATIC_REQUIRE((-0.75_s1_15).sub_sat(-0.5_s1_15) == -0.25_s1_15);
  STATIC_REQUIRE((0.75_s1_15).sub_sat(-0.5_s1_15) == (1_s1_15).pred());

  STATIC_REQUIRE((0.25_u0_16).add_sat(0.5_u0_16) == 0.75_u0_16);
  STATIC_REQUIRE((0.75_u0_16).add_sat(0.5_u0_16) == (1_u0_16).pred());
  STATIC_REQUIRE((0.75_u0_16).sub_sat(0.5_u0_16) == 0.25_u0_16);

  STATIC_REQUIRE((0.25_s1_31).add_sat(0.5_s1_31) == 0.75_s1_31);
  STATIC_REQUIRE((0.75_s1_31).add_sat(0.5_s1_31) == (1_s1_31).pred());
  STATIC_REQUIRE((-0.75_s1_31).add_sat(-0.5_s1_31) == 1_s1_31);
  STATIC_REQUIRE((-0.75_s1_31).add_sat(-0.5_s1_31) == 1_s1_31);

  STATIC_REQUIRE((0.25_s1_31).sub_sat(0.5_s1_31) == -0.25_s1_31);
  STATIC_REQUIRE((0.75_s1_31).sub_sat(0.5_s1_31) == 0.25_s1_31);
  STATIC_REQUIRE((-0.75_s1_31).sub_sat(-0.5_s1_31) == -0.25_s1_31);
  STATIC_REQUIRE((0.75_s1_31).sub_sat(-0.5_s1_31) == (1_s1_31).pred());

  STATIC_REQUIRE((0.25_u0_32).add_sat(0.5_u0_32) == 0.75_u0_32);
  STATIC_REQUIRE((0.75_u0_32).add_sat(0.5_u0_32) == (1_u0_32).pred());
  STATIC_REQUIRE((0.75_u0_32).sub_sat(0.5_u0_32) == 0.25_u0_32);
}

TEST_CASE("fixed div2") {
  STATIC_REQUIRE((0.5_u0_32).div2<1>() == 0.25_u0_32);
  STATIC_REQUIRE((0.5_u0_32).div2<4>() == 0.031250_u0_32);
  STATIC_REQUIRE((-0.5_s1_31).div2<4>() == -0.031250_s1_31);
}

TEST_CASE("(un)signed conversion") {
  STATIC_REQUIRE((0.75_u0_16).to_signed() == 0.75_s1_15);
  STATIC_REQUIRE((0.999_u0_16).to_signed() == 0.999_s1_15);
  STATIC_REQUIRE(s1_15::narrow(0.75_s1_31).to_unsigned() == 0.75_u0_16);
  STATIC_REQUIRE(s1_15::narrow(-0.75_s1_31).to_unsigned() == 0.25_u0_16);
  STATIC_REQUIRE(s1_15::narrow(-0.1_s1_31).to_unsigned() == 0.9_u0_16);

  STATIC_REQUIRE((0.75_u0_16).to_signed().to_unsigned() == 0.75_u0_16);
  STATIC_REQUIRE((0.33_u0_16).to_signed().to_unsigned() == 0.33_u0_16);
  STATIC_REQUIRE((0_u0_16).to_signed().to_unsigned() == 0_u0_16);
  // TODO: is this normal?
  STATIC_REQUIRE((max_val<u0_16>).to_signed().to_unsigned() == max_val<u0_16>.pred());

  STATIC_REQUIRE((0.75_s1_15).to_unsigned().to_signed() == 0.75_s1_15);
  STATIC_REQUIRE((0.33_s1_15).to_unsigned().to_signed() == 0.33_s1_15);
  STATIC_REQUIRE((0_s1_15).to_unsigned().to_signed() == 0_s1_15);
  STATIC_REQUIRE((max_val<s1_15>).to_unsigned().to_signed() == max_val<s1_15>);
}

TEST_CASE("fixed multiplication") {
  STATIC_REQUIRE(0.5_s1_15 * 0.5_s1_15 == 0.25_s1_31);
  STATIC_REQUIRE(0.25_s1_15 * 0.75_s1_15 == 0.1875_s1_31);
  STATIC_REQUIRE(0_s1_15 * -0.75_s1_15 == 0_s1_31);
  STATIC_REQUIRE(-0.5_s1_15 * 0.5_s1_15 == -0.25_s1_31);
  STATIC_REQUIRE(-0.25_s1_15 * -0.5_s1_15 == 0.125_s1_31);

  STATIC_REQUIRE(0.25_u0_16 * 0.5_u0_16 == 0.125_u0_32);
  STATIC_REQUIRE(0_u0_16 * 0.5_u0_16 == 0_u0_32);
  STATIC_REQUIRE(1_u0_16 * 0.5_u0_16 == 0_u0_32);

  STATIC_REQUIRE(32_s12_4 * 0.5_s1_15 == Fixed<SIGNED, 12, 20>(16_f));
  STATIC_REQUIRE(32_s20_12 * 0.5_s1_31 == Fixed<SIGNED, 20, 44>(16_f));
  STATIC_REQUIRE(f(2_s4_28 * 2_s4_28) == 4_f);
}

TEST_CASE("fixed widening/narrowing") {
  // pure widening
  STATIC_REQUIRE(u32(42_u) == u32(42_u));
  STATIC_REQUIRE(f(u32(42_u)) == 42_f);
  STATIC_REQUIRE(f(s32(42_s)) == 42_f);
  STATIC_REQUIRE(f(s32(-42_s)) == -42_f);
  // widening with shift
  STATIC_REQUIRE(u0_32(0.5_u0_16) == 0.5_u0_32);
  STATIC_REQUIRE(s10_22(0.5_s1_15) == 0.5_s10_22);
  STATIC_REQUIRE(s10_22(-0.5_s1_15) == -0.5_s10_22);
  // narrowing (fractional part)
  STATIC_REQUIRE(s1_15::narrow(0.50001_s1_31) == 0.5_s1_15);
  STATIC_REQUIRE(s1_15::narrow(-0.49999_s1_31) == -0.5_s1_15);

  STATIC_REQUIRE(std::is_same<decltype((0.25_u0_16).widen()), u16_16>::value);
  STATIC_REQUIRE(std::is_same<decltype((0.25_u1_15).widen()), u17_15>::value);
  STATIC_REQUIRE(std::is_same<decltype((0.25_s0_16).widen()), s16_16>::value);
  STATIC_REQUIRE(std::is_same<decltype((0.25_s1_15).widen()), s17_15>::value);
  STATIC_REQUIRE(std::is_same<decltype((0.25_u3_5).widen()), u11_5>::value);

  STATIC_REQUIRE((0.25_u0_16).widen() == 0.25_u16_16);
  STATIC_REQUIRE((0.25_u1_15).widen() == 0.25_u17_15);
  STATIC_REQUIRE((0.25_s0_16).widen() == 0.25_s16_16);
  STATIC_REQUIRE((0.25_s1_15).widen() == 0.25_s17_15);
  STATIC_REQUIRE((0.25_u3_5).widen() == 0.25_u11_5);
  STATIC_REQUIRE((0.25_u3_5).widen() != 0.5_u11_5);
}

// DYNAMIC TESTS

TEST_CASE("clip", "[numtypes]") {
  REQUIRE((0.9_u10_22).clip() == 0.9_u10_22);
  REQUIRE((1.2_u10_22).clip() == (1_u10_22).pred());
  REQUIRE((-0.9_s10_22).clip() == -0.9_s10_22);
  REQUIRE((-1.2_s10_22).clip() == -1_s10_22);
}

TEST_CASE("saturation arithmetic", "[numtypes]") {
  REQUIRE((0.25_s10_22).to_sat<1, 31>() == 0.25_s1_31);
  REQUIRE((-0.25_s10_22).to_sat<1, 31>() == -0.25_s1_31);
  REQUIRE((0.75_s10_22).to_sat<1, 31>() == 0.75_s1_31);
  REQUIRE((-0.75_s10_22).to_sat<1, 31>() == -0.75_s1_31);
  // TODO
  // REQUIRE(((1.25_s10_22).to_sat<1, 31>() == max_val<s1_31>));
  // TODO
  // REQUIRE(((-1.25_s10_22).to_sat<1, 31>() == max_val<s1_31>));
  REQUIRE((0.25_u10_22).to_sat<0, 32>() == 0.25_u0_32);
  REQUIRE((0.75_u10_22).to_sat<0, 32>() == 0.75_u0_32);
  // TODO
  // REQUIRE(((1.25_u10_22).to_sat<0, 32>() == max_val<u0_32>));
  REQUIRE((0.25_s1_15).add_sat(0.5_s1_15) == 0.75_s1_15);
  REQUIRE((0.75_s1_15).add_sat(0.5_s1_15) == (1_s1_15).pred());
  REQUIRE((-0.75_s1_15).add_sat(-0.5_s1_15) == -1_s1_15);
  REQUIRE((-0.75_s1_15).add_sat(-0.5_s1_15) == -1_s1_15);
  REQUIRE((0.25_s1_15).sub_sat(0.5_s1_15) == -0.25_s1_15);
  REQUIRE((0.75_s1_15).sub_sat(0.5_s1_15) == 0.25_s1_15);
  REQUIRE((-0.75_s1_15).sub_sat(-0.5_s1_15) == -0.25_s1_15);
  REQUIRE((0.75_s1_15).sub_sat(-0.5_s1_15) == (1_s1_15).pred());
  REQUIRE((0.25_u0_16).add_sat(0.5_u0_16) == 0.75_u0_16);
  REQUIRE((0.75_u0_16).add_sat(0.5_u0_16) == (1_u0_16).pred());
  REQUIRE((0.75_u0_16).sub_sat(0.5_u0_16) == 0.25_u0_16);
  REQUIRE((0.25_s1_31).add_sat(0.5_s1_31) == 0.75_s1_31);
  REQUIRE((0.75_s1_31).add_sat(0.5_s1_31) == (1_s1_31).pred());
  REQUIRE((-0.75_s1_31).add_sat(-0.5_s1_31) == 1_s1_31);
  REQUIRE((-0.75_s1_31).add_sat(-0.5_s1_31) == 1_s1_31);
  REQUIRE((0.25_s1_31).sub_sat(0.5_s1_31) == -0.25_s1_31);
  REQUIRE((0.75_s1_31).sub_sat(0.5_s1_31) == 0.25_s1_31);
  REQUIRE((-0.75_s1_31).sub_sat(-0.5_s1_31) == -0.25_s1_31);
  REQUIRE((0.75_s1_31).sub_sat(-0.5_s1_31) == (1_s1_31).pred());
}

TEST_CASE("sqrt", "[numtypes]") { REQUIRE(((25_f).sqrt() == 5_f)); }

TEST_CASE("sgn", "[numtypes]") {
  REQUIRE(infinity.sgn() == 1_f);
  REQUIRE((124334.42_f).sgn() == 1_f);
  REQUIRE((1.42_f).sgn() == 1_f);
  REQUIRE((0.000000001_f).sgn() == 1_f);
  REQUIRE((0_f).sgn() == 0_f);
  REQUIRE((-0.000000001_f).sgn() == -1_f);
  REQUIRE((-1.42_f).sgn() == -1_f);
  REQUIRE((-124334.42_f).sgn() == -1_f);
  REQUIRE(minus_infinity.sgn() == -1_f);
}
