#include <catch2/catch_all.hpp>

#include "units.h"

using namespace grm;

using namespace Catch::literals;
using namespace Catch::Generators;

TEST_CASE("semitonesToRatio", "[units]") {
  CHECK(Units::semitonesToRatio(0_f) == 1_f);
  CHECK(Units::semitonesToRatio(7_f).repr() == Catch::Approx(1.4983070769f).epsilon(0.001f));
  CHECK(Units::semitonesToRatio(12_f) == 2_f);
  CHECK(Units::semitonesToRatio(-12_f) == 0.5_f);
  CHECK(Units::semitonesToRatio(36_f) == 8_f);
  CHECK(Units::semitonesToRatio(-36_f) == 0.125_f);
}

TEST_CASE("midiToHz", "[units]") {
  CHECK(Units::midiToHz(69_f) == 440_f);
  CHECK(Units::midiToHz(57_f) == 220_f);
  CHECK(Units::midiToHz(81_f) == 880_f);
}

TEST_CASE("hzToMidi", "[units]") {
  CHECK(Units::HzToMidi(440_f).repr() == Catch::Approx(69.f).margin(0.01f));
  CHECK(Units::HzToMidi(220_f).repr() == Catch::Approx(57.f).margin(0.01f));
  CHECK(Units::HzToMidi(880_f).repr() == Catch::Approx(81.f).margin(0.01f));
}

TEST_CASE("hzToMidiToHz", "[units]") {
  auto note = GENERATE(-100_f, -10_f, 0_f, 1_f, 10_f, 20_f, 50_f, 80_f, 120_f, 200_f);
  INFO("note = " << note);
  CHECK(Units::HzToMidi(Units::midiToHz(note)).repr() == Catch::Approx(note.repr()).margin(0.02f));
}

TEST_CASE("ampToDb", "[units]") {
  SECTION("normal range") {
    auto gain = GENERATE(1_f, 2_f, 4_f, 32_f, 10_f, 100_f, 1000_f, 10000_f, 0.5_f, 0.125_f, 0.01_f,
                         0.001_f, 0.0001_f, 0.000001_f);
    INFO("gain = " << gain);
    CHECK(Units::ampToDb(gain).repr() ==
          Catch::Approx(20.f * std::log10(gain.repr())).epsilon(0.01f));
  }

  // under the limit, it clamps to -inf
  SECTION("limit range") {
    CHECK(Units::ampToDb(0.00000001_f) == minus_infinity);
    CHECK(Units::ampToDb(0.000000001_f) == minus_infinity);
  }
}

TEST_CASE("dbToAmp", "[units]") {
  SECTION("normal range") {
    auto db = GENERATE(0_f, 6_f, 100_f, 200_f, 500_f, -10_f, -50_f);
    INFO("dB = " << db);
    CHECK(Units::dbToAmp(db).repr() ==
          Catch::Approx(std::pow(10.f, db.repr() / 20.f)).epsilon(0.0001f));
  }

  // under the limit, it clamps to 0
  SECTION("limit range") {
    CHECK(Units::dbToAmp(-150_f) == 0_f);
    CHECK(Units::dbToAmp<16>(-97_f) == 0_f);
  }
}

TEST_CASE("ampToDbToAmp", "[units]") {
  SECTION("normal range") {
    auto amp = GENERATE(1_f, 0.1_f, 0.01_f, 0.000001_f, 10_f, 100_f, 1000_f, 10000_f);
    INFO("amp = " << amp);
    CHECK(Units::dbToAmp(Units::ampToDb(amp)).repr() == Catch::Approx(amp.repr()).epsilon(0.01));
  }

  SECTION("limit range") {
    auto amp = GENERATE(0.00000001_f, 0.0000000001_f);
    INFO("amp = " << amp);
    CHECK(Units::dbToAmp(Units::ampToDb(amp)) == 0_f);
  }
}
