#include <array>
#include <catch2/catch_all.hpp>
#include <iostream>
#include <list>
#include <vector>

#include "iter.h"
#include "numtypes.h"

using namespace grm;

TEST_CASE("zip") {
  std::vector<int> v1 = {1, 2, 3, 4, 5};
  std::array<int, 5> a1 = {5, 4, 3, 2, 1};
  int p1[] = {6, 7, 8, 9, 10};

  SECTION("read") {
    int s = 0;
    for (auto [x, y, z] : zip(v1, a1, p1)) { s += x + y; }
    REQUIRE(s == 30);
  }

  SECTION("write") {
    for (auto [x, y, z] : zip(v1, a1, p1)) { z = x + y; }
    REQUIRE(p1[0] == 6);
    REQUIRE(p1[4] == 6);
  }

  // TODO: test that this fails (different lengths):
  //  std::vector<int> v2 = {1,2,3};
  //  for (auto [x, y] : zip(v1, v2)) {
  //    std::cout << x << ", " << y << std::endl;
  //  }
}

TEST_CASE("count") {
  SECTION("simple count") {
    int s = 0;
    for (int i : count(10)) { s += i; }
    REQUIRE(s == 45);
  }

  SECTION("int count with start") {
    int s = 0;
    for (int i : count(10, 12)) { s += i; }
    REQUIRE(s == 21);
  }

  SECTION("float count with step") {
    float s = 0.;
    for (float i : count(0.f, 1.f, 0.1f)) { s += i; }
    REQUIRE(s == 4.5f);  // 1.f not included
  }

  SECTION("count with negative step") {
    float s = 0.;
    for (float i : count(1.f, 0.f, -0.1f)) { s += i; }
    REQUIRE(s == Catch::Approx(5.5f));
  }

  SECTION("count with negative value") {
    float s = 0.;
    for (float i : count(-1.f, 0.f, 0.1f)) s += i;
    REQUIRE(s == Catch::Approx(-5.5f));
  }

  SECTION("count with wrapped floats") {
    f s = 0_f;
    for (f i : count(-1_f, 0_f, 0.125_f)) s += i;
    REQUIRE(s == -4.5_f);
  }

  SECTION("count with Fixed") {
    s16_16 s = 0_s16_16;
    for (s16_16 i : count(-1_s16_16, 0_s16_16, 0.125_s16_16)) s += i;
    REQUIRE(s == -4.5_s16_16);
  }

  SECTION("count with unsigned Fixed") {
    u16_16 s = 0_u16_16;
    for (u16_16 i : count(1_u16_16, 2_u16_16, 0.125_u16_16)) s += i;
    REQUIRE(s == 11.5_u16_16);
  }

  SECTION("count with unsigned Fixed (high values)") {
    s17_15 s = 0_s17_15;
    for (s1_15 i : count(0_s1_15, 0.9_s1_15, 0.03125_s1_15)) s += s17_15(i);
    REQUIRE(s == 12.6875_s17_15);
  }
}

TEST_CASE("count and zip") {
  SECTION("count in zip") {
    std::vector<int> v1 = {1, 2, 3, 4, 5};
    for (auto [i, x] : zip(count(1, 6), v1)) { REQUIRE(i == x); }
  }

  SECTION("count to table") {
    int p1[] = {6, 7, 8, 9, 10};
    for (auto [i, x] : zip(count(0, 5), p1)) x = i;
    REQUIRE(p1[0] == 0);
    REQUIRE(p1[1] == 1);
    REQUIRE(p1[2] == 2);
    REQUIRE(p1[3] == 3);
  }

  SECTION("enumerate") {
    int p1[] = {0, 1, 2, 3, 4};
    for (auto [i, x] : enumerate(p1)) REQUIRE(x == i);
  }
}
