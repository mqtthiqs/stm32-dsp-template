#include <catch2/catch_all.hpp>

#include "ring.h"

using namespace grm;

TEST_CASE("ring array") {
  RingArray<int, 5> ring;
  SECTION("write then read") {
    // initialize with negative values: -3, -2, -1.
    for (int i : count(-ring.size(), 0)) ring << i;

    for (int i : count(16)) {
      ring << i;
      for (int j : count(ring.size())) REQUIRE(ring[j] == i - j);
    }
  }

  SECTION("view") {
    // initialize with negative values: -3, -2, -1.
    for (int i : count(-ring.size(), 0)) ring << i;

    auto v = ring.view();

    int cnt = GENERATE(1, 2, 3, 4);
    for (int i : count(cnt)) {
      ring << i;
      for (int j : count(ring.size() - cnt)) REQUIRE(v[j] == -j - 1);
    }
  }
}

TEST_CASE("ring array 8-specializations") {
  int max = GENERATE(5, 255, 256, 482);
  RingArray<int, 1 << 8> ring;
  for (int i : count(max)) ring << i;

  SECTION("copyTo") {
    DynArray<int> arr {5};
    ring.copyTo(arr);
    for (auto [i, x] : enumerate(arr)) REQUIRE(x == max - arr.size() + i);
  }
}

TEST_CASE("ring array 16-specializations") {
  SECTION("push") {
    RingArray<int, 1 << 16> ring;
    DynArray<int> buf;
    ring << buf;
  }

  int max = GENERATE(5, 65535, 65536, 482);
  RingArray<int, 1 << 16> ring;
  for (int i : count(max)) ring << i;

  SECTION("copyTo") {
    DynArray<int> arr {5};
    ring.copyTo(arr);
    for (auto [i, x] : enumerate(arr)) REQUIRE(x == max - arr.size() + i);
  }
}

TEST_CASE("ring dynarray") {
  RingDynArray<int> ring {5};

  SECTION("write then read") {
    // initialize with negative values: -3, -2, -1.
    for (int i : count(-ring.size(), 0)) ring << i;

    for (int i : count(16)) {
      ring << i;
      for (int j : count(ring.size())) REQUIRE(ring[j] == i - j);
    }
  }

  SECTION("view") {
    // initialize with negative values: -3, -2, -1.
    for (int i : count(-ring.size(), 0)) ring << i;

    auto v = ring.view();

    int cnt = GENERATE(1, 2, 3, 4);
    for (int i : count(cnt)) {
      ring << i;
      for (int j : count(ring.size() - cnt)) REQUIRE(v[j] == -j - 1);
    }
  }
}

TEST_CASE("ring copyTo") {
  RingArray<int, 10> ring;

  int upTo = GENERATE(3, 5, 9, 10, 11, 15, 19, 20, 25);

  for (int i : count(upTo)) ring << i;

  // TODO: dest size 0
  DynArray<int> dest {GENERATE(0, 1, 2, 5, 9, 10)};
  ring.copyTo(dest);

  INFO("upTo=" << upTo << ", dest size=" << dest.size());

  for (int i : count(dest.size())) REQUIRE(dest[i] == ring[dest.size() - 1 - i]);
}