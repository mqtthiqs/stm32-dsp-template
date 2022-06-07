#include <catch2/catch_all.hpp>

#include "array.h"
#include "iter.h"

using namespace grm;

TEST_CASE("array") {
  SECTION("init by filling") {
    Array<int, 5> arr {42};
    for (int x : arr) { REQUIRE(x == 42); }
    REQUIRE(arr[4] == 42);
  }

  SECTION("aggregate init") {
    Array<int, 5> arr = {42, 43, 44, 45, 46};
    int i = 0;
    for (int x : arr) {
      REQUIRE(x == 42 + i);
      i++;
    }
  }

  SECTION("enumerate") {
    Array<int, 5> arr = {1, 2, 3, 4, 5};
    for (auto [i, x] : enumerate(arr)) { REQUIRE(x == i + 1); }
  }

  SECTION("make_index_sequence") {
    struct C {
      C(int i) : i_(i) {}
      int i_;
    };
    auto arr = Array<C, 3>(42, std::make_index_sequence<3>());
    REQUIRE(arr[0].i_ == 42);
    REQUIRE(arr[1].i_ == 42);
    REQUIRE(arr[2].i_ == 42);
  }
}
