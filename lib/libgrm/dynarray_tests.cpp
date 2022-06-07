#include <catch2/catch_all.hpp>

#include "dynarray.h"
#include "iter.h"

using namespace grm;

TEST_CASE("dynarray") {
  DynArray<int> arr {10, 88};

  SECTION("write then reread") {
    for (auto [i, x] : enumerate(arr)) x = i;
    for (auto [i, x] : enumerate(arr)) REQUIRE(x == i);
  }

  SECTION("init fill") {
    DynArray<int> a1 {5, 1}, a2 {5, 2}, a3 {5, 3};
    for (auto [x, y, z] : zip(a1, a2, a3)) {
      REQUIRE(x == 1);
      REQUIRE(y == 2);
      REQUIRE(z == 3);
    }
  }

  SECTION("copy increment") {
    DynArray<int> arr2 {10};
    REQUIRE(arr.data() != arr2.data());
    for (auto [i, x] : enumerate(arr)) x = i;
    for (auto [x, y] : zip(arr, arr2)) y = x + 1;
    for (auto [x, y] : zip(arr, arr2)) REQUIRE(y == x + 1);
  }

  SECTION("copy") {
    DynArray<int> arr2 {10, 42};
    arr = arr2;  // copy assign
    for (int x : arr) REQUIRE(x == 42);
    DynArray<int> arr3 {arr2};  // copy cstr
    for (auto [x, y] : zip(arr2, arr3)) REQUIRE(x == y);
  }

  SECTION("move") {
    DynArray<int> arr2 {std::move(arr)};  // move cstr
    for (int i : arr2) REQUIRE(i == 88);
    REQUIRE(arr.size() == 0);

    DynArray<int> arr3 {7, 3};
    arr3 = std::move(arr2);
    REQUIRE(arr3.size() == 10);
    for (int i : arr3) REQUIRE(i == 88);
  }
}

TEST_CASE("dynarray ptr") {
  Array<int, 5> arr = {1, 2, 3, 4, 5};
  DynArray<int> dyn {arr};

  SECTION("read") {
    REQUIRE(dyn[0] == arr[0]);
    REQUIRE(dyn[4] == arr[4]);
  }

  SECTION("write") {
    dyn[0] = 0;
    REQUIRE(dyn[0] == 0);
  }

  SECTION("traverse read") {
    int sum = 0;
    for (int i : dyn) sum += i;
    REQUIRE(sum == 1 + 2 + 3 + 4 + 5);
  }

  SECTION("traverse write") {
    for (int& i : dyn) i = 0;
    REQUIRE(dyn[0] == 0);
    REQUIRE(dyn[4] == 0);
  }

  SECTION("zip") {
    for (auto [a, b] : zip(arr, dyn)) { REQUIRE(a == b); }
  }

  SECTION("zip+count") {
    for (auto [i, x] : zip(count(dyn.size()), dyn)) { REQUIRE(x == i + 1); }
  }

  SECTION("enumerate") {
    for (auto [i, x] : enumerate(dyn)) { REQUIRE(x == i + 1); }
  }
}

TEST_CASE("dynarray of dynarray") {
  Array<int, 5> o {0, 1, 2, 3, 4};

  DynArray<DynArray<int>> buffers {0};
  buffers = DynArray<DynArray<int>>(4);

  for (auto [i, buf] : enumerate(buffers)) { buf = DynArray<int>(o.data(), o.size()); }

  for (auto [i, buf] : enumerate(buffers))
    for (auto [j, x] : enumerate(buf)) REQUIRE(x == j);
}

void readFunction(DynArray<int> a) {
  for (auto [i, x] : enumerate(a)) REQUIRE(x == i);
}

void writeFunction(DynArray<int> a) {
  for (auto [i, x] : enumerate(a)) x = i + 1;
}

void readFunctionRef(DynArray<int>& a) {
  for (auto [i, x] : enumerate(a)) REQUIRE(x == i);
}

void writeFunctionRef(DynArray<int>& a) {
  for (auto [i, x] : enumerate(a)) x = i + 1;
}

TEST_CASE("dynarray copy cstr") {
  DynArray<int> arr {5};
  for (auto [i, x] : enumerate(arr)) x = i;

  SECTION("copy cstr") {
    DynArray<int> a2 {arr};  // copy construct
    for (auto [i, x] : enumerate(a2)) REQUIRE(x == i);
  }

  SECTION("read function") { readFunction(arr); }
  SECTION("write function") {
    writeFunction(arr);
    for (auto [i, x] : enumerate(arr)) REQUIRE(x == i + 1);
  }
  SECTION("read function ref") { readFunctionRef(arr); }
  SECTION("write function ref") {
    writeFunctionRef(arr);
    for (auto [i, x] : enumerate(arr)) REQUIRE(x == i + 1);
  }
}