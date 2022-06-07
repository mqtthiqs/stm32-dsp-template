#include <catch2/catch_all.hpp>

#include "buffer.h"

using namespace grm;

template <Interpolation::Type type>
void checkInterpolationIntegral(DynBuffer<f>& buf) {
  for (int i : count(2, buf.size() - Interpolation::kKernelSize<type>)) {
    f index = f(i) - Interpolation::kDelayAmount<type>;
    REQUIRE(buf.interpolate<type>(index).repr() == Catch::Approx(buf[i].repr()).epsilon(0.0001f));
  }
}

TEST_CASE("interpolation") {
  DynBuffer<f> buf {100};
  for (auto& x : buf) x = f(rand()) / f(RAND_MAX) * 4_f - 2_f;

  SECTION("interpolation at integral indices = access") {
    checkInterpolationIntegral<Interpolation::ZOH>(buf);
    checkInterpolationIntegral<Interpolation::LINEAR>(buf);
    checkInterpolationIntegral<Interpolation::HERMITE4>(buf);
    checkInterpolationIntegral<Interpolation::HERMITE6>(buf);
  }
}