#include "maths.h"

#include <cstdint>  // for uint32_t
#include <utility>  // for tuple_element<>::type, pair

#include "dsp.h"   // for MagicSine
#include "iter.h"  // for enumerate, operator!=, zip, zip_iterator

using namespace grm;

Math::Exp2Table::Exp2Table() noexcept {
  double x = 1.0f;
  for (auto& z : *this) {
    z = uint32_t(x * (1 << 23));
    x *= exp2_increment;
  }
}

Math::SineTable::SineTable() noexcept {
  MagicSine magic(1_f / f(sine_size));
  f previous = 0_f;
  for (auto& [v, d] : *this) {
    v = previous;
    previous = magic.process().second;
    d = previous - v;
  }
}

Math::CosineTable::CosineTable() noexcept {
  MagicSine magic(1_f / f(sine_size));
  f previous = 1_f;
  for (auto& [v, d] : *this) {
    v = previous;
    previous = magic.process().first;
    d = previous - v;
  }
}

Math::Log2Table::Log2Table() noexcept {
  for (auto [i, z] : enumerate(*this)) {
    f x = f(i) / f(log2_size) + 1_f;
    f y = f(std::log2f(x.repr()));
    z = uint32_t((y * f(1 << 23)).repr());
  }
}

Math::XFadeTable::XFadeTable() noexcept {
  MagicSine magic(0.25_f / f(xfade_size));
  f previous = 0_f;
  for (auto& [v, d] : *this) {
    v = previous;
    previous = magic.process().second;
    d = previous - v;
  }
}

Math::Exp2Table const Math::exp2_table;
Math::Log2Table const Math::log2_table;

Math::XFadeTable const Math::xfade_table;
Math::SineTable const Math::sine_table;
Math::CosineTable const Math::cosine_table;
