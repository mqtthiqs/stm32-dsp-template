#pragma once

#include "dynarray.h"
#include "iter.h"
#include "maths.h"
#include "numtypes.h"

namespace grm {

struct Window {
  enum Type {
    RECTANGLE,
    HAMMING,
    HANN,
    ROOT_HANN,
    BLACKMAN_HARRIS,
  };

  static f ncos(int order, int i, int size) noexcept {
    return Math::cos(f(order * i) * Math::pi / f(size - 1));
  }

  Window(int size, Type type) : window_(size) {
    switch (type) {
      case RECTANGLE: window_.fill(1_f); break;
      case HAMMING:
        for (auto [i, x] : enumerate(window_)) x = 0.54_f - 0.46_f * ncos(2, i, size);
        break;
      case HANN:
        for (auto [i, x] : enumerate(window_)) x = 0.5_f - 0.5_f * ncos(2, i, size);
        break;
      case ROOT_HANN:
        for (auto [i, x] : enumerate(window_)) x = (0.5_f - 0.5_f * ncos(2, i, size)).sqrt();
        break;
      case BLACKMAN_HARRIS:
        for (auto [i, x] : enumerate(window_))
          x = 0.35875_f - 0.48829_f * ncos(2, i, size) + 0.14128_f * ncos(4, i, size) -
              0.01168_f * ncos(6, i, size);

        break;
    }
  }

  void apply(DynArray<f> buf) noexcept {
    assert(buf.size() == window_.size());
    buf *= window_;
  }

 private:
  DynArray<f> window_;
};
}  // namespace grm