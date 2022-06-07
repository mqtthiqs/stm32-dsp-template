#pragma once

#include <array>
#include <cassert>
#include <bitset>

#include "dsp_util.h"
#include "iter.h"
#include "numtypes.h"
#include "ring.h"
#include "util.h"

namespace grm {

struct Interpolation {
  enum Type {
    ZOH,
    LINEAR,
    HERMITE4,
    HERMITE6,
    OPTIMAL4,
    OPTIMAL6,
  };

  template <Type>
  static constexpr int kKernelSize = 0;

  template <Type>
  static constexpr f kDelayAmount = 0_f;

};


  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::ZOH> = 0;
  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::LINEAR> = 1;
  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::HERMITE4> = 3;
  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::HERMITE6> = 5;
  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::OPTIMAL4> = 3;
  template <>
  constexpr int Interpolation::kKernelSize<Interpolation::OPTIMAL6> = 5;

  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::ZOH> = 0_f;
  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::LINEAR> = 0_f;
  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::HERMITE4> = 1_f;
  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::HERMITE6> = 2_f;
  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::OPTIMAL4> = 1_f;
  template <>
  constexpr f Interpolation::kDelayAmount<Interpolation::OPTIMAL6> = 2_f;

// Base should be iterable and randomly accessible
template <typename Base>
struct BufferTrait : Base {
  using Base::Base;
  using T = typename Base::value_type;

  template <typename B>
  constexpr explicit BufferTrait(BufferTrait<B>& that) : Base(static_cast<B&>(that)) {}

  explicit BufferTrait(T x) : Base() { this->fill(x); }

  template <Interpolation::Type TYPE = Interpolation::LINEAR, typename INDEX>
  constexpr T interpolate(INDEX const index) const noexcept {
    assert(index >= INDEX(0));
    assert(int(index.integral() + Interpolation::kKernelSize<TYPE>) < this->size());
    auto [integral, fractional] = index.integral_fractional();

    if constexpr (TYPE == Interpolation::ZOH) {
      return (*this)[int(integral)];
    }

    else if constexpr (TYPE == Interpolation::LINEAR) {
      const T x0 = (*this)[int(integral)];
      const T x1 = (*this)[int(integral + 1)];
      return DspUtil::crossfade(x0, x1, fractional);
    }

    // 4-points Hermite interpolation
    else if constexpr (TYPE == Interpolation::HERMITE4) {
      const T xm1 = (*this)[int(integral)];
      const T x0 = (*this)[int(integral) + 1];
      const T x1 = (*this)[int(integral) + 2];
      const T x2 = (*this)[int(integral) + 3];
      const T c = (x1 - xm1) * T(0.5_f);
      const T v = x0 - x1;
      const T w = c + v;
      const T a = w + v + (x2 - x0) * T(0.5_f);
      const T b_neg = w + a;
      return (((a * T(fractional)) - b_neg) * T(fractional) + c) * T(fractional) + x0;
    }

    // 6-point, 5th-order Hermite (z-form)
    else if constexpr (TYPE == Interpolation::HERMITE6) {
      const T x = T(fractional);
      const T xm2 = (*this)[int(integral)];
      const T xm1 = (*this)[int(integral) + 1];
      const T x0 = (*this)[int(integral) + 2];
      const T x1 = (*this)[int(integral) + 3];
      const T x2 = (*this)[int(integral) + 4];
      const T x3 = (*this)[int(integral) + 5];
      const T z = x - T(0.5_f);
      const T even1 = xm2 + x3, odd1 = xm2 - x3;
      const T even2 = xm1 + x2, odd2 = xm1 - x2;
      const T even3 = x0 + x1, odd3 = x0 - x1;
      const T c0 = T(3_f / 256_f) * even1 - T(25_f / 256_f) * even2 + T(75_f / 128_f) * even3;
      const T c1 = T(-3_f / 128_f) * odd1 + T(61_f / 384_f) * odd2 - T(87_f / 64_f) * odd3;
      const T c2 = T(-5_f / 96_f) * even1 + T(13_f / 32_f) * even2 - T(17_f / 48_f) * even3;
      const T c3 = T(5_f / 48_f) * odd1 - T(11_f / 16_f) * odd2 + T(37_f / 24_f) * odd3;
      const T c4 = T(1_f / 48_f) * even1 - T(1_f / 16_f) * even2 + T(1_f / 24_f) * even3;
      const T c5 = T(-1_f / 24_f) * odd1 + T(5_f / 24_f) * odd2 - T(5_f / 12_f) * odd3;
      return ((((c5 * z + c4) * z + c3) * z + c2) * z + c1) * z + c0;
    }

    // Optimal 2x (4-point, 4th-order) (z-form)
    else if constexpr (TYPE == Interpolation::OPTIMAL4) {
      const T x = T(fractional);
      const T xm1 = (*this)[int(integral)];
      const T x0 = (*this)[int(integral) + 1];
      const T x1 = (*this)[int(integral) + 2];
      const T x2 = (*this)[int(integral) + 3];
      const T z = x - T(0.5_f);
      const T even1 = x1 + x0, odd1 = x1 - x0;
      const T even2 = x2 + xm1, odd2 = x2 - xm1;
      const T c0 = even1 * T(0.45645918406487612_f) + even2 * T(0.04354173901996461_f);
      const T c1 = odd1 * T(0.47236675362442071_f) + odd2 * T(0.17686613581136501_f);
      const T c2 = even1 * T(-0.253674794204558521_f) + even2 * T(0.25371918651882464_f);
      const T c3 = odd1 * T(-0.37917091811631082_f) + odd2 * T(0.11952965967158000_f);
      const T c4 = even1 * T(0.04252164479749607_f) + even2 * T(-0.04289144034653719_f);
      return (((c4 * z + c3) * z + c2) * z + c1) * z + c0;

    }

    // Optimal 2x (6-point, 5th-order) (z-form)
    else if constexpr (TYPE == Interpolation::OPTIMAL6) {
      const T x = T(fractional);
      const T xm2 = (*this)[int(integral)];
      const T xm1 = (*this)[int(integral) + 1];
      const T x0 = (*this)[int(integral) + 2];
      const T x1 = (*this)[int(integral) + 3];
      const T x2 = (*this)[int(integral) + 4];
      const T x3 = (*this)[int(integral) + 5];
      const T z = x - T(0.5_f);
      const T even1 = x1 + x0, odd1 = x1 - x0;
      const T even2 = x2 + xm1, odd2 = x2 - xm1;
      const T even3 = x3 + xm2, odd3 = x3 - xm2;
      const T c0 = even1 * T(0.40513396007145713_f) + even2 * T(0.09251794438424393_f) +
                   even3 * T(0.00234806603570670_f);
      const T c1 = odd1 * T(0.28342806338906690_f) + odd2 * T(0.21703277024054901_f) +
                   odd3 * T(0.01309294748731515_f);
      const T c2 = even1 * T(-0.191337682540351941_f) + even2 * T(0.16187844487943592_f) +
                   even3 * T(0.02946017143111912_f);
      const T c3 = odd1 * T(-0.16471626190554542_f) + odd2 * T(-0.00154547203542499_f) +
                   odd3 * T(0.03399271444851909_f);
      const T c4 = even1 * T(0.03845798729588149_f) + even2 * T(-0.05712936104242644_f) +
                   even3 * T(0.01866750929921070_f);
      const T c5 = odd1 * T(0.04317950185225609_f) + odd2 * T(-0.01802814255926417_f) +
                   odd3 * T(0.00152170021558204_f);
      return ((((c5 * z + c4) * z + c3) * z + c2) * z + c1) * z + c0;
    }
  }

  // linear interpolation with variable kernel size. Requires access to cell
  // [index—kernelSize+1..index+kernelSize]
  template <typename INDEX>
  constexpr T interpolateVariableKernel(INDEX const index, int kernelSize) const noexcept {
    assert(kernelSize >= 1);
    assert(index.integral() + kernelSize < this->size());
    assert(index >= INDEX(0_f));
    auto [integral, fractional] = index.integral_fractional();

    T sum = T(0_f);
    for (int i = -kernelSize + 1; i <= kernelSize; i++) {
      T x = (*this)[integral + i];
      T a = i <= 0 ? -fractional : fractional;
      sum += x * (a + T(kernelSize) - T(std::abs(i)));
    }
    return sum / T(kernelSize * kernelSize);
  }

  // linear interpolation with normalized index in [0..1[
  template <Interpolation::Type TYPE>
  constexpr T interpolateNormalized(f const phase) const noexcept {
    assert(phase >= 0_f && phase < 1_f);
    return interpolate<TYPE>(phase * f(this->size() - 1));
  }

  // linear interpolation with normalized index in [0..1[
  template <int FRAC, Interpolation::Type TYPE>
  constexpr T interpolateNormalized(Fixed<UNSIGNED, 0, FRAC> const phase) const noexcept {
    assert(is_power_of_2(Base::size() - 1));
    const int BITS = ilog2(Base::size());
    Fixed<UNSIGNED, BITS, FRAC - BITS> index = phase.template movr<BITS>();
    return interpolate<TYPE>(index);
  }

  f acf(int source, int dest, int length, int stride) const noexcept {
    assert(source >= 0);
    assert(dest >= 0);
    assert(source + length < this->size());
    assert(dest + length < this->size());
    f acf = 0_f;
    int max = source + length;
    while (source < max) {
      acf += f((*this)[source] * (*this)[dest]);
      source += stride;
      dest += stride;
    }
    return acf;
  }
};  // namespace grm

template <typename T, int SIZE>
struct Buffer : BufferTrait<Array<T, SIZE>> {
  using BufferTrait<Array<T, SIZE>>::BufferTrait;
  using Ptr = BufferTrait<typename Array<T, SIZE>::Ptr>;
};

template <typename T>
struct DynBuffer : BufferTrait<DynArray<T>> {
  using BufferTrait<DynArray<T>>::BufferTrait;
};

template <typename T, int SIZE>
struct PairBuffer : Buffer<Pair<T>, SIZE> {
  using Buffer<Pair<T>, SIZE>::Buffer;

  template <typename U>
  explicit PairBuffer(PairBuffer<U, SIZE>& in) noexcept {
    for (auto [x, y] : zip(in, *this)) y = {T(x.first), T(x.second)};
  }

  void destruct(Buffer<T, SIZE>& b1, Buffer<T, SIZE>& b2) const noexcept {
    for (auto [p, a, b] : zip(*this, b1, b2)) {
      a = p.first;
      b = p.second;
    }
  }

  void first(Buffer<T, SIZE>& b) const noexcept {
    for (auto [p, a] : zip(*this, b)) a = p.first;
  }

  void second(Buffer<T, SIZE>& b) const noexcept {
    for (auto [p, a] : zip(*this, b)) a = p.second;
  }

  void recompose(Buffer<T, SIZE> const& b1, Buffer<T, SIZE> const& b2) noexcept {
    for (auto [x, a, b] : zip(*this, b1, b2)) x = {a, b};
  }

  constexpr PairBuffer<T, SIZE>& operator*=(T x) noexcept {
    for (auto& i : *this) {
      i.first *= x;
      i.second *= x;
    }
    return *this;
  }
};

template <typename T, int SIZE>
struct DiffBuffer : PairBuffer<T, SIZE> {
  constexpr T interpolateNormalized(f phase) const noexcept {
    constexpr f const maximum = f(SIZE - 1);
    phase *= maximum;
    auto [integral, fractional] = phase.integral_fractional();
    auto [a, d] = (*this)[integral];
    return DspUtil::crossfade_with_diff(a, d, fractional);
  }

  template <int FRAC>
  constexpr T interpolateNormalized(Fixed<UNSIGNED, 0, FRAC> const phase) const noexcept {
    static_assert(is_power_of_2(SIZE - 1), "only power-of-two-sized buffers");
    constexpr int BITS = ilog2(SIZE);
    Fixed<UNSIGNED, BITS, FRAC - BITS> p = phase.template movr<BITS>();
    auto [integral, fractional] = p.integral_fractional();
    auto [a, d] = (*this)[integral];
    return DspUtil::crossfade_with_diff(a, d, fractional);
  }
};

template <typename T, int SIZE>
struct RingBuffer : BufferTrait<RingArray<T, SIZE>> {
  using BufferTrait<RingArray<T, SIZE>>::BufferTrait;

  template <typename INDEX>
  struct View {
    using Index = INDEX;
    constexpr View(RingBuffer& buffer, Index const delay) noexcept
        : buffer_(buffer), delay_(delay) {}

    constexpr explicit View(RingBuffer& buffer) noexcept : View(buffer, Index(0_f)) {}

    // default constructor to make view arrays
    constexpr View() noexcept = default;

    template <Interpolation::Type TYPE>
    constexpr T interpolate(Index const shift) const noexcept {
      return buffer_.template interpolate<TYPE>(delay_ + shift);
    }

    int size() const noexcept { return buffer_.size(); }
    int sizeIncludingDelay() const noexcept { return buffer_.size() - delay_.integral(); }

    Index delay() const { return delay_; }

   private:
    BufferTrait<Ring<typename Array<T, SIZE>::Ptr>> buffer_;
    Index delay_;
  };

  template <typename INDEX>
  View<INDEX> view(INDEX delay) {
    return View(*this, delay);
  }
};

template <typename T>
struct RingDynBuffer : BufferTrait<RingDynArray<T>> {
  using BufferTrait<RingDynArray<T>>::BufferTrait;

  template <typename INDEX>
  struct View {
    using Index = INDEX;
    constexpr View(RingDynBuffer& buffer, Index const delay) noexcept
        : buffer_(buffer), delay_(delay) {}

    constexpr explicit View(RingDynBuffer& buffer) noexcept : View(buffer, Index(0_f)) {}

    // default constructor to make view arrays
    constexpr View() noexcept = default;

    template <Interpolation::Type TYPE>
    constexpr T interpolate(Index const shift) const noexcept {
      assert(shift + delay_ < Index(uint32_t(size() - Interpolation::kKernelSize<TYPE>)));
      return buffer_.template interpolate<TYPE>(delay_ + shift);
    }

    int size() const noexcept { return buffer_.size(); }
    int sizeIncludingDelay() const noexcept { return size() - int(delay_.integral()); }

    Index delay() const { return delay_; }

   private:
    BufferTrait<RingDynArray<T>> buffer_;
    Index delay_;
  };

  template <typename INDEX>
  View<INDEX> view(INDEX delay) {
    return View(*this, delay);
  }
};

// Stores a signal at different "zoom levels", which sizes are increasing powers of 2.
// We look up into this through a view, and at a specific zoom level;
// and get back minimum and maximum values for the window of the size of the zoom level.
struct ZoomableMinMaxBuffer {
  // a [Sample] is a pair of min/max values of the corresponding time span
  using Sample = Interval;

  ZoomableMinMaxBuffer(int numZoomLevels, int bufferSize)
      : levels_(numZoomLevels), mask_((1 << numZoomLevels) - 1) {
    for (auto& l : levels_) {
      l = ZoomLevel(bufferSize);
      // each zoom level has half the size of the previous one
      bufferSize >>= 1;
    }
  }

  void operator<<(f x) {
    // prepare data for the first level, which runs at the nominal sample rate.
    levels_[0].process(Sample {x}, Sample {x});

    // [changes] holds the "derivative" of sampleCounter_, ie. the bits that changed between two
    // consecutive values, eg. 1, 11, 1, 111, 1, 11, 1, 1111 etc. The number of bit set dictates
    // the number of levels to process, therefore each level runs at half the sample rate of its
    // predecessor.
    std::bitset<32> changes = (sampleCounter_ ^ (sampleCounter_ + 1)) & mask_;

    for (int i : count(1, int(changes.count()))) {
      // at each level, compute the new min/max from the last two min/max pairs of the previous
      // level
      Sample previous = levels_[i - 1].getBuffer()[1];
      Sample current = levels_[i - 1].getBuffer()[0];
      levels_[i].process(previous, current);
    }

    sampleCounter_++;
  }

  void operator<<(DynBuffer<f> buf) {
    // TODO: is this optimizable?
    for (f const& x : buf) *this << x;
  }

  int size(int zoomLevel) { return levels_[zoomLevel].size(); }

  template <typename IndexType>
  struct View {
    using Index = IndexType;
    View(ZoomableMinMaxBuffer& buf, IndexType delay) : views_(buf.levels_.size()) {
      for (auto [v, b] : zip(views_, buf.levels_)) {
        v = BufferView(b.getBuffer(), delay);
        // since each level is downsampled by 2, delays must be divided by 2
        delay = delay.div2(1);
      }
    }

    explicit View(ZoomableMinMaxBuffer& buf) : View(buf, IndexType(0_f)) {}
    View() = default;

    template <Interpolation::Type TYPE>
    Sample interpolate(Index index, int zoomLevel) {
      assert(zoomLevel < numZoomLevels());
      // since each level is downsampled by 2, indexes must be divided by 2^zoomLevel
      index = index.div2(zoomLevel);
      return views_[zoomLevel].template interpolate<TYPE>(index);
    }

    int size(int zoomLevel) { return views_[zoomLevel].size() << zoomLevel; }
    int sizeIncludingDelay(int zoomLevel) {
      return views_[zoomLevel].sizeIncludingDelay() << zoomLevel;
    }
    IndexType delay(int zoomLevel) { return views_[zoomLevel].delay(); }
    int numZoomLevels() { return views_.size(); }

   private:
    using BufferView = typename RingDynBuffer<Sample>::template View<IndexType>;
    DynArray<BufferView> views_;
  };

 private:
  struct ZoomLevel {
    ZoomLevel() = default;
    constexpr ZoomLevel(int size) noexcept : buffer_(size) { buffer_.fill({0_f}); }
    constexpr RingDynBuffer<Sample>& getBuffer() noexcept { return buffer_; }

    // samples [previous] and [current] from the previous level get combined into the current
    // level's sample
    constexpr void process(Sample previous, Sample current) {
      f min = current.first.min(previous.first);
      f max = current.second.max(previous.second);
      Sample s {min, max};
      buffer_ << s;
    }

    int size() { return buffer_.size(); }

   private:
    RingDynBuffer<Sample> buffer_;
  };

  // TODO: each level should have half the size of the previous one
  DynArray<ZoomLevel> levels_;
  uint32_t sampleCounter_ = 0;
  uint32_t mask_;
};

}  // namespace grm
