#pragma once

#include "dynarray.h"
#include "iter.h"

namespace grm {

template <typename Container, typename = void>
struct RingBase : Container {
  using Container::Container;
  using T = typename Container::value_type;

  template <typename, typename>
  friend struct RingBase;

  template <typename C>
  constexpr RingBase(RingBase<C>& that) noexcept
      : Container(static_cast<C&>(that)), cursor_(that.cursor_) {}

  constexpr void operator<<(T const x) noexcept {
    size_t index = ++cursor_ % size_t(Container::size());
    Container::operator[](int(index)) = x;
  }

  // drops top [n] element
  constexpr void drop(int n) noexcept { cursor_ += size_t(Container::size() - n); }

  constexpr T const& operator[](int n) const noexcept {
    assert(0 <= n && n < Container::size());
    size_t index = (cursor_ - size_t(n)) % size_t(Container::size());
    return Container::operator[](int(index));
  }

  constexpr T const& last() const noexcept {
    size_t index = (cursor_ + 1) % size_t(Container::size());
    return Container::operator[](int(index));
  }

  size_t getCursor() const { return size_t(cursor_); }

 private:
  // index of last element written.
  // the cursor begins high enough to allow subtractions, and keeps growing
  // indefintely; we ignore the glitch that will happen when it wraps over in
  // case SIZE != 2^n.
  size_t cursor_ = size_t(Container::size());
};

template <typename Container>
struct RingBase<Container, std::enable_if_t<Container::static_size == 1 << 8>> : Container {
  using Container::Container;
  using T = typename Container::value_type;

  template <typename, typename>
  friend struct RingBase;

  template <typename C>
  constexpr RingBase(RingBase<C>& that) noexcept
      : Container(static_cast<C&>(that)), cursor_(that.cursor_) {}

  constexpr void operator<<(T const x) noexcept { Container::operator[](++cursor_) = x; }

  // drops top [n] element
  constexpr void drop(int n) noexcept { cursor_ -= n; }

  constexpr T const& operator[](int n) const noexcept {
    assert(n < Container::size());
    return Container::operator[](uint8_t(cursor_ - uint8_t(n)));
  }

  constexpr T const& last() const noexcept { return Container::operator[](uint8_t(cursor_ + 1)); }

  size_t getCursor() const { return size_t(cursor_); }

 private:
  uint8_t cursor_;
};

template <typename Container>
struct RingBase<Container, std::enable_if_t<Container::static_size == 1 << 16>> : Container {
  using Container::Container;
  using T = typename Container::value_type;

  template <typename, typename>
  friend struct RingBase;

  template <typename C>
  constexpr RingBase(RingBase<C>& that) noexcept
      : Container(static_cast<C&>(that)), cursor_(that.cursor_) {}

  constexpr void operator<<(T const x) noexcept { Container::operator[](++cursor_) = x; }

  // drops top [n] element
  constexpr void drop(int n) noexcept { cursor_ -= n; }

  constexpr T const& operator[](int n) const noexcept {
    assert(n < Container::size());
    return Container::operator[](uint16_t(cursor_ - uint16_t(n)));
  }

  constexpr T const& last() const noexcept { return Container::operator[](uint16_t(cursor_ + 1)); }

  size_t getCursor() const { return size_t(cursor_); }

 private:
  uint16_t cursor_;
};

template <typename Container>
struct Ring : RingBase<Container> {
  using RingBase<Container>::RingBase;
  using T = typename Container::value_type;

  constexpr void operator<<(T const x) noexcept { RingBase<Container>::operator<<(x); }

  constexpr void operator<<(DynArray<T> const arr) noexcept {
    // TODO: optimize with std::reverse_copy
    for (T const& x : arr) *this << x;
  }

  // copies to [dest] the last [dest.size()] values in the ring buffer
  constexpr void copyTo(DynArray<T> dest) const noexcept {
    assert(dest.size() <= this->size());

    T const* src = this->data();
    int srcSize = this->size();
    T* dst = dest.data();
    int dstSize = dest.size();

    auto cursor = int(this->getCursor() % size_t(Container::size())) + 1;

    int toCopy = std::min(dstSize, cursor);
    std::copy(src + cursor - toCopy, src + cursor, dst + dest.size() - toCopy);
    int remaining = dstSize - toCopy;
    if (remaining > 0) std::copy(src + srcSize - remaining, src + srcSize, dst);
  }

  constexpr void copyToReversed(DynArray<T> dest) const noexcept {
    assert(dest.size() <= this->size());
    int size = dest.size();
    // TODO: optimize with std::reverse_copy
    for (int i = 0; i < size; i++) dest[i] = (*this)[i];
  }
};

template <typename Container>
struct RingView {
  static constexpr int static_size = Container::size();
  using T = typename Container::value_type;

  constexpr RingView() = default;
  constexpr explicit RingView(Ring<Container>& arr, int delay = 0) noexcept : arr_(arr) {
    assert(delay >= 0);
    arr_.drop(delay);
  }

  constexpr T operator[](int shift) const noexcept { return arr_[shift]; }
  constexpr void copyTo(DynArray<T> dest) const { arr_.copyTo(dest); }
  constexpr void copyToReversed(DynArray<T> dest) const { arr_.copyToReversed(dest); }

 private:
  Ring<typename Container::Ptr> arr_;
};

template <typename T, int SIZE>
struct RingArray : Ring<Array<T, SIZE>> {
  using Ring<Array<T, SIZE>>::Ring;
  using View = RingView<Array<T, SIZE>>;

  View view(int delay = 0) { return View(*this, delay); }
};

template <typename T>
struct RingDynArray : Ring<DynArray<T>> {
  using Ring<DynArray<T>>::Ring;
  using View = RingView<DynArray<T>>;

  View view(int delay = 0) { return View(*this, delay); }
};

}  // namespace grm