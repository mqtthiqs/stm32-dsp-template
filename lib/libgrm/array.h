#pragma once

#include <array>

#include "util.h"

namespace grm {
template <typename Value>
struct ArrayIterator {
  using iterator_category = std::forward_iterator_tag;
  using difference_type = std::ptrdiff_t;
  using value_type = Value;
  using pointer = value_type*;
  using reference = value_type&;

  explicit ArrayIterator(pointer x) : x_(x) {}

  reference operator*() const noexcept { return *x_; }
  pointer operator->() noexcept { return x_; }
  ArrayIterator& operator++() noexcept {
    x_++;
    return *this;
  }
  ArrayIterator const operator++(int) noexcept {
    ArrayIterator tmp = *this;
    ++(*this);
    return tmp;
  }

  friend bool operator==(const ArrayIterator& a, const ArrayIterator& b) noexcept {
    return a.x_ == b.x_;
  }
  friend bool operator!=(const ArrayIterator& a, const ArrayIterator& b) noexcept {
    return a.x_ != b.x_;
  }
  friend difference_type operator-(const ArrayIterator& a, const ArrayIterator& b) noexcept {
    return a.x_ - b.x_;
  }

 private:
  pointer x_;
};

template <typename T, int SIZE>
struct CopyableArray : std::array<T, size_t(SIZE)> {
  using Base = std::array<T, size_t(SIZE)>;
  using value_type = T;
  using iterator = ArrayIterator<value_type>;
  using const_iterator = ArrayIterator<value_type const>;

  static constexpr int static_size = SIZE;
  constexpr explicit CopyableArray(T x) noexcept { Base::fill(x); }

  // when T is not default constructible, this creates an array of T(arg), calling T's constructor
  // for each element. Usage: CopyableArray(arg, std::make_index_sequence<N>()). Caveat: works only
  // for one-argument constructors of T.
  template <typename Arg, size_t... Is>
  constexpr explicit CopyableArray(Arg&& arg, std::index_sequence<Is...>) noexcept
      : Base {((void)Is, T(std::forward<Arg>(arg)))...} {}
  template <typename Arg1, typename Arg2, size_t... Is>
  constexpr explicit CopyableArray(Arg1&& arg1, Arg2&& arg2, std::index_sequence<Is...>) noexcept
      : Base {((void)Is, T(std::forward<Arg1>(arg1), std::forward<Arg2>(arg2)))...} {}

  template <typename... Args>
  constexpr CopyableArray(Args&&... args) noexcept : Base {std::forward<Args>(args)...} {}

  constexpr int size() const noexcept { return SIZE; }

  constexpr typename Base::reference operator[](int pos) noexcept {
    return Base::operator[](size_t(pos));
  }

  constexpr typename Base::const_reference operator[](int pos) const noexcept {
    return Base::operator[](size_t(pos));
  }

  constexpr int binarySearch(T const x) const noexcept {
    int low = 0;
    int high = size() - 1;

    while (low + 1 < high) {
      int mid = (low + high) / 2;
      if (x < (*this)[mid]) high = mid;
      else low = mid;
    }
    return low;
  }

  struct Ptr {
    using value_type = CopyableArray::value_type;
    using iterator = CopyableArray::iterator;
    using const_iterator = CopyableArray::const_iterator;

    // default unsafe constructor for arrays
    Ptr() = default;

    explicit constexpr Ptr(CopyableArray<T, SIZE>& arr) noexcept : data_(arr.data()) {}

    explicit constexpr Ptr(T* data) noexcept : data_(data) {}
    constexpr int size() const noexcept { return SIZE; }
    static constexpr int static_size = SIZE;

    constexpr T* data() noexcept { return data_; }
    constexpr T const* data() const noexcept { return data_; }

    constexpr T& operator[](int const i) noexcept {
      assert(i < size());
      return data_[i];
    }

    constexpr T const& operator[](int const i) const noexcept {
      assert(i < size());
      return data_[i];
    }

    constexpr void fill(T const& x) noexcept { std::fill(data_, data_ + size(), x); }

    constexpr auto begin() noexcept { return iterator(data_); }
    constexpr auto end() noexcept { return iterator(data_ + size()); }

    constexpr auto begin() const noexcept { return const_iterator(data_); }
    constexpr auto end() const noexcept { return const_iterator(data_ + size()); }

   private:
    T* data_ = nullptr;
  };
};

template <typename T, int SIZE>
struct Array : CopyableArray<T, SIZE>, Nocopy {
  using CopyableArray<T, SIZE>::CopyableArray;

  // forward here to make it constexpr (an inherited constructor cannot be constexpr it seems)
  template <typename... Args>
  constexpr Array(Args&&... args) noexcept : CopyableArray<T, SIZE> {std::forward<Args>(args)...} {}
};
}  // namespace grm