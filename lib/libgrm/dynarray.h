#pragma once

#include "array.h"
#include "iter.h"

namespace grm {

template <typename T>
struct DynArray {
  using value_type = T;
  using iterator = ArrayIterator<value_type>;
  using const_iterator = ArrayIterator<value_type const>;

  using Ptr = DynArray;

  constexpr DynArray() = default;

  // Allocating constructors (owned)
  explicit constexpr DynArray(int size) noexcept : size_(size), data_(new T[size_t(size_)]) {}
  constexpr DynArray(int size, T fill) : DynArray(size) { this->fill(fill); }

  // Move constructor
  constexpr DynArray(DynArray<T>&& that) noexcept
      : owned_(that.owned_), size_(that.size_), data_(that.data_) {
    that.data_ = nullptr;
    that.size_ = 0;
    that.owned_ = false;
  }

  // Non-allocating constructors (not owned)
  constexpr DynArray(T* data, int size) noexcept : owned_(false), size_(size), data_(data) {}

  constexpr DynArray(DynArray<T> const& arr, int size) noexcept
      : owned_(false), size_(size), data_(arr.data_) {
    assert(size <= arr.size());
  }

  // Copy construct (does not actually copy the data)
  constexpr DynArray(DynArray<T> const& arr) noexcept : DynArray(arr, arr.size()) {}

  template <int SIZE>
  constexpr DynArray(CopyableArray<T, SIZE>& arr, int size) noexcept
      : owned_(false), size_(size), data_(arr.data()) {
    assert(size <= arr.size());
  }

  template <int SIZE>
  explicit constexpr DynArray(CopyableArray<T, SIZE>& arr) noexcept : DynArray(arr, arr.size()) {}

  // Destructor
  ~DynArray() {
    if (owned_) delete[] data_;
  }

  constexpr void copyTo(DynArray<T>& that) const noexcept {
    assert(that.size() >= this->size());
    std::copy(this->begin(), this->end(), that.begin());
  }

  constexpr int size() const noexcept { return size_; }

  constexpr T* data() noexcept { return data_; }
  constexpr T const* data() const noexcept { return data_; }

  constexpr auto begin() noexcept { return iterator(data_); }
  constexpr auto end() noexcept { return iterator(data_ + size()); }

  constexpr auto begin() const noexcept { return const_iterator(data_); }
  constexpr auto end() const noexcept { return const_iterator(data_ + size()); }

  constexpr void fill(T const& x) noexcept { std::fill(data_, data_ + size(), x); }

  constexpr T& operator[](int const i) noexcept {
    assert(i < size());
    return data_[size_t(i)];
  }

  constexpr T const& operator[](int const i) const noexcept {
    assert(i < size());
    return data_[size_t(i)];
  }

  // copy-assignment operator deep-copies data without a new allocation (i.e. only if they are the
  // same size)
  constexpr DynArray& operator=(DynArray const& that) noexcept {
    assert(this->size() == that.size());
    if (this != &that) that.copyTo(*this);
    return *this;
  }

  constexpr DynArray& operator=(DynArray&& that) noexcept {
    if (this != &that) {
      this->owned_ = that.owned_;
      this->size_ = that.size_;
      this->data_ = that.data_;
      that.owned_ = false;
      that.size_ = 0;
      that.data_ = nullptr;
    }
    return *this;
  }

  constexpr void operator+=(DynArray const& that) noexcept {
    assert(this->size() == that.size());
    for (auto [out, in] : zip(*this, that)) out += in;
  }

  constexpr DynArray& operator*=(T x) noexcept {
    for (T& i : *this) i *= x;
    return *this;
  }

  constexpr DynArray& operator*=(DynArray& that) noexcept {
    assert(this->size() == that.size());
    for (auto [x, y] : zip(*this, that)) x *= y;
    return *this;
  }

  // precondition: sorted [array]
  // postcondition: deleted all elements closer than [threshold] to
  // their predecessor
  constexpr void uniquify(T threshold) noexcept {
    int j = 1;
    for (int i = 1; i < size(); i++)
      if ((*this)[i] - (*this)[i - 1] > threshold) (*this)[j++] = (*this)[i];
    size_ = j;
  }

 private:
  bool owned_ = true;
  int size_ = 0;
  T* data_ = nullptr;
};
}  // namespace grm