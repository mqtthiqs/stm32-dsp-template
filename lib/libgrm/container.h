#pragma once

#include <array>
#include <atomic>
#include <bitset>
#include <memory>
#include <utility>

#include "array.h"
#include "dynarray.h"

namespace grm {

template <typename T>
struct IOArray {
  DynArray<T> input;
  DynArray<T> output;

  constexpr IOArray(DynArray<T>& data, int in_size, int out_size) noexcept
      : IOArray(data, data, in_size, out_size) {}

  constexpr IOArray(DynArray<T>& in, DynArray<T>& out, int in_size, int out_size) noexcept
      : input(in, in_size), output(out, out_size) {}
};

template <typename T, int SIZE>
struct LockFreeQueue : Array<T, SIZE> {
  using Array<T, SIZE>::Array;

  // copies elem at the top of the queue
  constexpr bool push(T const& elem) noexcept {
    int pos = writePos_.load();
    int next = (pos + 1) % this->size();

    if (next == readPos_.load()) return false;

    (*this)[pos] = elem;
    writePos_.store(next);
    return true;
  }

  // copies the last element into elem
  constexpr bool pop(T& elem) noexcept {
    auto pos = readPos_.load();

    if (pos == writePos_.load()) return false;

    elem = (*this)[pos];
    readPos_.store((pos + 1) % this->size());
    return true;
  }

  constexpr int available() noexcept {
    int d = writePos_.load() - readPos_.load();
    return d >= 0 ? readPos_.load() + (int(this->size()) - writePos_.load())
                  : readPos_.load() - writePos_.load();
  }

  constexpr int used() {
    int d = writePos_.load() - readPos_.load();
    return d >= 0 ? d : writePos_.load() + (int(this->size()) - readPos_.load());
  }

  constexpr void reset() noexcept { readPos_ = writePos_ = 0; }

  constexpr int size() noexcept { return SIZE; }

 private:
  std::atomic<int> readPos_ = 0, writePos_ = 0;
};

template <typename T=uint64_t>
struct AtomicBitset {
  constexpr AtomicBitset(int size, bool defaultValue) noexcept
      : size_(size), data_(defaultValue ? 0xFFFFFFFFFFFFFFFF : 0x0) {
    assert(size <= 64);
  }

  bool operator[](int pos) const noexcept {
    assert(pos < size());
    T mask = T(1) << pos;
    return (data_ & mask) != 0;
  }

  void set(int pos, bool value) noexcept {
    assert(pos < size());
    T mask = T(1) << pos;
    if (value) data_ |= mask;
    else data_ &= ~mask;
  }

  void setAll(T value) noexcept { data_ = value; }

  constexpr int size() const noexcept { return size_; }

 private:
  int size_;
  std::atomic<T> data_;
  static_assert(std::atomic<T>::is_always_lock_free);
};

}  // namespace grm
