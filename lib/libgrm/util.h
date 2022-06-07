#pragma once

// C++ utility

#include <algorithm>
#include <cstring>
#include <functional>
#include <tuple>
#include <cstdint>

namespace grm {

template <typename T>
using Pair = std::pair<T, T>;

template <typename T>
constexpr T max(T x, T y) {
  return x > y ? x : y;
}

constexpr bool is_power_of_2(int v) noexcept { return v && ((v & (v - 1)) == 0); }

constexpr int ipow(int a, int b) noexcept { return b == 0 ? 1 : a * ipow(a, b - 1); }
constexpr long lpow(long a, long b) noexcept { return b == 0 ? 1 : a * lpow(a, b - 1); }

constexpr float fpow2(int exp) noexcept {
  return exp > 0 ? 2.f * fpow2(exp - 1) : exp < 0 ? 0.5f * fpow2(exp + 1) : 1.f;
}

constexpr int ilog2(int n) {
  int res = 0;
  while ((n >>= 1) != 0) ++res;
  return res;
}

struct Nocopy {
  Nocopy(const Nocopy&) = delete;
  Nocopy& operator=(const Nocopy&) = delete;

 protected:
  constexpr Nocopy() = default;
  Nocopy(Nocopy&&) = default;
  Nocopy& operator=(Nocopy&&) = default;
  ~Nocopy() = default;
};

template <typename T, typename crtpType>
struct crtp {
  T& operator*() noexcept { return static_cast<T&>(*this); }
  T const& operator*() const noexcept { return static_cast<T const&>(*this); }

 private:
  crtp() = default;
  friend crtpType;
};

template <class... Args>
constexpr bool all_equal(Args const&... args) {
  if constexpr (sizeof...(Args) == 0) return true;
  else return [](auto const& a0, auto const&... rest) { return ((a0 == rest) && ...); }(args...);
}

// Observer pattern
template <typename... DATA>
struct Subject {
  void notify(DATA... args) { observer_(args...); }
  explicit Subject(std::function<void(DATA...)> observer) : observer_(observer) {}

 private:
  std::function<void(DATA...)> observer_;
};

// Overload
template <typename... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// from https://graphics.stanford.edu/~seander/bithacks.html
inline uint8_t reverse_8bits(uint8_t b) {
  // 3 operations, 64-bit multiply and modulus division
  return uint8_t((b * 0x0202020202ULL & 0x010884422010ULL) % 1023);
  // alternative (4 operations, 64-bit multiply, no division)
  // return ((b * 0x80200802ULL) & 0x0884422110ULL) * 0x0101010101ULL >> 32
}

template <class To, class From>
typename std::enable_if<(sizeof(To) == sizeof(From)) && std::is_trivially_copyable<From>::value &&
                            std::is_trivial<To>::value,
                        // this implementation requires that To is trivially default constructible
                        To>::type
// constexpr support needs compiler magic
bit_cast(const From& src) noexcept {
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace grm
