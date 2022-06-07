#pragma once

#include <cassert>
#include <tuple>

#include "util.h"

// Zip: a class for simultaneous iteration over structures

namespace grm {

namespace detail {
using std::begin, std::end;

template <typename Range>
struct range_traits {
  using iterator = decltype(begin(std::declval<Range>()));
  using value_type = typename std::iterator_traits<iterator>::value_type;
  using reference = typename std::iterator_traits<iterator>::reference;
};

template <typename... Its>
class zip_iterator {
 public:
  // technically lying
  using iterator_category =
      std::common_type_t<typename std::iterator_traits<Its>::iterator_category...>;
  using difference_type =
      std::common_type_t<typename std::iterator_traits<Its>::difference_type...>;
  using value_type = std::tuple<typename std::iterator_traits<Its>::value_type...>;
  using reference = std::tuple<typename std::iterator_traits<Its>::reference...>;
  using pointer = std::tuple<typename std::iterator_traits<Its>::pointer...>;

  constexpr zip_iterator() = default;
  constexpr explicit zip_iterator(Its... its) : base_its {its...} {}

  constexpr reference operator*() const {
    return std::apply([](auto const&... its) { return reference(*its...); }, base_its);
  }
  constexpr zip_iterator& operator++() {
    std::apply([](auto&... its) { (++its, ...); }, base_its);
    return *this;
  }
  constexpr zip_iterator const operator++(int) {
    return std::apply([](auto&... its) { return zip_iterator(its++...); }, base_its);
  }
  constexpr zip_iterator& operator--() {
    std::apply([](auto&... its) { (--its, ...); }, base_its);
    return *this;
  }
  constexpr zip_iterator const operator--(int) {
    return std::apply([](auto&... its) { return zip_iterator(its--...); }, base_its);
  }
  constexpr zip_iterator& operator+=(difference_type n) {
    std::apply([=](auto&... its) { ((its += n), ...); }, base_its);
    return *this;
  }
  constexpr zip_iterator& operator-=(difference_type n) {
    std::apply([=](auto&... its) { ((its -= n), ...); }, base_its);
    return *this;
  }
  friend constexpr zip_iterator operator+(const zip_iterator& it, difference_type n) {
    return std::apply([=](auto const&... its) { return zip_iterator(its + n...); }, it.base_its);
  }
  friend constexpr zip_iterator operator+(difference_type n, const zip_iterator& it) {
    return std::apply([=](auto const&... its) { return zip_iterator(n + its...); }, it.base_its);
  }
  friend constexpr zip_iterator operator-(const zip_iterator& it, difference_type n) {
    return std::apply([=](auto const&... its) { return zip_iterator(its - n...); }, it.base_its);
  }
  constexpr reference operator[](difference_type n) const {
    return std::apply([=](auto const&... its) { return reference(its[n]...); }, base_its);
  }

  // the following functions assume usual random access iterator semantics
  friend constexpr bool operator==(const zip_iterator& lhs, const zip_iterator& rhs) {
    return std::get<0>(lhs.base_its) == std::get<0>(rhs.base_its);
  }
  friend constexpr bool operator!=(const zip_iterator& lhs, const zip_iterator& rhs) {
    return !(lhs == rhs);
  }
  friend constexpr bool operator<(const zip_iterator& lhs, const zip_iterator& rhs) {
    return std::get<0>(lhs.base_its) < std::get<0>(rhs.base_its);
  }
  friend constexpr bool operator>(const zip_iterator& lhs, const zip_iterator& rhs) {
    return rhs < lhs;
  }
  friend constexpr bool operator<=(const zip_iterator& lhs, const zip_iterator& rhs) {
    return !(rhs < lhs);
  }
  friend constexpr bool operator>=(const zip_iterator& lhs, const zip_iterator& rhs) {
    return !(lhs < rhs);
  }

 private:
  std::tuple<Its...> base_its;
};
}  // namespace detail

template <typename... Ranges>
class zip {
  static_assert(sizeof...(Ranges) > 0, "Cannot zip zero ranges");

 public:
  using iterator = detail::zip_iterator<typename detail::range_traits<Ranges>::iterator...>;
  using value_type = typename iterator::value_type;
  using reference = typename iterator::reference;

  constexpr explicit zip(Ranges&&... rs) : ranges {std::forward<Ranges>(rs)...} {
    // assert(all_equal(int(std::size(rs))...));
  }

  constexpr iterator begin() {
    return std::apply([](auto&... rs) { return iterator(std::begin(rs)...); }, ranges);
  }

  constexpr iterator end() {
    return std::apply([](auto&... rs) { return iterator(std::end(rs)...); }, ranges);
  }

 private:
  std::tuple<Ranges...> ranges;
};

// by default, rvalue arguments are moved to prevent dangling references
template <typename... Ranges>
explicit zip(Ranges&&...) -> zip<Ranges...>;

// Count: a fake iterator class that updates a counter

template <typename T>
class count {
  T first = T(0);
  T last;
  T step = T(1);

 public:
  using value_type = T;

  class iterator {
    T value;
    T step;

   public:
    // lying again
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::intmax_t;
    using value_type = T;
    using reference = T;
    using pointer = T*;

    constexpr iterator() = default;
    // sentinel
    constexpr explicit iterator(T v) : value(v) {}
    constexpr explicit iterator(T v, T s) : value(v), step(s) {}

    constexpr reference operator*() const { return value; }
    constexpr iterator& operator++() {
      value += step;
      return *this;
    }

    constexpr iterator const operator++(int) {
      auto copy {*this};
      ++*this;
      return copy;
    }

    friend constexpr bool operator==(const iterator& lhs, const iterator& rhs) {
      return lhs.value * lhs.step >= rhs.value * lhs.step;
    }

    friend constexpr bool operator!=(const iterator& lhs, const iterator& rhs) {
      return !(lhs == rhs);
    }
  };

  constexpr explicit count(T e) : last(e) {}
  constexpr explicit count(T b, T e, T s = T(1)) : first(b), last(e), step(s) {}
  constexpr iterator begin() const { return iterator {first, step}; }
  constexpr iterator end() const { return iterator {last}; }
  constexpr T size() const { return (last - first) / step; }
};

// again, rvalue arguments are copied by default
template <typename Sequence>
auto enumerate(Sequence&& seq) {
  using std::begin, std::end;
  return zip(count(int(end(seq) - begin(seq))), std::forward<Sequence>(seq));
}

}  // namespace grm
