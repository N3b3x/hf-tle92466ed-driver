/**
 * @file tle92466ed_expected.hpp
 * @brief C++20-compatible expected<T,E> polyfill for TLE92466ED driver
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 *
 * @details
 * Provides `tle::expected<T,E>` and `tle::unexpected<E>` types that:
 *  - On C++23 (or later): alias directly to `std::expected` / `std::unexpected`
 *  - On C++20: supply a lightweight, self-contained implementation with the
 *    same API surface used by this driver (no exceptions, no allocations).
 *
 * This header is self-contained — it has no dependencies on other HardFOC
 * libraries and can be used by the driver alone.
 *
 * @note This polyfill only implements the subset of std::expected that the
 *       TLE92466ED driver actually uses. It is NOT a full std::expected
 *       replacement.
 */
#pragma once

// ── Feature detection ────────────────────────────────────────────────────────
#if defined(__has_include)
#if __has_include(<version>)
#include <version>
#endif
#endif

// ── C++23 path: alias to standard library ────────────────────────────────────
#if defined(__cpp_lib_expected) && __cpp_lib_expected >= 202211L

#include <expected>

namespace tle {

template <typename T, typename E>
using expected = std::expected<T, E>;

template <typename E>
using unexpected = std::unexpected<E>;

} // namespace tle

// ── C++20 fallback: lightweight polyfill ─────────────────────────────────────
#else

#include <cstdint>
#include <type_traits>
#include <utility>

namespace tle {

/**
 * @brief Wrapper for an error value (mirrors std::unexpected)
 * @tparam E Error type
 */
template <typename E> class unexpected {
  E error_;

public:
  constexpr explicit unexpected(const E &e) noexcept : error_(e) {}
  constexpr explicit unexpected(E &&e) noexcept : error_(std::move(e)) {}

  [[nodiscard]] constexpr const E &error() const &noexcept { return error_; }
  [[nodiscard]] constexpr E &error() &noexcept { return error_; }
  [[nodiscard]] constexpr E &&error() &&noexcept { return std::move(error_); }
};

/// Deduction guide
template <typename E> unexpected(E) -> unexpected<E>;

// Forward declaration
template <typename T, typename E> class expected;

/**
 * @brief Expected type — holds either a value of type T or an error of type E
 * @tparam T Value type
 * @tparam E Error type
 *
 * Stores both members (like tmc51x0::Result<T>). This is acceptable because
 * all types used in this driver are small and trivially copyable.
 */
template <typename T, typename E> class expected {
  T value_;
  E error_;
  bool has_value_;

public:
  // ── Value constructors ────────────────────────────────────────────────────
  constexpr expected(const T &v) noexcept // NOLINT(google-explicit-constructor)
      : value_(v), error_{}, has_value_(true) {}

  constexpr expected(T &&v) noexcept // NOLINT(google-explicit-constructor)
      : value_(std::move(v)), error_{}, has_value_(true) {}

  // ── Error constructors (from unexpected) ──────────────────────────────────
  constexpr expected( // NOLINT(google-explicit-constructor)
      const unexpected<E> &u) noexcept
      : value_{}, error_(u.error()), has_value_(false) {}

  constexpr expected( // NOLINT(google-explicit-constructor)
      unexpected<E> &&u) noexcept
      : value_{}, error_(std::move(u).error()), has_value_(false) {}

  // ── Copy / Move ───────────────────────────────────────────────────────────
  constexpr expected(const expected &) noexcept = default;
  constexpr expected(expected &&) noexcept = default;
  constexpr expected &operator=(const expected &) noexcept = default;
  constexpr expected &operator=(expected &&) noexcept = default;
  ~expected() = default;

  // ── Observers ─────────────────────────────────────────────────────────────
  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return has_value_;
  }
  [[nodiscard]] constexpr bool has_value() const noexcept {
    return has_value_;
  }

  [[nodiscard]] constexpr T &value() &noexcept { return value_; }
  [[nodiscard]] constexpr const T &value() const &noexcept { return value_; }
  [[nodiscard]] constexpr T &&value() &&noexcept {
    return std::move(value_);
  }

  [[nodiscard]] constexpr E &error() &noexcept { return error_; }
  [[nodiscard]] constexpr const E &error() const &noexcept { return error_; }
  [[nodiscard]] constexpr E &&error() &&noexcept {
    return std::move(error_);
  }

  [[nodiscard]] constexpr T &operator*() &noexcept { return value_; }
  [[nodiscard]] constexpr const T &operator*() const &noexcept {
    return value_;
  }
  [[nodiscard]] constexpr T &&operator*() &&noexcept {
    return std::move(value_);
  }

  [[nodiscard]] constexpr T *operator->() noexcept { return &value_; }
  [[nodiscard]] constexpr const T *operator->() const noexcept {
    return &value_;
  }

  // ── Monadic helpers ───────────────────────────────────────────────────────
  template <typename U>
  [[nodiscard]] constexpr T value_or(U &&default_val) const &noexcept {
    return has_value_ ? value_ : static_cast<T>(std::forward<U>(default_val));
  }

  // ── Converting constructor (e.g. expected<uint32_t,E> → expected<uint16_t,E>)
  template <typename U,
            typename = std::enable_if_t<!std::is_same_v<U, T> &&
                                         std::is_convertible_v<U, T>>>
  constexpr expected( // NOLINT(google-explicit-constructor)
      const expected<U, E> &other) noexcept
      : value_(other.has_value() ? static_cast<T>(*other) : T{}),
        error_(other.has_value() ? E{} : other.error()),
        has_value_(other.has_value()) {}
};

/**
 * @brief Specialization for void value type (expected<void, E>)
 * @tparam E Error type
 *
 * Used for operations that return success/failure with no associated value.
 */
template <typename E> class expected<void, E> {
  E error_;
  bool has_value_;

public:
  // ── Success constructor ───────────────────────────────────────────────────
  constexpr expected() noexcept : error_{}, has_value_(true) {}

  // ── Error constructors (from unexpected) ──────────────────────────────────
  constexpr expected( // NOLINT(google-explicit-constructor)
      const unexpected<E> &u) noexcept
      : error_(u.error()), has_value_(false) {}

  constexpr expected( // NOLINT(google-explicit-constructor)
      unexpected<E> &&u) noexcept
      : error_(std::move(u).error()), has_value_(false) {}

  // ── Copy / Move ───────────────────────────────────────────────────────────
  constexpr expected(const expected &) noexcept = default;
  constexpr expected(expected &&) noexcept = default;
  constexpr expected &operator=(const expected &) noexcept = default;
  constexpr expected &operator=(expected &&) noexcept = default;
  ~expected() = default;

  // ── Observers ─────────────────────────────────────────────────────────────
  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return has_value_;
  }
  [[nodiscard]] constexpr bool has_value() const noexcept {
    return has_value_;
  }

  [[nodiscard]] constexpr E &error() &noexcept { return error_; }
  [[nodiscard]] constexpr const E &error() const &noexcept { return error_; }
  [[nodiscard]] constexpr E &&error() &&noexcept {
    return std::move(error_);
  }
};

} // namespace tle

#endif // __cpp_lib_expected
