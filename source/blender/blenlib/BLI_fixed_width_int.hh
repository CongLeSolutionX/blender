/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <immintrin.h>
#include <iostream>

#include <gmpxx.h>

#include "BLI_string_ref.hh"
#include "BLI_utildefines.h"

namespace blender::fixed_width_int {

template<typename T, int S> struct UIntF {
  static_assert(std::is_unsigned_v<T>);
  static_assert(S >= 1);

  std::array<T, S> v;

  UIntF() = default;

  explicit UIntF(const uint64_t value)
  {
    constexpr int Count = std::min(S, int(sizeof(decltype(value)) / sizeof(T)));
    constexpr int BitsPerT = 8 * sizeof(T);

    for (int i = 0; i < Count; i++) {
      this->v[i] = T(value >> (BitsPerT * i));
    }
    for (int i = Count; i < S; i++) {
      this->v[i] = 0;
    }
  }

  explicit UIntF(const StringRefNull str, const int base = 10)
  {
    this->set_from_str(str, base);
  }

  explicit operator uint64_t() const
  {
    constexpr int Count = std::min(S, int(sizeof(uint64_t) / sizeof(T)));
    constexpr int BitsPerT = 8 * sizeof(T);

    uint64_t result = 0;
    for (int i = 0; i < Count; i++) {
      result |= uint64_t(this->v[i]) << (BitsPerT * i);
    }
    return result;
  }

  void set_from_str(const StringRefNull str, const int base = 10)
  {
    mpz_t x;
    mpz_init(x);
    mpz_set_str(x, str.c_str(), base);
    for (int i = 0; i < S; i++) {
      static_assert(sizeof(T) <= sizeof(decltype(mpz_get_ui(x))));
      this->v[i] = T(mpz_get_ui(x));
      mpz_div_2exp(x, x, 8 * sizeof(T));
    }
    mpz_clear(x);
  }

  void print(const int base = 10, const char *end = "\n") const
  {
    mpz_t x;
    mpz_init(x);
    for (int i = S - 1; i >= 0; i--) {
      static_assert(sizeof(T) <= sizeof(decltype(mpz_get_ui(x))));
      mpz_mul_2exp(x, x, 8 * sizeof(T));
      mpz_add_ui(x, x, this->v[i]);
    }
    mpz_out_str(stdout, base, x);
    mpz_clear(x);
    std::cout << end;
  }

  friend std::ostream &operator<<(std::ostream &stream, const UIntF &a)
  {
    /* Might not actually print to the stream currently. */
    a.print(10, "");
    return stream;
  }
};

template<typename T, int S> struct IntF {
  static_assert(std::is_unsigned_v<T>);
  static_assert(S >= 1);

  std::array<T, S> v;

  IntF() = default;

  explicit IntF(const int64_t value)
  {
    constexpr int Count = std::min(S, int(sizeof(decltype(value)) / sizeof(T)));
    constexpr int BitsPerT = 8 * sizeof(T);

    for (int i = 0; i < Count; i++) {
      this->v[i] = T(value >> (BitsPerT * i));
    }
    const T sign_extend_fill = value < 0 ? T(-1) : T(0);
    for (int i = Count; i < S; i++) {
      this->v[i] = sign_extend_fill;
    }
  }

  explicit IntF(const UIntF<T, S> &value) : v(value.v) {}

  explicit IntF(const StringRefNull str, const int base = 10)
  {
    this->set_from_str(str, base);
  }

  explicit operator int64_t() const
  {
    return int64_t(uint64_t(UIntF<T, S>(*this)));
  }

  void set_from_str(const StringRefNull str, const int base = 10)
  {
    if (str[0] == '-') {
      const UIntF<T, S> unsigned_value(str.c_str() + 1, base);
      this->v = unsigned_value.v;
      *this = -*this;
    }
    else {
      const UIntF<T, S> unsigned_value(str.c_str(), base);
      this->v = unsigned_value.v;
    }
  }

  explicit operator UIntF<T, S>() const
  {
    UIntF<T, S> result;
    result.v = this->v;
    return result;
  }

  void print(const int base = 10, const char *end = "\n") const
  {
    if (is_negative(*this)) {
      std::cout << "-";
      (-*this).print(base, end);
    }
    else {
      UIntF<T, S>(*this).print(base, end);
    }
  }

  friend std::ostream &operator<<(std::ostream &stream, const IntF &a)
  {
    /* Might not actually print to the stream currently. */
    a.print(10, "");
    return stream;
  }
};

template<typename T>
using double_uint_type = std::conditional_t<
    std::is_same_v<T, uint8_t>,
    uint16_t,
    std::conditional_t<
        std::is_same_v<T, uint16_t>,
        uint32_t,
        std::conditional_t<std::is_same_v<T, uint32_t>,
                           uint64_t,
                           std::conditional_t<std::is_same_v<T, uint64_t>, __uint128_t, void>>>>;

using UInt64_8 = UIntF<uint8_t, 8>;
using UInt64_16 = UIntF<uint16_t, 4>;
using UInt64_32 = UIntF<uint32_t, 2>;

using Int64_8 = IntF<uint8_t, 8>;
using Int64_16 = IntF<uint16_t, 4>;
using Int64_32 = IntF<uint32_t, 2>;

using UInt128_8 = UIntF<uint8_t, 16>;
using UInt128_16 = UIntF<uint16_t, 8>;
using UInt128_32 = UIntF<uint32_t, 4>;
using UInt128_64 = UIntF<uint64_t, 2>;

using UInt256_8 = UIntF<uint8_t, 32>;
using UInt256_16 = UIntF<uint16_t, 16>;
using UInt256_32 = UIntF<uint32_t, 8>;
using UInt256_64 = UIntF<uint64_t, 4>;

using Int128_8 = IntF<uint8_t, 16>;
using Int128_16 = IntF<uint16_t, 8>;
using Int128_32 = IntF<uint32_t, 4>;
using Int128_64 = IntF<uint64_t, 2>;

using Int256_8 = IntF<uint8_t, 32>;
using Int256_16 = IntF<uint16_t, 16>;
using Int256_32 = IntF<uint32_t, 8>;
using Int256_64 = IntF<uint64_t, 4>;

using UInt128 = UInt128_64;
using UInt256 = UInt256_64;

using Int128 = Int128_64;
using Int256 = Int256_64;

template<typename T, typename T2, int S>
inline void generic_add(T *__restrict dst, const T *a, const T *b)
{
  constexpr int shift = 8 * sizeof(T);
  T2 carry = 0;
  for (int i = 0; i < S; i++) {
    const T2 ai = T2(a[i]);
    const T2 bi = T2(b[i]);
    const T2 ri = ai + bi + carry;
    dst[i] = T(ri);
    carry = ri >> shift;
  }
}

template<typename T, typename T2, int S>
inline void generic_sub(T *__restrict dst, const T *a, const T *b)
{
  T2 carry = 0;
  for (int i = 0; i < S; i++) {
    const T2 ai = T2(a[i]);
    const T2 bi = T2(b[i]);
    const T2 ri = ai - bi - carry;
    dst[i] = T(ri);
    carry = ri > ai;
  }
}

template<typename T, typename T2, int S>
inline void generic_unsigned_mul(T *__restrict dst, const T *a, const T *b)
{
  constexpr int shift = 8 * sizeof(T);

  T2 r[S] = {};

  for (int i = 0; i < S; i++) {
    const T2 bi = T2(b[i]);
    T2 carry = 0;
    for (int j = 0; j < S - i; j++) {
      const T2 rji = T2(a[j]) * bi + carry;
      carry = rji >> shift;
      r[i + j] += T2(T(rji));
    }
  }

  T2 carry = 0;
  for (int i = 0; i < S; i++) {
    const T2 ri = r[i] + carry;
    carry = ri >> shift;
    dst[i] = T(ri);
  }
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline UIntF<T, Size> operator+(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_add<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

inline void operator+=(UInt256_64 &a, const UInt256_64 &b)
{
  asm("mov (%1), %%rax\n\t"
      "add %%rax, (%0)\n\t"
      "mov 8(%1), %%rax\n\t"
      "adc %%rax, 8(%0)\n\t"
      "mov 16(%1), %%rax\n\t"
      "adc %%rax, 16(%0)\n\t"
      "mov 24(%1), %%rax\n\t"
      "adc %%rax, 24(%0)\n\t" ::"r"(&a),
      "r"(&b)
      : "%rax");
}

inline UInt256_64 operator+(const UInt256_64 &a, const UInt256_64 &b)
{
  UInt256_64 r = a;
  r += b;
  return r;
}

// inline UInt256_64 operator+(const UInt256_64 &a, const UInt256_64 &b)
// {
//   UInt256_64 r;

//   const auto *a_ptr = reinterpret_cast<const unsigned long long *>(a.v.data());
//   const auto *b_ptr = reinterpret_cast<const unsigned long long *>(b.v.data());
//   auto *r_ptr = reinterpret_cast<unsigned long long *>(r.v.data());

//   const uint8_t carry0 = _addcarry_u64(0, a_ptr[0], b_ptr[0], r_ptr + 0);
//   const uint8_t carry1 = _addcarry_u64(carry0, a_ptr[1], b_ptr[1], r_ptr + 1);
//   const uint8_t carry2 = _addcarry_u64(carry1, a_ptr[2], b_ptr[2], r_ptr + 2);
//   r.v[3] = a_ptr[3] + b_ptr[3] + carry2;
//   return r;
// }

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline IntF<T, Size> operator+(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  IntF<T, Size> result;
  generic_add<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size>
inline UIntF<T, Size> operator-(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_sub<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size>
inline IntF<T, Size> operator-(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  IntF<T, Size> result;
  generic_sub<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline UIntF<T, Size> operator*(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  UIntF<T, Size> result;
  generic_unsigned_mul<T, double_uint_type<T>, Size>(result.v.data(), a.v.data(), b.v.data());
  return result;
}

template<typename T, int Size> bool is_negative(const IntF<T, Size> &a)
{
  return (a.v[Size - 1] & (T(1) << (sizeof(T) * 8 - 1))) != 0;
}

template<typename T, int Size, BLI_ENABLE_IF((!std::is_void_v<double_uint_type<T>>))>
inline IntF<T, Size> operator*(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  using UIntF = UIntF<T, Size>;
  using IntF = IntF<T, Size>;

  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a && is_negative_b) {
    return IntF(UIntF(-a) * UIntF(-b));
  }
  if (is_negative_a) {
    return -IntF(UIntF(-a) * UIntF(b));
  }
  if (is_negative_b) {
    return -IntF(UIntF(a) * UIntF(-b));
  }
  return IntF(UIntF(a) * UIntF(b));
}

template<typename T, int Size> inline IntF<T, Size> operator-(const IntF<T, Size> &a)
{
  IntF<T, Size> result;
  for (int i = 0; i < Size; i++) {
    result.v[i] = ~a.v[i];
  }
  return result + IntF<T, Size>(1);
}

template<typename T, int Size> inline void operator+=(UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  a = a + b;
}

template<typename T, int Size> inline void operator+=(IntF<T, Size> &a, const IntF<T, Size> &b)
{
  a = a + b;
}

template<typename T, int Size> inline void operator-=(UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  a = a - b;
}

template<typename T, int Size> inline void operator-=(IntF<T, Size> &a, const IntF<T, Size> &b)
{
  a = a - b;
}

template<typename T, int Size> inline void operator*=(UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  a = a * b;
}

template<typename T, int Size> inline void operator*=(IntF<T, Size> &a, const IntF<T, Size> &b)
{
  a = a * b;
}

template<typename T, int Size>
inline bool operator==(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  return a.v == b.v;
}

template<typename T, int Size>
inline bool operator==(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return a.v == b.v;
}

template<typename T, int Size>
inline bool operator!=(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  return a.v != b.v;
}

template<typename T, int Size>
inline bool operator!=(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return a.v != b.v;
}

template<typename T, size_t Size>
inline int compare_reversed_order(const std::array<T, Size> &a, const std::array<T, Size> &b)
{
  for (int i = Size - 1; i >= 0; i--) {
    if (a[i] < b[i]) {
      return -1;
    }
    if (a[i] > b[i]) {
      return 1;
    }
  }
  return 0;
}

template<typename T, int Size>
inline bool operator<(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a == is_negative_b) {
    return compare_reversed_order(a.v, b.v) < 0;
  }
  return is_negative_a;
}

template<typename T, int Size>
inline bool operator<=(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a == is_negative_b) {
    return compare_reversed_order(a.v, b.v) <= 0;
  }
  return is_negative_a;
}

template<typename T, int Size>
inline bool operator>(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a == is_negative_b) {
    return compare_reversed_order(a.v, b.v) > 0;
  }
  return is_negative_b;
}

template<typename T, int Size>
inline bool operator>=(const IntF<T, Size> &a, const IntF<T, Size> &b)
{
  const bool is_negative_a = is_negative(a);
  const bool is_negative_b = is_negative(b);
  if (is_negative_a == is_negative_b) {
    return compare_reversed_order(a.v, b.v) >= 0;
  }
  return is_negative_b;
}

template<typename T, int Size>
inline bool operator<(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return compare_reversed_order(a.v, b.v) < 0;
}

template<typename T, int Size>
inline bool operator<=(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return compare_reversed_order(a.v, b.v) <= 0;
}

template<typename T, int Size>
inline bool operator>(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return compare_reversed_order(a.v, b.v) > 0;
}

template<typename T, int Size>
inline bool operator>=(const UIntF<T, Size> &a, const UIntF<T, Size> &b)
{
  return compare_reversed_order(a.v, b.v) >= 0;
}

}  // namespace blender::fixed_width_int
