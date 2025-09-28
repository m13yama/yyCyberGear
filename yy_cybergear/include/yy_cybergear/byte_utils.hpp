// Copyright 2025 Yuki Yamamoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YY_CYBERGEAR__BYTE_UTILS_HPP_
#define YY_CYBERGEAR__BYTE_UTILS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace yy_cybergear
{
namespace byte_util
{

// -------------------------------------------------------------
// Basic utility
// -------------------------------------------------------------

/**
 * @brief Clamp a floating-point value within the specified range.
 *
 * @param v  Input value.
 * @param lo Lower bound.
 * @param hi Upper bound.
 * @return Clamped value within [lo, hi].
 */
inline constexpr float clipf(float v, float lo, float hi) { return std::clamp(v, lo, hi); }

// -------------------------------------------------------------
// Float <-> Unsigned 16-bit conversion
// -------------------------------------------------------------

/**
 * @brief Convert a float value to an unsigned 16-bit integer using linear mapping.
 *
 * The function maps the floating-point range [xmin, xmax] to the integer range [0, 65535].
 * Any input outside the range is clamped.
 *
 * @param x     Input float value.
 * @param xmin  Minimum float range.
 * @param xmax  Maximum float range.
 * @return Unsigned 16-bit integer representing the scaled value.
 */
inline constexpr uint16_t f2u16(float x, float xmin, float xmax)
{
  const float v = std::clamp(x, xmin, xmax);
  const float n = (v - xmin) / (xmax - xmin);
  const float q = std::round(n * 65535.0f);
  return static_cast<uint16_t>(std::clamp(q, 0.0f, 65535.0f));
}

/**
 * @brief Convert an unsigned 16-bit integer back to a float using linear mapping.
 *
 * @param u     Input 16-bit unsigned integer.
 * @param xmin  Minimum float range.
 * @param xmax  Maximum float range.
 * @return Float value mapped from the 16-bit integer.
 */
inline constexpr float u16toF(uint16_t u, float xmin, float xmax)
{
  const float n = static_cast<float>(u) / 65535.0f;
  return xmin + n * (xmax - xmin);
}

// -------------------------------------------------------------
// Bit-cast (type-safe memory copy)
// -------------------------------------------------------------

/**
 * @brief Reinterpret the bit pattern of one type as another (type-safe).
 *
 * Equivalent to `std::bit_cast` (C++20), implemented manually for compatibility.
 *
 * @tparam To   Target type.
 * @tparam From Source type.
 * @param src   Source object.
 * @return Object of type @p To with the same bit representation as @p src.
 */
template <typename To, typename From>
inline constexpr To bit_cast(const From & src) noexcept
{
  static_assert(sizeof(To) == sizeof(From), "bit_cast size mismatch");
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

// -------------------------------------------------------------
// 16-bit endian conversions
// -------------------------------------------------------------

/**
 * @brief Pack a 16-bit unsigned integer into big-endian byte order.
 *
 * @param p Destination buffer (must be at least 2 bytes).
 * @param v Input value.
 */
inline constexpr void packBE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFFu);
  p[1] = static_cast<uint8_t>(v & 0xFFu);
}

/**
 * @brief Read a 16-bit unsigned integer from a big-endian byte array.
 *
 * @param p Source buffer (must contain at least 2 bytes).
 * @return 16-bit unsigned integer.
 */
inline constexpr uint16_t readBE16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

/**
 * @brief Pack a 16-bit unsigned integer into little-endian byte order.
 *
 * @param p Destination buffer (must be at least 2 bytes).
 * @param v Input value.
 */
inline constexpr void packLE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

/**
 * @brief Read a 16-bit unsigned integer from a little-endian byte array.
 *
 * @param p Source buffer (must contain at least 2 bytes).
 * @return 16-bit unsigned integer.
 */
inline constexpr uint16_t readLE16(const uint8_t * p)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(p[0]) << 0) | (static_cast<uint16_t>(p[1]) << 8));
}

/**
 * @brief Read a 16-bit signed integer from a little-endian byte array.
 *
 * @param p Source buffer (must contain at least 2 bytes).
 * @return 16-bit signed integer.
 */
inline constexpr int16_t readLE16s(const uint8_t * p) { return static_cast<int16_t>(readLE16(p)); }

// -------------------------------------------------------------
// 32-bit endian conversions
// -------------------------------------------------------------

/**
 * @brief Pack a 32-bit unsigned integer into a little-endian 4-byte array.
 *
 * @param v Input value.
 * @return std::array<uint8_t, 4> representing the little-endian encoding.
 */
inline constexpr std::array<uint8_t, 4> packLE32(uint32_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu),
    static_cast<uint8_t>((v >> 16) & 0xFFu), static_cast<uint8_t>((v >> 24) & 0xFFu)};
}

/**
 * @brief Read a 32-bit unsigned integer from a little-endian byte array.
 *
 * @param p Source buffer (must contain at least 4 bytes).
 * @return 32-bit unsigned integer.
 */
inline constexpr uint32_t readLE32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// -------------------------------------------------------------
// Float encoding / decoding (IEEE754 32-bit)
// -------------------------------------------------------------

/**
 * @brief Pack a 32-bit IEEE754 float into little-endian byte order.
 *
 * @param v Input float value.
 * @return std::array<uint8_t, 4> representing the float in little-endian format.
 */
inline constexpr std::array<uint8_t, 4> packLEf(float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
  const uint32_t u = bit_cast<uint32_t>(v);
  return packLE32(u);
}

/**
 * @brief Read a 32-bit IEEE754 float from a little-endian byte array.
 *
 * @param p Source buffer (must contain at least 4 bytes).
 * @return Float value decoded from the buffer.
 */
inline constexpr float readLEf(const uint8_t * p)
{
  const uint32_t u = readLE32(p);
  return bit_cast<float>(u);
}

// -------------------------------------------------------------
// Special packing: 16-bit values padded to 4-byte arrays
// -------------------------------------------------------------

/**
 * @brief Pack a 16-bit unsigned integer into a 4-byte little-endian array.
 *
 * The upper two bytes are zero-padded.
 *
 * @param v Input 16-bit unsigned integer.
 * @return 4-byte array containing the encoded value.
 */
inline constexpr std::array<uint8_t, 4> packLE16u(uint16_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu),
    static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00)};
}

/**
 * @brief Pack a 16-bit signed integer into a 4-byte little-endian array.
 *
 * The upper two bytes are zero-padded.
 *
 * @param v Input 16-bit signed integer.
 * @return 4-byte array containing the encoded value.
 */
inline constexpr std::array<uint8_t, 4> packLE16s(int16_t v)
{
  const uint16_t u = static_cast<uint16_t>(v);
  return {
    static_cast<uint8_t>(u & 0xFFu), static_cast<uint8_t>((u >> 8) & 0xFFu),
    static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00)};
}

}  // namespace byte_util
}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__BYTE_UTILS_HPP_