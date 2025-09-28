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

inline constexpr float clipf(float v, float lo, float hi) { return std::clamp(v, lo, hi); }

inline constexpr uint16_t f2u16(float x, float xmin, float xmax)
{
  const float v = std::clamp(x, xmin, xmax);
  const float n = (v - xmin) / (xmax - xmin);
  const float q = std::round(n * 65535.0f);
  return static_cast<uint16_t>(std::clamp(q, 0.0f, 65535.0f));
}

inline constexpr float u16toF(uint16_t u, float xmin, float xmax)
{
  const float n = static_cast<float>(u) / 65535.0f;
  return xmin + n * (xmax - xmin);
}

inline constexpr void packBE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFFu);
  p[1] = static_cast<uint8_t>(v & 0xFFu);
}

inline constexpr uint16_t readBE16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

inline constexpr void packLE16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline constexpr uint16_t readLE16(const uint8_t * p)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(p[0]) << 0) | (static_cast<uint16_t>(p[1]) << 8));
}

inline constexpr uint32_t readLE32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline constexpr std::array<uint8_t, 4> packLE32(uint32_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu),
    static_cast<uint8_t>((v >> 16) & 0xFFu), static_cast<uint8_t>((v >> 24) & 0xFFu)};
}

template <typename To, typename From>
inline constexpr To bit_cast(const From & src) noexcept
{
  static_assert(sizeof(To) == sizeof(From), "bit_cast size mismatch");
  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

inline constexpr std::array<uint8_t, 4> packLEf(float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit IEEE754");
  const uint32_t u = bit_cast<uint32_t>(v);
  return packLE32(u);
}

inline constexpr std::array<uint8_t, 4> packLE16u(uint16_t v)
{
  return {
    static_cast<uint8_t>(v & 0xFFu), static_cast<uint8_t>((v >> 8) & 0xFFu),
    static_cast<uint8_t>(0x00), static_cast<uint8_t>(0x00)};
}

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