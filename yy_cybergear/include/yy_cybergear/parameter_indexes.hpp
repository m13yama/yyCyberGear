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

#ifndef YY_CYBERGEAR__PARAMETER_INDEXES_HPP_
#define YY_CYBERGEAR__PARAMETER_INDEXES_HPP_

#include <cstdint>

namespace yy_cybergear
{
// Parameter index constants used in read/write parameter requests.
// Names are based on the CyberGear protocol documentation.
namespace param
{
constexpr uint16_t RUN_MODE = 0x7005;      // Run mode (0: reset, 1: cali, 2: run)
constexpr uint16_t IQ_REFERENCE = 0x7006;  // Iq reference [A]

constexpr uint16_t SPEED_REFERENCE = 0x700A;  // Speed reference [rad/s]
constexpr uint16_t TORQUE_LIMIT = 0x700B;     // Torque limit [Nm]

constexpr uint16_t CURRENT_KP = 0x7010;           // Current loop Kp
constexpr uint16_t CURRENT_KI = 0x7011;           // Current loop Ki
constexpr uint16_t CURRENT_FILTER_GAIN = 0x7014;  // Current filter gain

constexpr uint16_t POSITION_REFERENCE = 0x7016;  // Position reference [rad]
constexpr uint16_t SPEED_LIMIT = 0x7017;         // Speed limit [rad/s]
constexpr uint16_t CURRENT_LIMIT = 0x7018;       // Current limit [A]

constexpr uint16_t MECHANICAL_POSITION = 0x7019;  // Mechanical position [rad]
constexpr uint16_t IQ_FILTER = 0x701A;            // Iq filtered [A]
constexpr uint16_t MECHANICAL_VELOCITY = 0x701B;  // Mechanical velocity [rad/s]
constexpr uint16_t BUS_VOLTAGE = 0x701C;          // Bus voltage [V]
constexpr uint16_t ROTATION_TURNS = 0x701D;       // Rotation turns (int16)

constexpr uint16_t POSITION_KP = 0x701E;  // Position loop Kp
constexpr uint16_t SPEED_KP = 0x701F;     // Speed loop Kp
constexpr uint16_t SPEED_KI = 0x7020;     // Speed loop Ki
}  // namespace param

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__PARAMETER_INDEXES_HPP_
