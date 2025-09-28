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

#pragma once

#include <string>

#include "yy_cybergear/cybergear.hpp"

namespace yy_cybergear
{
namespace logging
{
std::string formatStatusLine(const CyberGear & cg, double t_sec);

std::string formatParamsSummary(const CyberGear & cg);

std::string runModeToString(RunMode mode);
std::string runModeToString(uint32_t mode);
std::string statusModeToString(Status::StatusMode status_mode);
std::string statusModeToString(uint8_t status_mode);
std::vector<std::string> faultBitsToString(uint8_t fault_bits);
}  // namespace logging
}  // namespace yy_cybergear
