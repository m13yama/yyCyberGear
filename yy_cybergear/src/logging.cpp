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

#include "yy_cybergear/logging.hpp"

#include <iomanip>
#include <ostream>
#include <sstream>

namespace yy_cybergear
{
namespace logging
{

std::string format_status_line(const CyberGear & cg, double t_sec)
{
  const auto s = cg.getStatus();
  std::ostringstream oss;
  oss.setf(std::ios::fixed, std::ios::floatfield);
  oss << std::setprecision(3) << " t=" << t_sec << "s"
      << " ang=" << s.angle_rad << "rad"
      << " vel=" << s.vel_rad_s << "rad/s"
      << " tau=" << s.torque_Nm << "Nm"
      << " T=" << s.temperature_c << "C"
      << " mode=" << static_cast<unsigned>(s.mode) << " faults=0x" << std::uppercase << std::hex
      << std::setw(2) << std::setfill('0') << static_cast<unsigned>(s.fault_bits & 0xFFu)
      << std::dec << " mid=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<unsigned>(s.motor_can_id) << std::dec;
  return oss.str();
}

std::string format_params_summary(const CyberGear & cg)
{
  std::ostringstream oss;
  const auto uid = cg.uid();
  oss.setf(std::ios::fixed, std::ios::floatfield);
  oss << std::setprecision(3);
  oss << "  motor_id=0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<unsigned>(cg.motor_id()) << std::dec << "\n";

  oss << "    uid              = ";
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<unsigned>(uid[0]);
  for (int i = 1; i < 8; ++i) {
    oss << ":" << std::setw(2) << std::setfill('0') << static_cast<unsigned>(uid[i]);
  }
  oss << std::dec << "\n";

  oss << "    speed_limit      = " << cg.speed_limit() << " rad/s\n"
      << "    current_limit    = " << cg.current_limit() << " A\n"
      << "    torque_limit     = " << cg.torque_limit() << " Nm\n"
      << "    current_kp       = " << cg.current_kp() << "\n"
      << "    current_ki       = " << cg.current_ki() << "\n"
      << "    current_filter   = " << cg.current_filter_gain() << "\n"
      << "    position_kp      = " << cg.position_kp() << "\n"
      << "    speed_kp         = " << cg.speed_kp() << "\n"
      << "    speed_ki         = " << cg.speed_ki() << "\n";

  return oss.str();
}

}  // namespace logging
}  // namespace yy_cybergear
