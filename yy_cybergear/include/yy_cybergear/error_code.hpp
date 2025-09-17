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

#ifndef YY_CYBERGEAR__ERROR_CODE_HPP_
#define YY_CYBERGEAR__ERROR_CODE_HPP_

#include <string>

namespace yy_cybergear
{

// Concise error categories suitable for Result<T, ErrorCode>.
enum class ErrorCode : int {
  None = 0,     // No error
  NotOpen,      // Device/socket not open
  Timeout,      // Timed out waiting for response
  Io,           // I/O error (send/recv failure, OS error)
  InvalidFrame  // Malformed or unexpected CAN frame
};

inline std::string to_string(ErrorCode ec)
{
  switch (ec) {
    case ErrorCode::None:
      return "None";
    case ErrorCode::NotOpen:
      return "NotOpen";
    case ErrorCode::Timeout:
      return "Timeout";
    case ErrorCode::Io:
      return "Io";
    case ErrorCode::InvalidFrame:
      return "InvalidFrame";
    default:
      return "Unknown";
  }
}

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__ERROR_CODE_HPP_
