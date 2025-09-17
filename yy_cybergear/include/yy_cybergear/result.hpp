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

#ifndef YY_CYBERGEAR__RESULT_HPP_
#define YY_CYBERGEAR__RESULT_HPP_

#include <optional>
#include <utility>

#include "yy_cybergear/error_code.hpp"

namespace yy_cybergear
{

// Result<T, E>: optional value or error, with default error type ErrorCode.
template <typename T, typename E = ErrorCode>
class Result
{
public:
  // Factory methods
  static Result success(T value) { return Result(std::move(value)); }
  static Result failure(E error) { return Result(std::move(error)); }

  // State queries
  [[nodiscard]] bool ok() const { return value_.has_value(); }

  const std::optional<T> & value() const { return value_; }
  const std::optional<E> & error() const { return error_; }

private:
  std::optional<T> value_{};
  std::optional<E> error_{};

  explicit Result(T v) : value_(std::move(v)) {}
  explicit Result(E e) : error_(std::move(e)) {}
};

// Specialization for void result type
template <typename E>
class Result<void, E>
{
public:
  // Factory methods
  static Result success() { return Result(true); }
  static Result failure(E error) { return Result(false, std::move(error)); }

  // State queries
  [[nodiscard]] bool ok() const { return ok_; }

  const std::optional<E> & error() const { return error_; }

private:
  bool ok_{false};
  std::optional<E> error_{};

  explicit Result(bool ok) : ok_(ok) {}
  Result(bool ok, E e) : ok_(ok), error_(std::move(e)) {}
};

}  // namespace yy_cybergear

#endif  // YY_CYBERGEAR__RESULT_HPP_
