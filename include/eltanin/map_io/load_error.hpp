// Copyright 2026 RyuYamamoto.
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
// limitations under the License

#ifndef ELTANIN__MAP_IO__LOAD_ERROR_HPP_
#define ELTANIN__MAP_IO__LOAD_ERROR_HPP_

#include <stdexcept>
#include <string>

namespace eltanin::map_io
{

enum class LoadErrorKind
{
  FileNotFound,
  YamlParseError,
  MissingKey,
  InvalidValue,
  UnsupportedMode,
  UnsupportedOriginYaw,
  PgmBadMagic,
  PgmBadMaxval,
  PgmSizeMismatch,
  PgmTruncated,
  WriteFailed
};

/// Thrown by map_io only; eltanin_core and eltanin_map never throw.
class LoadError : public std::runtime_error
{
public:
  LoadError(LoadErrorKind kind, const std::string & message)
  : std::runtime_error(message), kind_(kind)
  {
  }

  LoadErrorKind kind() const noexcept { return kind_; }

private:
  LoadErrorKind kind_;
};

}  // namespace eltanin::map_io

#endif  // ELTANIN__MAP_IO__LOAD_ERROR_HPP_
