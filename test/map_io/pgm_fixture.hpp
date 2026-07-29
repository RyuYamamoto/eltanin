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

#ifndef ELTANIN_TEST__MAP_IO__PGM_FIXTURE_HPP_
#define ELTANIN_TEST__MAP_IO__PGM_FIXTURE_HPP_

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace eltanin_test
{

inline std::filesystem::path fixture_dir()
{
  const std::filesystem::path dir = std::filesystem::path(ELTANIN_TEST_TMP_DIR) / "fixtures";
  std::filesystem::create_directories(dir);
  return dir;
}

inline std::filesystem::path write_binary(
  const std::string & name, const std::string & header,
  const std::vector<std::uint8_t> & pixels)
{
  const std::filesystem::path path = fixture_dir() / name;
  std::ofstream out(path, std::ios::binary);
  out.write(header.data(), static_cast<std::streamsize>(header.size()));
  if (!pixels.empty()) {
    out.write(
      reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  }
  return path;
}

inline std::filesystem::path write_text(const std::string & name, const std::string & content)
{
  const std::filesystem::path path = fixture_dir() / name;
  std::ofstream out(path);
  out << content;
  return path;
}

/// Minimal independent P5 reader so that write_pgm round-trips are not verified by read_pgm alone.
inline std::vector<std::uint8_t> read_raw_pgm(
  const std::filesystem::path & path, int & width, int & height)
{
  std::ifstream in(path, std::ios::binary);
  std::string magic;
  int maxval = 0;
  in >> magic >> width >> height >> maxval;
  in.get();
  std::vector<std::uint8_t> pixels(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
  in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  return pixels;
}

}  // namespace eltanin_test

#endif  // ELTANIN_TEST__MAP_IO__PGM_FIXTURE_HPP_
