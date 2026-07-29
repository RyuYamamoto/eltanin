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

#include <eltanin/map_io/pgm.hpp>

#include <cstddef>
#include <fstream>
#include <istream>
#include <string>

namespace eltanin::map_io
{
namespace
{

bool is_pgm_space(int c)
{
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

void skip_space_and_comments(std::istream & in)
{
  while (true) {
    const int c = in.peek();
    if (c == std::istream::traits_type::eof()) {
      return;
    }
    if (is_pgm_space(c)) {
      in.get();
      continue;
    }
    if (c == '#') {
      std::string discarded;
      std::getline(in, discarded);
      continue;
    }
    return;
  }
}

std::string read_token(std::istream & in)
{
  skip_space_and_comments(in);
  std::string token;
  while (true) {
    const int c = in.peek();
    if (c == std::istream::traits_type::eof() || is_pgm_space(c) || c == '#') {
      break;
    }
    token.push_back(static_cast<char>(in.get()));
  }
  return token;
}

int read_int_token(std::istream & in, const std::string & what)
{
  const std::string token = read_token(in);
  if (token.empty()) {
    throw LoadError(LoadErrorKind::PgmSizeMismatch, "PGM header ended before " + what);
  }
  try {
    std::size_t consumed = 0;
    const int value = std::stoi(token, &consumed);
    if (consumed != token.size()) {
      throw LoadError(LoadErrorKind::PgmSizeMismatch, "PGM " + what + " is not an integer");
    }
    return value;
  } catch (const std::invalid_argument &) {
    throw LoadError(LoadErrorKind::PgmSizeMismatch, "PGM " + what + " is not an integer");
  } catch (const std::out_of_range &) {
    throw LoadError(LoadErrorKind::PgmSizeMismatch, "PGM " + what + " is out of range");
  }
}

}  // namespace

PgmImage read_pgm(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    throw LoadError(LoadErrorKind::FileNotFound, "cannot open PGM file: " + path.string());
  }

  const std::string magic = read_token(in);
  if (magic != "P5") {
    throw LoadError(LoadErrorKind::PgmBadMagic, "unsupported PGM magic: '" + magic + "'");
  }

  const int width = read_int_token(in, "width");
  const int height = read_int_token(in, "height");
  if (width <= 0 || height <= 0) {
    throw LoadError(LoadErrorKind::PgmSizeMismatch, "PGM dimensions must be positive");
  }

  const int maxval = read_int_token(in, "maxval");
  if (maxval != 255) {
    throw LoadError(
      LoadErrorKind::PgmBadMaxval, "only maxval 255 is supported, got " + std::to_string(maxval));
  }

  // Exactly one whitespace character separates the header from the pixel data.
  if (!is_pgm_space(in.peek())) {
    throw LoadError(LoadErrorKind::PgmTruncated, "PGM header is not terminated by whitespace");
  }
  in.get();

  PgmImage image;
  image.width = width;
  image.height = height;
  const std::size_t pixel_count =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  image.pixels.resize(pixel_count);
  in.read(reinterpret_cast<char *>(image.pixels.data()), static_cast<std::streamsize>(pixel_count));
  if (static_cast<std::size_t>(in.gcount()) != pixel_count) {
    throw LoadError(
      LoadErrorKind::PgmTruncated, "PGM pixel data is shorter than the declared size");
  }
  return image;
}

void write_pgm(const std::filesystem::path & path, const map::Costmap & costmap)
{
  const int width = costmap.size_x();
  const int height = costmap.size_y();
  if (width <= 0 || height <= 0) {
    throw LoadError(LoadErrorKind::PgmSizeMismatch, "cannot write a PGM for an empty map");
  }

  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw LoadError(LoadErrorKind::WriteFailed, "cannot open PGM file for writing: " + path.string());
  }

  out << "P5\n" << width << " " << height << "\n255\n";
  for (int row = 0; row < height; ++row) {
    const int my = height - 1 - row;
    const std::size_t offset = costmap.geometry().index(0, my);
    out.write(
      reinterpret_cast<const char *>(costmap.data().data() + offset),
      static_cast<std::streamsize>(width));
  }
  out.flush();
  if (!out) {
    throw LoadError(LoadErrorKind::WriteFailed, "failed to write PGM file: " + path.string());
  }
}

}  // namespace eltanin::map_io
