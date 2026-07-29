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

#include "pgm_fixture.hpp"

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map_io/pgm.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace
{

using eltanin::Vec2;
using eltanin::map::Costmap;
using eltanin::map::MapGeometry;
using eltanin::map_io::LoadError;
using eltanin::map_io::LoadErrorKind;
using eltanin::map_io::PgmImage;
using eltanin::map_io::read_pgm;
using eltanin::map_io::write_pgm;
using eltanin_test::read_raw_pgm;
using eltanin_test::write_binary;

}  // namespace

TEST(Pgm, ReadsPixelsInFileOrder)
{
  const std::vector<std::uint8_t> pixels{10, 20, 30, 40, 50, 60};
  const auto path = write_binary("plain.pgm", "P5\n3 2\n255\n", pixels);

  const PgmImage image = read_pgm(path);
  EXPECT_EQ(image.width, 3);
  EXPECT_EQ(image.height, 2);
  ASSERT_EQ(image.pixels.size(), 6u);
  EXPECT_EQ(image.pixels[0], 10);
  EXPECT_EQ(image.pixels[5], 60);
}

TEST(Pgm, SkipsCommentsAnywhereInHeader)
{
  const std::vector<std::uint8_t> pixels{1, 2, 3, 4};
  const auto path = write_binary(
    "comments.pgm", "P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n2 # inline\n2\n# before maxval\n255\n",
    pixels);

  const PgmImage image = read_pgm(path);
  EXPECT_EQ(image.width, 2);
  EXPECT_EQ(image.height, 2);
  ASSERT_EQ(image.pixels.size(), 4u);
  EXPECT_EQ(image.pixels[0], 1);
  EXPECT_EQ(image.pixels[3], 4);
}

TEST(Pgm, MissingFileReportsFileNotFound)
{
  try {
    read_pgm(eltanin_test::fixture_dir() / "does_not_exist.pgm");
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::FileNotFound);
  }
}

TEST(Pgm, WrongMagicIsRejected)
{
  const auto path = write_binary("ascii.pgm", "P2\n2 2\n255\n1 2 3 4\n", {});
  try {
    read_pgm(path);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::PgmBadMagic);
  }
}

TEST(Pgm, WrongMaxvalIsRejected)
{
  const auto path = write_binary("maxval.pgm", "P5\n2 2\n65535\n", {1, 2, 3, 4});
  try {
    read_pgm(path);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::PgmBadMaxval);
  }
}

TEST(Pgm, TruncatedPixelDataIsRejected)
{
  const auto path = write_binary("truncated.pgm", "P5\n4 4\n255\n", {1, 2, 3});
  try {
    read_pgm(path);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::PgmTruncated);
  }
}

TEST(Pgm, NonPositiveDimensionsAreRejected)
{
  const auto path = write_binary("zero.pgm", "P5\n0 4\n255\n", {});
  try {
    read_pgm(path);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::PgmSizeMismatch);
  }
}

TEST(Pgm, WriteFlipsRowsAndPreservesRawCellValues)
{
  Costmap costmap(MapGeometry(3, 2, 0.1, Vec2{-1.0, -1.0}), eltanin::map::FREE_SPACE);
  costmap(0, 0) = 253;
  costmap(1, 0) = 254;
  costmap(2, 0) = 255;
  costmap(0, 1) = 1;
  costmap(1, 1) = 2;
  costmap(2, 1) = 3;

  const auto path = eltanin_test::fixture_dir() / "dump.pgm";
  write_pgm(path, costmap);

  int width = 0;
  int height = 0;
  const std::vector<std::uint8_t> pixels = read_raw_pgm(path, width, height);
  ASSERT_EQ(width, 3);
  ASSERT_EQ(height, 2);
  ASSERT_EQ(pixels.size(), 6u);
  EXPECT_EQ(pixels[0], 1);
  EXPECT_EQ(pixels[1], 2);
  EXPECT_EQ(pixels[2], 3);
  EXPECT_EQ(pixels[3], 253);
  EXPECT_EQ(pixels[4], 254);
  EXPECT_EQ(pixels[5], 255);
}

TEST(Pgm, WriteThenReadRoundTripsEveryCell)
{
  Costmap costmap(MapGeometry(5, 4, 0.05, Vec2{2.0, 3.0}), eltanin::map::FREE_SPACE);
  std::uint8_t value = 0;
  for (int my = 0; my < costmap.size_y(); ++my) {
    for (int mx = 0; mx < costmap.size_x(); ++mx) {
      costmap(mx, my) = static_cast<std::uint8_t>(240 + value);
      ++value;
    }
  }

  const auto path = eltanin_test::fixture_dir() / "round_trip.pgm";
  write_pgm(path, costmap);
  const PgmImage image = read_pgm(path);

  ASSERT_EQ(image.width, costmap.size_x());
  ASSERT_EQ(image.height, costmap.size_y());
  for (int row = 0; row < image.height; ++row) {
    const int my = image.height - 1 - row;
    for (int mx = 0; mx < image.width; ++mx) {
      const std::size_t i =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(image.width) +
        static_cast<std::size_t>(mx);
      EXPECT_EQ(image.pixels[i], costmap(mx, my)) << "mx=" << mx << " my=" << my;
    }
  }
}

TEST(Pgm, WritingAnEmptyMapIsRejected)
{
  const Costmap empty;
  try {
    write_pgm(eltanin_test::fixture_dir() / "empty.pgm", empty);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::PgmSizeMismatch);
  }
}
