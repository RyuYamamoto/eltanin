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
#include <eltanin/map_io/map_loader.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{

using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::NO_INFORMATION;
using eltanin::map_io::load_map;
using eltanin::map_io::load_map_metadata;
using eltanin::map_io::LoadError;
using eltanin::map_io::LoadErrorKind;
using eltanin::map_io::MapMetadata;
using eltanin::map_io::occupancy_cost;
using eltanin_test::fixture_dir;
using eltanin_test::write_binary;
using eltanin_test::write_text;

constexpr double kTol = 1e-12;

/// 3 wide, 2 tall: the top image row is black, the bottom is white.
std::filesystem::path write_asymmetric_image(const std::string & name)
{
  return write_binary(name, "P5\n3 2\n255\n", {0, 0, 0, 255, 255, 255});
}

std::string valid_yaml(const std::string & image_reference)
{
  return "image: " + image_reference +
         "\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
         "occupied_thresh: 0.65\nfree_thresh: 0.196\n";
}

LoadErrorKind kind_of_failure(const std::filesystem::path & yaml_path)
{
  try {
    load_map_metadata(yaml_path);
  } catch (const LoadError & error) {
    return error.kind();
  }
  throw std::runtime_error("expected LoadError but none was thrown");
}

}  // namespace

TEST(MapLoader, ReadsMetadataAndResolvesRelativeImagePath)
{
  write_asymmetric_image("meta.pgm");
  const auto yaml = write_text("meta.yaml", valid_yaml("meta.pgm"));

  const MapMetadata metadata = load_map_metadata(yaml);
  EXPECT_EQ(metadata.image_path, fixture_dir() / "meta.pgm");
  EXPECT_NEAR(metadata.resolution, 0.05, kTol);
  EXPECT_NEAR(metadata.origin.x(), -1.0, kTol);
  EXPECT_NEAR(metadata.origin.y(), -2.0, kTol);
  EXPECT_FALSE(metadata.negate);
  EXPECT_NEAR(metadata.occupied_thresh, 0.65, kTol);
  EXPECT_NEAR(metadata.free_thresh, 0.196, kTol);
}

TEST(MapLoader, AcceptsAbsoluteImagePath)
{
  const auto image = write_asymmetric_image("absolute.pgm");
  const auto yaml = write_text("absolute.yaml", valid_yaml(image.string()));
  EXPECT_EQ(load_map_metadata(yaml).image_path, image);
}

TEST(MapLoader, MissingModeKeyIsAccepted)
{
  write_asymmetric_image("no_mode.pgm");
  const auto yaml = write_text("no_mode.yaml", valid_yaml("no_mode.pgm"));
  EXPECT_NO_THROW(load_map_metadata(yaml));
}

TEST(MapLoader, TrinaryModeIsAcceptedAndOthersRejected)
{
  write_asymmetric_image("mode.pgm");
  const auto trinary = write_text("mode_trinary.yaml", valid_yaml("mode.pgm") + "mode: trinary\n");
  EXPECT_NO_THROW(load_map_metadata(trinary));

  const auto scale = write_text("mode_scale.yaml", valid_yaml("mode.pgm") + "mode: scale\n");
  EXPECT_EQ(kind_of_failure(scale), LoadErrorKind::UnsupportedMode);
}

TEST(MapLoader, MissingFileAndMissingKeysAreReported)
{
  EXPECT_EQ(kind_of_failure(fixture_dir() / "absent.yaml"), LoadErrorKind::FileNotFound);

  const auto no_image = write_text(
    "no_image.yaml",
    "resolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
  EXPECT_EQ(kind_of_failure(no_image), LoadErrorKind::MissingKey);

  const auto no_thresh = write_text(
    "no_thresh.yaml", "image: mode.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n");
  EXPECT_EQ(kind_of_failure(no_thresh), LoadErrorKind::MissingKey);
}

TEST(MapLoader, InvalidValuesAreReported)
{
  const auto zero_resolution = write_text(
    "zero_resolution.yaml",
    "image: mode.pgm\nresolution: 0.0\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
  EXPECT_EQ(kind_of_failure(zero_resolution), LoadErrorKind::InvalidValue);

  const auto negative_resolution = write_text(
    "negative_resolution.yaml",
    "image: mode.pgm\nresolution: -0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
  EXPECT_EQ(kind_of_failure(negative_resolution), LoadErrorKind::InvalidValue);

  const auto short_origin = write_text(
    "short_origin.yaml",
    "image: mode.pgm\nresolution: 0.05\norigin: [0.0, 0.0]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
  EXPECT_EQ(kind_of_failure(short_origin), LoadErrorKind::InvalidValue);
}

TEST(MapLoader, NonZeroOriginYawIsRejected)
{
  const auto rotated = write_text(
    "rotated.yaml",
    "image: mode.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.3]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
  EXPECT_EQ(kind_of_failure(rotated), LoadErrorKind::UnsupportedOriginYaw);
}

TEST(MapLoader, MalformedYamlIsReported)
{
  const auto broken = write_text("broken.yaml", "image: [unclosed\nresolution: 0.05\n");
  EXPECT_EQ(kind_of_failure(broken), LoadErrorKind::YamlParseError);
}

TEST(MapLoader, OccupancyThresholdBoundariesFallToUnknown)
{
  MapMetadata metadata;
  metadata.negate = false;
  metadata.occupied_thresh = 100.0 / 255.0;
  metadata.free_thresh = 50.0 / 255.0;

  EXPECT_EQ(occupancy_cost(0, metadata), LETHAL_OBSTACLE);
  EXPECT_EQ(occupancy_cost(154, metadata), LETHAL_OBSTACLE);
  EXPECT_EQ(occupancy_cost(155, metadata), NO_INFORMATION);
  EXPECT_EQ(occupancy_cost(205, metadata), NO_INFORMATION);
  EXPECT_EQ(occupancy_cost(206, metadata), FREE_SPACE);
  EXPECT_EQ(occupancy_cost(255, metadata), FREE_SPACE);
}

TEST(MapLoader, NegateInvertsTheOccupancyMapping)
{
  MapMetadata metadata;
  metadata.negate = true;
  metadata.occupied_thresh = 100.0 / 255.0;
  metadata.free_thresh = 50.0 / 255.0;

  EXPECT_EQ(occupancy_cost(255, metadata), LETHAL_OBSTACLE);
  EXPECT_EQ(occupancy_cost(101, metadata), LETHAL_OBSTACLE);
  EXPECT_EQ(occupancy_cost(100, metadata), NO_INFORMATION);
  EXPECT_EQ(occupancy_cost(50, metadata), NO_INFORMATION);
  EXPECT_EQ(occupancy_cost(49, metadata), FREE_SPACE);
  EXPECT_EQ(occupancy_cost(0, metadata), FREE_SPACE);
}

TEST(MapLoader, LoadMapFlipsRowsSoRowZeroIsTheBottom)
{
  write_asymmetric_image("flip.pgm");
  const auto yaml = write_text("flip.yaml", valid_yaml("flip.pgm"));

  const Costmap costmap = load_map(yaml);
  ASSERT_EQ(costmap.size_x(), 3);
  ASSERT_EQ(costmap.size_y(), 2);
  EXPECT_NEAR(costmap.geometry().resolution(), 0.05, kTol);
  EXPECT_NEAR(costmap.geometry().origin().x(), -1.0, kTol);
  EXPECT_NEAR(costmap.geometry().origin().y(), -2.0, kTol);

  for (int mx = 0; mx < 3; ++mx) {
    EXPECT_EQ(costmap(mx, 0), FREE_SPACE) << "mx=" << mx;
    EXPECT_EQ(costmap(mx, 1), LETHAL_OBSTACLE) << "mx=" << mx;
  }
}

TEST(MapLoader, LoadMapReportsAMissingImage)
{
  const auto yaml = write_text("missing_image.yaml", valid_yaml("not_there.pgm"));
  try {
    load_map(yaml);
    FAIL() << "expected LoadError";
  } catch (const LoadError & error) {
    EXPECT_EQ(error.kind(), LoadErrorKind::FileNotFound);
  }
}
