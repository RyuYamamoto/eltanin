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

#include <eltanin/map/cost_values.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/map_io/pgm.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <system_error>

namespace eltanin::map_io
{
namespace
{

YAML::Node require(const YAML::Node & root, const std::string & key)
{
  if (!root[key]) {
    throw LoadError(LoadErrorKind::MissingKey, "missing required YAML key: " + key);
  }
  return root[key];
}

template <class T>
T require_as(const YAML::Node & root, const std::string & key)
{
  try {
    return require(root, key).as<T>();
  } catch (const YAML::Exception &) {
    throw LoadError(LoadErrorKind::InvalidValue, "YAML key has an unexpected type: " + key);
  }
}

}  // namespace

MapMetadata load_map_metadata(const std::filesystem::path & yaml_path)
{
  std::error_code ec;
  if (!std::filesystem::is_regular_file(yaml_path, ec)) {
    throw LoadError(LoadErrorKind::FileNotFound, "cannot open map YAML: " + yaml_path.string());
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception & e) {
    throw LoadError(
      LoadErrorKind::YamlParseError, "failed to parse " + yaml_path.string() + ": " + e.what());
  }

  MapMetadata metadata;

  const std::filesystem::path image = require_as<std::string>(root, "image");
  metadata.image_path = image.is_absolute() ? image : yaml_path.parent_path() / image;

  metadata.resolution = require_as<double>(root, "resolution");
  if (!std::isfinite(metadata.resolution) || metadata.resolution <= 0.0) {
    throw LoadError(LoadErrorKind::InvalidValue, "resolution must be positive");
  }

  const YAML::Node origin = require(root, "origin");
  if (!origin.IsSequence() || origin.size() != 3) {
    throw LoadError(LoadErrorKind::InvalidValue, "origin must be a sequence of three numbers");
  }
  double origin_values[3] = {0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < 3; ++i) {
    try {
      origin_values[i] = origin[i].as<double>();
    } catch (const YAML::Exception &) {
      throw LoadError(LoadErrorKind::InvalidValue, "origin elements must be numbers");
    }
  }
  if (origin_values[2] != 0.0) {
    throw LoadError(
      LoadErrorKind::UnsupportedOriginYaw, "a non-zero origin yaw is not supported yet");
  }
  metadata.origin = Vec2{origin_values[0], origin_values[1]};

  metadata.negate = require_as<int>(root, "negate") != 0;
  metadata.occupied_thresh = require_as<double>(root, "occupied_thresh");
  metadata.free_thresh = require_as<double>(root, "free_thresh");

  if (root["mode"]) {
    const std::string mode = require_as<std::string>(root, "mode");
    if (mode != "trinary") {
      throw LoadError(LoadErrorKind::UnsupportedMode, "only the trinary mode is supported");
    }
  }

  return metadata;
}

std::uint8_t occupancy_cost(std::uint8_t pixel, const MapMetadata & metadata) noexcept
{
  const double value = metadata.negate ? static_cast<double>(pixel)
                                       : 255.0 - static_cast<double>(pixel);
  const double occupancy = value / 255.0;
  if (occupancy > metadata.occupied_thresh) {
    return map::LETHAL_OBSTACLE;
  }
  if (occupancy < metadata.free_thresh) {
    return map::FREE_SPACE;
  }
  return map::NO_INFORMATION;
}

map::Costmap load_map(const std::filesystem::path & yaml_path)
{
  const MapMetadata metadata = load_map_metadata(yaml_path);
  const PgmImage image = read_pgm(metadata.image_path);

  const map::MapGeometry geometry(image.width, image.height, metadata.resolution, metadata.origin);
  map::Costmap costmap(geometry, map::NO_INFORMATION);

  for (int row = 0; row < image.height; ++row) {
    // PGM row 0 is the top of the image; the map's my = 0 is the bottom.
    const int my = image.height - 1 - row;
    const std::size_t source = static_cast<std::size_t>(row) *
                               static_cast<std::size_t>(image.width);
    const std::size_t destination = geometry.index(0, my);
    for (int mx = 0; mx < image.width; ++mx) {
      costmap[destination + static_cast<std::size_t>(mx)] =
        occupancy_cost(image.pixels[source + static_cast<std::size_t>(mx)], metadata);
    }
  }
  return costmap;
}

}  // namespace eltanin::map_io
