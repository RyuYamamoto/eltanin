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

#include <eltanin/core/footprint.hpp>
#include <eltanin/core/polygon.hpp>
#include <eltanin/core/traversability.hpp>
#include <eltanin/map/cost_model.hpp>
#include <eltanin/map/layers/inflation_layer.hpp>
#include <eltanin/map_io/map_loader.hpp>
#include <eltanin/map_io/pgm.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>

namespace
{

using Eigen::Vector2d;
using eltanin::DistanceTraversabilityModel;
using eltanin::Polygon2D;
using eltanin::Traversability;
using eltanin::map::Costmap;
using eltanin::map::CostTraversabilityModel;
using eltanin::map::InflationCostModel;
using eltanin::map::InflationLayer;

/// Cell count of FREE_SPACE in the reference map before inflation; see .plait/00-requirements.md.
constexpr std::size_t FREE_CELLS_BEFORE_INFLATION = 631664;

constexpr double TOLERANCE = 1e-9;

}  // namespace

TEST(InflationRealMap, ProducesAllThreeTraversabilityClasses)
{
  const std::filesystem::path yaml = std::filesystem::path(ELTANIN_TEST_MAP_DIR) / "map.yaml";
  if (!std::filesystem::is_regular_file(yaml)) {
    GTEST_SKIP() << "reference map not available at " << yaml;
  }

  Costmap costmap = eltanin::map_io::load_map(yaml);
  ASSERT_EQ(costmap.cell_count(), 16000000u);

  const Polygon2D footprint = {
    Vector2d{0.22, 0.15}, Vector2d{-0.22, 0.15}, Vector2d{-0.22, -0.15}, Vector2d{0.22, -0.15}};
  const auto distance_model = DistanceTraversabilityModel::from_footprint(footprint, 0.55);
  ASSERT_TRUE(distance_model.has_value());
  EXPECT_NEAR(distance_model->inscribed_radius(), 0.15, TOLERANCE);
  EXPECT_NEAR(distance_model->circumscribed_radius(), std::hypot(0.22, 0.15), TOLERANCE);

  const auto model = InflationCostModel::create(*distance_model, 10.0);
  ASSERT_TRUE(model.has_value());
  EXPECT_EQ(model->circumscribed_cost(), 78);

  InflationLayer layer(*model, false);
  layer.update_costs(costmap);

  const CostTraversabilityModel classifier(model->circumscribed_cost(), false);
  std::size_t free_cells = 0;
  std::size_t circumscribed_cells = 0;
  std::size_t inscribed_cells = 0;
  for (std::size_t i = 0; i < costmap.cell_count(); ++i) {
    switch (classifier.classify(costmap[i])) {
      case Traversability::Free:
        ++free_cells;
        break;
      case Traversability::Circumscribed:
        ++circumscribed_cells;
        break;
      case Traversability::Inscribed:
        ++inscribed_cells;
        break;
    }
  }

  const std::filesystem::path dump =
    std::filesystem::path(ELTANIN_TEST_TMP_DIR) / "real_map_inflated.pgm";
  eltanin::map_io::write_pgm(dump, costmap);

  std::cout << "circumscribed_cost " << static_cast<int>(model->circumscribed_cost())
            << " free " << free_cells << " circumscribed " << circumscribed_cells
            << " inscribed " << inscribed_cells << " dump " << dump << std::endl;

  EXPECT_GT(free_cells, 0u);
  EXPECT_GT(inscribed_cells, 0u);
  EXPECT_GT(circumscribed_cells, 0u);
  EXPECT_LT(circumscribed_cells, FREE_CELLS_BEFORE_INFLATION);
  EXPECT_EQ(free_cells + circumscribed_cells + inscribed_cells, costmap.cell_count());
}
