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

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>

namespace
{

using eltanin::map::Costmap;
using eltanin::map::FREE_SPACE;
using eltanin::map::LETHAL_OBSTACLE;
using eltanin::map::NO_INFORMATION;
using eltanin::map_io::load_map;

constexpr double kTol = 1e-12;

}  // namespace

TEST(RealMap, LoadsTheNavyuReferenceMap)
{
  const std::filesystem::path yaml = std::filesystem::path(ELTANIN_TEST_MAP_DIR) / "map.yaml";
  if (!std::filesystem::is_regular_file(yaml)) {
    GTEST_SKIP() << "reference map not available at " << yaml;
  }

  const Costmap costmap = load_map(yaml);

  EXPECT_EQ(costmap.size_x(), 4000);
  EXPECT_EQ(costmap.size_y(), 4000);
  EXPECT_EQ(costmap.cell_count(), 16000000u);
  EXPECT_NEAR(costmap.geometry().resolution(), 0.05, kTol);
  EXPECT_NEAR(costmap.geometry().origin().x(), -100.0, kTol);
  EXPECT_NEAR(costmap.geometry().origin().y(), -100.0, kTol);

  std::size_t free_cells = 0;
  std::size_t lethal_cells = 0;
  std::size_t unknown_cells = 0;
  for (std::size_t i = 0; i < costmap.cell_count(); ++i) {
    switch (costmap[i]) {
      case FREE_SPACE:
        ++free_cells;
        break;
      case LETHAL_OBSTACLE:
        ++lethal_cells;
        break;
      case NO_INFORMATION:
        ++unknown_cells;
        break;
      default:
        FAIL() << "unexpected cost value " << static_cast<int>(costmap[i]) << " at index " << i;
    }
  }

  EXPECT_GT(free_cells, 0u);
  EXPECT_GT(lethal_cells, 0u);
  EXPECT_GT(unknown_cells, 0u);
  EXPECT_EQ(free_cells + lethal_cells + unknown_cells, costmap.cell_count());
}
