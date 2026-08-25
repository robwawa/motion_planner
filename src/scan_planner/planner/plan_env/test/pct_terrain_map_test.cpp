#include <gtest/gtest.h>

#include <plan_env/pct_terrain_map.h>

namespace
{
  pct_planner::PctTerrainMap makeMap(const uint32_t rows, const uint32_t cols,
                                     const std::vector<float> &traversability,
                                     const std::vector<float> &ground_elevation)
  {
    pct_planner::PctTerrainMap msg;
    msg.header.frame_id = "map";
    msg.resolution = 0.1;
    msg.center_x = 0.0;
    msg.center_y = 0.0;
    msg.rows = rows;
    msg.cols = cols;
    msg.layers = static_cast<uint32_t>(ground_elevation.size() / (rows * cols));
    msg.traversability = traversability;
    msg.ground_elevation = ground_elevation;
    msg.elevation_valid.assign(ground_elevation.size(), 1);
    return msg;
  }

  PctTerrainMap loadMap(const pct_planner::PctTerrainMap &msg)
  {
    PctTerrainMap map;
    std::string error;
    EXPECT_TRUE(map.setFromMessage(msg, error)) << error;
    return map;
  }
} // namespace

TEST(PctTerrainMap, SelectsPythonRowAtPositiveAndNegativeHalfCell)
{
  PctTerrainMap map = loadMap(makeMap(3, 1, {0.0f, 0.0f, 0.0f},
                                      {10.0f, 20.0f, 30.0f}));
  PctTerrainMap::Cell cell;

  EXPECT_EQ(PctTerrainMap::QueryStatus::kFree,
            map.queryTraversableLayer(0.05, 0.0, 20.0, 0.1, 20.0, cell));
  EXPECT_FLOAT_EQ(20.0f, cell.ground_z);

  EXPECT_EQ(PctTerrainMap::QueryStatus::kFree,
            map.queryTraversableLayer(-0.05, 0.0, 20.0, 0.1, 20.0, cell));
  EXPECT_FLOAT_EQ(20.0f, cell.ground_z);
}

TEST(PctTerrainMap, SelectsPythonColumnAtHalfCell)
{
  PctTerrainMap map = loadMap(makeMap(1, 3, {0.0f, 0.0f, 0.0f},
                                      {10.0f, 20.0f, 30.0f}));
  PctTerrainMap::Cell cell;

  EXPECT_EQ(PctTerrainMap::QueryStatus::kFree,
            map.queryTraversableLayer(0.0, 0.05, 20.0, 0.1, 20.0, cell));
  EXPECT_FLOAT_EQ(20.0f, cell.ground_z);
}

TEST(PctTerrainMap, KeepsNearestCellForNonHalfCoordinate)
{
  PctTerrainMap map = loadMap(makeMap(3, 1, {0.0f, 0.0f, 0.0f},
                                      {10.0f, 20.0f, 30.0f}));
  PctTerrainMap::Cell cell;

  EXPECT_EQ(PctTerrainMap::QueryStatus::kFree,
            map.queryTraversableLayer(0.06, 0.0, 30.0, 0.1, 20.0, cell));
  EXPECT_FLOAT_EQ(30.0f, cell.ground_z);
}

TEST(PctTerrainMap, PreservesLayerAndTraversabilityFiltering)
{
  PctTerrainMap map = loadMap(makeMap(1, 1, {0.0f, 30.0f}, {0.0f, 2.0f}));
  PctTerrainMap::Cell cell;

  EXPECT_EQ(PctTerrainMap::QueryStatus::kHeightMismatch,
            map.queryTraversableLayer(0.0, 0.0, 2.0, 0.4, 20.0, cell));
}
