#include <cstdio>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "global_pct_planner/tomogram.h"
#include "global_pct_planner/tomogram_io.h"

namespace global_pct_planner {

TEST(Tomogram, GeneratesFlatCOrderMapAndRoundTrips) {
  TomogramProfile profile;
  profile.resolution = 0.25f;
  profile.slice_dh = 0.5f;
  profile.kernel_size = 3;
  profile.inflation = 0.1f;
  profile.safe_margin = 0.25f;
  std::vector<PointXYZ> points;
  for (int row = -4; row <= 4; ++row) {
    for (int col = -3; col <= 3; ++col) {
      const float x = static_cast<float>(row * profile.resolution);
      const float y = static_cast<float>(col * profile.resolution);
      points.push_back({x, y, 0.0f});
      points.push_back({x, y, 1.0f});
    }
  }
  TomogramEngine engine(profile);
  TomogramData map = engine.generate(points);
  ASSERT_GT(map.layers, 0u);
  ASSERT_GT(map.rows, 0u);
  ASSERT_GT(map.cols, 0u);
  ASSERT_EQ(map.cellCount(), map.traversability.size());
  ASSERT_EQ(map.cellCount(), map.ground_elevation.size());
  EXPECT_TRUE(std::isfinite(map.ground_elevation[map.index(0, map.rows / 2,
                                                            map.cols / 2)]));
  EXPECT_EQ(map.index(1, 2, 3),
            static_cast<std::size_t>((1u * map.rows + 2u) * map.cols + 3u));

  const std::string path = "/tmp/global_pct_planner_tomogram_test.pctm";
  std::string error;
  ASSERT_TRUE(TomogramIO::save(path, map, &error)) << error;
  TomogramData loaded;
  ASSERT_TRUE(TomogramIO::load(path, loaded, &error)) << error;
  EXPECT_EQ(map.layers, loaded.layers);
  EXPECT_EQ(map.rows, loaded.rows);
  EXPECT_EQ(map.cols, loaded.cols);
  EXPECT_FLOAT_EQ(map.resolution, loaded.resolution);
  EXPECT_FLOAT_EQ(map.center_x, loaded.center_x);
  EXPECT_FLOAT_EQ(map.center_y, loaded.center_y);
  ASSERT_EQ(map.traversability.size(), loaded.traversability.size());
  for (std::size_t i = 0; i < map.cellCount(); ++i) {
    EXPECT_FLOAT_EQ(map.traversability[i], loaded.traversability[i]);
    EXPECT_FLOAT_EQ(map.traversability_grad_x[i],
                    loaded.traversability_grad_x[i]);
    EXPECT_FLOAT_EQ(map.traversability_grad_y[i],
                    loaded.traversability_grad_y[i]);
    if (std::isnan(map.ground_elevation[i]))
      EXPECT_TRUE(std::isnan(loaded.ground_elevation[i]));
    else
      EXPECT_FLOAT_EQ(map.ground_elevation[i], loaded.ground_elevation[i]);
  }
  std::remove(path.c_str());
}

TEST(Tomogram, EmptyAndInvalidInputs) {
  TomogramProfile profile;
  TomogramEngine engine(profile);
  const TomogramData empty = engine.generate({});
  EXPECT_EQ(empty.layers, 1u);
  EXPECT_EQ(empty.rows, 4u);
  EXPECT_EQ(empty.cols, 4u);
  EXPECT_TRUE(std::isnan(empty.ground_elevation.front()));

  std::vector<PointXYZ> points{{std::numeric_limits<float>::quiet_NaN(), 0.0f,
                                0.0f},
                               {0.0f, 0.0f, std::numeric_limits<float>::infinity()}};
  const TomogramData invalid_points = engine.generate(points);
  EXPECT_EQ(invalid_points.cellCount(), 16u);
  profile.resolution = 0.0f;
  EXPECT_THROW(TomogramEngine(profile).generate({{0.0f, 0.0f, 0.0f}}),
               std::invalid_argument);
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
