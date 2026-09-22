#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "global_pct_planner/tomogram_io.h"

namespace global_pct_planner {
namespace {

#ifndef GLOBAL_PCT_SOURCE_DIR
#define GLOBAL_PCT_SOURCE_DIR "."
#endif

constexpr std::uint64_t kFnvOffset = 14695981039346656037ull;
constexpr std::uint64_t kFnvPrime = 1099511628211ull;

std::uint64_t hashFloats(const std::vector<float>& values) {
  std::uint64_t hash = kFnvOffset;
  for (const float value : values) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    for (std::size_t byte = 0; byte < sizeof(float); ++byte) {
      hash ^= bytes[byte];
      hash *= kFnvPrime;
    }
  }
  return hash;
}

std::string fixturePath() {
  return std::string(GLOBAL_PCT_SOURCE_DIR) +
         "/rsc/tomogram/building2_9.pctm";
}

std::size_t countFinite(const std::vector<float>& values) {
  std::size_t count = 0;
  for (const float value : values) {
    if (std::isfinite(value)) ++count;
  }
  return count;
}

std::size_t countNaN(const std::vector<float>& values) {
  std::size_t count = 0;
  for (const float value : values) {
    if (std::isnan(value)) ++count;
  }
  return count;
}

}  // namespace

TEST(PctmFixture, BuildingMapLayoutMetadataAndPayloadRegression) {
  TomogramData map;
  std::string error;
  ASSERT_TRUE(TomogramIO::load(fixturePath(), map, &error)) << error;

  EXPECT_EQ(map.layers, 11u);
  EXPECT_EQ(map.rows, 189u);
  EXPECT_EQ(map.cols, 184u);
  EXPECT_EQ(map.cellCount(), 382536u);
  EXPECT_FLOAT_EQ(map.resolution, 0.10000000149011612f);
  EXPECT_FLOAT_EQ(map.center_x, 1.4546852111816406f);
  EXPECT_FLOAT_EQ(map.center_y, -0.3868112564086914f);
  EXPECT_FLOAT_EQ(map.slice_h0, 0.5f);
  EXPECT_FLOAT_EQ(map.slice_dh, 0.5f);

  ASSERT_EQ(map.traversability.size(), map.cellCount());
  ASSERT_EQ(map.traversability_grad_x.size(), map.cellCount());
  ASSERT_EQ(map.traversability_grad_y.size(), map.cellCount());
  ASSERT_EQ(map.ground_elevation.size(), map.cellCount());
  ASSERT_EQ(map.ceiling_elevation.size(), map.cellCount());

  // These hashes cover every C-order [layer][row][col] float, including its
  // NaN bit pattern, and make an accidental channel reorder or truncation
  // fail the fixed-map regression immediately.
  EXPECT_EQ(hashFloats(map.traversability), 0xb76e674208c09b95ull);
  EXPECT_EQ(hashFloats(map.traversability_grad_x), 0xe8c62735f334a888ull);
  EXPECT_EQ(hashFloats(map.traversability_grad_y), 0x29fa2d332e257548ull);
  EXPECT_EQ(hashFloats(map.ground_elevation), 0xc01bab9994927c8bull);
  EXPECT_EQ(hashFloats(map.ceiling_elevation), 0x559c79f64e1c4045ull);

  EXPECT_EQ(countFinite(map.ground_elevation), 338186u);
  EXPECT_EQ(countNaN(map.ground_elevation), 44350u);
  EXPECT_EQ(countFinite(map.ceiling_elevation), 292561u);
  EXPECT_EQ(countNaN(map.ceiling_elevation), 89975u);
  EXPECT_EQ(countNaN(map.traversability), 0u);
  EXPECT_EQ(countNaN(map.traversability_grad_x), 0u);
  EXPECT_EQ(countNaN(map.traversability_grad_y), 0u);
}

TEST(PctmFixture, GlobalMapPctClean03PayloadRegression) {
  TomogramData map;
  std::string error;
  ASSERT_TRUE(TomogramIO::load(
                  std::string(GLOBAL_PCT_SOURCE_DIR) +
                      "/rsc/tomogram/GlobalMap_pct_clean03.pctm",
                  map, &error))
      << error;

  EXPECT_EQ(map.layers, 3u);
  EXPECT_EQ(map.rows, 306u);
  EXPECT_EQ(map.cols, 467u);
  EXPECT_EQ(map.cellCount(), 428706u);
  EXPECT_FLOAT_EQ(map.resolution, 0.10000000149011612f);
  EXPECT_FLOAT_EQ(map.center_x, 3.742161750793457f);
  EXPECT_FLOAT_EQ(map.center_y, -5.8690032958984375f);
  EXPECT_FLOAT_EQ(map.slice_h0, 0.5f);
  EXPECT_FLOAT_EQ(map.slice_dh, 0.5f);

  EXPECT_EQ(hashFloats(map.traversability), 0xfca637489f6db704ull);
  EXPECT_EQ(hashFloats(map.traversability_grad_x), 0x1336538572a3c641ull);
  EXPECT_EQ(hashFloats(map.traversability_grad_y), 0xf29149c8a8c09d1eull);
  EXPECT_EQ(hashFloats(map.ground_elevation), 0x6ed253f2abbdcafeull);
  EXPECT_EQ(hashFloats(map.ceiling_elevation), 0x717ef2994ef0ae06ull);
  EXPECT_EQ(countFinite(map.ground_elevation), 50719u);
  EXPECT_EQ(countNaN(map.ground_elevation), 377987u);
  EXPECT_EQ(countFinite(map.ceiling_elevation), 39659u);
  EXPECT_EQ(countNaN(map.ceiling_elevation), 389047u);
}

TEST(PctmFixture, BuildingMapCleanPayloadRegression) {
  TomogramData map;
  std::string error;
  ASSERT_TRUE(TomogramIO::load(
                  std::string(GLOBAL_PCT_SOURCE_DIR) +
                      "/rsc/tomogram/building2_9_clean.pctm",
                  map, &error))
      << error;

  // This fixture is regenerated from the PCD used by the Gazebo demo.  Keep
  // its layout and complete payload fixed so a stale .pctm cannot silently
  // make A* search a different grid than the active point cloud.
  EXPECT_EQ(map.layers, 9u);
  EXPECT_EQ(map.rows, 188u);
  EXPECT_EQ(map.cols, 183u);
  EXPECT_EQ(map.cellCount(), 309636u);
  EXPECT_FLOAT_EQ(map.resolution, 0.10000000149011612f);
  EXPECT_FLOAT_EQ(map.center_x, 1.4876441955566406f);
  EXPECT_FLOAT_EQ(map.center_y, -0.35709285736083984f);
  EXPECT_FLOAT_EQ(map.slice_h0, 0.5f);
  EXPECT_FLOAT_EQ(map.slice_dh, 0.5f);

  EXPECT_EQ(hashFloats(map.traversability), 0xb4c44e3b947a9151ull);
  EXPECT_EQ(hashFloats(map.traversability_grad_x), 0x452c8e7736856d4aull);
  EXPECT_EQ(hashFloats(map.traversability_grad_y), 0xfcd97b81f157b47dull);
  EXPECT_EQ(hashFloats(map.ground_elevation), 0x0ac0abaefdf2205aull);
  EXPECT_EQ(hashFloats(map.ceiling_elevation), 0xfa591cc342fd31afull);
  EXPECT_EQ(countFinite(map.ground_elevation), 237997u);
  EXPECT_EQ(countNaN(map.ground_elevation), 71639u);
  EXPECT_EQ(countFinite(map.ceiling_elevation), 192605u);
  EXPECT_EQ(countNaN(map.ceiling_elevation), 117031u);
  EXPECT_EQ(countNaN(map.traversability), 0u);
  EXPECT_EQ(countNaN(map.traversability_grad_x), 0u);
  EXPECT_EQ(countNaN(map.traversability_grad_y), 0u);
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
