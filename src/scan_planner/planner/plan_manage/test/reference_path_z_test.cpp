#include <gtest/gtest.h>

#include <plan_manage/reference_path_z.h>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace scan_planner
{
  namespace
  {
    Eigen::Vector3d point(const double x, const double y, const double z)
    {
      return Eigen::Vector3d(x, y, z);
    }

    TEST(ReferencePathZProfile, SamplesLinearSlope)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));
      EXPECT_DOUBLE_EQ(10.0, profile.totalProgress());
      EXPECT_NEAR(2.5, profile.sampleZ(2.5), 1e-9);
      EXPECT_NEAR(10.0, profile.sampleZ(20.0), 1e-9);
    }

    TEST(ReferencePathZProfile, ProjectsInLocalPathOrder)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));

      std::vector<Eigen::Vector3d> initial_path{
          point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0), point(4.0, 2.0, 0.0), point(4.0, 0.0, 0.0)};
      profile.applyToInitialPath(initial_path, 0.0, 0.0, 0.0, 4.0);

      EXPECT_NEAR(2.0, initial_path[1].z(), 1e-9);
      EXPECT_NEAR(4.0, initial_path[2].z(), 1e-9);
      EXPECT_NEAR(4.0, initial_path[3].z(), 1e-9);
    }

    TEST(ReferencePathZProfile, KeepsBoundaryHeights)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));

      std::vector<Eigen::Vector3d> initial_path{
          point(1.0, 0.0, 0.0), point(5.0, 0.0, 0.0), point(3.0, 0.0, 0.0), point(7.0, 0.0, 0.0)};
      profile.applyToInitialPath(initial_path, 1.0, 0.0, 1.2, 6.8);

      EXPECT_NEAR(1.2, initial_path.front().z(), 1e-9);
      EXPECT_NEAR(5.0, initial_path[1].z(), 1e-9);
      EXPECT_NEAR(5.0, initial_path[2].z(), 1e-9);
      EXPECT_NEAR(6.8, initial_path.back().z(), 1e-9);
    }

    TEST(ReferencePathZProfile, UsesLatestHeightAtZeroLengthXYSegment)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(0.0, 0.0, 2.0),
                                   point(4.0, 0.0, 6.0)}));

      EXPECT_NEAR(2.0, profile.sampleZ(0.0), 1e-9);
      EXPECT_NEAR(4.0, profile.sampleZ(2.0), 1e-9);
    }

    TEST(ReferencePathZProfile, ProjectsStartWithoutMovingBackward)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 0.0)}));

      EXPECT_NEAR(3.0, profile.projectProgress(point(1.0, 0.0, 0.0), 3.0, 7.0), 1e-9);
      EXPECT_NEAR(7.0, profile.projectProgress(point(9.0, 0.0, 0.0), 3.0, 7.0), 1e-9);
    }

    TEST(ReferencePathZProfile, ProjectionProgressNeverMovesBackward)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(5.0, 0.0, 1.0),
                                   point(5.0, 5.0, 2.0), point(10.0, 5.0, 3.0)}));

      double previous = 0.0;
      for (const Eigen::Vector3d &query : {point(1.0, 0.2, 0.0), point(5.0, 1.0, 0.0),
                                           point(8.0, 5.1, 0.0)})
      {
        const double current = profile.projectProgress(query, previous,
                                                       profile.totalProgress());
        EXPECT_GE(current, previous);
        previous = current;
      }
    }

    TEST(ReferencePathZProfile, ProjectionDoesNotDependOnOldHeight)
    {
      ReferencePathZProfile ascending;
      ASSERT_TRUE(ascending.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));
      const double low_query = ascending.projectProgress(
          point(5.0, 0.0, -100.0), 0.0, ascending.totalProgress());
      const double high_query = ascending.projectProgress(
          point(5.0, 0.0, 100.0), 0.0, ascending.totalProgress());
      EXPECT_NEAR(5.0, low_query, 1e-9);
      EXPECT_NEAR(low_query, high_query, 1e-9);
    }

    TEST(ReferencePathZProfile, DoesNotJumpToLaterSelfIntersection)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 1.0),
                                   point(10.0, 10.0, 2.0), point(0.0, 10.0, 3.0),
                                   point(0.0, 0.0, 4.0)}));

      const double progress = profile.projectProgress(
          point(0.0, 9.8, 0.0), 0.0, 5.0);
      EXPECT_LE(progress, 5.0);
      EXPECT_NEAR(0.0, profile.sampleZ(progress), 0.6);
    }

    TEST(ReferencePathZProfile, DoesNotCrossToParallelRouteAtDifferentHeight)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 0.0),
                                   point(10.0, 1.0, 10.0), point(0.0, 1.0, 10.0)}));

      const double progress = profile.projectProgress(
          point(2.0, 1.0, 100.0), 0.0, 4.0);
      EXPECT_NEAR(2.0, progress, 1e-9);
      EXPECT_NEAR(0.0, profile.sampleZ(progress), 1e-9);
    }

    TEST(ReferencePathZProfile, WalksAcrossConsecutiveShortSegments)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(1.0, 0.0, 1.0),
                                   point(2.0, 0.0, 2.0), point(3.0, 0.0, 3.0),
                                   point(4.0, 0.0, 4.0)}));

      EXPECT_NEAR(3.5, profile.projectProgress(point(3.5, 0.0, 0.0), 0.0, 3.5),
                  1e-9);
    }

    TEST(ReferencePathZProfile, ReportsProfileEndpointBeforeBoundaryOverride)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));
      std::vector<Eigen::Vector3d> initial_path{
          point(0.0, 0.0, 0.0), point(2.0, 0.0, 0.0), point(4.0, 0.0, 0.0)};

      const ReferencePathZApplyResult result = profile.applyToInitialPath(
          initial_path, 0.0, 0.5, 0.0, 8.0);
      EXPECT_NEAR(4.0, result.final_progress, 1e-9);
      EXPECT_NEAR(4.0, result.final_profile_z, 1e-9);
      EXPECT_NEAR(8.0, initial_path.back().z(), 1e-9);
    }
  } // namespace
} // namespace scan_planner
