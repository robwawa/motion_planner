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

    TEST(ReferencePathZProfile, UsesProjectedProgressInsteadOfDetourLength)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));

      std::vector<Eigen::Vector3d> initial_path{
          point(0.0, 0.0, 0.0), point(2.0, 2.0, 0.0), point(4.0, 2.0, 0.0), point(4.0, 0.0, 0.0)};
      profile.applyToInitialPath(initial_path, 0.0, 4.0, 0.0, 4.0);

      EXPECT_NEAR(2.0, initial_path[1].z(), 1e-9);
      EXPECT_NEAR(4.0, initial_path[2].z(), 1e-9);
      EXPECT_NEAR(4.0, initial_path[3].z(), 1e-9);
    }

    TEST(ReferencePathZProfile, KeepsBoundaryHeightsAndMonotonicProgress)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));

      std::vector<Eigen::Vector3d> initial_path{
          point(1.0, 0.0, 0.0), point(5.0, 0.0, 0.0), point(3.0, 0.0, 0.0), point(7.0, 0.0, 0.0)};
      profile.applyToInitialPath(initial_path, 1.0, 7.0, 1.2, 6.8);

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

    TEST(ReferencePathZProfile, ClampsProjectionToRequestedWindow)
    {
      ReferencePathZProfile profile;
      ASSERT_TRUE(profile.setPath({point(0.0, 0.0, 0.0), point(10.0, 0.0, 10.0)}));

      EXPECT_NEAR(3.0, profile.projectProgress(point(1.0, 0.0, 0.0), 3.0, 7.0), 1e-9);
      EXPECT_NEAR(7.0, profile.projectProgress(point(9.0, 0.0, 0.0), 3.0, 7.0), 1e-9);
    }
  } // namespace
} // namespace scan_planner
