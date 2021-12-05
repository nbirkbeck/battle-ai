#include "src/geometry/axis_aligned_box.h"
#include "src/geometry/plane.h"

#include "gtest/gtest.h"

TEST(PlaneTest, IntersectionMisses) {
  Plane plane = {nacb::Vec3d(0, 1, 0), -0.5};
  double t = 0;
  ASSERT_FALSE(
      plane.IntersectRay(nacb::Vec3d(0, 0, 0), nacb::Vec3d(1, 0, 0), &t));
}

TEST(PlaneTest, IntersectionHits) {
  Plane plane = {nacb::Vec3d(0, 1, 0), -0.5};
  double t = 0;
  ASSERT_TRUE(
      plane.IntersectRay(nacb::Vec3d(2, 0, 3), nacb::Vec3d(0, 1, 0), &t));
  ASSERT_NEAR(0.5, t, 1e-10);
}

TEST(AxisAlignedBoxTest, IntersectRay) {
  AxisAlignedBox box({10, 0, 0}, {2, 1, 4});

  double t = 0;
  EXPECT_TRUE(box.IntersectRay({0, 0, 0}, {1, 0, 0}, &t));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
