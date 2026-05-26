#include <gtest/gtest.h>
#include "Camera.h"

// ----------------------------------------------------------------
// 0. setup for testing
// ----------------------------------------------------------------

static constexpr float kEps = 1e-4f;

/**
 * @class CameraFixture
 * @brief test fixture for camera that provides a default setup
 */
class CameraFixture : public ::testing::Test
{
protected:
    Vector3df position{0.0f, 0.0f, 0.0f};
    Vector3df target{0.0f, 0.0f, -1.0f};
    Vector3df vup{0.0f, 1.0f, 0.0f};
    float vangle = 90.0f;
    float aspectRatio = 1.0f;

    Camera make_camera()
    {
        return Camera(position, target, vup, vangle, aspectRatio);
    }
};

// ================================================================
// 1. constructor tests
// ================================================================

// tests whether the camera is created successfully without throwing an error
TEST_F(CameraFixture, ConstructorDoesNotThrow)
{
    EXPECT_NO_THROW(make_camera());
}

// tests whether the camera is also created successfully with different aspect ratio
TEST_F(CameraFixture, ConstructorWideAspectRatio)
{
    Camera cam_square = make_camera();
    aspectRatio = 16.0f / 9.0f;
    Camera cam_wide = make_camera();

    Ray3df ray_square = cam_square.get_ray(0.0f, 0.5f);
    Ray3df ray_wide = cam_wide.get_ray(0.0f, 0.5f);

    // wider ratio -> higher x diversion on the left side
    EXPECT_GT(std::abs(ray_wide.direction[0]), std::abs(ray_square.direction[0]));
}

// tests whether the camera is also created successfully with smaller fov
TEST_F(CameraFixture, ConstructorSmallFOV)
{
    vangle = 10.0f;
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 0.5f);

    float len = ray.direction.length();
    EXPECT_NEAR(ray.direction[0] / len, 0.0f, kEps);
    EXPECT_NEAR(ray.direction[1] / len, 0.0f, kEps);
    EXPECT_NEAR(ray.direction[2] / len, -1.0f, kEps);
}

// tests whether the camera is also created successfully with other position
TEST_F(CameraFixture, ConstructorOffOrigin)
{
    position = Vector3df{1.0f, 2.0f, 3.0f};
    target = Vector3df{1.0f, 2.0f, 2.0f};
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 0.5f);

    EXPECT_NEAR(ray.origin[0], 1.0f, kEps);
    EXPECT_NEAR(ray.origin[1], 2.0f, kEps);
    EXPECT_NEAR(ray.origin[2], 3.0f, kEps);
}

// ================================================================
// 2. get_ray - origin of ray
// ================================================================

// tests whether every ray origin equals to the camera position
TEST_F(CameraFixture, RayOriginEqualsPosition)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 0.5f);

    EXPECT_NEAR(ray.origin[0], position[0], kEps);
    EXPECT_NEAR(ray.origin[1], position[1], kEps);
    EXPECT_NEAR(ray.origin[2], position[2], kEps);
}

// tests whether every ray origin equals to the camera position even if not centered
TEST_F(CameraFixture, RayOriginEqualsPositionOffCenter)
{
    position = Vector3df{1.0f, 2.0f, 3.0f};
    target = Vector3df{1.0f, 2.0f, 2.0f};
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.0f, 0.0f);

    EXPECT_NEAR(ray.origin[0], 1.0f, kEps);
    EXPECT_NEAR(ray.origin[1], 2.0f, kEps);
    EXPECT_NEAR(ray.origin[2], 3.0f, kEps);
}

// ================================================================
// 3. get_ray – direction of ray
// ================================================================

// tests whether a central ray points towards target
TEST_F(CameraFixture, CenterRayPointsTowardsTarget)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 0.5f);

    float len = ray.direction.length();

    EXPECT_NEAR(ray.direction[0] / len, 0.0f, kEps);
    EXPECT_NEAR(ray.direction[1] / len, 0.0f, kEps);
    EXPECT_NEAR(ray.direction[2] / len, -1.0f, kEps);
}

// tests whether the left edge of a ray points left (negative x component)
TEST_F(CameraFixture, LeftEdgeRayPointsLeft)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.0f, 0.5f);
    EXPECT_LT(ray.direction[0], 0.0f);
}

// tests whether the right edge of a ray points right (positive x component)
TEST_F(CameraFixture, RightEdgeRayPointsRight)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(1.0f, 0.5f);
    EXPECT_GT(ray.direction[0], 0.0f);
}

// tests whether the bottom edge of a ray points down (negative y component)
TEST_F(CameraFixture, BottomEdgeRayPointsDown)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 0.0f);
    EXPECT_LT(ray.direction[1], 0.0f);
}

// tests whether the upper edge of a ray points up (positive y component)
TEST_F(CameraFixture, TopEdgeRayPointsUp)
{
    Camera cam = make_camera();
    Ray3df ray = cam.get_ray(0.5f, 1.0f);
    EXPECT_GT(ray.direction[1], 0.0f);
}

// ================================================================
// 4. symmetry
// ================================================================

// tests whether rays that are symmetric to the center have a mirrored x component
TEST_F(CameraFixture, HorizontalSymmetry)
{
    Camera cam = make_camera();
    Ray3df left = cam.get_ray(0.2f, 0.5f);
    Ray3df right = cam.get_ray(0.8f, 0.5f);

    EXPECT_NEAR(left.direction[0], -right.direction[0], kEps);
    EXPECT_NEAR(left.direction[1], right.direction[1], kEps);
}

// tests whether rays that are symmetric to the center have a mirrored y component
TEST_F(CameraFixture, VerticalSymmetry)
{
    Camera cam = make_camera();
    Ray3df bottom = cam.get_ray(0.5f, 0.2f);
    Ray3df top = cam.get_ray(0.5f, 0.8f);

    EXPECT_NEAR(bottom.direction[0], top.direction[0], kEps);
    EXPECT_NEAR(bottom.direction[1], -top.direction[1], kEps);
}

// ================================================================
// 5. fov
// ================================================================

// tests whether a wider fov produces wider rays
TEST_F(CameraFixture, WiderFOVProducesWiderRays)
{
    Camera cam_narrow = make_camera();
    vangle = 120.0f;
    Camera cam_wide = make_camera();

    Ray3df ray_narrow = cam_narrow.get_ray(0.0f, 0.5f);
    Ray3df ray_wide = cam_wide.get_ray(0.0f, 0.5f);

    float ratio_narrow = std::abs(ray_narrow.direction[0] / ray_narrow.direction[2]);
    float ratio_wide = std::abs(ray_wide.direction[0] / ray_wide.direction[2]);

    EXPECT_GT(ratio_wide, ratio_narrow);
}
