#include <gtest/gtest.h>
#include "../core/AABB.h"
#include "../geometry/geometry.h"
#include "../math/math.h"

namespace
{
    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------
    constexpr float kEps = 1e-4f;

    /**
     * Creates a default AABB for testing.
     * @brief Creation of test AABB
     * @return new AABB with min (-1,-1,-1) and max (1,1,1)
     */
    AABB make_unit_aabb()
    {
        Vector3df min_pt{-1.0f, -1.0f, -1.0f};
        Vector3df max_pt{ 1.0f,  1.0f,  1.0f};
        return AABB(min_pt, max_pt);
    }

    /**
     * Creates a ray pointing in the -z direction from the origin.
     * @brief Creation of test ray
     * @return ray from (0,0,5) towards (0,0,-1)
     */
    Ray3df make_ray_along_neg_z()
    {
        return Ray3df{{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
    }

    // ================================================================
    // 1. constructor tests
    // ================================================================

    // tests whether default constructor creates an AABB without throwing
    TEST(AABBTest, DefaultConstructorDoesNotThrow)
    {
        EXPECT_NO_THROW(AABB());
    }

    // tests whether parameterized constructor stores min and max points correctly
    TEST(AABBTest, ParameterizedConstructorStoresMinMax)
    {
        Vector3df min_pt{-2.0f, -3.0f, -4.0f};
        Vector3df max_pt{ 2.0f,  3.0f,  4.0f};
        AABB box(min_pt, max_pt);

        EXPECT_NEAR(box.min_pt[0], -2.0f, kEps);
        EXPECT_NEAR(box.min_pt[1], -3.0f, kEps);
        EXPECT_NEAR(box.min_pt[2], -4.0f, kEps);

        EXPECT_NEAR(box.max_pt[0],  2.0f, kEps);
        EXPECT_NEAR(box.max_pt[1],  3.0f, kEps);
        EXPECT_NEAR(box.max_pt[2],  4.0f, kEps);
    }

    // ================================================================
    // 2. grow tests
    // ================================================================

    // tests whether grow does not throw
    TEST(AABBTest, GrowDoesNotThrow)
    {
        AABB box = make_unit_aabb();
        Vector3df point{2.0f, 2.0f, 2.0f};
        EXPECT_NO_THROW(box.grow(point));
    }

    // tests whether grow extends the max boundary when given a larger point
    TEST(AABBTest, GrowExpandsMaxBoundary)
    {
        AABB box = make_unit_aabb();
        Vector3df point{3.0f, 4.0f, 5.0f};
        box.grow(point);

        EXPECT_NEAR(box.max_pt[0], 3.0f, kEps);
        EXPECT_NEAR(box.max_pt[1], 4.0f, kEps);
        EXPECT_NEAR(box.max_pt[2], 5.0f, kEps);
    }

    // tests whether grow extends the min boundary when given a smaller point
    TEST(AABBTest, GrowExpandsMinBoundary)
    {
        AABB box = make_unit_aabb();
        Vector3df point{-3.0f, -4.0f, -5.0f};
        box.grow(point);

        EXPECT_NEAR(box.min_pt[0], -3.0f, kEps);
        EXPECT_NEAR(box.min_pt[1], -4.0f, kEps);
        EXPECT_NEAR(box.min_pt[2], -5.0f, kEps);
    }

    // tests whether grow does not shrink the box when given an interior point
    TEST(AABBTest, GrowWithInteriorPointDoesNotShrinkBox)
    {
        AABB box = make_unit_aabb();
        Vector3df interior{0.0f, 0.0f, 0.0f};
        box.grow(interior);

        EXPECT_NEAR(box.min_pt[0], -1.0f, kEps);
        EXPECT_NEAR(box.min_pt[1], -1.0f, kEps);
        EXPECT_NEAR(box.min_pt[2], -1.0f, kEps);

        EXPECT_NEAR(box.max_pt[0],  1.0f, kEps);
        EXPECT_NEAR(box.max_pt[1],  1.0f, kEps);
        EXPECT_NEAR(box.max_pt[2],  1.0f, kEps);
    }

    // tests whether growing with a point on the boundary does not change the box
    TEST(AABBTest, GrowWithBoundaryPointDoesNotChangeBox)
    {
        AABB box = make_unit_aabb();
        Vector3df boundary{1.0f, 1.0f, 1.0f};
        box.grow(boundary);

        EXPECT_NEAR(box.max_pt[0], 1.0f, kEps);
        EXPECT_NEAR(box.max_pt[1], 1.0f, kEps);
        EXPECT_NEAR(box.max_pt[2], 1.0f, kEps);
    }

    // ================================================================
    // 3. from_triangle tests
    // ================================================================

    // tests whether from_triangle does not throw
    TEST(AABBTest, FromTriangleDoesNotThrow)
    {
        Vector3df a{0.0f, 0.0f, 0.0f};
        Vector3df b{1.0f, 0.0f, 0.0f};
        Vector3df c{0.0f, 1.0f, 0.0f};
        EXPECT_NO_THROW(AABB::from_triangle(a, b, c));
    }

    // tests whether from_triangle computes the correct min point
    TEST(AABBTest, FromTriangleCorrectMinPoint)
    {
        Vector3df a{-1.0f, -2.0f, -3.0f};
        Vector3df b{ 2.0f,  1.0f,  0.0f};
        Vector3df c{ 0.0f, -1.0f,  3.0f};
        AABB box = AABB::from_triangle(a, b, c);

        EXPECT_NEAR(box.min_pt[0], -1.0f, kEps);
        EXPECT_NEAR(box.min_pt[1], -2.0f, kEps);
        EXPECT_NEAR(box.min_pt[2], -3.0f, kEps);
    }

    // tests whether from_triangle computes the correct max point
    TEST(AABBTest, FromTriangleCorrectMaxPoint)
    {
        Vector3df a{-1.0f, -2.0f, -3.0f};
        Vector3df b{ 2.0f,  1.0f,  0.0f};
        Vector3df c{ 0.0f, -1.0f,  3.0f};
        AABB box = AABB::from_triangle(a, b, c);

        EXPECT_NEAR(box.max_pt[0], 2.0f, kEps);
        EXPECT_NEAR(box.max_pt[1], 1.0f, kEps);
        EXPECT_NEAR(box.max_pt[2], 3.0f, kEps);
    }

    // tests whether from_triangle handles a degenerate (collinear) triangle
    TEST(AABBTest, FromTriangleDegenerate)
    {
        // all three points on the x-axis
        Vector3df a{-1.0f, 0.0f, 0.0f};
        Vector3df b{ 0.0f, 0.0f, 0.0f};
        Vector3df c{ 1.0f, 0.0f, 0.0f};
        AABB box = AABB::from_triangle(a, b, c);

        EXPECT_NEAR(box.min_pt[0], -1.0f, kEps);
        EXPECT_NEAR(box.max_pt[0],  1.0f, kEps);

        // y and z extents should be zero (flat)
        EXPECT_NEAR(box.min_pt[1], box.max_pt[1], kEps);
        EXPECT_NEAR(box.min_pt[2], box.max_pt[2], kEps);
    }

    // ================================================================
    // 4. intersect – hitting the box
    // ================================================================

    // tests whether intersect does not throw on a valid ray
    TEST(AABBTest, IntersectDoesNotThrow)
    {
        AABB box = make_unit_aabb();
        Ray3df ray = make_ray_along_neg_z();
        float t_min, t_max;
        EXPECT_NO_THROW(box.intersect(ray, t_min, t_max));
    }

    // tests whether a ray aimed at the center of the box reports an intersection
    TEST(AABBTest, IntersectRayHitsBox)
    {
        AABB box = make_unit_aabb();
        Ray3df ray = make_ray_along_neg_z();
        float t_min, t_max;
        bool hit = box.intersect(ray, t_min, t_max);
        EXPECT_TRUE(hit);
    }

    // tests whether t_min is smaller than t_max on a valid intersection
    TEST(AABBTest, IntersectTMinSmallerThanTMax)
    {
        AABB box = make_unit_aabb();
        Ray3df ray = make_ray_along_neg_z();
        float t_min, t_max;
        box.intersect(ray, t_min, t_max);
        EXPECT_LT(t_min, t_max);
    }

    // tests whether t values are positive when the ray originates outside the box
    TEST(AABBTest, IntersectTValuesPositiveFromOutside)
    {
        AABB box = make_unit_aabb();
        Ray3df ray = make_ray_along_neg_z();   // origin at z=5, box at z in [-1,1]
        float t_min, t_max;
        box.intersect(ray, t_min, t_max);
        EXPECT_GT(t_min, 0.0f);
        EXPECT_GT(t_max, 0.0f);
    }

    // tests whether intersection is detected correctly along all three axes
    TEST(AABBTest, IntersectHitsAlongAllAxes)
    {
        AABB box = make_unit_aabb();
        float t_min, t_max;

        // along +x
        Ray3df ray_x{{5.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}};
        EXPECT_TRUE(box.intersect(ray_x, t_min, t_max));

        // along +y
        Ray3df ray_y{{0.0f, 5.0f, 0.0f}, {0.0f, -1.0f, 0.0f}};
        EXPECT_TRUE(box.intersect(ray_y, t_min, t_max));

        // along +z
        Ray3df ray_z{{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
        EXPECT_TRUE(box.intersect(ray_z, t_min, t_max));
    }

    // ================================================================
    // 5. intersect – missing the box
    // ================================================================

    // tests whether a ray that clearly misses the box returns false
    TEST(AABBTest, IntersectRayMissesBox)
    {
        AABB box = make_unit_aabb();
        // ray travels parallel to the box, well outside in x
        Ray3df ray{{5.0f, 0.0f, 5.0f}, {0.0f, 0.0f, -1.0f}};
        float t_min, t_max;
        bool hit = box.intersect(ray, t_min, t_max);
        EXPECT_FALSE(hit);
    }

    // tests whether a ray pointing away from the box returns false
    TEST(AABBTest, IntersectRayPointingAwayFromBox)
    {
        AABB box = make_unit_aabb();
        // ray starts in front of the box and points further away
        Ray3df ray{{0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 1.0f}};
        float t_min, t_max;
        bool hit = box.intersect(ray, t_min, t_max);
        EXPECT_FALSE(hit);
    }

    // ================================================================
    // 6. intersect – ray origin inside the box
    // ================================================================

    // tests whether a ray originating inside the box still reports an intersection
    TEST(AABBTest, IntersectRayOriginInsideBox)
    {
        AABB box = make_unit_aabb();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        float t_min, t_max;
        bool hit = box.intersect(ray, t_min, t_max);
        EXPECT_TRUE(hit);
    }

    // tests whether t_max is positive when the ray origin is inside the box
    TEST(AABBTest, IntersectInsideBoxTMaxIsPositive)
    {
        AABB box = make_unit_aabb();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        float t_min, t_max;
        box.intersect(ray, t_min, t_max);
        EXPECT_GT(t_max, 0.0f);
    }

} // namespace