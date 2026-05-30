#include <gtest/gtest.h>
#include "../core/KDTree.h"
#include "../core/AABB.h"
#include "../geometry/geometry.h"
#include "../math/math.h"
#include <vector>

namespace
{
    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------
    constexpr float kEps = 1e-4f;

    /**
     * @brief Fixture for tests that need a single triangle.
     *
     * The triangle data lives as member variables so that the KDTree's
     * internal raw pointers remain valid for the lifetime of the test.
     */
    class KDTreeSingleTriangleTest : public ::testing::Test
    {
    protected:
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        KDTree                   tree;

        void SetUp() override
        {
            Vector3df a{-1.0f, -1.0f, -3.0f};
            Vector3df b{ 1.0f, -1.0f, -3.0f};
            Vector3df c{ 0.0f,  1.0f, -3.0f};
            triangles.emplace_back(a, b, c);
            va.push_back(a);
            vb.push_back(b);
            vc.push_back(c);
            tree.build(triangles, va, vb, vc);
        }
    };

    /**
     * @brief Fixture for tests that need multiple triangles.
     *
     * Eight triangles spread along the -z axis at z = -2, -4, ..., -16.
     * Triangle at index 0 is the nearest (z = -2).
     */
    class KDTreeMultipleTrianglesTest : public ::testing::Test
    {
    protected:
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        KDTree                   tree;

        void SetUp() override
        {
            for (int i = 1; i <= 8; ++i)
            {
                float z = static_cast<float>(-i) * 2.0f;
                Vector3df a{-1.0f, -1.0f, z};
                Vector3df b{ 1.0f, -1.0f, z};
                Vector3df c{ 0.0f,  1.0f, z};
                triangles.emplace_back(a, b, c);
                va.push_back(a);
                vb.push_back(b);
                vc.push_back(c);
            }
            tree.build(triangles, va, vb, vc);
        }
    };

    // ================================================================
    // 1. constructor tests
    // ================================================================

    // tests whether the default constructor does not throw
    TEST(KDTreeTest, DefaultConstructorDoesNotThrow)
    {
        EXPECT_NO_THROW(KDTree());
    }

    // ================================================================
    // 2. build tests
    // ================================================================

    // tests whether build with a single triangle does not throw
    TEST(KDTreeTest, BuildSingleTriangleDoesNotThrow)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{ 1.0f, -1.0f, -3.0f};
        Vector3df c{ 0.0f,  1.0f, -3.0f};
        triangles.emplace_back(a, b, c);
        va.push_back(a); vb.push_back(b); vc.push_back(c);
        KDTree tree;
        EXPECT_NO_THROW(tree.build(triangles, va, vb, vc));
    }

    // tests whether build with multiple triangles does not throw
    TEST(KDTreeTest, BuildMultipleTrianglesDoesNotThrow)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        for (int i = 1; i <= 8; ++i)
        {
            float z = static_cast<float>(-i) * 2.0f;
            Vector3df a{-1.0f, -1.0f, z};
            Vector3df b{ 1.0f, -1.0f, z};
            Vector3df c{ 0.0f,  1.0f, z};
            triangles.emplace_back(a, b, c);
            va.push_back(a); vb.push_back(b); vc.push_back(c);
        }
        KDTree tree;
        EXPECT_NO_THROW(tree.build(triangles, va, vb, vc));
    }

    // tests whether build with an empty triangle list does not throw
    TEST(KDTreeTest, BuildEmptyTriangleListDoesNotThrow)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        KDTree tree;
        EXPECT_NO_THROW(tree.build(triangles, va, vb, vc));
    }

    // tests whether build can be called multiple times without throwing
    TEST(KDTreeTest, BuildCalledTwiceDoesNotThrow)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{ 1.0f, -1.0f, -3.0f};
        Vector3df c{ 0.0f,  1.0f, -3.0f};
        triangles.emplace_back(a, b, c);
        va.push_back(a); vb.push_back(b); vc.push_back(c);
        KDTree tree;
        tree.build(triangles, va, vb, vc);
        EXPECT_NO_THROW(tree.build(triangles, va, vb, vc));
    }

    // ================================================================
    // 3. intersect – without prior build
    // ================================================================

    // tests whether intersect on an unbuilt tree does not throw
    TEST(KDTreeTest, IntersectUnbuiltTreeDoesNotThrow)
    {
        KDTree tree;
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_NO_THROW(tree.intersect(ray, ctx, mat_index));
    }

    // tests whether intersect on an unbuilt tree returns false
    TEST(KDTreeTest, IntersectUnbuiltTreeReturnsFalse)
    {
        KDTree tree;
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_FALSE(hit);
    }

    // ================================================================
    // 4. intersect – empty scene
    // ================================================================

    // tests whether intersect on an empty tree does not throw
    TEST(KDTreeTest, IntersectEmptyTreeDoesNotThrow)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        KDTree tree;
        tree.build(triangles, va, vb, vc);
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_NO_THROW(tree.intersect(ray, ctx, mat_index));
    }

    // tests whether intersect on an empty tree returns false
    TEST(KDTreeTest, IntersectEmptyTreeReturnsFalse)
    {
        std::vector<Triangle3df> triangles;
        std::vector<Vector3df>   va, vb, vc;
        KDTree tree;
        tree.build(triangles, va, vb, vc);
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_FALSE(hit);
    }

    // ================================================================
    // 5. intersect – single triangle, hit
    // ================================================================

    // tests whether a ray aimed at the triangle reports a hit
    TEST_F(KDTreeSingleTriangleTest, IntersectSingleTriangleHit)
    {
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_TRUE(hit);
    }

    // tests whether a hit sets mat_index to a valid (non-negative) value
    TEST_F(KDTreeSingleTriangleTest, IntersectHitSetsMaterialIndex)
    {
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        tree.intersect(ray, ctx, mat_index);
        EXPECT_GE(mat_index, 0);
    }

    // ================================================================
    // 6. intersect – single triangle, miss
    // ================================================================

    // tests whether a ray that misses the triangle returns false
    TEST_F(KDTreeSingleTriangleTest, IntersectSingleTriangleMiss)
    {
        Ray3df ray{{10.0f, 10.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_FALSE(hit);
    }

    // tests whether a ray pointing away from the triangle returns false
    TEST_F(KDTreeSingleTriangleTest, IntersectRayPointingAwayReturnsFalse)
    {
        // triangle is at z=-3, ray points in +z direction away from it
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_FALSE(hit);
    }

    // ================================================================
    // 7. intersect – multiple triangles
    // ================================================================

    // tests whether a hit is detected in a tree with multiple triangles
    TEST_F(KDTreeMultipleTrianglesTest, IntersectMultipleTrianglesHit)
    {
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_TRUE(hit);
    }

    // tests whether a miss is correctly reported in a tree with multiple triangles
    TEST_F(KDTreeMultipleTrianglesTest, IntersectMultipleTrianglesMiss)
    {
        Ray3df ray{{10.0f, 10.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        bool hit = tree.intersect(ray, ctx, mat_index);
        EXPECT_FALSE(hit);
    }

    // tests whether the nearest triangle is hit (not one behind it)
    TEST_F(KDTreeMultipleTrianglesTest, IntersectMultipleTrianglesReturnsNearest)
    {
        // triangles are at z = -2, -4, -6, ... — the nearest is index 0 (z=-2)
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        tree.intersect(ray, ctx, mat_index);
        EXPECT_EQ(mat_index, 0);
    }

} // namespace