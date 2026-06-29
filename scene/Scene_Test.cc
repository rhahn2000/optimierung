#include "Scene.h"
#include "gtest/gtest.h"

namespace
{
    constexpr float kEps = 1e-4f;

    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------

    /**
     * @brief creates a triangle in xy plane
     * @return created triangle
     */
    Triangle3df make_triangle_z0()
    {
        return Triangle3df(
            Vector3df{-1.0f, -1.0f, 0.0f},
            Vector3df{1.0f, -1.0f, 0.0f},
            Vector3df{0.0f, 1.0f, 0.0f});
    }

    /**
     * @brief creates a triangle
     * @return created triangle
     */
    Triangle3df make_triangle_z_minus2()
    {
        return Triangle3df(
            Vector3df{-1.0f, -1.0f, -2.0f},
            Vector3df{1.0f, -1.0f, -2.0f},
            Vector3df{0.0f, 1.0f, -2.0f});
    }

    /**
     * @brief creates a red material
     * @return created material
     */
    Material make_red_material()
    {
        return Material({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
    }

    /**
     * @brief creates a blue material
     * @return created material
     */
    Material make_blue_material()
    {
        return Material({0.0f, 0.0f, 1.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
    }

    /**
     * @brief creates a light
     * @return created light
     */
    Light make_light()
    {
        return Light(Vector3df{0.0f, 5.0f, 5.0f}, Vector3df{1.0f, 1.0f, 1.0f});
    }

    /**
     * @brief creates a ray hitting the center
     * @return created ray
     */
    Ray3df make_ray_hitting_center()
    {
        return Ray3df{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
    }

    /**
     * @brief creates a ray missing the triangles
     * @return created ray
     */
    Ray3df make_ray_missing()
    {
        return Ray3df{{5.0f, 5.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
    }

    /**
     * @class SceneTest
     * @brief creates a scene for testing
     */
    class SceneTest : public ::testing::Test
    {
    protected:
        Scene scene;
    };

    // ================================================================
    // 1. lights
    // ================================================================

    // tests whether an empty scene has no lights
    TEST_F(SceneTest, EmptySceneHasNoLights)
    {
        EXPECT_EQ(0u, scene.getLights().size());
    }

    // tests whether adding a light increases the amount of lights in a scene
    TEST_F(SceneTest, AddLightIncreasesLightCount)
    {
        Light light = make_light();
        scene.addLight(light);
        EXPECT_EQ(1u, scene.getLights().size());
    }

    // tests whether multiple lights are added correctly
    TEST_F(SceneTest, AddMultipleLights)
    {
        Light l1 = make_light();
        Light l2 = make_light();
        scene.addLight(l1);
        scene.addLight(l2);
        EXPECT_EQ(2u, scene.getLights().size());
    }

    // ================================================================
    // 2. material
    // ================================================================

    // tests whether the returned material has the correct color
    TEST_F(SceneTest, GetMaterialReturnsCorrectColor)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        const Material &result = scene.getMaterial(0);
        EXPECT_NEAR(1.0f, result.color[0], kEps);
        EXPECT_NEAR(0.0f, result.color[1], kEps);
        EXPECT_NEAR(0.0f, result.color[2], kEps);
    }

    // tests whether the returned material has the correct properties
    TEST_F(SceneTest, GetMaterialReturnsCorrectProperties)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat({0.5f, 0.5f, 0.5f}, 0.3f, 0.8f, 1.5f, 0.2f);
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        const Material &result = scene.getMaterial(0);
        EXPECT_NEAR(0.3f, result.reflectivity, kEps);
        EXPECT_NEAR(0.8f, result.shininess, kEps);
        EXPECT_NEAR(1.5f, result.refraction_index, kEps);
        EXPECT_NEAR(0.2f, result.transparency, kEps);
    }

    // ================================================================
    // 3. intersect (badouel)
    // ================================================================

    // tests whether using intersect on an empty scene returns false
    TEST_F(SceneTest, IntersectEmptySceneReturnsFalse)
    {
        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether the intersect hits a triangle correctly
    TEST_F(SceneTest, IntersectHitsTriangle)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_TRUE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether a ray not aimed at triangle misses
    TEST_F(SceneTest, IntersectMissesTriangle)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        Ray3df ray = make_ray_missing();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether intersect returns correct t
    TEST_F(SceneTest, IntersectReturnsCorrectT)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));
        EXPECT_NEAR(1.0f, ctx.t, kEps);
    }

    // tests whether intersects sets matindex
    TEST_F(SceneTest, IntersectSetsMatIndex)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));
        EXPECT_GE(mat_index, 0);
    }

    // tests whether only closest triangle is hit
    TEST_F(SceneTest, IntersectReturnsNearestTriangle)
    {
        Triangle3df front = make_triangle_z0();      // t=1
        Triangle3df back = make_triangle_z_minus2(); // t=3
        Material mat_front = make_red_material();
        Material mat_back = make_blue_material();
        scene.addTriangle(front, mat_front);
        scene.addTriangle(back, mat_back);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));
        EXPECT_NEAR(1.0f, ctx.t, kEps);
    }

    // tests whether the normal is flipped when ray hits triangle from behind
    TEST_F(SceneTest, IntersectBackfaceNormalPointsTowardRay)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        scene.addTriangle(tri, mat);
        scene.build_kd_tree();

        // ray comes from behind (negative z) pointing in +z direction
        Ray3df ray{{0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));

        // normal must point against the ray direction after flip
        float dot = ctx.normal * ray.direction;
        EXPECT_LT(dot, 0.0f);
    }

    // ================================================================
    // 4. intersect (moeller trumbore)
    // ================================================================

    // tests whether using intersect on an empty scene returns false
    TEST_F(SceneTest, IntersectMtEmptySceneReturnsFalse)
    {
        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect_mt(ray, ctx, mat_index));
    }

    // tests whether the intersect hits a triangle correctly
    TEST_F(SceneTest, IntersectMtHitsTriangle)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        Vector3df a{-1.0f, -1.0f, 0.0f}, b{1.0f, -1.0f, 0.0f}, c{0.0f, 1.0f, 0.0f};
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_TRUE(scene.intersect_mt(ray, ctx, mat_index));
    }

    // tests whether a ray not aimed at triangle misses
    TEST_F(SceneTest, IntersectMtMissesTriangle)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        Vector3df a{-1.0f, -1.0f, 0.0f}, b{1.0f, -1.0f, 0.0f}, c{0.0f, 1.0f, 0.0f};
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();

        Ray3df ray = make_ray_missing();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect_mt(ray, ctx, mat_index));
    }

    // tests whether intersect returns correct t
    TEST_F(SceneTest, IntersectMtReturnsCorrectT)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        Vector3df a{-1.0f, -1.0f, 0.0f}, b{1.0f, -1.0f, 0.0f}, c{0.0f, 1.0f, 0.0f};
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect_mt(ray, ctx, mat_index));
        EXPECT_NEAR(1.0f, ctx.t, kEps);
    }

    // tests whether only closest triangle is hit
    TEST_F(SceneTest, IntersectMtReturnsNearestTriangle)
    {
        Triangle3df front = make_triangle_z0();
        Triangle3df back = make_triangle_z_minus2();
        Material mat_front = make_red_material();
        Material mat_back = make_blue_material();
        Vector3df af{-1.0f, -1.0f, 0.0f}, bf{1.0f, -1.0f, 0.0f}, cf{0.0f, 1.0f, 0.0f};
        Vector3df ab{-1.0f, -1.0f, -2.0f}, bb{1.0f, -1.0f, -2.0f}, cb{0.0f, 1.0f, -2.0f};
        scene.addTriangle(front, mat_front, af, bf, cf);
        scene.addTriangle(back, mat_back, ab, bb, cb);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect_mt(ray, ctx, mat_index));
        EXPECT_NEAR(1.0f, ctx.t, kEps);
    }

    // tests whether b and mt return the same t
    TEST_F(SceneTest, IntersectAndIntersectMtAgree)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        Vector3df a{-1.0f, -1.0f, 0.0f}, b{1.0f, -1.0f, 0.0f}, c{0.0f, 1.0f, 0.0f};
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();

        Ray3df ray = make_ray_hitting_center();
        Intersection_Context<float, 3> ctx1, ctx2;
        int idx1 = -1, idx2 = -1;

        bool hit1 = scene.intersect(ray, ctx1, idx1);
        bool hit2 = scene.intersect_mt(ray, ctx2, idx2);

        EXPECT_EQ(hit1, hit2);
        if (hit1 && hit2)
        {
            EXPECT_NEAR(ctx1.t, ctx2.t, kEps);
        }
    }

    // tests whether an intersection from the backside of the triangle is calculated correctly
    TEST_F(SceneTest, IntersectMtBackfaceNormalPointsTowardRay)
    {
        Triangle3df tri = make_triangle_z0();
        Material mat = make_red_material();
        Vector3df a{-1.0f, -1.0f, 0.0f}, b{1.0f, -1.0f, 0.0f}, c{0.0f, 1.0f, 0.0f};
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();

        Ray3df ray{{0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect_mt(ray, ctx, mat_index));

        float dot = ctx.normal * ray.direction;
        EXPECT_LT(dot, 0.0f);
    }

}