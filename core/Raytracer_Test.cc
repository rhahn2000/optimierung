#include <gtest/gtest.h>
#include "Raytracer.h"
#include "Camera.h"
#include "../scene/Scene.h"
#include "../scene/Material.h"
#include "../scene/Light.h"
#include "../math/math.h"

namespace
{
    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------
    constexpr float kEps = 1e-4f;

    /**
     * Creates a default camera for testing.
     * @brief Creation of test camera
     * @return new camera
     */
    Camera make_default_camera()
    {
        Vector3df pos{0.0f, 0.0f, 0.0f};
        Vector3df target{0.0f, 0.0f, -1.0f};
        Vector3df vup{0.0f, 1.0f, 0.0f};
        return Camera(pos, target, vup, 90.0f, 1.0f);
    }

    /**
     * Creates an empty scene for testing.
     * @brief Creation of test scene
     * @return empty scene
     */
    Scene make_empty_scene()
    {
        return Scene();
    }

    /**
     * Fills the given scene with a single triangle and a light for testing.
     * @brief Fills test scene
     * @param scene the scene to fill with a triangle and a light
     */
    void make_scene_with_triangle(Scene& scene)
    {
        // create triangle
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{1.0f, -1.0f, -3.0f};
        Vector3df c{0.0f, 1.0f, -3.0f};
        Triangle3df tri(a, b, c);
        // add triangle to scene
        Material mat({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
        scene.addTriangle(tri, mat, a, b, c);
        // add light to scene
        Vector3df light_pos{0.0f, 5.0f, 0.0f};
        Vector3df light_int{1.0f, 1.0f, 1.0f};
        Light light(light_pos, light_int);
        scene.addLight(light);
    }

    // ================================================================
    // 1. framebuffer tests
    // ================================================================

    // tests whether the create framebuffer of raytracer matches expected size
    TEST(RaytracerTest, FramebufferSizeMatchesConstructorDimensions)
    {
        int w = 10, h = 10;
        Raytracer rt(w, h, 1);
        EXPECT_EQ(rt.getFramebuffer().size(), static_cast<size_t>(w * h));
    }

    // tests whether the framebuffer of raytracer matches expected size after render
    TEST(RaytracerTest, FramebufferSizeAfterRender)
    {
        int w = 10, h = 10;
        Raytracer rt(w, h, 1);
        Scene s1 = make_empty_scene();
        rt.render(make_default_camera(), s1);
        EXPECT_EQ(rt.getFramebuffer().size(), static_cast<size_t>(w * h));
    }

    // tests whether the framebuffer of raytracer matches expected size if rectangular image
    TEST(RaytracerTest, FramebufferSizeRectangular)
    {
        int w = 16, h = 9;
        Raytracer rt(w, h, 1);
        Scene s2 = make_empty_scene();
        rt.render(make_default_camera(), s2);
        EXPECT_EQ(rt.getFramebuffer().size(), static_cast<size_t>(w * h));
    }

    // ================================================================
    // 2. trace – empty scene
    // ================================================================

    // tests whether tracing an empty scene does not throw an error
    TEST(RaytracerTest, TraceEmptySceneDoesNotThrow)
    {
        Raytracer rt(10, 10, 3);
        Scene scene = make_empty_scene();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        EXPECT_NO_THROW(rt.trace(ray, scene, 0));
    }

    // tests whether tracing an empty scene returns the background color
    TEST(RaytracerTest, TraceEmptySceneReturnsBackground)
    {
        Raytracer rt(10, 10, 3);
        Scene scene = make_empty_scene();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color = rt.trace(ray, scene, 0);
        EXPECT_NEAR(0.2f, color[0], kEps);
        EXPECT_NEAR(0.2f, color[1], kEps);
        EXPECT_NEAR(0.2f, color[2], kEps);
    }

    // ================================================================
    // 3. trace – scene with triangle
    // ================================================================

    // tests if the color differs when hitting or missing during tracing
    TEST(RaytracerTest, TraceHitDiffersFromMiss)
    {
        Raytracer rt(10, 10, 3);
        Scene scene;
        make_scene_with_triangle(scene);
        scene.build_kd_tree();

        Ray3df ray_hit{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Ray3df ray_miss{{0.0f, 0.0f, 0.0f}, {10.0f, 10.0f, -1.0f}};

        Vector3df color_hit = rt.trace(ray_hit, scene, 3);
        Vector3df color_miss = rt.trace(ray_miss, scene, 3);

        bool differs = (std::abs(color_hit[0] - color_miss[0]) > kEps ||
                        std::abs(color_hit[1] - color_miss[1]) > kEps ||
                        std::abs(color_hit[2] - color_miss[2]) > kEps);
        EXPECT_TRUE(differs);
    }

    // tests if tracing returns a non negative color
    TEST(RaytracerTest, TraceHitNonNegativeColor)
    {
        Raytracer rt(10, 10, 3);
        Scene scene;
        make_scene_with_triangle(scene);
        scene.build_kd_tree();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color = rt.trace(ray, scene, 0);
        EXPECT_GE(color[0], 0.0f);
        EXPECT_GE(color[1], 0.0f);
        EXPECT_GE(color[2], 0.0f);
    }

    // ================================================================
    // 4. 0 depth for recursion
    // ================================================================

    // tests whether tracing with 0 depth/recursion throws no error
    TEST(RaytracerTest, TraceDepthZeroDoesNotThrow)
    {
        Raytracer rt(10, 10, 5);
        Scene scene;
        make_scene_with_triangle(scene);
        scene.build_kd_tree();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        EXPECT_NO_THROW(rt.trace(ray, scene, 0));
    }

    // tests whether tracing with 0 depth/recursion returns background
    TEST(RaytracerTest, TraceDepthZeroReturnsBackground)
    {
        Raytracer rt(10, 10, 5);
        Scene scene;
        make_scene_with_triangle(scene);
        scene.build_kd_tree();
        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color = rt.trace(ray, scene, 0);

        EXPECT_NEAR(0.2f, color[0], kEps);
        EXPECT_NEAR(0.2f, color[1], kEps);
        EXPECT_NEAR(0.2f, color[2], kEps);
    }

    // ================================================================
    // 5. reflection
    // ================================================================

    // tests whether reflective material returns different color than same material without reflection
    TEST(RaytracerTest, TraceReflectiveMaterialDiffersFromMatte)
    {
        Raytracer rt(10, 10, 3);

        Scene scene_matte;
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{1.0f, -1.0f, -3.0f};
        Vector3df c{0.0f, 1.0f, -3.0f};
        Triangle3df tri_matte(a, b, c);
        Material mat_matte({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
        scene_matte.addTriangle(tri_matte, mat_matte, a, b, c);
        scene_matte.build_kd_tree();
        Light light({0.0f, 5.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
        scene_matte.addLight(light);

        Scene scene_refl;
        Triangle3df tri_refl(a, b, c);
        Material mat_refl({1.0f, 0.0f, 0.0f}, 0.8f, 0.0f, 1.0f, 0.0f);
        scene_refl.addTriangle(tri_refl, mat_refl, a, b, c);
        scene_refl.build_kd_tree();
        scene_refl.addLight(light);

        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color_matte = rt.trace(ray, scene_matte, 3);
        Vector3df color_refl = rt.trace(ray, scene_refl, 3);

        bool differs = (std::abs(color_matte[0] - color_refl[0]) > kEps ||
                        std::abs(color_matte[1] - color_refl[1]) > kEps ||
                        std::abs(color_matte[2] - color_refl[2]) > kEps);
        EXPECT_TRUE(differs);
    }

    // tests whether a reflective material does not return a negative color
    TEST(RaytracerTest, TraceReflectiveMaterialNonNegativeColor)
    {
        Raytracer rt(10, 10, 5);
        Scene scene;
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{1.0f, -1.0f, -3.0f};
        Vector3df c{0.0f, 1.0f, -3.0f};
        Triangle3df tri(a, b, c);
        Material mat({1.0f, 0.0f, 0.0f}, 0.8f, 0.0f, 1.0f, 0.0f);
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();
        scene.addLight(Light({0.0f, 5.0f, 0.0f}, {1.0f, 1.0f, 1.0f}));

        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color = rt.trace(ray, scene, 5);
        EXPECT_GE(color[0], 0.0f);
        EXPECT_GE(color[1], 0.0f);
        EXPECT_GE(color[2], 0.0f);
    }

    // ================================================================
    // 6. transparancy / transmission
    // ================================================================

    // tests whether transparent material returns different color than same opaque material
    TEST(RaytracerTest, TraceTransparentMaterialDiffersFromOpaque)
    {
        Raytracer rt(10, 10, 5);

        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{1.0f, -1.0f, -3.0f};
        Vector3df c{0.0f, 1.0f, -3.0f};
        Light light({0.0f, 5.0f, 0.0f}, {1.0f, 1.0f, 1.0f});

        Scene scene_opaque;
        Triangle3df tri_opaque(a, b, c);
        Material mat_opaque({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
        scene_opaque.addTriangle(tri_opaque, mat_opaque, a, b, c);
        scene_opaque.build_kd_tree();
        scene_opaque.addLight(light);

        Scene scene_transp;
        Triangle3df tri_transp(a, b, c);
        Material mat_transp({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.5f, 0.8f);
        scene_transp.addTriangle(tri_transp, mat_transp, a, b, c);
        scene_transp.build_kd_tree();
        scene_transp.addLight(light);

        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color_opaque = rt.trace(ray, scene_opaque, 5);
        Vector3df color_transp = rt.trace(ray, scene_transp, 5);

        bool differs = (std::abs(color_opaque[0] - color_transp[0]) > kEps ||
                        std::abs(color_opaque[1] - color_transp[1]) > kEps ||
                        std::abs(color_opaque[2] - color_transp[2]) > kEps);
        EXPECT_TRUE(differs);
    }

    // tests whether a transparent material does not return a negative color
    TEST(RaytracerTest, TraceTransparentMaterialNonNegativeColor)
    {
        Raytracer rt(10, 10, 5);
        Scene scene;
        Vector3df a{-1.0f, -1.0f, -3.0f};
        Vector3df b{1.0f, -1.0f, -3.0f};
        Vector3df c{0.0f, 1.0f, -3.0f};
        Triangle3df tri(a, b, c);
        Material mat({1.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.5f, 0.8f);
        scene.addTriangle(tri, mat, a, b, c);
        scene.build_kd_tree();
        scene.addLight(Light({0.0f, 5.0f, 0.0f}, {1.0f, 1.0f, 1.0f}));

        Ray3df ray{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color = rt.trace(ray, scene, 5);
        EXPECT_GE(color[0], 0.0f);
        EXPECT_GE(color[1], 0.0f);
        EXPECT_GE(color[2], 0.0f);
    }

    // ================================================================
    // 7. shadows
    // ================================================================

    // tests whether material in shadow returns different color than same material in light
    TEST(RaytracerTest, TraceShadowedTriangleDarkerThanLit)
    {
        Raytracer rt(10, 10, 5);

        Light light({0.0f, 2.0f, -3.0f}, {1.0f, 1.0f, 1.0f});

        Vector3df a2{-1.0f, -1.0f, -5.0f};
        Vector3df b2{1.0f, -1.0f, -5.0f};
        Vector3df c2{0.0f, 1.0f, -5.0f};
        Material mat_back({1.0f, 1.0f, 1.0f}, 0.0f, 0.0f, 1.0f, 0.0f);

        Scene scene_lit;
        Triangle3df tri_back_lit(a2, b2, c2);
        scene_lit.addTriangle(tri_back_lit, mat_back, a2, b2, c2);
        scene_lit.build_kd_tree();
        scene_lit.addLight(light);

        Scene scene_shadowed;
        Vector3df a1{-1.0f, -1.0f, -3.0f};
        Vector3df b1{1.0f, -1.0f, -3.0f};
        Vector3df c1{0.0f, 1.0f, -3.0f};
        Triangle3df tri_front(a1, b1, c1);
        Material mat_front({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.0f);
        Triangle3df tri_back_shadowed(a2, b2, c2);
        scene_shadowed.addTriangle(tri_front, mat_front, a1, b1, c1);
        scene_shadowed.addTriangle(tri_back_shadowed, mat_back, a2, b2, c2);
        scene_shadowed.build_kd_tree();
        scene_shadowed.addLight(light);

        Ray3df ray{{0.0f, -0.5f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        Vector3df color_lit = rt.trace(ray, scene_lit, 5);
        Vector3df color_shadowed = rt.trace(ray, scene_shadowed, 5);

        float brightness_lit = color_lit[0] + color_lit[1] + color_lit[2];
        float brightness_shadowed = color_shadowed[0] + color_shadowed[1] + color_shadowed[2];
        EXPECT_GT(brightness_lit, brightness_shadowed);
    }

}