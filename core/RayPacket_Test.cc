#include "RayPacket.h"
#include "AABB.h"
#include "gtest/gtest.h"

namespace
{
    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------

    /**
     * @brief Fixture for tests that intersect a RayPacket with an AABB.
     */
    class RayPacketAABB : public ::testing::Test
    {
    protected:
        /**
         * @brief builds a RayPacket from 8 individual rays.
         * @param rays array of 8 rays to pack
         * @return the resulting RayPacket
         */
        RayPacket make_packet(Ray3df rays[8])
        {
            RayPacket p;
            p.set_rays(rays);
            return p;
        }
    };

    /**
     * @brief Fixture for tests that intersect a RayPacket with a triangle.
     */
    class RayPacketTriangle : public ::testing::Test
    {
    protected:
        /**
         * @brief builds a RayPacket from 8 individual rays.
         * @param rays array of 8 rays to pack
         * @return the resulting RayPacket
         */
        RayPacket make_packet(Ray3df rays[8])
        {
            RayPacket p;
            p.set_rays(rays);
            return p;
        }
    };

    // ================================================================
    // 1. set_rays tests
    // ================================================================

    // tests whether set_rays correctly stores the origin of each ray
    TEST(RAY_PACKET, SetRaysStoresOrigins)
    {
        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {float(i), float(i + 1), float(i + 2)};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p;
        p.set_rays(rays);

        float ox[8], oy[8], oz[8];
        _mm256_storeu_ps(ox, p.origin_x);
        _mm256_storeu_ps(oy, p.origin_y);
        _mm256_storeu_ps(oz, p.origin_z);

        for (int i = 0; i < 8; ++i)
        {
            EXPECT_NEAR(float(i), ox[i], 0.00001f);
            EXPECT_NEAR(float(i + 1), oy[i], 0.00001f);
            EXPECT_NEAR(float(i + 2), oz[i], 0.00001f);
        }
    }

    // tests whether set_rays correctly precomputes the reciprocal directions
    TEST(RAY_PACKET, SetRaysStoresReciprocalDirections)
    {
        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.0f, 0.0f, 0.0f};
            rays[i].direction = {2.0f, 4.0f, 0.5f};
        }

        RayPacket p;
        p.set_rays(rays);

        float rcp_dx[8], rcp_dy[8], rcp_dz[8];
        _mm256_storeu_ps(rcp_dx, p.rp_direction_x);
        _mm256_storeu_ps(rcp_dy, p.rp_direction_y);
        _mm256_storeu_ps(rcp_dz, p.rp_direction_z);

        for (int i = 0; i < 8; ++i)
        {
            EXPECT_NEAR(0.5f, rcp_dx[i], 0.00001f);
            EXPECT_NEAR(0.25f, rcp_dy[i], 0.00001f);
            EXPECT_NEAR(2.0f, rcp_dz[i], 0.00001f);
        }
    }

    // ================================================================
    // 2. intersect_aabb tests
    // ================================================================

    // tests whether all 8 rays report a hit when all aim at the box
    TEST_F(RayPacketAABB, AllRaysHitBox)
    {
        AABB box({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.0f, 0.0f, 5.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 t_min, t_max;
        int mask = p.intersect_aabb(box, t_min, t_max);

        EXPECT_EQ(0xFF, mask);
    }

    // tests whether the bitmask is zero when no ray hits the box
    TEST_F(RayPacketAABB, NoRayHitsBox)
    {
        AABB box({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {5.0f, 5.0f, 5.0f};
            rays[i].direction = {1.0f, 0.0f, 0.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 t_min, t_max;
        int mask = p.intersect_aabb(box, t_min, t_max);

        EXPECT_EQ(0x00, mask);
    }

    // tests whether the bitmask correctly marks only the rays that hit the box
    TEST_F(RayPacketAABB, SomeRaysHitBox)
    {
        AABB box({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }
        rays[0].origin = {0.0f, 0.0f, 5.0f};
        rays[1].origin = {0.0f, 0.0f, 5.0f};
        rays[2].origin = {0.0f, 0.0f, 5.0f};
        rays[3].origin = {0.0f, 0.0f, 5.0f};
        rays[4].origin = {5.0f, 5.0f, 5.0f};
        rays[5].origin = {5.0f, 5.0f, 5.0f};
        rays[6].origin = {5.0f, 5.0f, 5.0f};
        rays[7].origin = {5.0f, 5.0f, 5.0f};

        RayPacket p = make_packet(rays);
        __m256 t_min, t_max;
        int mask = p.intersect_aabb(box, t_min, t_max);

        EXPECT_EQ(0x0F, mask);
    }

    // tests whether t_min and t_max hold plausible entry and exit distances
    TEST_F(RayPacketAABB, TMinAndTMaxPlausible)
    {
        AABB box({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.0f, 0.0f, 5.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 t_min, t_max;
        p.intersect_aabb(box, t_min, t_max);

        float tmin_vals[8], tmax_vals[8];
        _mm256_storeu_ps(tmin_vals, t_min);
        _mm256_storeu_ps(tmax_vals, t_max);

        for (int i = 0; i < 8; ++i)
        {
            EXPECT_NEAR(4.0f, tmin_vals[i], 0.00001f);
            EXPECT_NEAR(6.0f, tmax_vals[i], 0.00001f);
        }
    }

    // tests whether a ray originating inside the box still reports a hit
    TEST_F(RayPacketAABB, RayOriginInsideBox)
    {
        AABB box({-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.0f, 0.0f, 0.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 t_min, t_max;
        int mask = p.intersect_aabb(box, t_min, t_max);

        EXPECT_EQ(0xFF, mask);
    }

    // ================================================================
    // 3. intersect_mt tests
    // ================================================================

    // tests whether all 8 rays report a hit when all aim at the triangle
    TEST_F(RayPacketTriangle, AllRaysHitTriangle)
    {
        Triangle3df triangle({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {3.0f, 0.0f, 0.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.5f, 0.5f, 5.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 out_t;
        int mask = p.intersect_mt(triangle, out_t);

        EXPECT_EQ(0xFF, mask);
    }

    // tests whether the bitmask is zero when no ray hits the triangle
    TEST_F(RayPacketTriangle, NoRayHitsTriangle)
    {
        Triangle3df triangle({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {3.0f, 0.0f, 0.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {5.0f, 5.0f, 5.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 out_t;
        int mask = p.intersect_mt(triangle, out_t);

        EXPECT_EQ(0x00, mask);
    }

    // tests whether out_t holds the correct intersection distance for every ray
    TEST_F(RayPacketTriangle, TValueCorrect)
    {
        Triangle3df triangle({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {3.0f, 0.0f, 0.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.5f, 0.5f, 2.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 out_t;
        p.intersect_mt(triangle, out_t);

        float t_vals[8];
        _mm256_storeu_ps(t_vals, out_t);

        for (int i = 0; i < 8; ++i)
        {
            EXPECT_NEAR(2.0f, t_vals[i], 0.00001f);
        }
    }

    // tests whether rays whose triangle intersection lies behind their origin are rejected
    TEST_F(RayPacketTriangle, RaysBehindTriangleMissed)
    {
        Triangle3df triangle({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {3.0f, 0.0f, 0.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
        {
            rays[i].origin = {0.5f, 0.5f, -5.0f};
            rays[i].direction = {0.0f, 0.0f, -1.0f};
        }

        RayPacket p = make_packet(rays);
        __m256 out_t;
        int mask = p.intersect_mt(triangle, out_t);

        EXPECT_EQ(0x00, mask);
    }

    // tests whether the bitmask correctly marks only the rays that hit the triangle
    TEST_F(RayPacketTriangle, SomeRaysHitTriangle)
    {
        Triangle3df triangle({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {3.0f, 0.0f, 0.0f});

        Ray3df rays[8];
        for (int i = 0; i < 8; ++i)
            rays[i].direction = {0.0f, 0.0f, -1.0f};

        rays[0].origin = {0.5f, 0.5f, 5.0f};
        rays[1].origin = {0.5f, 0.5f, 5.0f};
        rays[2].origin = {0.5f, 0.5f, 5.0f};
        rays[3].origin = {0.5f, 0.5f, 5.0f};
        rays[4].origin = {5.0f, 5.0f, 5.0f};
        rays[5].origin = {5.0f, 5.0f, 5.0f};
        rays[6].origin = {5.0f, 5.0f, 5.0f};
        rays[7].origin = {5.0f, 5.0f, 5.0f};

        RayPacket p = make_packet(rays);
        __m256 out_t;
        int mask = p.intersect_mt(triangle, out_t);

        EXPECT_EQ(0x0F, mask);
    }

} // namespace