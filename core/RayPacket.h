#ifndef RAY_PACKET_H
#define RAY_PACKET_H

#include <immintrin.h>
#include "geometry/geometry.h"
#include "AABB.h"

/**
 * @struct RayPacket
 * @brief A packet of 8 coherent rays stored as SoA. Used for AVX processing.
 */
struct RayPacket
{
    // __m256 is avx datatyp that holds 8 floats (256 bit)
    __m256 origin_x, origin_y, origin_z;                   // ray origins
    __m256 direction_x, direction_y, direction_z;          // ray directions
    __m256 rp_direction_x, rp_direction_y, rp_direction_z; // precomputed reciprocal directions

    /**
     * @brief fills the ray packet.
     * @param Ray3df Array of 9 rays
     * @returns void
     */
    void set_rays(const Ray3df rays[8]);

    /**
     * @brief intersect method for ray packet with kd-tree AABB (min_pt/max_pt layout)
     * @param aabb the bounding box to test against
     * @param t_min entry distance
     * @param t_max exit distance
     * @return bitmask
     */
    int intersect_aabb(const AABB &aabb, __m256 &t_min, __m256 &t_max) const;

    /**
     * @brief intersect method for ray packet with mt
     * @param out_t intersection distance
     * @return bitmask
     */
    int intersect_mt(const Triangle3df &triangle, __m256 &out_t) const;
};

#endif // RAY_PACKET_H