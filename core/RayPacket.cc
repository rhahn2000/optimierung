#include "RayPacket.h"

void RayPacket::set_rays(const Ray3df rays[8])
{
    float packet_origin_x[8], packet_origin_y[8], packet_origin_z[8];
    float packet_direction_x[8], packet_direction_y[8], packet_direction_z[8];

    // copy each ray into the packet arrays
    for (int i = 0; i < 8; ++i)
    {
        packet_origin_x[i] = rays[i].origin[0];
        packet_origin_y[i] = rays[i].origin[1];
        packet_origin_z[i] = rays[i].origin[2];
        packet_direction_x[i] = rays[i].direction[0];
        packet_direction_y[i] = rays[i].direction[1];
        packet_direction_z[i] = rays[i].direction[2];
    }

    // copy each float from storage to register
    origin_x = _mm256_loadu_ps(packet_origin_x);
    origin_y = _mm256_loadu_ps(packet_origin_y);
    origin_z = _mm256_loadu_ps(packet_origin_z);
    direction_x = _mm256_loadu_ps(packet_direction_x);
    direction_y = _mm256_loadu_ps(packet_direction_y);
    direction_z = _mm256_loadu_ps(packet_direction_z);

    // precompute the directions for aabb
    const __m256 one = _mm256_set1_ps(1.0f);
    rp_direction_x = _mm256_div_ps(one, direction_x);
    rp_direction_y = _mm256_div_ps(one, direction_y);
    rp_direction_z = _mm256_div_ps(one, direction_z);
}

int RayPacket::intersect_aabb(const AABB &aabb, __m256 &t_min, __m256 &t_max) const
{

    const __m256 box_min_x = _mm256_set1_ps(aabb.min_pt[0]);
    const __m256 box_max_x = _mm256_set1_ps(aabb.max_pt[0]);
    const __m256 box_min_y = _mm256_set1_ps(aabb.min_pt[1]);
    const __m256 box_max_y = _mm256_set1_ps(aabb.max_pt[1]);
    const __m256 box_min_z = _mm256_set1_ps(aabb.min_pt[2]);
    const __m256 box_max_z = _mm256_set1_ps(aabb.max_pt[2]);

    // calculate t for each ray and border
    const __m256 tx0 = _mm256_mul_ps(_mm256_sub_ps(box_min_x, origin_x), rp_direction_x);
    const __m256 tx1 = _mm256_mul_ps(_mm256_sub_ps(box_max_x, origin_x), rp_direction_x);
    const __m256 ty0 = _mm256_mul_ps(_mm256_sub_ps(box_min_y, origin_y), rp_direction_y);
    const __m256 ty1 = _mm256_mul_ps(_mm256_sub_ps(box_max_y, origin_y), rp_direction_y);
    const __m256 tz0 = _mm256_mul_ps(_mm256_sub_ps(box_min_z, origin_z), rp_direction_z);
    const __m256 tz1 = _mm256_mul_ps(_mm256_sub_ps(box_max_z, origin_z), rp_direction_z);

    const __m256 tmin_x = _mm256_min_ps(tx0, tx1);
    const __m256 tmax_x = _mm256_max_ps(tx0, tx1);
    const __m256 tmin_y = _mm256_min_ps(ty0, ty1);
    const __m256 tmax_y = _mm256_max_ps(ty0, ty1);
    const __m256 tmin_z = _mm256_min_ps(tz0, tz1);
    const __m256 tmax_z = _mm256_max_ps(tz0, tz1);

    const __m256 zero = _mm256_setzero_ps();
    t_min = _mm256_max_ps(zero, _mm256_max_ps(tmin_x, _mm256_max_ps(tmin_y, tmin_z)));
    t_max = _mm256_min_ps(tmax_x, _mm256_min_ps(tmax_y, tmax_z));

    return _mm256_movemask_ps(_mm256_cmp_ps(t_max, t_min, _CMP_GE_OQ));
}

int RayPacket::intersect_mt(const Triangle3df &triangle, __m256 &out_t, __m256 &out_u, __m256 &out_v) const
{
    // define struct to get data of triangle3df
    struct TriangleAccessor : Triangle3df
    {
        const Vector3df &vertices_a() const { return a; }
        const Vector3df &vertices_b() const { return b; }
        const Vector3df &vertices_c() const { return c; }
    };
    const TriangleAccessor &tri = reinterpret_cast<const TriangleAccessor &>(triangle);

    const __m256 ax = _mm256_set1_ps(tri.vertices_a()[0]);
    const __m256 ay = _mm256_set1_ps(tri.vertices_a()[1]);
    const __m256 az = _mm256_set1_ps(tri.vertices_a()[2]);
    const __m256 bx = _mm256_set1_ps(tri.vertices_b()[0]);
    const __m256 by = _mm256_set1_ps(tri.vertices_b()[1]);
    const __m256 bz = _mm256_set1_ps(tri.vertices_b()[2]);
    const __m256 cx = _mm256_set1_ps(tri.vertices_c()[0]);
    const __m256 cy = _mm256_set1_ps(tri.vertices_c()[1]);
    const __m256 cz = _mm256_set1_ps(tri.vertices_c()[2]);

    // calculate edges between vertices
    const __m256 edge_1_x = _mm256_sub_ps(bx, ax);
    const __m256 edge_1_y = _mm256_sub_ps(by, ay);
    const __m256 edge_1_z = _mm256_sub_ps(bz, az);
    const __m256 edge_2_x = _mm256_sub_ps(cx, ax);
    const __m256 edge_2_y = _mm256_sub_ps(cy, ay);
    const __m256 edge_2_z = _mm256_sub_ps(cz, az);

    // calculate cross product 
    const __m256 hx = _mm256_sub_ps(_mm256_mul_ps(direction_y, edge_2_z), _mm256_mul_ps(direction_z, edge_2_y));
    const __m256 hy = _mm256_sub_ps(_mm256_mul_ps(direction_z, edge_2_x), _mm256_mul_ps(direction_x, edge_2_z));
    const __m256 hz = _mm256_sub_ps(_mm256_mul_ps(direction_x, edge_2_y), _mm256_mul_ps(direction_y, edge_2_x));

    // calculate determinante
    const __m256 det = _mm256_add_ps(_mm256_mul_ps(edge_1_x, hx),
                                     _mm256_add_ps(_mm256_mul_ps(edge_1_y, hy),
                                                   _mm256_mul_ps(edge_1_z, hz)));

    // remove rays parallel to triangle
    const __m256 eps = _mm256_set1_ps(1e-7f);
    const __m256 abs_det = _mm256_andnot_ps(_mm256_set1_ps(-0.0f), det);
    __m256 active = _mm256_cmp_ps(abs_det, eps, _CMP_GE_OQ);

    // inverte det to avoid divide
    const __m256 inv_det = _mm256_div_ps(_mm256_set1_ps(1.0f), det);

    // calculate scalar product
    const __m256 sx = _mm256_sub_ps(origin_x, ax);
    const __m256 sy = _mm256_sub_ps(origin_y, ay);
    const __m256 sz = _mm256_sub_ps(origin_z, az);
    const __m256 sdoth = _mm256_add_ps(_mm256_mul_ps(sx, hx),
                                       _mm256_add_ps(_mm256_mul_ps(sy, hy),
                                                     _mm256_mul_ps(sz, hz)));

    // calculate u coordinate
    const __m256 u = _mm256_mul_ps(sdoth, inv_det);

    // check if u is inbetween 0 and 1 and filter active accordingly
    const __m256 zero = _mm256_setzero_ps();
    const __m256 one = _mm256_set1_ps(1.0f);
    active = _mm256_and_ps(active, _mm256_cmp_ps(u, zero, _CMP_GE_OQ));
    active = _mm256_and_ps(active, _mm256_cmp_ps(u, one, _CMP_LE_OQ));

    // calculate q coordinate
    const __m256 qx = _mm256_sub_ps(_mm256_mul_ps(sy, edge_1_z), _mm256_mul_ps(sz, edge_1_y));
    const __m256 qy = _mm256_sub_ps(_mm256_mul_ps(sz, edge_1_x), _mm256_mul_ps(sx, edge_1_z));
    const __m256 qz = _mm256_sub_ps(_mm256_mul_ps(sx, edge_1_y), _mm256_mul_ps(sy, edge_1_x));

    // calculate v coordinate
    const __m256 ddotq = _mm256_add_ps(_mm256_mul_ps(direction_x, qx),
                                       _mm256_add_ps(_mm256_mul_ps(direction_y, qy),
                                                     _mm256_mul_ps(direction_z, qz)));
    const __m256 v = _mm256_mul_ps(ddotq, inv_det);

    // check if u and v are greater than 0 and their sum is smaller than 1 and filter accordingly
    active = _mm256_and_ps(active, _mm256_cmp_ps(v, zero, _CMP_GE_OQ));
    active = _mm256_and_ps(active, _mm256_cmp_ps(_mm256_add_ps(u, v), one, _CMP_LE_OQ));

    // calculate distance and filter rays if distance <= 0
    const __m256 e2dotq = _mm256_add_ps(_mm256_mul_ps(edge_2_x, qx),
                                        _mm256_add_ps(_mm256_mul_ps(edge_2_y, qy),
                                                      _mm256_mul_ps(edge_2_z, qz)));
    out_t = _mm256_mul_ps(e2dotq, inv_det);
    active = _mm256_and_ps(active, _mm256_cmp_ps(out_t, zero, _CMP_GT_OQ));

    out_u = u;
    out_v = v;

    return _mm256_movemask_ps(active);
}