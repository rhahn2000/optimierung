#ifndef CORE_AABB_H
#define CORE_AABB_H

#include "../geometry/geometry.h"
#include "../math/math.h"
#include <vector>

/**
 * @struct AABB
 * @brief AABB struct. Contains the definition of the axis aligned bounding box. Used for kd tree optimization.
 */
struct AABB {
    Vector3df min_pt;
    Vector3df max_pt;

    /**
     * Creates a new AABB.
     * @brief Constructor of AABB
     */
    AABB();
    
    /**
     * Creates a new AABB with min and max points.
     * @brief Constructor of AABB
     * @param min_p the min point of the box
     * @param max_p the max point of the box
     */
    AABB(const Vector3df& min_p, const Vector3df& max_p);

    /**
     * @brief extends the box dynamically with the given point.
     * @param point the point used to extend the box
     * @returns void
     */
    void grow(const Vector3df& point);

    /**
     * @brief calculates aabb for a specific triangle
     * @param a corner of triangle
     * @param b corner of triangle
     * @param c corner of triangle
     * @return AABB for triangle
     */
    static AABB from_triangle(const Vector3df& a, const Vector3df& b, const Vector3df& c);

    /**
     * @brief checks whether ray has intersection with aabb
     * @param ray the ray to check
     * @param t_min time distance between origin and entry point of box
     * @param t_max time distance between origin and exit point of box
     * @returns bool; true if intersection exists
     */
    bool intersect(const Ray3df& ray, float& t_min, float& t_max) const;
};

#endif