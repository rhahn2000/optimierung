#ifndef MATERIAL_H
#define MATERIAL_H

#include "../math/math.h"
#include "../geometry/geometry.h"

/**
 * @struct Material
 * @brief Structure for the material of objects. The material describes how objects look like.
 */
struct Material {
    /**
     * Creates a new material object based on a color, reflectivity, shineness, refraction index and transparancy values.
     * @brief Constructor of material
     * @param color the color of the material
     * @param reflectivity how much of the environment should be reflected
     * @param shininess how polished the material is supposed to be
     * @param refraction_index how much rays should divert when entering the material
     * @param transparency how transparent a material should be
     */
    Material(Vector3df color, float reflectivity, float shininess, float refraction_index, float transparency);
    Vector3df color;
    float reflectivity;
    float shininess;
    float refraction_index;
    float transparency;
};

#endif