#ifndef LIGHT_H
#define LIGHT_H

#include "../math/math.h"
#include "../geometry/geometry.h"

/**
 * @struct Light
 * @brief Structure for the scene light. 
 */
struct Light {
    /**
     * Creates a new light object based on position and intensity.
     * @brief Constructor of light
     * @param position the position of the light
     * @param intensity the intensity of the light
     */
    Light(Vector3df position, Vector3df intensity);
    Vector3df position;
    Vector3df intensity;
};    

#endif