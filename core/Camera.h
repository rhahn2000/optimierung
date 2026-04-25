#ifndef CAMERA_H
#define CAMERA_H

#include "../math/math.h"
#include "../geometry/geometry.h"

/**
 * @class Camera
 * @brief Camera class / ray origin. Converts 2d pixel coordinates into 3d rays.
 */
class Camera {
    public:
        /** Creates a new camera object based on different params.
         *  Calculates local coordinate system and creates the private variables.
         * @brief Constructor of camera
         * @param position The camera position
         * @param target The point the camera is looking at
         * @param vup The orientation vector
         * @param vangle The camera opening angle / vertical field of view in degrees
         * @param aspectRatio 
         */
        Camera(Vector3df& position, Vector3df& target, Vector3df& vup, float vangle, float aspectRatio);
        /**
         * @brief Calculates ray for coordinates.
         * @param u coordinate u
         * @param v coordinate v
         * @return the calculated ray for the coordinates (u, v)
         */
        Ray3df get_ray(float u, float v);
    private:
        Vector3df origin; // Origin of ray / camera position.
        Vector3df lower_left_corner; // position of the view's bottom left corner
        /**
         * Size of the image
         */
        Vector3df horizontal; 
        Vector3df vertical;
};
#endif