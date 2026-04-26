#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include "Camera.h"
#include "../scene/Scene.h"
#include <vector>

/**
 * @class Raytracer
 * @brief Raytracer class. Core engine that manages the rendering and coloring.
 */
class Raytracer {
    public:
        /** Creates a new Raytracer object based on the image width & height and the max depth of recursion.
         * @brief Constructor of Raytracer
         * @param width the width of the image
         * @param height the height of the image
         * @param max_depth the maxium amount of recursive calls
         */
        Raytracer(int width, int height, int max_depth);
        /**
         * @brief Render loop over all pixels.
         * @param cam the camera with the rays
         * @param scene the scene containing lights, triangles and materials
         * @return void
         */
        void render(Camera cam, Scene scene);
        /**
         * @brief Calculate the color for one ray
         * @param ray the ray to calculate the color for
         * @param scene
         * @param depth the current recursive depth
         * @return vector containing the color information 
         */
        Vector3df trace(Ray3df& ray, Scene& scene, int depth);
        /**
         * @brief Getter for frame buffer
         * @return framebuffer
         */
        const std::vector<Vector3df>& getFramebuffer() const { return framebuffer; }
    private:
        int max_depth; // maximum of recursive calls
        std::vector<Vector3df> framebuffer; // stores the final image's colors
        /**
         * image resolution
         */
        int width;
        int height;
};

#endif