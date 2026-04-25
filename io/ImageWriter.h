#ifndef IMAGEWRITER_H
#define IMAGEWRITER_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include <string>
#include <vector>

/**
 * @class ImageWriter
 * @brief ImageWriter class. Write colors from framebuffer into a file.
 */
class ImageWriter {
    public:
        /**
         * @brief Writes the color values into a ppm file.
         * @param path the path of the new file
         * @param framebuffer the color data 
         * @param width the image width
         * @param height the image height
         * @return void
         */
        void write_ppm(std::string path, std::vector<Vector3df> framebuffer, int width, int height);
};
#endif