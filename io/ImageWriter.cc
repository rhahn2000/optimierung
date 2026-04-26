#include "ImageWriter.h"
#include <fstream>
#include <iostream>
#include <algorithm>

void ImageWriter::write_ppm(std::string path, std::vector<Vector3df> framebuffer, int width, int height) {
    std::ofstream file(path, std::ios::out);
    if (!file.is_open()) {
        std::cerr << "ImageWriter: Cannot create file: " << path << std::endl;
        return;
    }

    // PPM P3 header
    file << "P3\n";
    file << width << " " << height << "\n";
    file << "255\n";

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vector3df color = framebuffer[y * width + x];

            // Clamp auf [0, 1]
            float r = std::clamp(color[0], 0.0f, 1.0f);
            float g = std::clamp(color[1], 0.0f, 1.0f);
            float b = std::clamp(color[2], 0.0f, 1.0f);

            // Umrechnung in [0, 255]
            int ir = static_cast<int>(r * 255.999f);
            int ig = static_cast<int>(g * 255.999f);
            int ib = static_cast<int>(b * 255.999f);

            file << ir << " " << ig << " " << ib << "\n";
        }
    }

    std::cout << "ImageWriter: Written " << path << " (" << width << "x" << height << ")" << std::endl;
}