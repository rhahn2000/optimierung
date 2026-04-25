#ifndef OBJLOADER_H
#define OBJLOADER_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include <string>
#include "Scene.h"

/**
 * @class ObjLoader
 * @brief ObjLoader class. Parser for wavefront .obj files. 
 */
class ObjLoader {
    public:
        /**
         * @brief Loads objects from an obj file into the scene.
         * @param path the path to the obj file
         * @param scene the scene where the objects need to be saved to
         * @return void
         */
        void load(std::string path, Scene& scene);
};

#endif