#ifndef OBJLOADER_H
#define OBJLOADER_H

#include "../math/math.h"
#include "../geometry/geometry.h"
#include <string>
#include <map>
#include "../scene/Scene.h"

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
    private:
        /**
         * @brief Loads materials from a mtl file
         * @param path path to the mtl file
         * @param material_map output map with the loaded materials
         */
        void loadMTL(const std::string& path, std::map<std::string, Material>& material_map);    
};

#endif