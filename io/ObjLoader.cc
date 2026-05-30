#include "ObjLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <algorithm>

void ObjLoader::load(std::string path, Scene& scene) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ObjLoader: Cannot open file: " << path << std::endl;
        return;
    }

    // raw object data
    std::vector<Vector3df> vertices;
    std::vector<Vector3df> normals;

    std::map<std::string, Material> material_map;
    std::string current_material = "";

    Material default_material(
        Vector3df{0.8f, 0.8f, 0.8f},    // color
        0.0f,                           // reflectivity
        1.0f,                           // shininess
        1.0f,                           // refraction_index
        0.0f                            // transparency
    );

    std::string line;
    while (std::getline(file, line)) {
        // Strip Windows-style carriage returns
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "mtllib") {
            std::string mtl_filename;
            iss >> mtl_filename;

            // Build path: same directory as .obj file
            std::string dir = path.substr(0, path.find_last_of("/\\") + 1);
            std::string mtl_path = dir + mtl_filename;

            loadMTL(mtl_path, material_map);

        } else if (token == "usemtl") {
            iss >> current_material;

        } else if (token == "v") {
            // vertex position
            float x, y, z;
            iss >> x >> y >> z;
            vertices.push_back(Vector3df{x, y, z});

        } else if (token == "vn") {
            // vertex normal
            float x, y, z;
            iss >> x >> y >> z;
            normals.push_back(Vector3df{x, y, z});

        } else if (token == "f") {
            // face
            std::vector<int> v_idx, vn_idx;
            std::string part;

            while (iss >> part) {
                // Detect format before replacing slashes:
                //   "v"         -> only vertex
                //   "v/vt"      -> vertex + texcoord, no normal
                //   "v//vn"     -> vertex + normal, no texcoord
                //   "v/vt/vn"   -> vertex + texcoord + normal
                bool has_double_slash = (part.find("//") != std::string::npos);

                std::replace(part.begin(), part.end(), '/', ' ');
                std::istringstream part_iss(part);
                std::vector<int> indices;
                int idx;
                while (part_iss >> idx) {
                    indices.push_back(idx);
                }

                // OBJ indices are 1-based
                v_idx.push_back(indices[0] - 1);

                if (indices.size() == 3) {
                    // v/vt/vn: normal is third index
                    vn_idx.push_back(indices[2] - 1);
                } else if (indices.size() == 2 && has_double_slash) {
                    // v//vn: normal is second index
                    vn_idx.push_back(indices[1] - 1);
                } else {
                    // "v" or "v/vt": no normal available
                    vn_idx.push_back(-1);
                }
            }

            // Triangulate
            for (size_t i = 1; i + 1 < v_idx.size(); i++) {
                Vector3df va = vertices[v_idx[0]];
                Vector3df vb = vertices[v_idx[i]];
                Vector3df vc = vertices[v_idx[i + 1]];

                // Look up material
                Material mat = default_material;
                if (!current_material.empty()) {
                    auto it = material_map.find(current_material);
                    if (it != material_map.end()) {
                        mat = it->second;
                    }
                }

                // Build triangle with or without per-vertex normals
                bool has_normals = !vn_idx.empty()
                                   && vn_idx[0] >= 0
                                   && vn_idx[i] >= 0
                                   && vn_idx[i + 1] >= 0;

                Triangle3df tri = has_normals
                    ? Triangle3df(va, vb, vc,
                                  normals[vn_idx[0]],
                                  normals[vn_idx[i]],
                                  normals[vn_idx[i + 1]])
                    : Triangle3df(va, vb, vc);

                scene.addTriangle(tri, mat, va, vb, vc);
            }
        }
    }
    
    scene.build_kd_tree();
    std::cout << "ObjLoader: Loaded " << path << std::endl;
}

void ObjLoader::loadMTL(const std::string& path, std::map<std::string, Material>& material_map) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ObjLoader: Cannot open MTL file: " << path << std::endl;
        return;
    }

    std::string current_name = "";
    Vector3df Kd{0.8f, 0.8f, 0.8f};
    Vector3df Ks{0.0f, 0.0f, 0.0f};
    float Ns = 1.0f;
    float d  = 1.0f;
    float Ni = 1.0f;

    // lambda to save material in map
    auto flush_material = [&]() {
        if (current_name.empty()) return;
        float reflectivity  = (Ks[0] + Ks[1] + Ks[2]) / 3.0f;
        float transparency  = 1.0f - d;
        material_map.emplace(current_name,
            Material(Kd, reflectivity, Ns, Ni, transparency));
    };

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();

        // Strip inline comments
        size_t comment = line.find('#');
        if (comment != std::string::npos) line = line.substr(0, comment);

        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "newmtl") {
            flush_material();
            iss >> current_name;
            // reset defaults for new material
            Kd = Vector3df{0.8f, 0.8f, 0.8f};
            Ks = Vector3df{0.0f, 0.0f, 0.0f};
            Ns = 1.0f; d = 1.0f; Ni = 1.0f;

        } else if (token == "Kd") {
            iss >> Kd[0] >> Kd[1] >> Kd[2];
        } else if (token == "Ks") {
            iss >> Ks[0] >> Ks[1] >> Ks[2];
        } else if (token == "Ns") {
            iss >> Ns;
        } else if (token == "d") {
            iss >> d;
        } else if (token == "Ni") {
            iss >> Ni;
        }
    }
    flush_material(); 
}
