#include "ObjLoader.h"
#include "gtest/gtest.h"

#include <fstream>
#include <string>
#include <filesystem>
#include <cstdio>

namespace
{
    constexpr float kEps = 1e-4f;

    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------

    /**
     * @brief builds an absolute path to a temp file
     * @param filename name of file including file extension
     * @return absolute path
     */
    std::string tmp(const std::string &filename)
    {
        return (std::filesystem::temp_directory_path() / filename).string();
    }

    /**
     * @brief writes content into the file
     * @param path path to write to
     * @param content content to write
     * @return void
     */
    void write_file(const std::string &path, const std::string &content)
    {
        std::ofstream f(path);
        f << content;
    }

    /**
     * @brief writes a simple obj file
     * @param path path to write to
     * @return void
     */
    void write_single_triangle_obj(const std::string &path)
    {
        write_file(path,
                   "v  0.0  0.0  0.0\n"
                   "v  1.0  0.0  0.0\n"
                   "v  0.0  1.0  0.0\n"
                   "f 1 2 3\n");
    }

    /**
     * @brief writes a simple obj file with two triangles
     * @param path path to write to
     * @return void
     */
    void write_two_triangle_obj(const std::string &path)
    {
        write_file(path,
                   "v  0.0  0.0  0.0\n"
                   "v  1.0  0.0  0.0\n"
                   "v  0.0  1.0  0.0\n"
                   "v -1.0  0.0  0.0\n"
                   "v  0.0 -1.0  0.0\n"
                   "f 1 2 3\n"
                   "f 1 4 5\n");
    }

    /**
     * @brief writes a simple material file
     * @param path path to write to
     * @return void
     */
    void write_red_mtl(const std::string &path)
    {
        write_file(path,
                   "newmtl RedMat\n"
                   "Kd 1.0 0.0 0.0\n");
    }

    /**
     * @brief writes an obj file that references material
     * @param obj_path path to obj file
     * @param mtl_path path to material
     * @return void
     */
    void write_obj_with_material(const std::string &obj_path, const std::string &mtl_path)
    {
        std::string mtl_filename = std::filesystem::path(mtl_path).filename().string();
        write_file(obj_path,
                   "mtllib " + mtl_filename + "\n"
                                              "v  0.0  0.0 -1.0\n"
                                              "v  1.0  0.0 -1.0\n"
                                              "v  0.0  1.0 -1.0\n"
                                              "usemtl RedMat\n"
                                              "f 1 2 3\n");
    }

    /**
     * @class ObjLoaderTest
     * @brief creates temporary obj and mtl files for the tests and removes them after
     */
    class ObjLoaderTest : public ::testing::Test
    {
    protected:
        ObjLoader loader;
        Scene scene;

        std::string obj_path = tmp("objloader_test.obj");
        std::string obj2_path = tmp("objloader_test2.obj");
        std::string mtl_path = tmp("objloader_test.mtl");

        void TearDown() override
        {
            std::remove(obj_path.c_str());
            std::remove(obj2_path.c_str());
            std::remove(mtl_path.c_str());
        }
    };

    // ================================================================
    // 1. invalid path
    // ================================================================

    // tests whether a missing file does not throw an error / causes a crash
    TEST_F(ObjLoaderTest, InvalidPathDoesNotCrash)
    {
        ASSERT_NO_THROW(loader.load("/nonexistent/file.obj", scene));
    }

    // tests whether an invalid path leads to an empty scene
    TEST_F(ObjLoaderTest, InvalidPathLeavesSceneEmpty)
    {
        loader.load("/nonexistent/file.obj", scene);
        Ray3df ray{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect(ray, ctx, mat_index));
    }

    // ================================================================
    // 2. loading
    // ================================================================

    // tests whether a file with a single triangle leads to exactly one hit
    TEST_F(ObjLoaderTest, LoadsSingleTriangle)
    {
        write_single_triangle_obj(obj_path);
        loader.load(obj_path, scene);

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_TRUE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether a file with two triangles leads to exactly two hits
    TEST_F(ObjLoaderTest, LoadsTwoTriangles)
    {
        write_two_triangle_obj(obj_path);
        loader.load(obj_path, scene);

        // Strahl trifft erstes Dreieck
        Ray3df ray1{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx1;
        int idx1 = -1;
        EXPECT_TRUE(scene.intersect(ray1, ctx1, idx1));

        // Strahl trifft zweites Dreieck
        Ray3df ray2{{-0.2f, -0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx2;
        int idx2 = -1;
        EXPECT_TRUE(scene.intersect(ray2, ctx2, idx2));
    }

    // tests whether a ray missing the triangle(s) leads to zero hits
    TEST_F(ObjLoaderTest, RayMissesLoadedTriangle)
    {
        write_single_triangle_obj(obj_path);
        loader.load(obj_path, scene);

        Ray3df ray{{5.0f, 5.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_FALSE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether loading the same file twice does not lead to a crash
    TEST_F(ObjLoaderTest, LoadSameFileTwiceDoesNotCrash)
    {
        write_single_triangle_obj(obj_path);
        ASSERT_NO_THROW(loader.load(obj_path, scene));
        ASSERT_NO_THROW(loader.load(obj_path, scene));
    }

    // ================================================================
    // 3. correct intersection geometry
    // ================================================================

    // tests whether the ray hits the triangle at the correct depth
    TEST_F(ObjLoaderTest, IntersectionAtCorrectDepth)
    {
        write_single_triangle_obj(obj_path);
        loader.load(obj_path, scene);

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));
        EXPECT_NEAR(ctx.t, 1.0f, kEps);
    }

    // ================================================================
    // 4. material
    // ================================================================

    // tests whether material color is loaded correctly from file
    TEST_F(ObjLoaderTest, MaterialColorFromMTL)
    {
        write_red_mtl(mtl_path);
        write_obj_with_material(obj_path, mtl_path);
        loader.load(obj_path, scene);

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));

        const Material &mat = scene.getMaterial(mat_index);
        // Rotes Material: R dominiert, G und B sind 0
        EXPECT_GT(mat.color[0], mat.color[1]);
        EXPECT_GT(mat.color[0], mat.color[2]);
    }

    // tests whether not using a material file while not cause a crash
    TEST_F(ObjLoaderTest, LoadWithoutMTLDoesNotCrash)
    {
        write_single_triangle_obj(obj_path);
        ASSERT_NO_THROW(loader.load(obj_path, scene));
    }

    // ================================================================
    // 5. vertex normals
    // ================================================================

    // tests whether obj with vertex normals is correctly loaded
    TEST_F(ObjLoaderTest, LoadsObjWithVertexNormals)
    {
        write_file(obj_path,
                   "v  0.0  0.0  0.0\n"
                   "v  1.0  0.0  0.0\n"
                   "v  0.0  1.0  0.0\n"
                   "vn 0.0  0.0  1.0\n"
                   "f 1//1 2//1 3//1\n");
        ASSERT_NO_THROW(loader.load(obj_path, scene));

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_TRUE(scene.intersect(ray, ctx, mat_index));
    }

    // tests whether obj with vertex normals in vt vn format is correctly loaded
    TEST_F(ObjLoaderTest, LoadsObjWithVtVnFormat)
    {
        write_file(obj_path,
                   "v  0.0  0.0  0.0\n"
                   "v  1.0  0.0  0.0\n"
                   "v  0.0  1.0  0.0\n"
                   "vn 0.0  0.0  1.0\n"
                   "f 1/1/1 2/1/1 3/1/1\n");
        ASSERT_NO_THROW(loader.load(obj_path, scene));

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        EXPECT_TRUE(scene.intersect(ray, ctx, mat_index));
    }

    // ================================================================
    // 6. remaining mtl fields
    // ================================================================

    // tests whether mtl files are loaded successfully with all common fields
    TEST_F(ObjLoaderTest, LoadsMTLWithAllFields)
    {
        write_file(mtl_path,
                   "newmtl FullMat\n"
                   "Kd 0.0 0.8 0.0\n"
                   "Ks 0.5 0.5 0.5\n"
                   "Ns 32.0\n"
                   "d  0.9\n"
                   "Ni 1.5\n");
        std::string mtl_filename = std::filesystem::path(mtl_path).filename().string();
        write_file(obj_path,
                   "mtllib " + mtl_filename + "\n"
                                              "v  0.0  0.0 -1.0\n"
                                              "v  1.0  0.0 -1.0\n"
                                              "v  0.0  1.0 -1.0\n"
                                              "usemtl FullMat\n"
                                              "f 1 2 3\n");

        ASSERT_NO_THROW(loader.load(obj_path, scene));

        Ray3df ray{{0.2f, 0.2f, 1.0f}, {0.0f, 0.0f, -1.0f}};
        Intersection_Context<float, 3> ctx;
        int mat_index = -1;
        ASSERT_TRUE(scene.intersect(ray, ctx, mat_index));

        const Material &mat = scene.getMaterial(mat_index);
        EXPECT_GT(mat.color[1], mat.color[0]);
        EXPECT_GT(mat.color[1], mat.color[2]);
    }

}