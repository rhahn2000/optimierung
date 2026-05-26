#include "ImageWriter.h"
#include "gtest/gtest.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>     // std::remove
#include <filesystem> // std::filesystem::temp_directory_path (C++17)

namespace
{
    // ----------------------------------------------------------------
    // 0. setup for testing
    // ----------------------------------------------------------------

    /**
     * @struct RGB
     * @brief RGB integer triplets
     */
    struct RGB
    {
        int r, g, b;
    };

    /**
     * @brief parses a p3 ppm file into a list of rgbs.
     * @param path path to the ppm file
     * @param out_width image width in pixels
     * @param out_height image height in pixels
     * @param out_max_val max channel value
     * @param out_pixels pixels as rgb list
     * @returns true if reading was successful
     */
    bool parse_ppm(const std::string &path,
                   int &out_width, int &out_height, int &out_max_val,
                   std::vector<RGB> &out_pixels)
    {
        std::ifstream f(path);
        if (!f.is_open())
            return false;

        std::string magic;
        f >> magic;
        if (magic != "P3")
            return false;

        f >> out_width >> out_height >> out_max_val;

        out_pixels.clear();
        out_pixels.reserve(out_width * out_height);

        RGB px{};
        while (f >> px.r >> px.g >> px.b)
        {
            out_pixels.push_back(px);
        }
        return true;
    }

    /**
     * @class ImageWriterTest
     * @brief creates a unique temporary path and removes the file after the test.
     */
    class ImageWriterTest : public ::testing::Test
    {
    protected:
        std::string tmp_path = (std::filesystem::temp_directory_path() / "imagewriter_test.ppm").string();
        ImageWriter writer;

        void TearDown() override
        {
            std::remove(tmp_path.c_str());
        }
    };

    // ================================================================
    // 1. ppm structure tests
    // ================================================================

    // tests whether the file is parseable as p3 and contains the correct dimensions
    TEST_F(ImageWriterTest, WritesValidP3Header)
    {
        std::vector<Vector3df> fb = {{0.0f, 0.0f, 0.0f}};
        writer.write_ppm(tmp_path, fb, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        EXPECT_EQ(1, w);
        EXPECT_EQ(1, h);
    }

    // tests whether the files pixel count equals width * height
    TEST_F(ImageWriterTest, PixelCountMatchesDimensions)
    {
        const int W = 4, H = 3;
        std::vector<Vector3df> fb(W * H, {0.5f, 0.5f, 0.5f});
        writer.write_ppm(tmp_path, fb, W, H);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        EXPECT_EQ(W, w);
        EXPECT_EQ(H, h);
        EXPECT_EQ(W * H, static_cast<int>(pixels.size()));
    }

    // ================================================================
    // 2. color value tests
    // ================================================================

    // tests whether a pure red returns a maximum channel value for red and 0 for green/blue
    TEST_F(ImageWriterTest, PureRedPixel)
    {
        writer.write_ppm(tmp_path, {{1.0f, 0.0f, 0.0f}}, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(1u, pixels.size());

        EXPECT_GT(pixels[0].r, 0);
        EXPECT_EQ(0, pixels[0].g);
        EXPECT_EQ(0, pixels[0].b);
        EXPECT_EQ(pixels[0].r, maxval);
    }

    // tests whether a pure black returns 0 for all channels
    TEST_F(ImageWriterTest, PureBlackPixel)
    {
        writer.write_ppm(tmp_path, {{0.0f, 0.0f, 0.0f}}, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(1u, pixels.size());

        EXPECT_EQ(0, pixels[0].r);
        EXPECT_EQ(0, pixels[0].g);
        EXPECT_EQ(0, pixels[0].b);
    }

    // tests whether a pure white returns maximum value for all channels
    TEST_F(ImageWriterTest, PureWhitePixel)
    {
        writer.write_ppm(tmp_path, {{1.0f, 1.0f, 1.0f}}, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(1u, pixels.size());

        EXPECT_EQ(maxval, pixels[0].r);
        EXPECT_EQ(maxval, pixels[0].g);
        EXPECT_EQ(maxval, pixels[0].b);
    }

    // tests whether the pixel order in the framebuffer equals the pixel order of the file
    TEST_F(ImageWriterTest, PixelOrderIsPreserved)
    {
        // 2x1 image: red pixel, then blue pixel
        std::vector<Vector3df> fb = {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        writer.write_ppm(tmp_path, fb, 2, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(2u, pixels.size());

        // first pixel: red dominates
        EXPECT_GT(pixels[0].r, pixels[0].b);
        // second pixel: blue dominates
        EXPECT_GT(pixels[1].b, pixels[1].r);
    }

    // ================================================================
    // 3. out-of-range / clamping tests
    // ================================================================

    // tests whether a value above 1.0 is clamped to maxval and not overflow
    TEST_F(ImageWriterTest, OverbrightValueIsClamped)
    {
        writer.write_ppm(tmp_path, {{1.5f, 0.0f, 0.0f}}, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(1u, pixels.size());

        EXPECT_LE(pixels[0].r, maxval); // must not exceed maxval
        EXPECT_GE(pixels[0].r, 0);      // must not be negative
    }

    // tests whether a negative value is clamped to 0
    TEST_F(ImageWriterTest, NegativeValueIsClamped)
    {
        writer.write_ppm(tmp_path, {{-0.2f, 0.0f, 0.0f}}, 1, 1);

        int w, h, maxval;
        std::vector<RGB> pixels;
        ASSERT_TRUE(parse_ppm(tmp_path, w, h, maxval, pixels));
        ASSERT_EQ(1u, pixels.size());

        EXPECT_GE(pixels[0].r, 0);
        EXPECT_LE(pixels[0].r, maxval);
    }

    // ================================================================
    // 4. Edge case: invalid (non-writable) path
    // ================================================================

    // tests whether writing into a non existent directory does not throw an error & does not create a file
    TEST_F(ImageWriterTest, InvalidPathDoesNotCrash)
    {
        std::string invalid_path = "/nonexistent_dir/image.ppm";
        writer.write_ppm(invalid_path, {{1.0f, 0.0f, 0.0f}}, 1, 1);

        std::ifstream f(invalid_path);
        EXPECT_FALSE(f.is_open());
    }

}