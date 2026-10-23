#include <gtest/gtest.h>

#include <filesystem>
#include <string>

static std::string getModelsDir()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path().string() +
         "/models";
}

TEST(SdfModelTest, GroundPlaneModelSdfExists) {
  std::string path = getModelsDir() + "/ground_plane.sdf";
  ASSERT_TRUE(std::filesystem::exists(path));
}

TEST(SdfModelTest, GroundPlaneTexturesExist) {
  std::string dir = getModelsDir() + "/ground_plane/materials/textures";
  ASSERT_TRUE(std::filesystem::is_directory(dir));
  int pngCount = 0;
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().extension() == ".png") {pngCount++;}
  }
  ASSERT_GE(pngCount, 1);
}
