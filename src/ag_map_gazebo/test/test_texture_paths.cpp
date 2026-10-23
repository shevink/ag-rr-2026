#include <gtest/gtest.h>

#include <filesystem>
#include <string>

static std::string getTexturesDir()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path().string() +
         "/textures";
}

TEST(TexturePathsTest, TextureDirectoryExists) {
  ASSERT_TRUE(std::filesystem::exists(getTexturesDir()));
}

TEST(TexturePathsTest, TextureDirIsDirectory) {
  ASSERT_TRUE(std::filesystem::is_directory(getTexturesDir()));
}
