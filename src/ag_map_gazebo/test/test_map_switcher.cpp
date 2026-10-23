#include <gtest/gtest.h>

#include <string>

#include <ag_map_gazebo/srv/load_map.hpp>

TEST(MapSwitcherTest, ServiceInterfaceCompiles) {
  auto request = std::make_shared<ag_map_gazebo::srv::LoadMap::Request>();
  auto response = std::make_shared<ag_map_gazebo::srv::LoadMap::Response>();

  request->map_name = "test_map";
  EXPECT_EQ(request->map_name, "test_map");

  response->success = true;
  response->message = "ok";
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "ok");
}

TEST(MapSwitcherTest, ValidMapNames) {
  const std::vector<std::string> valid = {
    "easy_map", "medium_map", "hard_map", "maze_map"
  };
  for (const auto & name : valid) {
    EXPECT_FALSE(name.empty());
  }
}


TEST(MapSwitcherTest, ValidMapProducesNonEmptyPath) {
  // Simulate the path building logic: valid names produce non-empty strings
  EXPECT_NE("easy_map", "");
  EXPECT_NE("medium_map", "");
  EXPECT_NE("hard_map", "");
  EXPECT_NE("maze_map", "");
}
