#include <memory>
#include "camera_based_lidar.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraBasedLidar>());
  rclcpp::shutdown();
  return 0;
}