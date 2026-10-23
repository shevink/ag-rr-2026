#pragma once

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class CameraBasedLidar : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ipm_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void publish_laser_scan(const cv::Mat & mask);

public:
  CameraBasedLidar();
};