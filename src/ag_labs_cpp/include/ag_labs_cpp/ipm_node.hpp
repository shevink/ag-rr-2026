#ifndef AG_LABS_CPP__IPM_NODE_HPP_
#define AG_LABS_CPP__IPM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

namespace ag_labs_cpp
{
class IPMNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new IPM Node object
   */
  IPMNode();

  /**
   * @brief Destroy the IPM Node object
   */
  ~IPMNode() = default;

private:
  /**
   * @brief Callback function for processing incoming camera images
   * @param msg Incoming raw image message
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Shared processing path used by both camera streams
   * @param msg Incoming raw image message
   * @param publisher Output publisher for transformed images
   */
  void processImage(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher);

  /**
   * @brief Load and parse ROS parameters
   */
  void loadParameters();

  /**
   * @brief Initialize OpenCV transformation matrices
   */
  void initializeTransforms();

  /**
   * @brief Parse source and destination points from parameter arrays
   * @param src_array Source points parameter array
   * @param dst_array Destination points parameter array
   */
  void parseTransformPoints(const std::vector<double>& src_array,
                           const std::vector<double>& dst_array);

  // ROS2 Communication
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // OpenCV Transformation matrices
  cv::Mat perspective_matrix_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Transform points (trapezoid -> rectangle)
  std::vector<cv::Point2f> src_points_;
  std::vector<cv::Point2f> dst_points_;

  // Configuration parameters
  std::string input_topic_;
  std::string output_topic_;
  int output_width_;
  int output_height_;
  bool enable_undistortion_;
  std::string frame_id_;
};

}  // namespace ag_labs_cpp

#endif  // AG_LABS_CPP__IPM_NODE_HPP_
