#include "ag_labs_cpp/ipm_node.hpp"

#include <sensor_msgs/image_encodings.hpp>

namespace ag_labs_cpp
{

IPMNode::IPMNode()
: Node("ipm_node"),
  output_width_(640),
  output_height_(480),
  enable_undistortion_(false),
  frame_id_("ipm_ground_virtuallink")
{
  loadParameters();

  initializeTransforms();

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, 10,
    std::bind(&IPMNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "IPM node mapping %s -> %s (%dx%d)",
    input_topic_.c_str(), output_topic_.c_str(), output_width_, output_height_);
}

void IPMNode::loadParameters()
{
  this->declare_parameter("input_topic", "/camera/image_raw");
  this->declare_parameter("output_topic", "/camera/ipm_image");
  this->declare_parameter("output_width", 640);
  this->declare_parameter("output_height", 480);
  this->declare_parameter("frame_id", "ipm_ground_virtuallink");
  this->declare_parameter("enable_undistortion", false);

  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();

  output_width_ = this->get_parameter("output_width").as_int();
  output_height_ = this->get_parameter("output_height").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  enable_undistortion_ = this->get_parameter("enable_undistortion").as_bool();

  std::vector<double> default_src = {235.0, 290.0, 405.0, 290.0, 639.0, 475.0, 0.0, 475.0};
  std::vector<double> default_dst = {0.0, 0.0, 640.0, 0.0, 640.0, 480.0, 0.0, 480.0};

  this->declare_parameter("src_points", default_src);
  this->declare_parameter("dst_points", default_dst);

  auto src_array = this->get_parameter("src_points").as_double_array();
  auto dst_array = this->get_parameter("dst_points").as_double_array();

  parseTransformPoints(src_array, dst_array);

  if (enable_undistortion_) {
    this->declare_parameter("camera_matrix", std::vector<double>{});
    this->declare_parameter("distortion_coefficients", std::vector<double>{});

    auto cam_matrix_array = this->get_parameter("camera_matrix").as_double_array();
    auto dist_coeffs_array = this->get_parameter("distortion_coefficients").as_double_array();

    if (cam_matrix_array.size() == 9) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 9; i++) {
        camera_matrix_.at<double>(i / 3, i % 3) = cam_matrix_array[i];
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Camera matrix not properly configured, disabling undistortion");
      enable_undistortion_ = false;
    }

    if (dist_coeffs_array.size() >= 4) {
      dist_coeffs_ = cv::Mat(dist_coeffs_array.size(), 1, CV_64F);
      for (size_t i = 0; i < dist_coeffs_array.size(); i++) {
        dist_coeffs_.at<double>(i, 0) = dist_coeffs_array[i];
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Distortion coefficients not properly configured, disabling undistortion");
      enable_undistortion_ = false;
    }
  }
}

void IPMNode::parseTransformPoints(const std::vector<double>& src_array,
                                   const std::vector<double>& dst_array)
{
  if (src_array.size() != 8 || dst_array.size() != 8) {
    RCLCPP_ERROR(this->get_logger(),
      "Transform points must have exactly 8 values each (4 points with x,y coordinates)");
    RCLCPP_ERROR(this->get_logger(),
      "Got src_points size: %zu, dst_points size: %zu", src_array.size(), dst_array.size());
    throw std::runtime_error("Invalid transform point configuration");
  }

  src_points_.clear();
  for (size_t i = 0; i < src_array.size(); i += 2) {
    src_points_.emplace_back(static_cast<float>(src_array[i]),
                            static_cast<float>(src_array[i + 1]));
  }

  dst_points_.clear();
  for (size_t i = 0; i < dst_array.size(); i += 2) {
    dst_points_.emplace_back(static_cast<float>(dst_array[i]),
                            static_cast<float>(dst_array[i + 1]));
  }
}

void IPMNode::initializeTransforms()
{
  if (src_points_.size() == 4 && dst_points_.size() == 4) {
    perspective_matrix_ = cv::getPerspectiveTransform(src_points_, dst_points_);
  } else {
    throw std::runtime_error("Invalid transform point configuration");
  }
}

void IPMNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  processImage(msg, image_pub_);
}

void IPMNode::processImage(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher)
{
  try {
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat & input_image = cv_ptr->image;
    if (input_image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    cv::Mat processed_image;

    if (enable_undistortion_ && !camera_matrix_.empty() && !dist_coeffs_.empty()) {
      cv::undistort(input_image, processed_image, camera_matrix_, dist_coeffs_);
    } else {
      processed_image = input_image;
    }

    cv::Mat ipm_image;
    cv::warpPerspective(processed_image, ipm_image, perspective_matrix_,
                       cv::Size(output_width_, output_height_),
                       cv::INTER_CUBIC,
                       cv::BORDER_CONSTANT,
                       cv::Scalar(255, 0, 255));

    sensor_msgs::msg::Image::SharedPtr out_msg =
      cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, ipm_image).toImageMsg();
    out_msg->header.frame_id = frame_id_;

    publisher->publish(*out_msg);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }
}

}  // namespace ag_labs_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ag_labs_cpp::IPMNode>());
  rclcpp::shutdown();
  return 0;
}
