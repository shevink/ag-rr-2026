#include "camera_based_lidar.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <memory>

namespace {
  std::optional<cv::Mat> to_cv_mat(const sensor_msgs::msg::Image::SharedPtr & msg)
  {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      return cv_ptr->image;
    } catch (const cv_bridge::Exception &e) {
      return std::nullopt;
    }
  }

  cv::Mat apply_gray_filter(const cv::Mat & bgr)
  {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  cv::Mat apply_blur(const cv::Mat & gray, int ksize, double sigma)
  {
    cv::Mat gray_blur;
    int k = ksize;
    if (k <= 0) {
      k = 1;
    }
    if (k % 2 == 0) {
      ++k;
    }
    cv::GaussianBlur(gray, gray_blur, cv::Size(k, k), sigma);
    return gray_blur;
  }

  std::pair<cv::Mat, cv::Mat> compute_gradients(const cv::Mat & gray_blur)
  {
    cv::Mat grad_x;
    cv::Mat grad_y;
    cv::Sobel(gray_blur, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(gray_blur, grad_y, CV_32F, 0, 1, 3);
    return {grad_x, grad_y};
  }

  std::pair<cv::Mat, cv::Mat> compute_edge_strength_masks(
    const cv::Mat & grad_x,
    const cv::Mat & grad_y,
    int grad_thresh)
  {
    cv::Mat abs_grad_x;
    cv::Mat abs_grad_y;
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::Mat mask_x;
    cv::Mat mask_y;
    cv::threshold(abs_grad_x, mask_x, grad_thresh, 255, cv::THRESH_BINARY);
    cv::threshold(abs_grad_y, mask_y, grad_thresh, 255, cv::THRESH_BINARY);
    return {mask_x, mask_y};
  }

  cv::Mat compute_orientation_mask(const cv::Mat & grad_x, const cv::Mat & grad_y, bool filter_for_x, float gradient_scale)
  {
    cv::Mat abs_x = cv::abs(grad_x);
    cv::Mat abs_y = cv::abs(grad_y);
    cv::Mat orientation_mask;
    if (filter_for_x) {
      orientation_mask = abs_x > (gradient_scale * abs_y);
    } else {
      orientation_mask = abs_y > (gradient_scale * abs_x);
    }
    return orientation_mask;
  }

  cv::Mat combine_edge_masks(const cv::Mat & edge_strength_mask, const cv::Mat & orientation_mask)
  {
    cv::Mat edge_mask;
    cv::bitwise_and(edge_strength_mask, orientation_mask, edge_mask);
    return edge_mask;
  }

  void remove_bottom_rectangle(cv::Mat & edge_mask, float fraction)
  {
    const int height = edge_mask.rows;
    const int width = edge_mask.cols;
    int rem = static_cast<int>(height * fraction);
    if (rem <= 0) {
      return;
    }
    if (rem > height) {
      rem = height;
    }
    cv::Rect r(0, height - rem, width, rem);
    cv::rectangle(edge_mask, r, cv::Scalar(0), cv::FILLED);
  }

  void remove_ipm_caused_triangles(cv::Mat & edge_mask,
    float height_left_frac,
    float height_right_frac,
    float width_left_frac,
    float width_right_frac)
  {
    const int height = edge_mask.rows;
    const int width = edge_mask.cols;

    // Proportions measured for the IPM artifacts. Adjust via parameters.
    const int height_left_triangle = static_cast<int>(height * height_left_frac);
    const int height_right_triangle = static_cast<int>(height * height_right_frac);
    const int width_left_triangle = static_cast<int>(width * width_left_frac);
    const int width_right_triangle = static_cast<int>(width * width_right_frac);

    std::vector<cv::Point> left_tri = {
      cv::Point(0, height - 1),
      cv::Point(std::max(0, width_left_triangle), height - 1),
      cv::Point(0, std::max(0, height - height_left_triangle))
    };

    std::vector<cv::Point> right_tri = {
      cv::Point(width - 1, height - 1),
      cv::Point(std::max(0, width - width_right_triangle), height - 1),
      cv::Point(width - 1, std::max(0, height - height_right_triangle))
    };

    cv::fillConvexPoly(edge_mask, left_tri.data(), static_cast<int>(left_tri.size()), cv::Scalar(0));
    cv::fillConvexPoly(edge_mask, right_tri.data(), static_cast<int>(right_tri.size()), cv::Scalar(0));
  }

  void cleanup_edge_mask(cv::Mat & edge_mask, int open_size, int close_size)
  {
    int os = open_size <= 0 ? 1 : open_size;
    int cs = close_size <= 0 ? 1 : close_size;
    if (os % 2 == 0) ++os;
    if (cs % 2 == 0) ++cs;
    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(os, os));
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(cs, cs));
    cv::morphologyEx(edge_mask, edge_mask, cv::MORPH_OPEN, open_kernel);
    cv::morphologyEx(edge_mask, edge_mask, cv::MORPH_CLOSE, close_kernel);
  }

  /**
  // create and publish a color overlay: vertical=red, horizontal=green
  cv::Mat DEBUG_create_orientation_overlay(const cv::Mat & mask_x, const cv::Mat & mask_y)
  {
    cv::Mat mx;
    cv::Mat my;
    if (mask_x.type() != CV_8U) {
      mask_x.convertTo(mx, CV_8U);
    } else {
      mx = mask_x;
    }
    if (mask_y.type() != CV_8U) {
      mask_y.convertTo(my, CV_8U);
    } else {
      my = mask_y;
    }

    cv::Mat color_overlay(mask_x.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<cv::Mat> ch(3);
    ch[0] = cv::Mat::zeros(mx.size(), CV_8U); // B channel
    ch[1] = my; // G channel -> Y-direction (horizontal) edges shown green
    ch[2] = mx; // R channel -> X-direction (vertical) edges shown red
    cv::merge(ch, color_overlay);
    return color_overlay;
  }
  **/

  /**
  void DEBUG_publish_mask(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & debug_mask_pub,
    const std_msgs::msg::Header & header,
    const cv::Mat & edge_mask)
  {
    if (!debug_mask_pub) {
      return;
    }

    cv::Mat mask_to_pub;
    if (edge_mask.type() != CV_8UC1) {
      edge_mask.convertTo(mask_to_pub, CV_8U);
    } else {
      mask_to_pub = edge_mask;
    }

    auto out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask_to_pub).toImageMsg();
    debug_mask_pub->publish(*out_msg);
  }
  **/
} // namespace

CameraBasedLidar::CameraBasedLidar() : Node("camera_based_lidar")
{
  ipm_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/ipm_image", 10, std::bind(&CameraBasedLidar::on_image, this, std::placeholders::_1));
  debug_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_based_lidar/debug_white_mask", 10);
  laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/image_based_lidar/lidar", 10);
  
  // Meters per pixel for the IPM image. Used to convert pixel distances to real-world distances for LaserScan generation.
  this->declare_parameter("meters_per_pixel", 0.003f);

  // Horizontal field of view of the "fake" laser scanner (radians).
  // This is at 180° because the IPM image sees a trapazoidal area in front of the vehicle,
  // while the lidar only sees a triangular area. The extra FOV allows the generated LaserScan
  // to cover the entire IPM area.
  this->declare_parameter("horizontal_fov", 3.1415926535f);

  // The fake laser scanner starts at the left edge of the horizontal_fov_ and moves to the right.
  // This variable defines the angle increment between each "laser ray" in radians.
  // In practice,  this controls the resolution of the generated LaserScan.
  // A smaller angle increment means more rays and higher resolution.
  this->declare_parameter("angle_increment", 0.0000174532925f);

  // Image processing parameters (exposed as ROS parameters)
  // Gaussian blur kernel size (odd integer). Values <=0 are clamped to 1; even values are incremented.
  // Used to blurr the image before gradient computation, which can help reduce noise.
  this->declare_parameter("blur_kernel_size", 5);

  // Gaussian blur sigma (standard deviation). If 0, OpenCV derives sigma from kernel size.
  // No idea what that does, sorry. Just exposing it for tuning in case the default behavior isn't good.
  this->declare_parameter("blur_sigma", 0.0);

  // Threshold applied to absolute gradient magnitude (0..255) to detect edges.
  // Higher values means stricter edge detection, which can result in fewer false positives but also more false negatives.
  this->declare_parameter("gradient_threshold", 35);

  // Orientation mask scale factor: how much stronger one gradient direction must be
  // compared to the other to be considered dominant.
  // Used to diffrenciate between vertical and horizontal edges. The actual value is try-and-error.
  this->declare_parameter("gradient_scale", 1.3f);

  // Morphological opening kernel size (odd integer) to remove small noise in the mask.
  // Used to remove small false positives. A larger kernel will remove more noise but
  // can also remove small true positives.
  this->declare_parameter("morph_open_size", 3);

  // Morphological closing kernel size (odd integer) to close small gaps in the mask.
  // If a lane is detected by its two edges with a small gap in between, this can help fill the gap
  // and create a more complete lane marking.
  this->declare_parameter("morph_close_size", 7);

  // Fraction of image height to remove from the bottom (e.g., vehicle hood). Range [0,1].
  this->declare_parameter("bottom_remove_fraction", 0.04f);

  // Fractions (0..1) used to compute left/right triangular masks that remove IPM artifacts:
  // height fraction for the left triangle.
  this->declare_parameter("ipm_left_tri_height_fraction", 0.77f);
  // height fraction for the right triangle.
  this->declare_parameter("ipm_right_tri_height_fraction", 0.78f);
  // width fraction for the left triangle base.
  this->declare_parameter("ipm_left_tri_width_fraction", 0.375);
  // width fraction for the right triangle base.
  this->declare_parameter("ipm_right_tri_width_fraction", 0.366779f);

  // Laser scan parameters
  // Minimum and maximum valid scan ranges in meters.
  this->declare_parameter("scan_range_min", 0.05f);
  this->declare_parameter("scan_range_max", 10.0f);
  this->declare_parameter("lidar_frame_id", std::string("lidar_front_link"));
}

void CameraBasedLidar::publish_laser_scan(const cv::Mat & mask)
{
  if (!laserscan_pub_) {
    return;
  }

  if (mask.empty()) {
    return;
  }

  cv::Mat mono_mask;
  if (mask.type() != CV_8UC1) {
    mask.convertTo(mono_mask, CV_8U);
  } else {
    mono_mask = mask;
  }

  const float meters_per_pixel = static_cast<float>(this->get_parameter("meters_per_pixel").as_double());
  const float horizontal_fov = static_cast<float>(this->get_parameter("horizontal_fov").as_double());
  const float angle_increment = static_cast<float>(this->get_parameter("angle_increment").as_double());
  const float scan_range_min = static_cast<float>(this->get_parameter("scan_range_min").as_double());
  const float scan_range_max = static_cast<float>(this->get_parameter("scan_range_max").as_double());
  const std::string lidar_frame_id = this->get_parameter("lidar_frame_id").as_string();

  const float angle_min = -0.5f * horizontal_fov;
  const float angle_max = 0.5f * horizontal_fov;
  const float range_min = scan_range_min;
  const float range_max = scan_range_max;

  if (angle_increment <= 0.0f || horizontal_fov <= 0.0f || meters_per_pixel <= 0.0f) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Invalid scan parameters: meters_per_pixel=%f horizontal_fov=%f angle_increment=%f",
      meters_per_pixel, horizontal_fov, angle_increment);
    return;
  }

  const int beam_count = static_cast<int>(std::floor((angle_max - angle_min) / angle_increment)) + 1;

  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp = this->now();
  scan_msg.header.frame_id = lidar_frame_id;
  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = angle_increment;
  scan_msg.time_increment = 0.0f;
  scan_msg.scan_time = 0.0f;
  scan_msg.range_min = range_min;
  scan_msg.range_max = range_max;
  scan_msg.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

  const float not_transformed_new_origin_x = 0.5f * static_cast<float>(mono_mask.cols - 1);
  const float not_transformed_new_origin_y = static_cast<float>(mono_mask.rows - 1);

  std::vector<cv::Point> lane_points;
  cv::findNonZero(mono_mask, lane_points);

  for (const cv::Point & pt : lane_points) {
    const float not_transformed_x = static_cast<float>(pt.x);
    const float not_transformed_y = static_cast<float>(pt.y);

    // Robot frame: x forward (image up), y left (image left).
    const float transformed_x = (not_transformed_new_origin_y - not_transformed_y) * meters_per_pixel;
    const float transformed_y = (not_transformed_new_origin_x - not_transformed_x) * meters_per_pixel;

    const float range = std::hypot(transformed_x, transformed_y);
    if (range < range_min || range > range_max) {
      continue;
    }

    const float angle = std::atan2(transformed_y, transformed_x);
    if (angle < angle_min || angle > angle_max) {
      continue;
    }

    int beam_index = static_cast<int>(std::floor((angle - angle_min) / angle_increment));
    if (beam_index == beam_count && std::abs(angle - angle_max) < 1e-6f) {
      beam_index = beam_count - 1;
    }
    if (beam_index < 0 || beam_index >= beam_count) {
      continue;
    }

    if (range < scan_msg.ranges[beam_index]) {
      scan_msg.ranges[beam_index] = range;
    }
  }

  laserscan_pub_->publish(scan_msg);
}

void CameraBasedLidar::on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const int blur_kernel_size = this->get_parameter("blur_kernel_size").as_int();
  const double blur_sigma = this->get_parameter("blur_sigma").as_double();
  const int gradient_threshold = this->get_parameter("gradient_threshold").as_int();
  const float gradient_scale = static_cast<float>(this->get_parameter("gradient_scale").as_double());
  const int morph_open_size = this->get_parameter("morph_open_size").as_int();
  const int morph_close_size = this->get_parameter("morph_close_size").as_int();
  const float bottom_remove_fraction = static_cast<float>(this->get_parameter("bottom_remove_fraction").as_double());
  const float ipm_left_tri_height_frac = static_cast<float>(
    this->get_parameter("ipm_left_tri_height_fraction").as_double());
  const float ipm_right_tri_height_frac = static_cast<float>(
    this->get_parameter("ipm_right_tri_height_fraction").as_double());
  const float ipm_left_tri_width_frac = static_cast<float>(
    this->get_parameter("ipm_left_tri_width_fraction").as_double());
  const float ipm_right_tri_width_frac = static_cast<float>(
    this->get_parameter("ipm_right_tri_width_fraction").as_double());

  std::optional<cv::Mat> bgr_opt = to_cv_mat(msg);
  if (!bgr_opt) {
    RCLCPP_WARN(this->get_logger(), "Failed to convert incoming image to BGR");
    return;
  }

  cv::Mat bgr = std::move(*bgr_opt);

  cv::Mat gray = apply_gray_filter(bgr);
  cv::Mat gray_blur = apply_blur(gray, blur_kernel_size, blur_sigma);

  auto [grad_x, grad_y] = compute_gradients(gray_blur);

  auto [edge_strength_x, edge_strength_y] = compute_edge_strength_masks(grad_x, grad_y, gradient_threshold);

  cv::Mat orientation_mask_x = compute_orientation_mask(grad_x, grad_y, true, gradient_scale);
  cv::Mat orientation_mask_y = compute_orientation_mask(grad_x, grad_y, false, gradient_scale);

  cv::Mat edges_x = combine_edge_masks(edge_strength_x, orientation_mask_x);
  cv::Mat edges_y = combine_edge_masks(edge_strength_y, orientation_mask_y);

  cleanup_edge_mask(edges_x, morph_open_size, morph_close_size);
  cleanup_edge_mask(edges_y, morph_open_size, morph_close_size);

  remove_ipm_caused_triangles(edges_x, ipm_left_tri_height_frac, ipm_right_tri_height_frac, ipm_left_tri_width_frac, ipm_right_tri_width_frac);
  remove_ipm_caused_triangles(edges_y, ipm_left_tri_height_frac, ipm_right_tri_height_frac, ipm_left_tri_width_frac, ipm_right_tri_width_frac);

  remove_bottom_rectangle(edges_x, bottom_remove_fraction);
  remove_bottom_rectangle(edges_y, bottom_remove_fraction);

  cv::Mat edge_mask;
  cv::bitwise_or(edges_x, edges_y, edge_mask);

  cv::Mat mask_to_pub;
  if (edge_mask.type() != CV_8UC1) {
    edge_mask.convertTo(mask_to_pub, CV_8U);
  } else {
    mask_to_pub = edge_mask;
  }
  auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, mask_to_pub).toImageMsg();
  if (debug_mask_pub_) {
    debug_mask_pub_->publish(*out_msg);
  }

  publish_laser_scan(edge_mask);
}
