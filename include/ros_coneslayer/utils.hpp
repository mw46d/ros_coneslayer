#pragma once

#include "opencv2/core/mat.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rclcpp {
class Logger;
}

namespace ros_coneslayer {
namespace utils {
cv::Mat msgToMat(const rclcpp::Logger& logger, const sensor_msgs::msg::Image::ConstSharedPtr& img, const std::string& encoding);
void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y);
}  // namespace utils
}  // namespace ros_coneslayer
