#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace ros_coneslayer {
class SpatialBB : public rclcpp::Node {
   public:
    explicit SpatialBB(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                   const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections);

    message_filters::Subscriber<sensor_msgs::msg::Image> previewSub;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> detSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection3DArray>
        syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub;

    uint32_t last_secs;
    uint32_t last_nsecs;
};

}  // namespace ros_coneslayer
