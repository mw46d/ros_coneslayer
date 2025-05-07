#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace ros_coneslayer {
class SpatialBB : public rclcpp::Node {
   public:
    explicit SpatialBB(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                   const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections);
    void gpsCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
    void loadAvgTimerCB();

    message_filters::Subscriber<sensor_msgs::msg::Image> previewSub;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> detSub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection3DArray>
        syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub;

    rclcpp::TimerBase::SharedPtr loadAvgTimer;

    double latitude = 0.0;
    double longitude = 0.0;
    float loadAvg1 = 0.0;
    float loadAvg5 = 0.0;
    float loadAvg15 = 0.0;

    uint32_t last_secs;
    uint32_t last_nsecs;
};

}  // namespace ros_coneslayer
