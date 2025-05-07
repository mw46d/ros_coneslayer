#include <iostream>
#include <fstream>

#include "ros_coneslayer/spatial_bb.hpp"
#include "ros_coneslayer/utils.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

rmw_qos_profile_t qos_profile{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth
    ),
    qos_profile);

namespace ros_coneslayer {

SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    detSub.subscribe(this, "nn/spatial_detections");
    gpsSub = create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",
        qos,
        std::bind(&SpatialBB::gpsCB, this, std::placeholders::_1));

    last_secs = 0;
    last_nsecs = 0;

    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    sync->registerCallback(std::bind(&SpatialBB::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("spatial_bb/overlay", 10);

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    loadAvgTimer = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&SpatialBB::loadAvgTimerCB, this));
}

void SpatialBB::loadAvgTimerCB() {
    std::string loadAvg;
    std::ifstream loadAvgFile("/proc/loadavg", std::ifstream::in);

    if (getline(loadAvgFile, loadAvg)) {
        sscanf(loadAvg.c_str(), "%f %f %f ", &loadAvg1, &loadAvg5, &loadAvg15);
    }
}

void SpatialBB::gpsCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
    latitude = msg->latitude;
    longitude = msg->longitude;
}

void SpatialBB::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                          const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);
    auto blue = cv::Scalar(255, 0, 0);
    double dt = 0.0;

    if (last_secs > 0) {
        dt = (double)(detections->header.stamp.sec - last_secs) + (double)(detections->header.stamp.nanosec - last_nsecs) * 0.000000001;
    }
    last_secs = detections->header.stamp.sec;
    last_nsecs = detections->header.stamp.nanosec; 

    for(auto& detection : detections->detections) {
        auto confidence = detection.results[0].hypothesis.score;

	if (confidence > 0.6) {
            auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size.x / 2.0;
            auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size.x / 2.0;
            auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size.y / 2.0;
            auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size.y / 2.0;

            cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
            auto labelStr = "Cone";
            utils::addTextToFrame(previewMat, labelStr, x1 + 10, y1 + 10);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << confidence * 100;
            utils::addTextToFrame(previewMat, confStr.str(), x1 + 10, y1 + 40);

            std::stringstream depthX;
            depthX << "X: " << std::setprecision(3) << detection.results[0].pose.pose.position.x << " m";
            utils::addTextToFrame(previewMat, depthX.str(), x1 + 10, y1 + 60);

            std::stringstream depthY;
            depthY << "Y: " << std::setprecision(3) << detection.results[0].pose.pose.position.y << " m";
            utils::addTextToFrame(previewMat, depthY.str(), x1 + 10, y1 + 75);
            std::stringstream depthZ;
            depthZ << "Z: " << std::setprecision(3) << detection.results[0].pose.pose.position.z << " m";
            utils::addTextToFrame(previewMat, depthZ.str(), x1 + 10, y1 + 90);
	}
    }

    std::stringstream fps;
    fps << "FPS: " << std::setprecision(3) << std::fixed << (1.0 / dt);
    utils::addTextToFrame(previewMat, fps.str(), 5, previewMat.cols - 70);

    if (latitude != 0.0) {
        std::stringstream gpss;
        gpss << "Pos: " << std::setprecision(8) << std::fixed << latitude << " " << longitude;
        utils::addTextToFrame(previewMat, gpss.str(), 5, previewMat.cols - 50);
    }

    if (loadAvg1 > 0.0) {
        std::stringstream las;
        las << "LoadAvg: " << std::setprecision(2) << loadAvg1 << " " << loadAvg5 << " " << loadAvg15;
	utils::addTextToFrame(previewMat, las.str(), 5, previewMat.cols - 30);
    }

    std::stringstream ts;
    ts << "Time: " << preview->header.stamp.sec;
    utils::addTextToFrame(previewMat, ts.str(), 5, previewMat.cols - 10);

    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace ros_coneslayer
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_coneslayer::SpatialBB);
