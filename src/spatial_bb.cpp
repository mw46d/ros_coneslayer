#include "ros_coneslayer/spatial_bb.hpp"
#include "ros_coneslayer/utils.hpp"

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point32.hpp"
#include "opencv2/opencv.hpp"

namespace ros_coneslayer {

SpatialBB::SpatialBB(const rclcpp::NodeOptions& options) : rclcpp::Node("spatial_bb_node", options) {
    onInit();
}
void SpatialBB::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    detSub.subscribe(this, "nn/spatial_detections");

    last_secs = 0;
    last_nsecs = 0;

    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    sync->registerCallback(std::bind(&SpatialBB::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("spatial_bb/overlay", 10);
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
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size.x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size.x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size.y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size.y / 2.0;

        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
        auto labelStr = "Cone";
        utils::addTextToFrame(previewMat, labelStr, x1 + 10, y1 + 10);
        auto confidence = detection.results[0].hypothesis.score;
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

    std::stringstream fps;
    fps << "FPS: " << std::setprecision(3) << std::fixed << (1.0 / dt);
    utils::addTextToFrame(previewMat, fps.str(), 10, previewMat.cols - 10);

    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace ros_coneslayer
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_coneslayer::SpatialBB);
