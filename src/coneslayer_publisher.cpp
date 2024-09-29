#include <cstdio>
#include <iostream>
#include <nlohmann/json.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath, std::string configPath) {
    std::ifstream configIfs(configPath);
    nlohmann::json j_config = nlohmann::json::parse(configIfs);
    nlohmann::json j_nnConfig;
    nlohmann::json j_metadata;
    int classes = 0;
    int coordinates = 0;
    std::vector<float> anchors;
    nlohmann::json j_anchorMasks;
    double iouThreshold = 0.0;
    double confidenceThreshold = 0.0;
    nlohmann::json j_nnMappings;

    if (j_config.contains("nn_config")) {
        j_nnConfig = j_config["nn_config"];
    }

    //  extract metadata
    if (j_nnConfig.contains("NN_specific_metadata")) {
        j_metadata = j_nnConfig["NN_specific_metadata"];
    }
    if (j_metadata.contains("classes")) {
        classes = j_metadata["classes"];
    }
    if (j_metadata.contains("coordinates")) {
        coordinates = j_metadata["coordinates"];
    }
    if (j_metadata.contains("anchors") && j_metadata["anchors"].is_array()) {
        anchors = j_metadata["anchors"].get<std::vector<float>>();
    }
    else {
        anchors = std::vector<float>(0.0);
    }
    if (j_metadata.contains("anchor_masks")) {
        j_anchorMasks = j_metadata["anchor_masks"];
    }
    if (j_metadata.contains("iou_threshold")) {
        iouThreshold = j_metadata["iou_threshold"];
    }
    if (j_metadata.contains("confidence_threshold")) {
        confidenceThreshold = j_metadata["confidence_threshold"];
    }

    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    xoutNN->setStreamName("detections");
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");

    // Properties
    camRgb->setPreviewSize(1920, 1080);
    // 1 camRgb->setPreviewSize(1280, 720);
    // 1 camRgb->setPreviewKeepAspectRatio(true);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(25);

    imageManip->initialConfig.setResizeThumbnail(416, 416);
    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    // Align depth map to the perspective of RGB camera, on which inference is done
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->setOutputSize(monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());

    /// setting node configs
    // ?? stereo->initialConfig.setConfidenceThreshold(confidence);
    // ?? stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    // ?? stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    // ?? stereo->setSubpixel(subpixel);
    // ?? stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(confidenceThreshold);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(11000);
    spatialDetectionNetwork->setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm(1));

    // Yolo specific parameters
    spatialDetectionNetwork->setNumClasses(classes);
    spatialDetectionNetwork->setCoordinateSize(coordinates);
    spatialDetectionNetwork->setAnchors(anchors);  // ->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks(j_anchorMasks); // >setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(iouThreshold);
    spatialDetectionNetwork->setNumInferenceThreads(2);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    camRgb->preview.link(imageManip->inputImage);
    imageManip->out.link(spatialDetectionNetwork->input);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    if (syncNN) {
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
        spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    }
    else {
        imageManip->out.link(xoutRgb->input);
        stereo->depth.link(xoutDepth->input);
    }

    spatialDetectionNetwork->out.link(xoutNN->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("coneslayer_node");

    std::string tfPrefix, resourceBaseFolder;
    std::string nnName(BLOB_NAME), configName(CONFIG_NAME);
    bool syncNN;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("sync_nn", true);
    node->declare_parameter("nnName", "");
    node->declare_parameter("configName", "");
    node->declare_parameter("resourceBaseFolder", "");

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }
    node->get_parameter("configName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("configName", configName);
    }

    std::string nnPath = resourceBaseFolder + "/" + nnName;
    std::string configPath = resourceBaseFolder + "/" + configName;
    dai::Pipeline pipeline = createPipeline(syncNN, nnPath, configPath);
    dai::Device device(pipeline);

    auto rgbQueue = device.getOutputQueue("rgb", 30, false);
    auto detectionQueue = device.getOutputQueue("detections", 30, false);
    auto depthQueue = device.getOutputQueue("depth", 30, false);
    auto calibrationHandler = device.readCalibration();

    int width = 640, height = 480;

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>
        rgbPublish(rgbQueue,
                   node,
                   std::string("color/image"),
                   std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                             &rgbConverter,  // since the converter has the same frame name
                                             // and image type is also same we can reuse it
                             std::placeholders::_1,
                             std::placeholders::_2),
                   30,
                   rgbCameraInfo,
                   "color");

    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
    // dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
    dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection3DArray, dai::SpatialImgDetections>
        detectionPublish(detectionQueue,
                   node,
                   std::string("color/spatial_detections"),
                   // std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                   std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosVisionMsg,
                             &detConverter,
			     std::placeholders::_1,
			     std::placeholders::_2),
                   30);

    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>
        depthPublish(depthQueue,
                   node,
                   std::string("stereo/depth"),
                   std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
			     &depthConverter,
			     std::placeholders::_1,
			     std::placeholders::_2),
                   30,
                   rightCameraInfo,
                   "stereo");

    depthPublish.addPublisherCallback();
    detectionPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    rclcpp::spin(node);

    return 0;
}
