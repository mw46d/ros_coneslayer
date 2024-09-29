cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(ros_coneslayer VERSION 0.0.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

set(coneslayer_name "coneslayer_openvino_2022.1_6shave.blob")
set(config_name "coneslayer_openvino_2022.1_6shave.json")

set(coneslayer_blob "${PROJECT_SOURCE_DIR}/resources/${coneslayer_name}")
set(config_file "${PROJECT_SOURCE_DIR}/resources/${config_name}")
file(DOWNLOAD "https://github.com/mkrupczak3/Coneslayer/raw/main/coneslayer_openvino_2022.1_6shave.blob"
  ${coneslayer_blob}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA256=0ecaccfd9457ad8e7ff5c301a5df31465a8f9be068d8b1cbf85e96b3d48c47a9
  TLS_VERIFY ON)
file(DOWNLOAD "https://github.com/mkrupczak3/Coneslayer/raw/main/coneslayer.json"
  ${config_file}
  TIMEOUT 60  # seconds
  EXPECTED_HASH SHA256=5f8b0f9fc4b802f6f68973ff0b9d36bdd88dbf28b2196eea5df6fa4725a510b0
  TLS_VERIFY ON)

find_package(ament_cmake REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  depthai_ros_msgs
  depthai_bridge
  nlohmann_json
  rclcpp
  rclcpp_components
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  visualization_msgs
)

include_directories(include)

macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

  target_link_libraries("${node_name}"
                        depthai::core
                        opencv_imgproc
                        opencv_highgui)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

dai_add_node_ros2(coneslayer_node src/coneslayer_publisher.cpp)

target_compile_definitions(coneslayer_node PRIVATE BLOB_NAME="${coneslayer_name}" CONFIG_NAME="${config_name}")

add_library(${PROJECT_NAME} SHARED src/spatial_bb.cpp src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::SpatialBB")

install(TARGETS coneslayer_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY params resources rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch FILES_MATCHING PATTERN "*.py")

ament_export_include_directories(include)
install( DIRECTORY include/ DESTINATION include)

ament_package()

