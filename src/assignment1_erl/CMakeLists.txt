cmake_minimum_required(VERSION 3.20)
project(assignment1_erl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(assignment_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Navigation component
add_library(navigation_comp SHARED src/navigation.cpp)
target_include_directories(navigation_comp PRIVATE include)
target_link_libraries(navigation_comp PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${assignment_msg_TARGETS}
)
rclcpp_components_register_node(navigation_comp
  PLUGIN "assignment1_erl::NavigationManager"
  EXECUTABLE navigation_node
)

# FSM component
add_library(fsm_comp SHARED src/fsm.cpp)
target_include_directories(fsm_comp PRIVATE include)
target_link_libraries(fsm_comp PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${assignment_msg_TARGETS}
)
rclcpp_components_register_node(fsm_comp
  PLUGIN "assignment1_erl::FsmManager"
  EXECUTABLE fsm_node
)

# Image detection component
add_library(image_detection_comp SHARED src/image_detection.cpp)
target_include_directories(image_detection_comp PRIVATE include)
target_link_libraries(image_detection_comp PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${aruco_opencv_msgs_TARGETS}
  ${assignment_msg_TARGETS}
)
rclcpp_components_register_node(image_detection_comp
  PLUGIN "assignment1_erl::DetectionManager"
  EXECUTABLE image_detection_node
)

# Image annotator component
add_library(image_annotator_comp SHARED src/image_annotator.cpp)
target_include_directories(image_annotator_comp PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_annotator_comp PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  ${assignment_msg_TARGETS}
)
rclcpp_components_register_node(image_annotator_comp
  PLUGIN "assignment1_erl::ImageAnnotator"
  EXECUTABLE image_annotator_node
)

# Install
install(TARGETS
  navigation_comp
  fsm_comp
  image_detection_comp
  image_annotator_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  models
  worlds
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()