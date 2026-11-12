
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
find_package(rclcpp_action REQUIRED)

# Dependencies
macro(link_ros2_dependencies target)
  foreach(dep ${ARGN})
    if(${dep}_INCLUDE_DIRS)
      target_include_directories(
        ${target} PUBLIC ${${dep}_INCLUDE_DIRS}
      )
    endif()
    if(${dep}_LIBRARIES)
      target_link_libraries(
        ${target} PUBLIC ${${dep}_LIBRARIES}
      )
    endif()
  endforeach()
endmacro()

# Navigation component
add_library(navigation_comp SHARED src/navigation.cpp)

target_include_directories(navigation_comp PRIVATE include)

link_ros2_dependencies(navigation_comp
  assignment_msg
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  aruco_opencv_msgs
  rclcpp_action
)

rclcpp_components_register_node(navigation_comp
  PLUGIN "assignment1_erl::NavigationManager"
  EXECUTABLE navigation_node
)

# FSM component
add_library(fsm_comp SHARED src/fsm.cpp)

target_include_directories(fsm_comp PRIVATE include)

link_ros2_dependencies(fsm_comp
  assignment_msg
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  aruco_opencv_msgs
  rclcpp_action
)

rclcpp_components_register_node(fsm_comp
  PLUGIN "assignment1_erl::FsmManager"
  EXECUTABLE fsm_node
)

# Image detection component
add_library(image_detection_comp SHARED src/image_detection.cpp)

target_include_directories(image_detection_comp PRIVATE include)

link_ros2_dependencies(image_detection_comp
  assignment_msg
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  aruco_opencv_msgs
  rclcpp_action
)

rclcpp_components_register_node(image_detection_comp
  PLUGIN "assignment1_erl::DetectionManager"
  EXECUTABLE image_detection_node
)

# Install
install(TARGETS
  navigation_comp
  fsm_comp
  image_detection_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch models worlds urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
