cmake_minimum_required(VERSION 3.20)
project(gazebo_ath)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# 1. GENERAZIONE DEI MESSAGGI CUSTOM (Fondamentale per creare gli .hpp)
# ==============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoInfo.msg"
  "msg/FsmStatus.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

# ==============================================================================
# CUSTOM MACRO: IL SOSTITUTO INFALLIBILE PER ament_target_dependencies
# ==============================================================================
macro(link_ros2_dependencies target)
  foreach(dep ${ARGN})
    if(${dep}_INCLUDE_DIRS)
      target_include_directories(${target} PUBLIC ${${dep}_INCLUDE_DIRS})
    endif()
    if(${dep}_LIBRARIES)
      target_link_libraries(${target} PUBLIC ${${dep}_LIBRARIES})
    endif()
  endforeach()
endmacro()

# ===============================
# Component: Navigation
# ===============================
add_library(navigation_comp SHARED src/navigation.cpp)

# Aggiunte le dipendenze mancanti (sensor_msgs, aruco_opencv_msgs) e il target dei messaggi locali
target_link_libraries(navigation_comp PRIVATE "${cpp_typesupport_target}")
target_include_directories(navigation_comp PRIVATE include)
link_ros2_dependencies(navigation_comp 
  rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs std_msgs aruco_opencv_msgs rclcpp_action action_msg)

rclcpp_components_register_node(navigation_comp 
  PLUGIN "gazebo_ath::NavigationManager" EXECUTABLE navigation_node)


# ===============================
# Component: Fsm
# ===============================
add_library(fsm_comp SHARED src/fsm.cpp)

target_link_libraries(fsm_comp PRIVATE "${cpp_typesupport_target}")
target_include_directories(fsm_comp PRIVATE include)
link_ros2_dependencies(fsm_comp 
  rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs std_msgs aruco_opencv_msgs rclcpp_action action_msg)

rclcpp_components_register_node(fsm_comp 
  PLUGIN "gazebo_ath::FsmManager" EXECUTABLE fsm_node)


# ===============================
# Component: Image detection
# ===============================
add_library(image_detection_comp SHARED src/image_detection.cpp)

target_link_libraries(image_detection_comp PRIVATE "${cpp_typesupport_target}")
target_include_directories(image_detection_comp PRIVATE include)
link_ros2_dependencies(image_detection_comp 
  rclcpp rclcpp_components geometry_msgs sensor_msgs nav_msgs std_msgs aruco_opencv_msgs rclcpp_action action_msg)

rclcpp_components_register_node(image_detection_comp 
  PLUGIN "gazebo_ath::DetectionManager" EXECUTABLE image_detection_node)


# ===============================
# Install targets
# ===============================
install(TARGETS
  navigation_comp
  fsm_comp
  image_detection_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch models worlds urdf rviz config msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()