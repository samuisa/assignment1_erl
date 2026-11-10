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
find_package(action_msg REQUIRED)

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

link_ros2_dependencies(navigation_comp 
  rclcpp rclcpp_components geometry_msgs nav_msgs std_msgs rclcpp_action action_msg)

rclcpp_components_register_node(navigation_comp 
  PLUGIN "gazebo_ath::NavigationManager" EXECUTABLE navigation_node)


# ===============================
# Install targets
# ===============================
install(TARGETS
  navigation_comp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch models worlds urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()