
cmake_minimum_required(VERSION 3.20)
project(assignment1_erl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(assignment_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)

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


# Navigation standalone node
add_executable(navigation_node
  src/navigation.cpp
)

target_include_directories(navigation_node PRIVATE include)

ament_target_dependencies(navigation_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  assignment_msg
)


# FSM standalone node
add_executable(fsm_node
  src/fsm.cpp
)

target_include_directories(fsm_node PRIVATE include)

ament_target_dependencies(fsm_node
  rclcpp
  std_msgs
  assignment_msg
)


# Image detection standalone node
add_executable(image_detection_node
  src/image_detection.cpp
)

target_include_directories(image_detection_node PRIVATE include)

ament_target_dependencies(image_detection_node
  rclcpp
  sensor_msgs
  aruco_opencv_msgs
  assignment_msg
)


# Install executables
install(TARGETS
  navigation_node
  fsm_node
  image_detection_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install project resources
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
