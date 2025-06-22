cmake_minimum_required(VERSION 3.8)
project(image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# add exe
add_executable(image_undistort src/image_undistort.cc)

# include dirs
include_directories(include)

# install launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# add depend
ament_target_dependencies(image_undistort
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# install targets
install(TARGETS
  image_undistort
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  image_processing/image_brighten_node.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()