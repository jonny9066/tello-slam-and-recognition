cmake_minimum_required(VERSION 3.8)
project(webcam_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(webcam_node src/webcam_node.cpp)
target_include_directories(webcam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${${OpenCV_INCLUDE_DIRS}})
target_compile_features(webcam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(webcam_node
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_msgs"
    "camera_info_manager"
    "image_transport"
    "camera_calibration_parsers"
    "OpenCV"
    "cv_bridge"
    )

install(TARGETS webcam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

