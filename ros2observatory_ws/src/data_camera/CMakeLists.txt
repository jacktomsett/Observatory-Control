cmake_minimum_required(VERSION 3.8)
project(data_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# set include paths for gphoto2
include_directories(/usr/include/gphoto2)
# set include path for other files containing functions ect.
include_directories(src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(GPHOTO2 REQUIRED)



# Add executables
add_executable(camera src/camera.cpp)
target_link_libraries(camera -lgphoto2 -lgphoto2_port)
ament_target_dependencies(camera rclcpp rclcpp_action rclcpp_components interfaces OpenCV cv_bridge sensor_msgs)

# Install executables
install(TARGETS
  camera

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
