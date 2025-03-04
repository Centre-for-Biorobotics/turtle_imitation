cmake_minimum_required(VERSION 3.8)
project(turtle_motion_rft)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_PATH}
)
#install python modules
install(DIRECTORY
  launch
  ml_models
  DESTINATION share/${PROJECT_NAME}/
)
file(GLOB_RECURSE SCRIPT_FILES scripts/*.py)

install(PROGRAMS ${SCRIPT_FILES}
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
