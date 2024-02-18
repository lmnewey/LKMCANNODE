cmake_minimum_required(VERSION 3.8)
project(lkmcannode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)



# Declare the package dependencies
ament_export_dependencies(rclcpp)
ament_export_dependencies(can_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create a ROS 2 executable
add_executable(lkmcannode src/lkmcannode.cpp)

# Link the executable with ROS 2 libraries
ament_target_dependencies(
lkmcannode 
rclcpp 
can_msgs 
std_msgs 
geometry_msgs
tf2_ros
nav_msgs
)

# Install the executable#differential_drive_node
install(TARGETS lkmcannode
  
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_export_dependencies(rclcpp tf2_ros geometry_msgs)



ament_package()
