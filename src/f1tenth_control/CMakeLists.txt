cmake_minimum_required(VERSION 3.8)
project(f1tenth_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# include directories
include_directories(${IPOPT_INCLUDE_DIRS})

# Build an executable named 'slash_mpc' from the source file at src/slash_mpc.cpp
# This will be the main entry point for the MPC control node 
add_executable(slash_mpc src/slash_mpc.cpp)

# Link the executable against the rclcpp library (ROS 2 C++ client library)
# This gives the node access to ROS 2 APIs like publishing, subscribing, etc.
ament_target_dependencies(slash_mpc rclcpp)

# Link the executable against the Ipopt optimization library
# Ipopt will be used to solve the nonlinear optimization problem in the MPC controller
target_link_libraries(slash_mpc ${IPOPT_LIBRARIES})

# Install the compiled slash_mpc binary into the ROS2 install path
# This ensures it can be run using the `ros2 run f1tenth_control slash_mpc` command
install(TARGETS slash_mpc DESTINATION lib/${PROJECT_NAME})

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
