cmake_minimum_required(VERSION 3.8)
project(sprint1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


# Build the executable
add_executable(sprint1_node src/sprint1.cpp)

# Link dependencies
ament_target_dependencies(sprint1_node
  rclcpp
  sensor_msgs
  
)

# Install the executable
install(TARGETS
  sprint1_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
