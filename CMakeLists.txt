cmake_minimum_required(VERSION 3.8)
project(colcon_debug_helper)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
install(
        DIRECTORY src/
        DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

# Package configuration
ament_package()
