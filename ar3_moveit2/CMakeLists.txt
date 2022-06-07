cmake_minimum_required(VERSION 3.5)
project(ar3_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch meshes DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
