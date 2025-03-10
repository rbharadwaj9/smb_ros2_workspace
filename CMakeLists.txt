cmake_minimum_required(VERSION 3.5)
project(smb_bringup)

# Use C++17
add_compile_options(-std=c++17)

# Find required packages
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
