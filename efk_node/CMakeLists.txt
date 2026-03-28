cmake_minimum_required(VERSION 3.8)
project(efk_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the config directory to the share folder
# This is what allows the launch file to find your ekf.yaml
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()