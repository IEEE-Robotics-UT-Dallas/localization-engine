cmake_minimum_required(VERSION 3.8)
project(localization_engine)

# Force modern C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# --- Tell CMake to look inside the Src/ folder ---
add_executable(particle_filter_node
        Src/main.cpp
        Src/localization_node.cpp
        Src/mcl.cpp
        Src/particle.cpp
        Src/mapCreator.cpp
        Src/map.cpp
        Src/tof_sensor.cpp
)

# Link it all together
ament_target_dependencies(particle_filter_node
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
)
target_link_libraries(particle_filter_node Eigen3::Eigen)

# --- Tell CMake your headers live in the Inc/ folder AND where Eigen is ---
target_include_directories(particle_filter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
        ${EIGEN3_INCLUDE_DIRS} # <-- NEW: Include Eigen's math headers
)

ament_target_dependencies(particle_filter_node rclcpp)

# <-- NEW: Link Eigen to your node
target_link_libraries(particle_filter_node Eigen3::Eigen)

install(TARGETS
        particle_filter_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()