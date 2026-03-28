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
find_package(visualization_msgs REQUIRED)

# --- NEW: Add the TF2 libraries ---
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(particle_filter_node
        Src/main.cpp
        Src/localization_node.cpp
        Src/mcl.cpp
        Src/particle.cpp
        Src/mapCreator.cpp
        Src/map.cpp
        Src/tof_sensor.cpp
)

ament_target_dependencies(particle_filter_node
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        visualization_msgs
        # --- NEW: Link them to your node ---
        tf2
        tf2_ros
        tf2_geometry_msgs
)
target_link_libraries(particle_filter_node Eigen3::Eigen)

# --- Tell CMake your headers live in the Inc/ folder AND where Eigen is ---
target_include_directories(particle_filter_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
        ${EIGEN3_INCLUDE_DIRS} # <-- NEW: Include Eigen's math headers
)

# Install the Python bridge script from the NEW local scripts folder
install(PROGRAMS
        scripts/tof_bridge.py
        scripts/coverage_client.py
        scripts/master_sweeper.py
        DESTINATION lib/${PROJECT_NAME}
)

# Keep your launch install as is
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
        particle_filter_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()