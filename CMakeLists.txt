cmake_minimum_required(VERSION 3.22)
project(localizationAlgorithm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 1. Find the math library
find_package(Eigen3 REQUIRED)

# 2. Create the program
add_executable(localizationAlgorithm
        Src/main.cpp
        Src/mapCreator.cpp
        Src/map.cpp
        Src/particle.cpp
        Src/mcl.cpp
        Src/localization_node.cpp)

# Add Include Directory for custom headers
target_include_directories(localizationAlgorithm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_include_directories(localizationAlgorithm SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})

# 3. Attach the math library to the target
target_link_libraries(localizationAlgorithm Eigen3::Eigen)
