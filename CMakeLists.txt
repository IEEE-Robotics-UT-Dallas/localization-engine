cmake_minimum_required(VERSION 4.1)
project(localizationAlgorithm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 1. Find the math library
find_package(Eigen3 REQUIRED)

# 2. Create the program (listing main.cpp only once)
add_executable(localizationAlgorithm main.cpp)

# 3. Attach the math library to the exact name of the program we just created
target_link_libraries(localizationAlgorithm Eigen3::Eigen)