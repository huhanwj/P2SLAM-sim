cmake_minimum_required(VERSION 3.0)

# Set the project name
project(P2SLAM-sim)

# Find GTSAM and its dependencies
find_package(GTSAM REQUIRED)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the source files to the executable
add_executable(${PROJECT_NAME} main.cpp general_functions.cpp RobotBetweenFactor.cpp)

# Link GTSAM to the executable
target_link_libraries(${PROJECT_NAME} gtsam)
