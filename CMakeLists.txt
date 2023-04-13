cmake_minimum_required(VERSION 3.10)

# Set the project name
project(P2SLAM-sim)

# Find GTSAM and its dependencies
find_package(GTSAM REQUIRED)

# Find the matio library
find_package(PkgConfig REQUIRED)
pkg_check_modules(MATIO REQUIRED IMPORTED_TARGET matio)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the source files to the executable
add_executable(${PROJECT_NAME} main.cpp general_functions.cpp RobotBetweenFactor.cpp APRobotFactors.cpp CSI_process.cpp noise_model.cpp)

# Link GTSAM, matio, and OpenMP to the executable
target_link_libraries(${PROJECT_NAME} gtsam PkgConfig::MATIO OpenMP::OpenMP_CXX)
