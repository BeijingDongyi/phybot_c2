# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Set the project name and language to C++ (you can also specify C if needed)
project(hihost CXX C)

# Set C++ standard to C++11 (or any other version you prefer)
# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

set(CMAKE_CXX_STANDARD 17)
# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../ThirdParty/eigen3)


# Define source files (Make sure C++ files use .cpp extension)
set(SOURCES
    test.cpp
    IMUReader.cpp
    serial_port.c  # Change this to C++ source if serial_port.c actually contains C++ code
    log.c
)

# Create executable
add_executable(hihost ${SOURCES})

# Link required libraries (if any)
target_link_libraries(hihost m pthread)  # Link math library

# 让 CMake 去寻找 yaml-cpp 库
find_package(yaml-cpp REQUIRED)

# 链接库文件
target_link_libraries(hihost m pthread yaml-cpp)
# If you are using C++ features (like STL), link C++ standard library
# target_link_libraries(hihost stdc++)
