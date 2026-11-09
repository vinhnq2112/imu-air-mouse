cmake_minimum_required(VERSION 3.14)
project(gyro_pointer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Header-only library: link it to get the include path.
add_library(gyro_pointer INTERFACE)
target_include_directories(gyro_pointer INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(gyro_pointer INTERFACE cxx_std_17)

# Standalone example with a simulated sensor.
add_executable(gyro_pointer_example example/main.cpp)
target_link_libraries(gyro_pointer_example PRIVATE gyro_pointer)

if(MSVC)
    target_compile_options(gyro_pointer_example PRIVATE /W4)
else()
    target_compile_options(gyro_pointer_example PRIVATE -Wall -Wextra -Wpedantic)
endif()
