cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr")
find_package(OpenCV 4.2.0 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
