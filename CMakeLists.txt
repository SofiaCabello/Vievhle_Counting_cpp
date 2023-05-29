cmake_minimum_required(VERSION 3.25)
project(OpenCV_Based_Project)
find_package(OpenCV)
set(CMAKE_CXX_STANDARD 98)
include_directories(${OpenCV_INCLUDE_DIRS})
set(SOURCE_FILES main.cpp)


add_executable(OpenCV_Based_Project main.cpp data_processing.cpp vis.cpp geometry.cpp)

target_link_libraries(OpenCV_Based_Project ${OpenCV_LIBS})
