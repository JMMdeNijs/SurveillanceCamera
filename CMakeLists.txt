cmake_minimum_required(VERSION 3.1)
project( Detector )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED PATHS "/home/jan/opencv/build/")


include_directories( ${OpenCV_INCLUDE_DIRS} ) 

add_definitions(-DDISABLE_OPENCV_24_COMPATIBILITY=1)  # Avoid C-like legacy API

add_executable(  Detector  DetectorV1.0.cpp survcamlibV1.0.cpp)

target_link_libraries(   Detector realsense2 ${OpenCV_LIBS} )
