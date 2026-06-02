cmake_minimum_required(VERSION 3.10)
project(zed_sender)

set(CMAKE_CXX_STANDARD 17)

find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# OpenCV
pkg_check_modules(OPENCV REQUIRED opencv4)

include_directories(
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
)

add_executable(zed_sender zed_sender.cpp)

target_link_libraries(zed_sender
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${OPENCV_LIBRARIES}
    pthread
)