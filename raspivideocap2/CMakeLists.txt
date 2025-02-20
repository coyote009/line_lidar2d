project(raspivideocap2)

cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)

set(TARGET raspivideocap2)

file(GLOB SRCS src/*.cpp)

set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -O0 -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++17")

# to enable debug mode (!!Note: Clean build directory before changing mode)
#  > cmake -Ddebug=1 ..
if( debug )
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(${TARGET} ${SRCS})
target_link_libraries(${TARGET} PUBLIC camera camera-base ${OpenCV_LIBRARIES})
target_include_directories(${TARGET} PUBLIC /usr/include/libcamera ${OpenCV_INCLUDE_DIRS})
