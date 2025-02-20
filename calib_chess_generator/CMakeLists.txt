cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)

set(TARGET calib_chess_generator)

file(GLOB SRCS src/*.cpp)

set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# to enable debug mode (!!Note: Clean build directory before changing mode)
#  > cmake -Ddebug=1 ..
if( debug )
  set( CMAKE_BUILD_TYPE Debug )
else()
  set( CMAKE_BUILD_TYPE Release )
endif()

add_executable(${TARGET} ${SRCS})
#target_link_libraries( ${TARGET} ceres python3.6m boost_numpy3 boost_python3 )
target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})
target_include_directories(${TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS})
