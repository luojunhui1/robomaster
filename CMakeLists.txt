cmake_minimum_required(VERSION 2.8)

project(HERO)
SET(CMAKE_BUILE_TYPE DEBUG)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++14 -pthread -lv4l1 -lv4l2 -lrealsense2")
SET(BIN_NAME "run")

FIND_PACKAGE(OpenCV 4.4.0 REQUIRED)
FIND_PACKAGE(Threads)
find_package(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/Thread/include)
include_directories(${PROJECT_SOURCE_DIR}/Serials/include)
include_directories(${PROJECT_SOURCE_DIR}/Armor/include)
include_directories(${PROJECT_SOURCE_DIR}/Other/include)
include_directories(${PROJECT_SOURCE_DIR}/Pose/include)
include_directories(${PROJECT_SOURCE_DIR}/Filter/include)
include_directories(${PROJECT_SOURCE_DIR}/Drivers/V4L2KAS/include)
include_directories(${PROJECT_SOURCE_DIR}/Drivers/RealSense/include)
include_directories(/usr/include/eigen3)
include_directories(${realsense2_INCLUDE_DIR})


MESSAGE(STATUS "Project:INFANTRY")
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")
MESSAGE(STATUS "OpenCVlibrarystatus:")
MESSAGE(STATUS "version: ${OpenCV_VERSION}")
MESSAGE(STATUS " libraries: ${OpenCVLIBS}")
MESSAGE(STATUS "includepath:${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}")


FILE(GLOB_RECURSE sourcefiles "Other/src/*.cpp" "Energy/src/*.cpp" "Armor/src/*.cpp" "Serials/src/*.cpp" "Thread/src/*.cpp" "Pose/src/*.cpp" "Filter/src/*.cpp" "Drivers/V4L2KAS/src/*.cpp" "Drivers/RealSense/src/*.cpp")
ADD_EXECUTABLE(${BIN_NAME} main.cpp ${sourcefiles} )

list(APPEND PSMOVEAPI_TRACKER_REQUIRED_LIBS v4l2)

TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${BIN_NAME} ${OpenCV_LIBS})
target_link_libraries(${BIN_NAME} ${realsense2_LIBRARY})
