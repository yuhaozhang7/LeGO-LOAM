cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
# set(CMAKE_BUILD_TYPE "Debug")

# find_package(GTSAM REQUIRED QUIET)
# find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)

SET(EIGEN3_INCLUDE_DIRS "/home/yuhao/slambench/deps/eigen3/include/eigen3")
SET(PCL_INCLUDE_DIRS "/home/yuhao/slambench/deps/pcl/include/pcl-1.8")
SET(OpenCV_INCLUDE_DIRS "/home/yuhao/slambench/deps/opencv/include")
SET(GTSAM_INCLUDE_DIRS "/home/yuhao/Downloads/gtsam/include")

SET(PCL_LIBRARY_DIRS "/home/yuhao/slambench/deps/pcl/lib")
SET(OpenCV_LIBRARY_DIRS "/home/yuhao/slambench/deps/opencv/lib")
SET(GTSAM_LIBRARY_DIRS "/home/yuhao/Downloads/gtsam/lib")

file(GLOB PCL_LIBRARIES "${PCL_LIBRARY_DIRS}/libpcl_*.so*")
file(GLOB OpenCV_LIBRARIES "${OpenCV_LIBRARY_DIRS}/libopencv_*.so*")
file(GLOB GTSAM_LIBRARIES "${GTSAM_LIBRARY_DIRS}/lib*.*")

include_directories(
	include
    ${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
./include/cloud_info.h 
./include/imageProjection.h 
./include/utility.h 
./src/imageProjection.cpp)

add_executable(run ./src/legoloam.cpp)
target_link_libraries(run
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GTSAM_LIBRARIES})
