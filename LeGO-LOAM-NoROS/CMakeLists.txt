cmake_minimum_required(VERSION 2.8.3)
project(legoloam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

# find_package(GTSAM REQUIRED QUIET)
# find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)

include_directories(
	include
    ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
	${SLAMBENCH_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
./include/cloud_info.h
./include/utility.h
./include/imageProjection.h
./include/featureAssociation.h
./include/mapOptimization.h
./include/legoloam.h
./src/imageProjection.cpp
./src/featureAssociation.cpp
./src/mapOptimization.cpp
./src/legoloam.cpp)


add_library(${PROJECT_NAME}-original SHARED legoloam_library.cpp)
install(TARGETS ${PROJECT_NAME}  DESTINATION  lib/)
install(TARGETS ${PROJECT_NAME}-original  DESTINATION  lib/)

target_link_libraries(${PROJECT_NAME}-original
	${PROJECT_NAME}
	${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
	${YAML_CPP_LIBRARIES}
    ${GTSAM_LIBRARIES})

SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${WRAPPER_PATH}/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "${LIB_PATH}")
    install(TARGETS ${library_name}-library  DESTINATION  lib/)

endfunction()

generate_slam_library(${PROJECT_NAME}-original "-Wl,--disable-new-dtags")
