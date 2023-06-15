cmake_minimum_required(VERSION 3.16)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(USE_G2O_NEWEST 0)
set(USE_SOPHUS_NEWEST 1)
set(USE_OPENCV_NEWEST 1)

IF (NOT ROS_BUILD_TYPE)
    SET(ROS_BUILD_TYPE Release)
ENDIF ()
message(STATUS "${PROJECT_NAME} Build type: " ${CMAKE_BUILD_TYPE})
if (${USE_G2O_NEWEST})
    message("build without march=native")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
endif ()
# Use c++17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_compile_definitions(COMPILEDWITHC17=1)
    set(COMPILEDWITHC17 1)
    message(STATUS "Using c++17")
else ()
    message(FATAL_ERROR "The complier ${CMAKE_CXX_COMPILER} has no C++17 suuport.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

if (${USE_OPENCV_NEWEST})
    find_package(OpenCV 4.5 QUIET)
else ()
    find_package(OpenCV 3.2 QUIET)
    if (NOT OpenCV_FOUND)
        find_package(OpenCV 2.4.3 QUIET)
        if (NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif ()
    endif ()
endif ()

find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED COMPONENT common io filters)

if (${USE_SOPHUS_NEWEST})
    add_definitions(-DUSE_SOPHUS_NEWEST)
    # Find Sophus, we found only newest master branch code could be directly compiled with eigen 3.3.7
    find_package(Sophus REQUIRED)
    message(STATUS "Sophus_INCLUDE_DIR=${Sophus_INCLUDE_DIR}; LIBS=${Sophus_LIBRARIES}; Ver=${Sophus_VERSION}")

    message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}; LIBS=${OpenCV_LIBS}; ${OpenCV_VERSION}")
    message(STATUS "EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIR}; LIBS=${EIGEN3_LIBS}; ${EIGEN3_VERSION}")
else ()
    message(FATAL_ERROR "Sophus not found.")
endif ()
#find g2o here for Eigen>3.2.10
if (${USE_G2O_NEWEST})
    add_definitions(-DUSE_G2O_NEWEST)
    add_definitions(-DG2O_USE_VENDORED_CERES)
    find_package(G2O REQUIRED)
    SET(G2O_LIBRARIES ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SBA} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_EIGEN})
    message(STATUS "g2o_Version=" ${G2O_INCLUDE_DIR}${G2O_LIBRARIES})
endif ()

include_directories(
        ${PROJECT_SOURCE_DIR}/../../..
        ${PROJECT_SOURCE_DIR}/../../../include
        ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
        ${PROJECT_SOURCE_DIR}/../../../src/Odom
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
)
add_definitions(${PCL_DEFINITIONS})

set(LIBS
        ${PROJECT_SOURCE_DIR}/../../../lib/libVIEO_SLAM.so
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system
        )

# Node for stereo(-inertial)/rgbd(-inertial) camera
rosbuild_add_executable(Stereo_Inertial
        src/ros_stereo_inertial.cc
        )
target_link_libraries(Stereo_Inertial
        ${LIBS}
        )
