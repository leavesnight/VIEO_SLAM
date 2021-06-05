cmake_minimum_required(VERSION 2.8)
project(VIEO_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(USE_G2O_NEWEST 0)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if (${USE_G2O_NEWEST})
   message("build without march=native")
   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
   set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
   set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
else()
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")
   SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
   SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
endif()

## Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

# Use c++17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_STANDARD 17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using c++17")
else()
   message(FATAL_ERROR "The complier ${CMAKE_CXX_COMPILER} has no C++17 suuport.")
endif()

message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV") 
find_package(OpenCV 3.2 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 MODULE REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)#added for cholmod linear solver

find_package(PCL REQUIRED COMPONENT common io filters)

# Find Sophus
find_package(Sophus REQUIRED)
message(STATUS "Sophus_INCLUDE_DIR=${Sophus_INCLUDE_DIR}; LIBS=${Sophus_LIBRARIES}; Ver=${Sophus_VERSION}!")

message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}; LIBS=${OpenCV_LIBS}!")
message(STATUS "EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIR}; LIBS=${EIGEN3_LIBS}; ${EIGEN3_VERSION}")

#find g2o here for Eigen>3.2.10
if (${USE_G2O_NEWEST})
   add_definitions(-DUSE_G2O_NEWEST)
   find_package(G2O REQUIRED)
   SET(G2O_LIBRARIES ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SBA} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_EIGEN})
   message(STATUS "g2o_Version="${G2O_INCLUDE_DIR}${G2O_LIBRARIES})
else()
   SET(G2O_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
endif()

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}

        ${PROJECT_SOURCE_DIR}/src/Odom
        ${CHOLMOD_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS} )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc

src/Odom/OdomData.cpp
src/Odom/OdomPreIntegrator.cpp
src/Odom/IMUInitialization.cpp
        src/Odom/so3.cpp

        src/Odom/g2otypes.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${G2O_LIBRARIES}

        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

