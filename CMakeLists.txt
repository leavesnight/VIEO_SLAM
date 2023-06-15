cmake_minimum_required(VERSION 3.16)
project(VIEO_SLAM)

set(USE_G2O_NEWEST 0)
set(USE_SOPHUS_NEWEST 1)
set(USE_OPENCV_NEWEST 1)
set(ADD_VR_EXAMPLE 0)

# default built type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "${PROJECT_NAME} Build type: " ${CMAKE_BUILD_TYPE})
if (${USE_G2O_NEWEST})
    message("build without march=native")
    #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")#DEBUG default is -O0
    #set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")#RELEASE default has -O3
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -march=native")
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    #set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
    #set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")
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

set(${PROJECT_NAME}_LIB_TYPE SHARED)
message(STATUS "${PROJECT_NAME} shared=" ${${PROJECT_NAME}_LIB_TYPE})

# Set the output directory for the build executables and libraries
if (WIN32)
    set(${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
else (WIN32)
    set(${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
endif (WIN32)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "${PROJECT_NAME} lib out=" ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${PROJECT_NAME} CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV")
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
find_package(Cholmod REQUIRED)#added for cholmod linear solver

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
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_RELEASE=${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_RELWITHDEBINFO=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/src/Odom
        ${CHOLMOD_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
)
add_definitions(${PCL_DEFINITIONS})
message(STATUS "${PROJECT_NAME} PCL definitions=" ${PCL_DEFINITIONS})

add_subdirectory(common)
add_subdirectory(loop/DBoW2)
LIST(APPEND ${PROJECT_NAME}_deps
        ${PROJECT_NAME}_common
        DBoW2
        )
message("g2o newest=" ${USE_G2O_NEWEST})
#find g2o here for Eigen>3.2.10
if (${USE_G2O_NEWEST})
    add_definitions(-DUSE_G2O_NEWEST)
    add_definitions(-DG2O_USE_VENDORED_CERES)
    find_package(G2O REQUIRED)
    SET(G2O_LIBRARIES ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SBA} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_EIGEN})
    message(STATUS "g2o_Version=" ${G2O_INCLUDE_DIR}${G2O_LIBRARIES})
else ()
    message(STATUS "Use Local Submodule g2o!")
    add_subdirectory(optimizer/g2o)

    LIST(APPEND ${PROJECT_NAME}_deps
            g2o
            )
endif ()

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_TYPE}
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/FrameBase.cpp
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

        src/Odom/g2otypes.cpp

        src/CameraModels/GeometricCamera.cpp
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/CameraModels/radtan.cpp)

target_link_libraries(${PROJECT_NAME}
        ${${PROJECT_NAME}_deps}
        ${CHOLMOD_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${G2O_LIBRARIES}
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

if (${ADD_VR_EXAMPLE})
    find_package(RapidJSON)
    add_executable(stereo_vr
            Examples/Stereo/stereo_vr.cc)
    target_link_libraries(stereo_vr ${PROJECT_NAME})
endif ()

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
