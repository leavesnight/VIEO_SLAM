cmake_minimum_required(VERSION 3.16)
project(VIEO_SLAM)

# if 1 please: echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib" > ~/.bashrc
# else may cannot find g2o soft linked .so
set(USE_G2O_NEWEST 0)
set(USE_SOPHUS_NEWEST 1)
set(USE_OPENCV_NEWEST 1)
set(ADD_VR_EXAMPLE 0)
set(SET_AFFINITY_LINUX 0)#TODO(zzh)
set(USE_PCL 1)

# Detect OS and define macros appropriately
if (WIN32)
    add_definitions(-DWINDOWS)
    message(STATUS "Compiling on Windows")
elseif (UNIX)
    ADD_DEFINITIONS(-DUNIX)
    MESSAGE(STATUS "Compiling on Unix")
ENDIF (UNIX)

# default built type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "${PROJECT_NAME} Build type: " ${CMAKE_BUILD_TYPE})
if (SET_AFFINITY_LINUX)
    add_compile_definitions(SET_AFFINITY_LINUX=1)
endif ()
# To check if c++X is supported
include(CheckCXXCompilerFlag)
if (${USE_G2O_NEWEST})
    message("build without march=native")
    #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")#DEBUG default is -O0
    #set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")#RELEASE default has -O3
else ()
    message(STATUS "CXX Compiler=${CMAKE_CXX_COMPILER_ID}")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -march=native")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -march=native")
        #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
        #set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
        #set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")

        # Use gnu c++17
        CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /W4 /Od /D_USE_MATH_DEFINES /bigobj")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /Od /D_USE_MATH_DEFINES /bigobj")

        message(STATUS "MSVC VERS=${MSVC_VERSION}")
        if (MSVC_VERSION VERSION_GREATER_EQUAL 1932)
            set(COMPILER_SUPPORTS_CXX17 1)
        endif ()
    else ()
        message(FATAL_ERROR "Unsupported CXX Compiler!")
    endif ()
endif ()
if (COMPILER_SUPPORTS_CXX17)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(CMAKE_CXX_STANDARD 20)
        # please choose VS2022 in CMakeSettings.json!
        add_compile_options("/std:c++20")
    else ()
        message(FATAL_ERROR "Unsupported CXX17(GNU like)!")
    endif ()
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
    set(${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Target for the ${PROJECT_NAME} libraries")
else (WIN32)
    set(${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Target for the ${PROJECT_NAME} libraries")
endif (WIN32)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "${PROJECT_NAME} lib out=" ${${PROJECT_NAME}_LIBRARY_OUTPUT_DIRECTORY})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${PROJECT_NAME} CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV")
# Or set it in windows environment variables config: add OpenCV_DIR term
# set(OpenCV_DIR "C:/zzh/4Life/opencv/build/install/lib")
message(STATUS "OpenCV_DIR=$ENV{OpenCV_DIR} OR Not ENV: ${OpenCV_DIR}")
if (${USE_OPENCV_NEWEST})
    find_package(OpenCV 4.5 REQUIRED)
else ()
    find_package(OpenCV 3.2 QUIET)
    if (NOT OpenCV_FOUND)
        find_package(OpenCV 2.4.3 QUIET)
        if (NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif ()
    endif ()
endif ()
message(STATUS "${PROJECT_NAME} OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}; LIBS=${OpenCV_LIBS}; ${OpenCV_VERSION}")

find_package(Pangolin REQUIRED)
message(STATUS "Pangolin_INCLUDE_DIRS=${Pangolin_INCLUDE_DIRS}; LIBS=${Pangolin_LIBRARIES}; ${Pangolin_VERSION}")

# set(ENV{Eigen3_DIR} "C:/zzh/4Life/eigen/build/x64/Release/Eigen3/share/eigen3/cmake")
message(STATUS "Eigen3_DIR=$ENV{Eigen3_DIR} OR Not ENV: ${Eigen3_DIR}")
find_package(Eigen3 3.3.7 MODULE REQUIRED)
#set(EIGEN3_INCLUDE_DIR "C:/zzh/4Life/eigen")
message(STATUS "${PROJECT_NAME} EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIR}; LIBS=${EIGEN3_LIBS}; ${EIGEN3_VERSION}")
if (${USE_SOPHUS_NEWEST})
    add_definitions(-DUSE_SOPHUS_NEWEST)
    # Find Sophus, we found only newest master branch code could be directly compiled with eigen 3.3.7
    if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        set(Sophus_INCLUDE_DIR "C:/zzh/4Life/Sophus/build/x64/Sophus/include")
    endif ()
    find_package(Sophus REQUIRED)
    message(STATUS "Sophus_INCLUDE_DIR=${Sophus_INCLUDE_DIR}; LIBS=${Sophus_LIBRARIES}; Ver=${Sophus_VERSION}")
else ()
    message(FATAL_ERROR "Sophus not found.")
endif ()
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_DEBUG=${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_RELEASE=${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "${PROJECT_NAME} CMAKE_CXX_FLAGS_RELWITHDEBINFO=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

#find_package(Cholmod REQUIRED)#added for cholmod linear solver

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/src/Odom
        #${CHOLMOD_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
)

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
if (${USE_PCL})
    add_compile_definitions(USE_PCL=1)
    add_subdirectory(map)

    LIST(APPEND ${PROJECT_NAME}_deps
            ${PROJECT_NAME}_map
            )
endif ()

LIST(APPEND ${PROJECT_NAME}_SRCS
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
        src/CameraModels/radtan.cpp
        )

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_TYPE}
        ${${PROJECT_NAME}_SRCS}
        )
target_link_libraries(${PROJECT_NAME}
        ${${PROJECT_NAME}_deps}
        #${CHOLMOD_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${EIGEN3_LIBS}
        ${Sophus_LIBRARIES}
        ${G2O_LIBRARIES}
        )
if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE COMPILE_LIB_VIEO_SLAM)
endif ()

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
