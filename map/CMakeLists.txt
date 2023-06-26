cmake_minimum_required(VERSION 3.16)
set(PARENT_PROJECT_NAME ${PARENT_PROJECT_NAME} ${PROJECT_NAME})
project(map)
set(PARENT_PROJECT_NAME_STR)
foreach (tmp ${PARENT_PROJECT_NAME})
    set(PARENT_PROJECT_NAME_STR "${PARENT_PROJECT_NAME_STR} ${tmp}")
endforeach ()
message(STATUS "${PROJECT_NAME} parent project=" ${PARENT_PROJECT_NAME_STR})

if (NOT USE_G2O_NEWEST)
    set(USE_G2O_NEWEST 0)
endif ()
if (NOT USE_SOPHUS_NEWEST)
    set(USE_SOPHUS_NEWEST 1)
endif ()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "${PROJECT_NAME} Build type: " ${CMAKE_BUILD_TYPE})
list(GET PARENT_PROJECT_NAME 0 root_project_name)
if (NOT root_project_name)
    message(WARNING "${PROJECT_NAME} is root project!")
    if (${USE_G2O_NEWEST})
        message("build without march=native")
    else ()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -march=native")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -march=native")
    endif ()
endif ()
if (NOT COMPILEDWITHC17)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
    if (COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
        add_compile_definitions(COMPILEDWITHC17=1)
        set(COMPILEDWITHC17 1)
        message(STATUS "Using c++17")
    else ()
        message(FATAL_ERROR "${PROJECT_NAME} The complier ${CMAKE_CXX_COMPILER} has no C++17 suuport.")
    endif ()
else ()
    message(STATUS "${PROJECT_NAME} Using c++17")
endif ()

set(map_LIB_TYPE SHARED)
message(STATUS "${PROJECT_NAME} shared=" ${${PROJECT_NAME}_LIB_TYPE})

if (WIN32)
    set(${root_project_name}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Target for the ${PROJECT_NAME} libraries")
else (WIN32)
    set(${root_project_name}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Target for the ${PROJECT_NAME} libraries")
endif (WIN32)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "${PROJECT_NAME} lib out=" ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${PROJECT_NAME} CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

find_package(Eigen3 3.3.7 MODULE REQUIRED)
message(STATUS "${PROJECT_NAME} EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIR}; LIBS=${EIGEN3_LIBS}; ${EIGEN3_VERSION}")
if (${USE_SOPHUS_NEWEST})
    add_definitions(-DUSE_SOPHUS_NEWEST)
    # Find Sophus, we found only newest master branch code could be directly compiled with eigen 3.3.7
    find_package(Sophus REQUIRED)
    message(STATUS "${PROJECT_NAME} Sophus_INCLUDE_DIR=${Sophus_INCLUDE_DIR}; LIBS=${Sophus_LIBRARIES}; Ver=${Sophus_VERSION}")
else ()
    message(FATAL_ERROR "${PROJECT_NAME} Sophus not found.")
endif ()

find_package(PCL REQUIRED COMPONENT common io filters)
add_definitions(${PCL_DEFINITIONS})
message(STATUS "${PROJECT_NAME} PCL definitions=${PCL_DEFINITIONS}, include=${PCL_INCLUDE_DIRS}, libs=${PCL_LIBRARIES}, pclcompileoptions=${PCL_COMPILE_OPTIONS}")
list(FIND PCL_COMPILE_OPTIONS "-ffloat-store" PCL_FFLOAT_STORE)
if (NOT ${PCL_FFLOAT_STORE} EQUAL -1)
    message(FATAL_ERROR "Please remove -ffloat-store for speed 30% improve! Now pclcompileoptions=${PCL_COMPILE_OPTIONS},${PCL_FFLOAT_STORE}")
endif ()

LIST(APPEND deps
        ${EIGEN3_LIBS}
        ${Sophus_LIBRARIES}
        ${PCL_LIBRARIES}
        )

LIST(APPEND ${PROJECT_NAME}_SRCS
        pcl/map_sl.cpp
        )

message(STATUS "pjs_dir=" ${PROJECT_SOURCE_DIR})
include_directories(
        ${PROJECT_SOURCE_DIR}/../
        ${PROJECT_SOURCE_DIR}/../include
        ${PROJECT_SOURCE_DIR}/../src/Odom
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

# Create library
message(STATUS "pjn=" ${PARENT_PROJECT_NAME}_${PROJECT_NAME})
add_library(${PARENT_PROJECT_NAME}_${PROJECT_NAME} ${${PROJECT_NAME}_LIB_TYPE} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PARENT_PROJECT_NAME}_${PROJECT_NAME} ${deps})
