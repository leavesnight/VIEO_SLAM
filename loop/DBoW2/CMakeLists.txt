cmake_minimum_required(VERSION 3.16)
set(PARENT_PROJECT_NAME ${PARENT_PROJECT_NAME} ${PROJECT_NAME})
project(DBoW2)
set(PARENT_PROJECT_NAME_STR)
foreach (tmp ${PARENT_PROJECT_NAME})
    set(PARENT_PROJECT_NAME_STR "${PARENT_PROJECT_NAME_STR} ${tmp}")
endforeach ()
message(STATUS "${PROJECT_NAME} parent project=" ${PARENT_PROJECT_NAME_STR})

if (NOT USE_G2O_NEWEST)
    set(USE_G2O_NEWEST 0)
endif ()
if (NOT USE_OPENCV_NEWEST)
    set(USE_OPENCV_NEWEST 1)
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
if (WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    add_definitions(-DCOMPILE_LIB_DBoW2)
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

#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
if (WIN32)
    set(${root_project_name}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin CACHE PATH "Target for the ${PROJECT_NAME} libraries")
else (WIN32)
    set(${root_project_name}_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Target for the ${PROJECT_NAME} libraries")
endif (WIN32)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "${PROJECT_NAME} lib out=" ${${root_project_name}_LIBRARY_OUTPUT_DIRECTORY})

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
message(STATUS "${PROJECT_NAME} OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}; LIBS=${OpenCV_LIBS}; ${OpenCV_VERSION}")

set(HDRS_DBOW2
        DBoW2/BowVector.h
        DBoW2/FORB.h
        DBoW2/FClass.h
        DBoW2/FeatureVector.h
        DBoW2/ScoringObject.h
        DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
        DBoW2/BowVector.cpp
        DBoW2/FORB.cpp
        DBoW2/FeatureVector.cpp
        DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
        DUtils/Random.h
        DUtils/Timestamp.h)
set(SRCS_DUTILS
        DUtils/Random.cpp
        DUtils/Timestamp.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
