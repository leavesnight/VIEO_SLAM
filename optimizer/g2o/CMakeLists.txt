cmake_minimum_required(VERSION 3.16)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(PARENT_PROJECT_NAME ${PARENT_PROJECT_NAME} ${PROJECT_NAME})
project(g2o)
set(PARENT_PROJECT_NAME_STR)
foreach (tmp ${PARENT_PROJECT_NAME})
    set(PARENT_PROJECT_NAME_STR "${PARENT_PROJECT_NAME_STR} ${tmp}")
endforeach ()
message(STATUS "${PROJECT_NAME} parent project=" ${PARENT_PROJECT_NAME_STR})

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if (MSVC90)
    add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
    message(STATUS "Disabling memory alignment for MSVC8")
endif (MSVC90)
# Detect OS and define macros appropriately
IF (UNIX)
    ADD_DEFINITIONS(-DUNIX)
    MESSAGE(STATUS "Compiling on Unix")
ENDIF (UNIX)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "${PROJECT_NAME} Build type: " ${CMAKE_BUILD_TYPE})
list(GET PARENT_PROJECT_NAME 0 root_project_name)
if (NOT root_project_name)
    message(WARNING "${PROJECT_NAME} is root project!")
    if (${USE_G2O_NEWEST})
        message("build without march=native")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
        set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
    else ()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")
        SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
        SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
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
# Compiler specific options for gcc
# activate warnings !!!
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF (OPENMP_FOUND AND G2O_USE_OPENMP)
    SET(G2O_OPENMP 1)
    SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF (OPENMP_FOUND AND G2O_USE_OPENMP)
# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")
message(STATUS "CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")
message(STATUS "CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

set(g2o_LIB_TYPE SHARED)
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

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
message(STATUS "${PROJECT_NAME} source dir= ${g2o_SOURCE_DIR} Or ${PROJECT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${PROJECT_NAME} CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")

# For building the CHOLMOD / CSPARSE solvers added
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(LAPACK REQUIRED)

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR ${g2o_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3 3.3.7 REQUIRED)
IF (EIGEN3_FOUND)
    SET(g2o_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE (EIGEN3_FOUND)
    SET(g2o_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF (EIGEN3_FOUND)
message(STATUS "${PROJECT_NAME} EIGEN3_INCLUDE_DIRS=${g2o_EIGEN3_INCLUDE}; LIBS=${EIGEN3_LIBS}; ${EIGEN3_VERSION}")

# Generate config.h
SET(g2o_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
message(STATUS "${PROJECT_NAME} CXX COMPILER= ${${PROJECT_NAME}_CXX_COMPILER}")
configure_file(config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up the top-level include directories
include_directories(
        ${g2o_SOURCE_DIR}/../../
        ${g2o_EIGEN3_INCLUDE}
)

# Include the subdirectories
add_library(g2o ${${PROJECT_NAME}_LIB_TYPE}
        #types
        g2o/types/types_sba.h
        g2o/types/types_six_dof_expmap.h
        g2o/types/types_sba.cpp
        g2o/types/types_six_dof_expmap.cpp
        g2o/types/types_seven_dof_expmap.cpp
        g2o/types/types_seven_dof_expmap.h
        g2o/types/se3quat.h
        g2o/types/se3_ops.h
        g2o/types/se3_ops.hpp
        #core
        g2o/core/base_edge.h
        g2o/core/base_binary_edge.h
        g2o/core/hyper_graph_action.cpp
        g2o/core/base_binary_edge.hpp
        g2o/core/hyper_graph_action.h
        g2o/core/base_multi_edge.h
        g2o/core/hyper_graph.cpp
        g2o/core/base_multi_edge.hpp
        g2o/core/hyper_graph.h
        g2o/core/base_unary_edge.h
        g2o/core/linear_solver.h
        g2o/core/base_unary_edge.hpp
        g2o/core/marginal_covariance_cholesky.cpp
        g2o/core/base_vertex.h
        g2o/core/marginal_covariance_cholesky.h
        g2o/core/base_vertex.hpp
        g2o/core/matrix_structure.cpp
        g2o/core/batch_stats.cpp
        g2o/core/matrix_structure.h
        g2o/core/batch_stats.h
        g2o/core/openmp_mutex.h
        g2o/core/block_solver.h
        g2o/core/block_solver.hpp
        g2o/core/parameter.cpp
        g2o/core/parameter.h
        g2o/core/cache.cpp
        g2o/core/cache.h
        g2o/core/optimizable_graph.cpp
        g2o/core/optimizable_graph.h
        g2o/core/solver.cpp
        g2o/core/solver.h
        g2o/core/creators.h
        g2o/core/optimization_algorithm_factory.cpp
        g2o/core/estimate_propagator.cpp
        g2o/core/optimization_algorithm_factory.h
        g2o/core/estimate_propagator.h
        g2o/core/factory.cpp
        g2o/core/optimization_algorithm_property.h
        g2o/core/factory.h
        g2o/core/sparse_block_matrix.h
        g2o/core/sparse_optimizer.cpp
        g2o/core/sparse_block_matrix.hpp
        g2o/core/sparse_optimizer.h
        g2o/core/hyper_dijkstra.cpp
        g2o/core/hyper_dijkstra.h
        g2o/core/parameter_container.cpp
        g2o/core/parameter_container.h
        g2o/core/optimization_algorithm.cpp
        g2o/core/optimization_algorithm.h
        g2o/core/optimization_algorithm_with_hessian.cpp
        g2o/core/optimization_algorithm_with_hessian.h
        g2o/core/optimization_algorithm_levenberg.cpp
        g2o/core/optimization_algorithm_levenberg.h
        g2o/core/jacobian_workspace.cpp
        g2o/core/jacobian_workspace.h
        g2o/core/robust_kernel.cpp
        g2o/core/robust_kernel.h
        g2o/core/robust_kernel_factory.cpp
        g2o/core/robust_kernel_factory.h
        g2o/core/robust_kernel_impl.cpp
        g2o/core/robust_kernel_impl.h
        #added
        g2o/core/optimization_algorithm_gauss_newton.cpp
        #stuff
        g2o/stuff/string_tools.h
        g2o/stuff/color_macros.h
        g2o/stuff/macros.h
        g2o/stuff/timeutil.cpp
        g2o/stuff/misc.h
        g2o/stuff/timeutil.h
        g2o/stuff/os_specific.c
        g2o/stuff/os_specific.h
        g2o/stuff/string_tools.cpp
        g2o/stuff/property.cpp
        g2o/stuff/property.h

        #added
        g2o/stuff/sparse_helper.cpp
        )
