CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
set(PARENT_PROJECT_NAME ${PROJECT_NAME})
PROJECT(common)

LIST(APPEND deps
        ${EIGEN3_LIBS}
        ${Sophus_LIBRARIES}
        ${OpenCV_LIBS}
        )

LIST(APPEND SOURCEFILES
        ${CMAKE_CURRENT_SOURCE_DIR}/mlog/log.cpp
        )

INCLUDE_DIRECTORIES(
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

# Create library
message(STATUS "pjn=" ${PARENT_PROJECT_NAME}_${PROJECT_NAME})
ADD_LIBRARY(${PARENT_PROJECT_NAME}_${PROJECT_NAME} SHARED ${SOURCEFILES})
target_link_libraries(${PARENT_PROJECT_NAME}_${PROJECT_NAME} ${deps})


