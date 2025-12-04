# CMake Minimum Version
cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "ApaPsdFusion")

message("APA_ROOT: " ${APA_ROOT})

set(PROTOBUF_ROOT "${APA_ROOT}/3rdparty/x86-64/protobuf-3.19.6")

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${APA_ROOT}/3rdparty/x86-64/eigen3/cmake")
find_package(Eigen3 REQUIRED PATHS ${CMAKE_PREFIX_PATH} NO_DEFAULT_PATH)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(
    ${APA_ROOT}/interfaces
    ${APA_ROOT}/pacos/LOG/
    ${APA_ROOT}/3rdparty/x86-64/eigen3
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

file(GLOB_RECURSE SRC_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/psd_fusion_process.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(${PROJECT_NAME}
    ${Protobuf_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${APA_ROOT}/modules/psd/lib/"
    LIBRARY_OUTPUT_DIRECTORY "${APA_ROOT}/modules/psd/lib/"
    RUNTIME_OUTPUT_DIRECTORY "${APA_ROOT}/modules/psd/bin/"
)
