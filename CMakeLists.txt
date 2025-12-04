cmake_minimum_required(VERSION 3.10)
project(psd_project)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置三方库路径
set(APA_ROOT "../../")
get_filename_component(APA_ROOT ${APA_ROOT} ABSOLUTE)
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${APA_ROOT}/3rdparty/x86-64/protobuf-3.19.6")
message("APA_ROOT:" "${APA_ROOT}")

find_package(Protobuf 3.19.6 REQUIRED)
message(STATUS "Found Protobuf: ${Protobuf_VERSION}")

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${APA_ROOT}/interfaces
    ${APA_ROOT}/pacos/v1.0.0/include/pacos/log
    ${APA_ROOT}/pacos/v1.0.0/include/pacos/com
    ${APA_ROOT}/pacos/v1.0.0/include/pacos/com/proto
    ${APA_ROOT}/pacos/v1.0.0/lib/include/iceoryx/v2.0.6
    ${Protobuf_INCLUDE_DIRS}
    ${APA_ROOT}/modules
)

# 库目录
link_directories(
    ${APA_ROOT}/pacos/v1.0.0/lib
    ${APA_ROOT}/pacos/v1.0.0/lib/lib
    ${Protobuf_LIBRARY_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

# 单行定义并查找源文件
file(GLOB GENERATED_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_subdirectory(swc_process)

add_executable(psd ${GENERATED_SOURCES})
target_link_libraries(psd
    pacos_com
    pacos_log
    ${Protobuf_LIBRARIES}
    iceoryx_posh
    iceoryx_hoofs
    iceoryx_platform
    ApaPsdFusion
)

target_compile_options(psd PRIVATE $<$<CONFIG:Debug>:-g -O0>)
