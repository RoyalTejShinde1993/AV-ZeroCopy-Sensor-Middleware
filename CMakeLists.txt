cmake_minimum_required(VERSION 3.16)
project(AV_ZeroCopy_Sensor_Middleware VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AV_ENABLE_CUDA "Enable CUDA pinned host memory handoff" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined")

find_package(Threads REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(av_middleware_node src/main.cpp)
target_link_libraries(av_middleware_node PRIVATE Threads::Threads rt)

if(AV_ENABLE_CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_compile_definitions(av_middleware_node PRIVATE AV_ENABLE_CUDA)
    target_link_libraries(av_middleware_node PRIVATE CUDA::cudart)
endif()

enable_testing()
add_executable(av_middleware_tests tests/test_suite.cpp)
target_link_libraries(av_middleware_tests PRIVATE Threads::Threads rt)
add_test(NAME CorePipelineTests COMMAND av_middleware_tests)
