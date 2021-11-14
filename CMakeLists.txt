cmake_minimum_required(VERSION 3.16)
project(myrt C CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

include(compile_flags)

find_package(OpenMP REQUIRED)

add_subdirectory(vendor/stb)
add_subdirectory(vendor/tinyobjloader)

add_executable(myrt src/myrt.cpp src/image.cpp src/random.cpp)
myrt_cxx_flags(myrt)

target_link_libraries(myrt PRIVATE OpenMP::OpenMP_CXX stb tinyobjloader)
