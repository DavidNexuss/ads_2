cmake_minimum_required(VERSION 3.10)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

project(ads)

find_package(OpenMP)
include_directories(src)

file(GLOB_RECURSE SRC 
  src/*.cpp
)

add_executable(sweep ${SRC})
target_include_directories(sweep PUBLIC src lib)
