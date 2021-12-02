cmake_minimum_required(VERSION 3.6)

project(Physics2D LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PHYSICS2D_ALL_HEADER_FILES)
set(PHYSICS2D_ALL_SOURCE_FILES)

file(GLOB_RECURSE PHYSICS2D_ALL_HEADER_FILES ./include/*.*)
file(GLOB_RECURSE PHYSICS2D_ALL_SOURCE_FILES ./source/*.*)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(Physics2D STATIC ${PHYSICS2D_ALL_HEADER_FILES} ${PHYSICS2D_ALL_SOURCE_FILES})

target_include_directories(Physics2D
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )