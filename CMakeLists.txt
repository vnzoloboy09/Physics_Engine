cmake_minimum_required(VERSION 3.5)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)

add_executable(Physics_Engine Physics_Engine/src/main.cpp)

target_link_libraries(Physics_Engine raylib)
