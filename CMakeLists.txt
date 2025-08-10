cmake_minimum_required(VERSION 3.5)
project(Physics_Engine)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Automatically download and build raylib
include(FetchContent)
FetchContent_Declare(
  raylib
  URL https://github.com/raysan5/raylib/archive/master.zip
)
FetchContent_MakeAvailable(raylib)

# Add your executable
add_executable(Physics_Engine Physics_Engine/src/main.cpp)

# Link raylib
target_link_libraries(Physics_Engine raylib)
