cmake_minimum_required(VERSION 3.16)
project(calicofx)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use pkg-config to find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(PIPEWIRE REQUIRED libpipewire-0.3)
pkg_check_modules(LILV REQUIRED lilv-0)



add_executable(calicofx)
target_compile_options(calicofx PRIVATE -g -O0  -Wall -Wextra -Wpedantic)

add_subdirectory(src)
