cmake_minimum_required(VERSION 3.25)
project(OpenMac VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OPENMAC_BUILD_SHELL "Build the SDL3 + Dear ImGui shell" ON)
option(OPENMAC_BUILD_TESTS "Build tests" ON)

# Warnings apply to OpenMac targets only, never to fetched dependencies.
add_library(openmac_warnings INTERFACE)
if(MSVC)
    target_compile_options(openmac_warnings INTERFACE /W4 /WX /permissive- /utf-8 /Zc:__cplusplus)
else()
    target_compile_options(openmac_warnings INTERFACE -Wall -Wextra -Werror)
endif()

add_subdirectory(core)

if(OPENMAC_BUILD_SHELL)
    add_subdirectory(shell)
endif()

if(OPENMAC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
