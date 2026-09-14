cmake_minimum_required(VERSION 3.25)

# Link the C++ runtime statically with MSVC. openmac_c.dll ships beside the app
# in a zip with no installer, and a DLL linked to the shared runtime needs the
# Visual C++ Redistributable on the PC -- without it the core fails to load and
# the app falls back to its stub. Every target, fetched dependencies included,
# uses the same runtime so they link together.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

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
