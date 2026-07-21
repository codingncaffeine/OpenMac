include(FetchContent)

# doctest's CMakeLists predates CMake 3.5; CMake 4 needs this floor relaxed.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
FetchContent_Declare(doctest
    URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
    SYSTEM
)
FetchContent_MakeAvailable(doctest)

add_executable(openmac_unit
    unit/main.cpp
    unit/test_cpu_reset.cpp
)
target_link_libraries(openmac_unit PRIVATE openmac_core doctest::doctest openmac_warnings)
add_test(NAME unit COMMAND openmac_unit)
