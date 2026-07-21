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

FetchContent_Declare(miniz
    URL https://github.com/richgel999/miniz/archive/refs/tags/3.0.2.tar.gz
    SYSTEM
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
    SYSTEM
)
FetchContent_MakeAvailable(miniz nlohmann_json)

add_executable(openmac_unit
    unit/main.cpp
    unit/test_cpu_reset.cpp
    unit/test_machine.cpp
    unit/test_hfs.cpp
)
target_link_libraries(openmac_unit PRIVATE openmac_core doctest::doctest openmac_warnings)
add_test(NAME unit COMMAND openmac_unit)

add_executable(openmac_trace
    boottrace/main.cpp
)
target_link_libraries(openmac_trace PRIVATE openmac_core openmac_warnings)

# SingleStepTests runner: point it at a local clone of SingleStepTests/680x0.
add_executable(openmac_sst
    cpu_sst/main.cpp
)
target_link_libraries(openmac_sst PRIVATE openmac_core miniz nlohmann_json::nlohmann_json openmac_warnings)
