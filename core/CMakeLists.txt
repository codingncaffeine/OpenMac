add_library(openmac_core STATIC
    src/cpu.cpp
)

target_include_directories(openmac_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openmac_core PRIVATE openmac_warnings)
