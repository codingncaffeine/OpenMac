add_library(openmac_core STATIC
    src/cpu.cpp
    src/cpu_decode.cpp
    src/cpu_ops_move.cpp
    src/cpu_ops_alu.cpp
    src/cpu_ops_shift_bits.cpp
    src/cpu_ops_muldiv_bcd.cpp
    src/cpu_ops_flow.cpp
)

target_include_directories(openmac_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openmac_core PRIVATE openmac_warnings)
