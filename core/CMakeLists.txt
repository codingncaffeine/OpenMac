add_library(openmac_core STATIC
    src/cpu.cpp
    src/cpu_decode.cpp
    src/cpu_ops_move.cpp
    src/cpu_ops_alu.cpp
    src/cpu_ops_shift_bits.cpp
    src/cpu_ops_muldiv_bcd.cpp
    src/cpu_ops_flow.cpp
    src/cpu040.cpp
    src/cpu040_decode.cpp
    src/cpu040_ea.cpp
    src/cpu040_ops_move.cpp
    src/cpu040_ops_alu.cpp
    src/cpu040_ops_bits.cpp
    src/cpu040_ops_muldiv.cpp
    src/cpu040_ops_flow.cpp
    src/cpu040_ops_system.cpp
    src/cpu040_fpu.cpp
    src/hardware_trace.cpp
    src/r65c02.cpp
    src/machine/iifx/adb_bus.cpp
    src/machine/iifx/am29000.cpp
    src/machine/iifx/iop.cpp
    src/machine/iifx/iifx.cpp
    src/machine/iifx/state.cpp
    src/machine/quadra/quadra.cpp
    src/machine/machine.cpp
    src/machine/via.cpp
    src/machine/rtc.cpp
    src/hfs/hfsformat.cpp
    src/hfs/hfsbuild.cpp
    src/hfs/hfsread.cpp
    src/hfs/hfsrepair.cpp
    src/debugger.cpp
)

target_include_directories(openmac_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openmac_core PRIVATE openmac_warnings)

# C ABI shared library (openmac_c.dll) for the .NET WPF GUI via P/Invoke.
# Windows-only -- the GUI is Windows-only. Its Release DLL is emitted into
# gui/native so the GUI project can copy it beside the app.
if(WIN32)
    add_library(openmac_c SHARED src/capi.cpp)
    target_link_libraries(openmac_c PRIVATE openmac_core openmac_warnings)
    target_include_directories(openmac_c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set_target_properties(openmac_c PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/gui/native
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/gui/native)
endif()
