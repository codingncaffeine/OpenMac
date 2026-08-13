include(FetchContent)

# doctest/miniz still declare CMake 3.0. Treat their policy surface as 3.10:
# CMake 4 can configure them without compatibility deprecation noise, while
# their pinned project files remain untouched under FetchContent.
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
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
    unit/test_cpu040.cpp
    unit/test_cpu030.cpp
    unit/test_r65c02.cpp
    unit/test_iifx.cpp
    unit/test_hardware_trace.cpp
    unit/test_trace_compare.cpp
    unit/test_machine.cpp
    unit/test_rtc.cpp
    unit/test_hfs.cpp
    unit/test_hfsbuild.cpp
    unit/test_scsi.cpp
    unit/test_scsicd.cpp
    unit/test_scsinet.cpp
    unit/test_gcr.cpp
    unit/test_sony.cpp
    unit/test_mfm.cpp
    unit/test_dc42.cpp
    unit/test_macbinary.cpp
)
target_link_libraries(openmac_unit PRIVATE openmac_core doctest::doctest openmac_warnings)
target_link_libraries(openmac_unit PRIVATE nlohmann_json::nlohmann_json)
add_test(NAME unit COMMAND openmac_unit)

add_executable(openmac_trace
    boottrace/main.cpp
)
target_link_libraries(openmac_trace PRIVATE openmac_core openmac_warnings)

add_executable(openmac_trace040
    boottrace040/main.cpp
)
target_link_libraries(openmac_trace040 PRIVATE openmac_core openmac_warnings)

add_executable(openmac_trace030
    boottrace030/main.cpp
)
target_link_libraries(openmac_trace030 PRIVATE openmac_core openmac_warnings)

add_executable(openmac_tracecmp
    tracecmp/main.cpp
)
target_link_libraries(openmac_tracecmp PRIVATE nlohmann_json::nlohmann_json openmac_warnings)

add_executable(openmac_iifx_matrix
    iifx_matrix/main.cpp
)
target_link_libraries(openmac_iifx_matrix PRIVATE
    openmac_core nlohmann_json::nlohmann_json openmac_warnings)
add_test(NAME iifx_public_boot_matrix
    COMMAND openmac_iifx_matrix
        "${CMAKE_CURRENT_SOURCE_DIR}/data/iifx-public-matrix.jsonl")
set_tests_properties(iifx_public_boot_matrix PROPERTIES
    TIMEOUT 180 LABELS "iifx;matrix")

# Proprietary ROM/System media never enter the repository. Developers who own
# local dumps can opt the exhaustive, native-device boot matrix into CTest at
# configure time; ordinary public builds simply run the synthetic 54-case
# matrix above. OPENMAC_IIFX_SYSTEM_DISK remains as a compatibility alias for
# the original single-image smoke-test setting.
set(OPENMAC_IIFX_ROM "" CACHE FILEPATH
    "Local 512 KiB Macintosh IIfx ROM for gated tests")
set(OPENMAC_IIFX_SYSTEM_DISK "" CACHE FILEPATH
    "Deprecated alias for OPENMAC_IIFX_SCSI80")
set(OPENMAC_IIFX_SCSI80 "" CACHE FILEPATH
    "Local bootable approximately 80 MiB IIfx SCSI image")
set(OPENMAC_IIFX_SCSI160 "" CACHE FILEPATH
    "Local bootable approximately 160 MiB IIfx SCSI image")
set(OPENMAC_IIFX_SYSTEM71_FLOPPY "" CACHE FILEPATH
    "Local bootable System 7.1 SuperDrive image (raw/DC42/MacBinary)")
set(OPENMAC_IIFX_VIDEO_ROM "" CACHE FILEPATH
    "Local 32/64 KiB Macintosh Display Card 8/24 GC declaration ROM")
set(OPENMAC_IIFX_MATRIX_RAMS "4;8;16;20;32;64;68;80;128" CACHE STRING
    "Semicolon-separated legal IIfx RAM sizes used by the gated boot matrix")
set(OPENMAC_IIFX_MATRIX_FRAMES 1800 CACHE STRING
    "Base frame allowance for each gated IIfx application boot")
set(OPENMAC_IIFX_MATRIX_FRAMES_PER_RAM_MB 26 CACHE STRING
    "Additional ROM memory-test frames allowed per installed IIfx RAM MiB")

if(NOT OPENMAC_IIFX_SCSI80 AND OPENMAC_IIFX_SYSTEM_DISK)
    set(OPENMAC_IIFX_SCSI80 "${OPENMAC_IIFX_SYSTEM_DISK}")
endif()

function(openmac_add_iifx_boot_profile profile disk_image floppy_image expected_app)
    if(NOT OPENMAC_IIFX_ROM)
        return()
    endif()
    foreach(ram_mb IN LISTS OPENMAC_IIFX_MATRIX_RAMS)
        # The real IIfx ROM destructively tests every installed longword before
        # starting the Toolbox. A fixed deadline falsely fails larger physical
        # SIMM layouts while the ROM is still making legitimate progress.
        math(EXPR frame_budget
            "${OPENMAC_IIFX_MATRIX_FRAMES} + ${ram_mb} * ${OPENMAC_IIFX_MATRIX_FRAMES_PER_RAM_MB}")
        set(_iifx_boot_args
            "${OPENMAC_IIFX_ROM}" "${frame_budget}"
            "${disk_image}" "${ram_mb}" "${floppy_image}" no-frame
            --expect-app "${expected_app}" --stop-on-milestone
            --native-storage --quiet)
        if(OPENMAC_IIFX_VIDEO_ROM)
            list(APPEND _iifx_boot_args --video-rom "${OPENMAC_IIFX_VIDEO_ROM}")
        endif()
        add_test(NAME "iifx_rom_${profile}_ram${ram_mb}"
            COMMAND openmac_trace030 ${_iifx_boot_args})
        set_tests_properties("iifx_rom_${profile}_ram${ram_mb}" PROPERTIES
            TIMEOUT 180
            RUN_SERIAL TRUE
            LABELS "iifx;matrix;private;rom;${profile}")
    endforeach()
endfunction()

if(OPENMAC_IIFX_ROM)
    set(_iifx_chime_args
        "${OPENMAC_IIFX_ROM}" 300 none 8 none no-frame
        --native-storage --quiet --expect-startup-chime)
    if(OPENMAC_IIFX_VIDEO_ROM)
        list(APPEND _iifx_chime_args --video-rom "${OPENMAC_IIFX_VIDEO_ROM}")
    endif()
    add_test(NAME iifx_rom_startup_chime
        COMMAND openmac_trace030 ${_iifx_chime_args})
    set_tests_properties(iifx_rom_startup_chime PROPERTIES
        TIMEOUT 30 RUN_SERIAL TRUE
        LABELS "iifx;private;rom;audio")

    if(OPENMAC_IIFX_SCSI80)
        openmac_add_iifx_boot_profile(scsi80
            "${OPENMAC_IIFX_SCSI80}" none Finder)
    endif()
    if(OPENMAC_IIFX_SCSI160)
        openmac_add_iifx_boot_profile(scsi160
            "${OPENMAC_IIFX_SCSI160}" none Finder)
    endif()
    if(OPENMAC_IIFX_SYSTEM71_FLOPPY)
        openmac_add_iifx_boot_profile(superdrive71
            none "${OPENMAC_IIFX_SYSTEM71_FLOPPY}" Installer)
        if(OPENMAC_IIFX_SCSI80)
            openmac_add_iifx_boot_profile(scsi80_superdrive71
                "${OPENMAC_IIFX_SCSI80}" "${OPENMAC_IIFX_SYSTEM71_FLOPPY}" Installer)
        endif()
        if(OPENMAC_IIFX_SCSI160)
            openmac_add_iifx_boot_profile(scsi160_superdrive71
                "${OPENMAC_IIFX_SCSI160}" "${OPENMAC_IIFX_SYSTEM71_FLOPPY}" Installer)
        endif()
    endif()
endif()

# Differential traces are also asset-gated. CTest first captures a bounded
# OpenMac trace with the exact configured ROM/media/topology, then normalizes
# and compares it with an independently captured JSONL trace. A reference may
# contain only the fields its capture source knows; those fields become the
# comparison contract.
set(OPENMAC_IIFX_REFERENCE_TRACE "" CACHE FILEPATH
    "Normalized JSONL trace captured from another emulator or real hardware")
set(OPENMAC_IIFX_REFERENCE_DISK "" CACHE FILEPATH
    "Disk image used to reproduce OPENMAC_IIFX_REFERENCE_TRACE")
set(OPENMAC_IIFX_REFERENCE_FLOPPY "" CACHE FILEPATH
    "Floppy image used to reproduce OPENMAC_IIFX_REFERENCE_TRACE")
set(OPENMAC_IIFX_REFERENCE_RAM 8 CACHE STRING
    "RAM size in MiB used by the differential trace capture")
set(OPENMAC_IIFX_REFERENCE_FRAMES 60 CACHE STRING
    "Frames executed by the differential trace capture")
set(OPENMAC_IIFX_REFERENCE_TRIGGER_CYCLE 1 CACHE STRING
    "CPU cycle that triggers the bounded differential trace")
set(OPENMAC_IIFX_REFERENCE_TRACE_EVENTS 131072 CACHE STRING
    "Maximum events retained by the differential trace")
set(OPENMAC_IIFX_REFERENCE_POST_EVENTS 131071 CACHE STRING
    "Events retained after the differential trace trigger")
set(OPENMAC_IIFX_REFERENCE_CYCLE_TOLERANCE 0 CACHE STRING
    "Permitted cycle delta for differential trace events")
set(OPENMAC_IIFX_REFERENCE_RESYNC 0 CACHE STRING
    "Actual-only events skipped while aligning a differential trace")
set(OPENMAC_IIFX_REFERENCE_SOURCE "" CACHE STRING
    "Optional source filter used by the differential trace comparator")
set(OPENMAC_IIFX_REFERENCE_KIND "" CACHE STRING
    "Optional event-kind filter used by the differential trace comparator")

if(OPENMAC_IIFX_ROM AND OPENMAC_IIFX_REFERENCE_TRACE)
    set(_iifx_reference_disk "${OPENMAC_IIFX_REFERENCE_DISK}")
    if(NOT _iifx_reference_disk)
        set(_iifx_reference_disk none)
    endif()
    set(_iifx_reference_floppy "${OPENMAC_IIFX_REFERENCE_FLOPPY}")
    if(NOT _iifx_reference_floppy)
        set(_iifx_reference_floppy none)
    endif()
    set(_iifx_actual_trace
        "${CMAKE_CURRENT_BINARY_DIR}/iifx-reference-actual.jsonl")
    add_test(NAME iifx_reference_trace_capture
        COMMAND openmac_trace030
            "${OPENMAC_IIFX_ROM}" "${OPENMAC_IIFX_REFERENCE_FRAMES}"
            "${_iifx_reference_disk}" "${OPENMAC_IIFX_REFERENCE_RAM}"
            "${_iifx_reference_floppy}" no-frame --native-storage
            --hw-trace "${_iifx_actual_trace}" --structured-only
            --trace-cycle "${OPENMAC_IIFX_REFERENCE_TRIGGER_CYCLE}"
            --trace-capacity "${OPENMAC_IIFX_REFERENCE_TRACE_EVENTS}"
            --trace-post "${OPENMAC_IIFX_REFERENCE_POST_EVENTS}")
    set_tests_properties(iifx_reference_trace_capture PROPERTIES
        TIMEOUT 180
        FIXTURES_SETUP iifx_reference_trace
        LABELS "iifx;private;trace;differential")

    set(_iifx_compare_args
        "${_iifx_actual_trace}" "${OPENMAC_IIFX_REFERENCE_TRACE}"
        --cycle-tolerance "${OPENMAC_IIFX_REFERENCE_CYCLE_TOLERANCE}"
        --resync "${OPENMAC_IIFX_REFERENCE_RESYNC}")
    if(OPENMAC_IIFX_REFERENCE_SOURCE)
        list(APPEND _iifx_compare_args --source "${OPENMAC_IIFX_REFERENCE_SOURCE}")
    endif()
    if(OPENMAC_IIFX_REFERENCE_KIND)
        list(APPEND _iifx_compare_args --kind "${OPENMAC_IIFX_REFERENCE_KIND}")
    endif()
    add_test(NAME iifx_reference_trace_compare
        COMMAND openmac_tracecmp ${_iifx_compare_args})
    set_tests_properties(iifx_reference_trace_compare PROPERTIES
        TIMEOUT 30
        FIXTURES_REQUIRED iifx_reference_trace
        LABELS "iifx;private;trace;differential")
endif()

# SingleStepTests runner: point it at a local clone of SingleStepTests/680x0.
add_executable(openmac_sst
    cpu_sst/main.cpp
)
target_link_libraries(openmac_sst PRIVATE openmac_core miniz nlohmann_json::nlohmann_json openmac_warnings)
