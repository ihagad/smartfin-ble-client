cmake_minimum_required(VERSION 3.21)

project(smartfin_ble_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SMARTFIN_PROTOCOL_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol)
set(SMARTFIN_PIPELINE_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline)
set(SMARTFIN_PROCESSING_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing)
set(SMARTFIN_MATH_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing/math)

option(SMARTFIN_ENABLE_SIMPLEBLE "Enable SimpleBLE transport" ON)

set(SMARTFIN_WELCH_NPERSEG "256" CACHE STRING
    "DFT length for the Welch PSD estimator (must be a power of 2, >= 4)")

set(SMARTFIN_DEMO_NPERSEG "512" CACHE STRING
    "DFT length used by smartfin_demo (must be a power of 2, >= 4; default 512)")

if(SMARTFIN_ENABLE_SIMPLEBLE)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/SimpleBLE/simpleble/CMakeLists.txt")
        message(FATAL_ERROR
            "SMARTFIN_ENABLE_SIMPLEBLE is ON, but external/SimpleBLE is missing. "
            "Run: git submodule update --init --recursive")
    endif()
    add_subdirectory(external/SimpleBLE/simpleble)
endif()

#  Protocol layer 
# Pure C++20, no platform or BLE dependencies.
# Apple clients (watchOS/iOS) can link this without SimpleBLE.
add_library(smartfin_protocol STATIC
    src/protocol/ensemble_decoder.cpp
)
target_include_directories(smartfin_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROTOCOL_DIR}
)
target_compile_features(smartfin_protocol PUBLIC cxx_std_20)

set_source_files_properties(
    src/proccessing/math/constants.hpp
    src/proccessing/math/lin_alg.hpp
    src/proccessing/math/matrix.hpp
    src/proccessing/math/matrix.tpp
    PROPERTIES HEADER_FILE_ONLY TRUE
)

add_library(smartfin_math STATIC
    src/proccessing/math/lin_alg.cpp
    src/proccessing/math/constants.hpp
    src/proccessing/math/lin_alg.hpp
    src/proccessing/math/matrix.hpp
    src/proccessing/math/matrix.tpp
)
target_include_directories(smartfin_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
    ${SMARTFIN_MATH_DIR}
)
target_compile_features(smartfin_math PUBLIC cxx_std_20)

add_library(smartfin_filter STATIC
    src/proccessing/filter/butterworth.cpp
    src/proccessing/filter/decimator.cpp
    src/proccessing/filter/filtfilt.cpp
)
target_include_directories(smartfin_filter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_compile_features(smartfin_filter PUBLIC cxx_std_20)

add_library(smartfin_gps STATIC
    src/proccessing/gps/geo_utils.cpp
    src/proccessing/gps/sensorlog_parser.cpp
    src/proccessing/gps/session_align.cpp
)
target_include_directories(smartfin_gps PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(smartfin_gps PUBLIC
    smartfin_math
    smartfin_pipeline
)
target_compile_features(smartfin_gps PUBLIC cxx_std_20)

add_library(smartfin_fusion STATIC
    src/proccessing/fusion/ekf_fusion.cpp
    src/proccessing/fusion/trajectory_export.cpp
)
target_include_directories(smartfin_fusion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(smartfin_fusion PUBLIC
    smartfin_gps
    smartfin_proc_core
)
target_compile_features(smartfin_fusion PUBLIC cxx_std_20)

add_library(smartfin_ahrs STATIC
    src/proccessing/AHRS/madgwick_ahrs.cpp
)
target_include_directories(smartfin_ahrs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proccessing/AHRS
)
target_link_libraries(smartfin_ahrs PUBLIC
    smartfin_math
    smartfin_protocol
)
target_compile_features(smartfin_ahrs PUBLIC cxx_std_20)

#  Pipeline layer 
# ISampleSink implementations (FileSink, etc). Header-only sinks (BufferSink,
# LoggingSink) need no compilation unit; FileSink does.
add_library(smartfin_pipeline STATIC
    src/pipeline/file_sink.cpp
)
target_include_directories(smartfin_pipeline PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PIPELINE_DIR}
)
target_link_libraries(smartfin_pipeline PUBLIC smartfin_protocol)

#  Processing layer
# smartfin_welch: FFT and Welch PSD, compiled with the production nperseg.
# smartfin_proc_core: data types and pipeline coordinator (no welch sources).
# smartfin_proc: INTERFACE bundle of proc_core + welch for all production consumers.
# sort/timsort.hpp is header-only; no compilation unit required.
add_library(smartfin_welch STATIC
    src/proccessing/welch/fft.cpp
    src/proccessing/welch/welch.cpp
)
target_include_directories(smartfin_welch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_compile_definitions(smartfin_welch PUBLIC
    SMARTFIN_WELCH_NPERSEG=${SMARTFIN_WELCH_NPERSEG}
)
target_compile_features(smartfin_welch PUBLIC cxx_std_20)

add_library(smartfin_proc_core STATIC
    src/proccessing/proc_types.cpp
    src/proccessing/processor.cpp
    src/proccessing/orient_ride.cpp
    src/proccessing/ride_orient.cpp
)
target_include_directories(smartfin_proc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(smartfin_proc_core PUBLIC
    smartfin_ahrs
    smartfin_filter
    smartfin_pipeline
)
target_compile_features(smartfin_proc_core PUBLIC cxx_std_20)

add_library(smartfin_proc INTERFACE)
target_link_libraries(smartfin_proc INTERFACE
    smartfin_proc_core
    smartfin_welch
)

# Demo welch: same sources compiled with SMARTFIN_DEMO_NPERSEG (default 1024).
add_library(smartfin_welch_demo STATIC
    src/proccessing/welch/fft.cpp
    src/proccessing/welch/welch.cpp
)
target_include_directories(smartfin_welch_demo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_compile_definitions(smartfin_welch_demo PUBLIC
    SMARTFIN_WELCH_NPERSEG=${SMARTFIN_DEMO_NPERSEG}
)
target_compile_features(smartfin_welch_demo PUBLIC cxx_std_20)

#  Session layer
# Depends on protocol; transport adapter is injected at runtime via IBleAdapter.
add_library(smartfin_session STATIC
    src/session/session.cpp
)
target_include_directories(smartfin_session PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(smartfin_session PUBLIC smartfin_protocol)

# Static libraries linked into smartfin_demo must be compiled as PIC on ELF
# platforms because smartfin_demo is a shared library loaded by ctypes.
set_target_properties(
    smartfin_protocol
    smartfin_math
    smartfin_filter
    smartfin_gps
    smartfin_fusion
    smartfin_ahrs
    smartfin_pipeline
    smartfin_welch_demo
    smartfin_proc_core
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)

#  SimpleBLE transport adapter (optional) 
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_library(smartfin_transport STATIC
        src/transport/simpleble_adapter.cpp
    )
    target_include_directories(smartfin_transport PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(smartfin_transport PUBLIC
        smartfin_protocol
        simpleble::simpleble
    )
endif()

#  Receiver executable (SimpleBLE desktop only)
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_executable(ble_receiver
        src/receiver/receiver.cpp
    )
    target_include_directories(ble_receiver PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SMARTFIN_PROTOCOL_DIR}
        ${SMARTFIN_MATH_DIR}
    )
    target_link_libraries(ble_receiver PRIVATE
        smartfin_math
        smartfin_protocol
        smartfin_session
        smartfin_transport
    )
endif()

#  BLE logger (interactive test tool) 
# Streams ensembles from the fin and prints each one to stdout in real time.
# Usage: ./ble_logger   (Ctrl-C to stop)
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_executable(ble_logger
        tools/ble_logger.cpp
    )
    target_include_directories(ble_logger PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(ble_logger PRIVATE
        smartfin_protocol
        smartfin_session
        smartfin_transport
    )
endif()

#  GPS inspect (SensorLog CSV summary; no BLE dependency)
add_executable(gps_inspect
    tools/gps_inspect.cpp
)
target_include_directories(gps_inspect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(gps_inspect PRIVATE smartfin_gps)

#  Oriented ride inspect (load_ride → orient_ride summary; no BLE)
add_executable(orient_ride_inspect
    tools/orient_ride_inspect.cpp
)
target_include_directories(orient_ride_inspect PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(orient_ride_inspect PRIVATE smartfin_proc_core)

#  Fusion export (trajectory CSVs from paired .sfdat + SensorLog GPS)
add_executable(fusion_export
    tools/fusion_export.cpp
)
target_include_directories(fusion_export PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SMARTFIN_PROCESSING_DIR}
)
target_link_libraries(fusion_export PRIVATE
    smartfin_fusion
    smartfin_proc_core
    smartfin_gps
    smartfin_pipeline
)

#  Ride recorder (writes binary .sfdat session file) 
if(SMARTFIN_ENABLE_SIMPLEBLE)
    add_executable(ride_recorder
        tools/ride_recorder.cpp
    )
    target_include_directories(ride_recorder PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(ride_recorder PRIVATE
        smartfin_pipeline
        smartfin_protocol
        smartfin_session
        smartfin_transport
    )
endif()

#  C bridge (Swift/watchOS entry point — no BLE dependency)
# Swift imports src/bridge/smartfin_c_api.h via a bridging header and calls
# into this library directly; CoreBluetooth handles the BLE layer instead.
option(SMARTFIN_ENABLE_BRIDGE "Build the pure-C Swift bridge library" OFF)
if(SMARTFIN_ENABLE_BRIDGE)
    add_library(smartfin_bridge STATIC
        src/bridge/smartfin_c_api.cpp
    )
    target_include_directories(smartfin_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SMARTFIN_PROCESSING_DIR}
    )
    target_link_libraries(smartfin_bridge PUBLIC
        smartfin_protocol
        smartfin_pipeline
        smartfin_proc
    )
    target_compile_features(smartfin_bridge PUBLIC cxx_std_20)
endif()

# Processing demo shared library (no BLE dependency)
# Exposes pipeline stages as a flat C API loaded by tools/demo/run_demo.py
# via ctypes. Links smartfin_welch_demo so the FFT runs at SMARTFIN_DEMO_NPERSEG
# (default 1024) independently of the production SMARTFIN_WELCH_NPERSEG (256).
# Build: cmake --build build --target smartfin_demo
add_library(smartfin_demo SHARED
    tools/demo/demo_api.cpp
)
target_include_directories(smartfin_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/demo
)
target_link_libraries(smartfin_demo PRIVATE
    smartfin_proc_core
    smartfin_welch_demo
    smartfin_pipeline
)
target_compile_definitions(smartfin_demo PRIVATE
    SMARTFIN_WELCH_NPERSEG=${SMARTFIN_DEMO_NPERSEG}
)
target_compile_features(smartfin_demo PRIVATE cxx_std_20)

enable_testing()
add_subdirectory(tests)
