if(EXISTS "${CMAKE_SOURCE_DIR}/external/googletest/CMakeLists.txt")
    add_subdirectory(${CMAKE_SOURCE_DIR}/external/googletest googletest)
else()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_subdirectory(filtfilt)
add_subdirectory(butterworth)
add_subdirectory(welch)
add_subdirectory(gps)

# Phase A/B: GPS fixtures, geo golden (Python) + time-align pairing
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(
    NAME gps_geo_golden_python
    COMMAND ${Python3_EXECUTABLE} -m unittest test_geo_golden -v
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/gps
)
set_tests_properties(gps_geo_golden_python PROPERTIES LABELS "gps;python")

add_test(
    NAME gps_align_sessions_python
    COMMAND ${Python3_EXECUTABLE} -m unittest test_align_sessions -v
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/gps
)
set_tests_properties(gps_align_sessions_python PROPERTIES LABELS "gps;python")

add_test(
    NAME gps_ahrs_orient_python
    COMMAND ${Python3_EXECUTABLE} -m unittest test_ahrs_orient -v
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/gps
)
set_tests_properties(gps_ahrs_orient_python PROPERTIES LABELS "gps;python;ahrs")

add_test(
    NAME gps_data_generation
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_SOURCE_DIR}/tests/gps -v
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/gps
)
set_tests_properties(gps_data_generation PROPERTIES LABELS "gps;python")
