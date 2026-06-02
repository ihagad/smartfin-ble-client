find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GEN_GEO_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/gen_geo_golden.py)
set(GEN_GEO_HEADER ${CMAKE_CURRENT_BINARY_DIR}/geo_golden_vectors.hpp)

add_custom_command(
    OUTPUT  ${GEN_GEO_HEADER}
    COMMAND ${Python3_EXECUTABLE} ${GEN_GEO_SCRIPT} ${GEN_GEO_HEADER}
    DEPENDS ${GEN_GEO_SCRIPT}
            ${CMAKE_SOURCE_DIR}/tools/generate_fake_gps.py
    COMMENT "Generating geo golden vectors"
    VERBATIM
)

add_custom_target(geo_golden_vectors_header DEPENDS ${GEN_GEO_HEADER})

add_executable(gps_test
    gps_test.cpp
)

add_dependencies(gps_test geo_golden_vectors_header)

target_include_directories(gps_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(gps_test PRIVATE
    SMARTFIN_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(gps_test PRIVATE
    smartfin_gps
    smartfin_pipeline
    GTest::gtest_main
)

add_executable(ahrs_orient_test
    ahrs_orient_test.cpp
)

target_include_directories(ahrs_orient_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_compile_definitions(ahrs_orient_test PRIVATE
    SMARTFIN_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(ahrs_orient_test PRIVATE
    smartfin_proc_core
    smartfin_pipeline
    GTest::gtest_main
)

add_executable(ekf_fusion_test
    ekf_fusion_test.cpp
)

target_include_directories(ekf_fusion_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_compile_definitions(ekf_fusion_test PRIVATE
    SMARTFIN_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(ekf_fusion_test PRIVATE
    smartfin_fusion
    smartfin_proc_core
    smartfin_gps
    smartfin_pipeline
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(gps_test)
gtest_discover_tests(ahrs_orient_test)
gtest_discover_tests(ekf_fusion_test)
