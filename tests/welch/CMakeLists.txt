add_executable(fft_test
    fft_test.cpp
)

add_executable(welch_test
    welch_test.cpp
)

target_include_directories(fft_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_include_directories(welch_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/proccessing
)

target_link_libraries(fft_test PRIVATE
    smartfin_proc
    GTest::gtest_main
)

target_link_libraries(welch_test PRIVATE
    smartfin_proc
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(fft_test)
gtest_discover_tests(welch_test)
