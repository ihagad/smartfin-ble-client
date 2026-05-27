add_executable(contract_test contract_test.cpp)

target_link_libraries(contract_test PRIVATE
    smartfin_pipeline
    smartfin_protocol
    gtest_main
)

target_include_directories(contract_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

gtest_discover_tests(contract_test
    PROPERTIES ENVIRONMENT "SF_CONTRACT_MANIFEST=${SF_CONTRACT_MANIFEST};SF_CONTRACT_OUT=${CMAKE_BINARY_DIR}/contract_out"
)
