cmake_minimum_required(VERSION 3.28.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ch375_unit_tests)

add_compile_options(-Wno-error=deprecated-declarations)

get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)

# Include satubs first
target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
)

# Include directories
target_include_directories(app PRIVATE
    ${PROJECT_ROOT}/drivers/ch375/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)

# Source files for test
target_sources(app PRIVATE
    ${PROJECT_ROOT}/drivers/ch375/src/ch375.c
    ${PROJECT_ROOT}/drivers/ch375/src/ch375_host.c
)

# Mocks
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks/mock_ch375_hw.c
)

# Test files
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ch375_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/test_transfers.c
)