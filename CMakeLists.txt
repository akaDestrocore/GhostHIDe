cmake_minimum_required(VERSION 3.28.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

# Add source files
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
)

# Add include dirs
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Needed for 9-bit UART configuration
zephyr_library_include_directories(
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/soc
)