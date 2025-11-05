cmake_minimum_required(VERSION 3.28.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

# Add source files
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_host.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/hid_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hid/hid_mouse.c
)

# Add include dirs
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hid
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/include
)

# Needed for 9-bit UART configuration
zephyr_library_include_directories(
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include
    ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/soc
)