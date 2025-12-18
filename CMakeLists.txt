cmake_minimum_required(VERSION 3.28.1)

if(BOARD STREQUAL "stm32f4_disco")
    set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/stm32f4_disco.overlay")
elseif(BOARD MATCHES "^rpi_pico2")
    set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/rpi_pico2.overlay")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_hid_proxy)

# Common source files
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_host.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hid/src/hid_parser.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hid/src/hid_mouse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hid/src/hid_keyboard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hid/src/hid_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_hid_proxy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/input_patterns.c
)

# Platform-specific UART implementation
if(CONFIG_SOC_SERIES_STM32F4X)

    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_uart_stm32.c
    )
    
    # STM32 LL drivers
    zephyr_library_include_directories(
        ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include
        ${ZEPHYR_BASE}/modules/hal/stm32/stm32cube/stm32f4xx/soc
    )
    
elseif(CONFIG_SOC_RP2350A_M33)

    target_sources(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/src/ch375_uart_rp2.c
    )
    
    # RP2350 PIO support
    zephyr_library_include_directories(
        ${ZEPHYR_BASE}/../modules/hal/rpi_pico/src/rp2_common/hardware_pio/include
        ${ZEPHYR_BASE}/../modules/hal/rpi_pico/src/rp2_common/hardware_clocks/include
        ${ZEPHYR_BASE}/../modules/hal/rpi_pico/src/rp2_common/hardware_gpio/include
        ${ZEPHYR_BASE}/../modules/hal/rpi_pico/src/common/hardware_claim/include
        ${ZEPHYR_BASE}/../modules/hal/rpi_pico/src/rp2_common/pico_base/include
    )
    
else()
    message(FATAL_ERROR "Unsupported platform! Only STM32F4X and RP2XXX are supported.")
endif()

# Common include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/hid/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ch375/include
)