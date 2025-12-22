/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           ch375_uart.h
 * @brief          CH375 UART hardware interface definitions
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Hardware abstraction layer for CH375 communication via UART. Defines
 * pin mappings, USART indices, and initialization functions for manual
 * UART configuration in 9-bit mode.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef CH375_UART_H
#define CH375_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "ch375.h"

/**
 * Platform-specific includes
 */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    #include <zephyr/drivers/uart.h>
    #include <stm32f4xx_ll_rcc.h>
    #include <stm32f4xx_ll_bus.h>
    #include <stm32f4xx_ll_gpio.h>
    #include <stm32f4xx_ll_usart.h>
#elif defined(CONFIG_SOC_RP2350A_M33) || defined(CONFIG_SOC_RP2040) || defined(CONFIG_SOC_SERIES_RP2XXX)
    #include <hardware/pio.h>
    #include <hardware/clocks.h>
    #include <hardware/gpio.h>
#else
    #error "Unsupported platform"
#endif

/**
 * Hardware context structure
 */
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    #define CH375_A_USART_INDEX 2
    #define CH375_B_USART_INDEX 3

    typedef struct {
        const char *name;
        const struct device *uart_dev;
        USART_TypeDef *huart;
        struct gpio_dt_spec int_gpio;
    } ch375_HwContext_t;

#elif defined(CONFIG_SOC_RP2350A_M33) || defined(CONFIG_SOC_RP2040) || defined(CONFIG_SOC_SERIES_RP2XXX)
    #define CH375_A_USART_INDEX 0
    #define CH375_B_USART_INDEX 1   

    #define PIO_UART_TX_PIN_CH375A 4
    #define PIO_UART_RX_PIN_CH375A 5
    #define PIO_UART_TX_PIN_CH375B 8
    #define PIO_UART_RX_PIN_CH375B 9

    #define PIO_UART_SM_TX 0
    #define PIO_UART_SM_RX 1

    typedef struct {
        const char *name;
        uint32_t baudrate;
        struct gpio_dt_spec int_gpio;
        PIO pio;
        uint sm_tx;
        uint sm_rx;
        uint tx_pin;
        uint rx_pin;
        uint offset_tx;
        uint offset_rx;
    } ch375_HwContext_t;

#endif

// Abstract wrapper for CH375 hardware initialization
int ch375_hwInitManual(const char *name, int uart_index, const struct gpio_dt_spec *int_gpio, uint32_t initial_baudrate, struct ch375_Context_t **ppCtxOut);

// Abstract wrapper for CH375 baudrate modification
int ch375_hwSetBaudrate(struct ch375_Context_t *pCtx, uint32_t baudrate);


#ifdef __cplusplus
}
#endif

#endif /* CH375_UART_H */