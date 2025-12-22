/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           ch375_uart.c
 * @brief          CH375 UART hardware interface implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Manual UART configuration using MCU's SDK drivers for 9-bit mode operation.
 * Bypasses Zephyr's UART API to enable command/data differentiation via
 * 9th bit. Implements clock setup, GPIO configuration, and baudrate management.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "ch375_uart.h"

LOG_MODULE_REGISTER(ch375_uart, LOG_LEVEL_DBG);

#if defined(CONFIG_SOC_SERIES_STM32F4X)
    extern int ch375_stm32_hw_init(const char *name, int usart_index, 
                                   const struct gpio_dt_spec *int_gpio, 
                                   uint32_t initial_baudrate, 
                                   struct ch375_Context_t **ppCtxOut);
    extern int ch375_stm32_set_baudrate(struct ch375_Context_t *pCtx, uint32_t baudrate);
#elif defined(CONFIG_SOC_RP2350A_M33) || defined(CONFIG_SOC_RP2040) || defined(CONFIG_SOC_SERIES_RP2XXX)
    extern int ch375_rp2_hw_init(const char *name, int uart_index, 
                                const struct gpio_dt_spec *int_gpio, 
                                uint32_t initial_baudrate, 
                                struct ch375_Context_t **ppCtxOut);
    extern int ch375_rp2_set_baudrate(struct ch375_Context_t *pCtx, uint32_t baudrate);
#else
    #error "Unsupported platform. Please build for a supported platform."
#endif

/**
 * @brief Wrapper for CH375 hardware initialization
 */
int ch375_hwInitManual(const char *name, int usart_index, const struct gpio_dt_spec *int_gpio, uint32_t initial_baudrate, struct ch375_Context_t **ppCtxOut) {
    
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    LOG_INF("Platform: STM32F4X");
    return ch375_stm32_hw_init(name, usart_index, int_gpio, initial_baudrate, ppCtxOut);
#elif defined(CONFIG_SOC_RP2350A_M33)
        LOG_INF("Platform: RP2350 (RPI Pico 2)");
    return ch375_rp2_hw_init(name, usart_index, int_gpio, initial_baudrate, ppCtxOut);
#elif defined(CONFIG_SOC_RP2040)
    LOG_INF("Platform: RP2040 (RPI Pico)");
    return ch375_rp2_hw_init(name, usart_index, int_gpio, initial_baudrate, ppCtxOut);
#else
    LOG_ERR("ERROR: No platform defined!");
    return -ENOTSUP;
#endif
}

/**
 * @brief Set baudrate
 */
int ch375_hwSetBaudrate(struct ch375_Context_t *pCtx, uint32_t baudrate) 
{
    LOG_INF("ch375_hwSetBaudrate called: baud=%u", baudrate);
    
#if defined(CONFIG_SOC_SERIES_STM32F4X)
    return ch375_stm32_set_baudrate(pCtx, baudrate);
#elif defined(CONFIG_SOC_RP2350A_M33) || defined(CONFIG_SOC_RP2040) || defined(CONFIG_SOC_SERIES_RP2XXX)
    return ch375_rp2_set_baudrate(pCtx, baudrate);
#else
    LOG_ERR("ERROR: No platform defined!");
    return -ENOTSUP;
#endif
}