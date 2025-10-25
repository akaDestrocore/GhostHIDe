#ifndef CH375_UART_H
#define CH375_UART_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_usart.h>

#include "ch375.h"

/**
 * use these defined values in function `ch375_hwInitManual()`
 */
#define CH375_A_USART_INDEX 2
#define CH375_B_USART_INDEX 3

typedef struct {
    const char *name;
    const struct device *uart_dev;
    USART_TypeDef *huart;
    struct gpio_dt_spec int_gpio;
} ch375_HwContext_t;

/**
 * @brief UART bare metal functions
 */

int ch375_hwInitManual(const char *name, int usart_index, const struct gpio_dt_spec *int_gpio,
                         uint32_t initial_baudrate, struct ch375_Context_t **ctx_out);

int ch375_hwSetBaudrate(struct ch375_Context_t *ctx, uint32_t baudrate);

#endif /* CH375_UART_H */