#include "ch375_uart.h"

LOG_MODULE_REGISTER(ch375_uart, LOG_LEVEL_DBG);

/* Private variables ---------------------------------------------------------*/
static bool usart2_init_done = false;
static bool usart3_init_done = false;

/* Private function prototypes -----------------------------------------------*/
static USART_TypeDef *get_uart_instance_from_index(int usart_index);
static int ch375_configure_9bit_instance(USART_TypeDef *huart, uint32_t baudrate);
static int ch375_instance_write_u16_timeout(USART_TypeDef *huart, uint16_t data, k_timeout_t timeout);
static int ch375_instance_read_u16_timeout(USART_TypeDef *huart, uint16_t *data, k_timeout_t timeout);
static int ch375_write_cmd_cb(struct ch375_Context_t *ctx, uint8_t cmd);
static int ch375_write_data_cb(struct ch375_Context_t *ctx, uint8_t data);
static int ch375_read_data_cb(struct ch375_Context_t *ctx, uint8_t *data);
static int ch375_query_int_cb(struct ch375_Context_t *ctx);

/**
 * @brief CH375 UART harware functions
 */

/**
  * @brief Initialize the CH375 context
  * @param name the name of the UART peripheral
  * @param usart_index the index of the USART peripheral
  * @param int_gpio the GPIO pin to use as interrupt
  * @param initial_baudrate the initial baud rate value
  * @param ctx_out the context to initialize
  * @retval 0 on success, error code otherwise
  */
int ch375_hwInitManual(const char *name, int usart_index, const struct gpio_dt_spec *int_gpio, 
                                uint32_t initial_baudrate, struct ch375_Context_t **ctx_out) {
    
    ch375_HwContext_t *hw = NULL;
    struct ch375_Context_t *ctx = NULL;
    USART_TypeDef *huart = NULL;
    int ret = -1;

    if ( CH375_A_USART_INDEX != usart_index && CH375_B_USART_INDEX != usart_index) {
        LOG_ERR("Invalid USART index: %d. Use macros CH375_A_USART_INDEX or CH375_B_USART_INDEX instead.", usart_index);
        return -EINVAL;
    }

    huart = get_uart_instance_from_index(usart_index);
    if (NULL == huart) {
        LOG_ERR("UART peripheral with index %d not defined.", usart_index);
        return -ENOTSUP;
    }

    // LL initialization done only for USART2/USART3
    if (CH375_A_USART_INDEX == usart_index && true != usart2_init_done) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

       /** USART2 GPIO Configuration
        * PA2   ------> USART2_TX
        * PA3   ------> USART2_RX
        */
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);

        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);
        LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);

        usart2_init_done = true;
        LOG_INF("Done initializing clock/pins for USART2");
    } else if (CH375_B_USART_INDEX == usart_index && true != usart3_init_done) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
        /** USART3 GPIO Configuration
         * PB10   ------> USART3_TX
         * PB11   ------> USART3_RX
         */

        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_7);

        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_7);

        usart3_init_done = true;
        LOG_INF("Done initializing clock/pins for USART3");
    }

    hw = k_malloc(sizeof(ch375_HwContext_t));
    if (NULL == hw) {
        LOG_ERR("Failed to allocate memory for ch375_HwContext_t");
        return -ENOMEM;
    }

    memset(hw, 0x00, sizeof(ch375_HwContext_t));

    hw->name = name;
    hw->uart_dev = NULL;
    hw->huart = huart;

    if(NULL != int_gpio) {
        hw->int_gpio = *int_gpio;
    } else {
        memset(&hw->int_gpio, 0x00, sizeof(hw->int_gpio));
    }

    // Configure 9bit mode
    ret = ch375_configure_9bit_instance(huart, initial_baudrate);
    if (ret < 0) {
        LOG_ERR("%s: Failed to configure 9bit mode for instance %d", name, ret);
        k_free(hw);
        return ret;
    }

    // Configure INT GPIO pin
    if (NULL != int_gpio) {
        if (true != device_is_ready(int_gpio->port)) {
            LOG_ERR("%s: INT GPIO not ready", name);
            k_free(hw);
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("%s: Failed to configure INT GPIO: %d", name, ret);
            k_free(hw);
            return ret;
        }
    }

    // Open CH375 context (callback will use hw->huart via instance functions)
    ret = ch375_openContext(&ctx, ch375_write_cmd_cb, ch375_write_data_cb, 
                                ch375_read_data_cb, ch375_query_int_cb, hw);

     if (CH375_SUCCESS != ret) {
        LOG_ERR("%s: ch375_openContext failed: %d", name, ret);
        k_free(hw);
        return -EIO;
    }

    *ctx_out = ctx;
    LOG_INF("%s: Manual hardware initialized (USART%d)", name, usart_index);

    return 0;
}


/**
  * @brief Set the baud rate of the UART
  * @param ctx CH375 context
  * @param baudrate Baud rate
  * @retval 0 on success, error code otherwise
  */
int ch375_hwSetBaudrate(struct ch375_Context_t *ctx, uint32_t baudrate) {
    
    ch375_HwContext_t *hw = (ch375_HwContext_t *)ch375_getPriv(ctx);

    if (NULL == hw || NULL == hw->huart) {
        return -ENOTSUP;
    }

    LOG_INF("%s: Changing baudrate to %d", hw->name, (int)baudrate);
    return ch375_configure_9bit_instance(hw->huart, baudrate);
}

/* --------------------------------------------------------------------------
 * INSTANCE helpers
 * -------------------------------------------------------------------------*/
static USART_TypeDef *get_uart_instance_from_index(int usart_index) {
    
    switch(usart_index) {
        case 1: {
            #if defined(USART1)
                return USART1;
            #else
                return NULL;
            #endif
        }

        case 2: {
            #if defined(USART2)
                return USART2;
            #else
                return NULL;
            #endif
        }

        case 3: {
            #if defined(USART3)
                return USART3;
            #else
                return NULL;
            #endif
        }

        case 4: {
            #if defined(UART4)
                return UART4;
            #else
                return NULL;
            #endif
        }

        case 5: {
            #if defined(UART5)
                return UART5;
            #else
                return NULL;
            #endif
        }

        case 6: {
            #if defined(USART6)
                return USART6;
            #else
                return NULL;
            #endif
        }

        default:  {
            return NULL;
        }
    }
}

static int ch375_configure_9bit_instance(USART_TypeDef *huart, uint32_t baudrate) {
    
    if (NULL == huart) {
        return -ENOTSUP;
    }

    uint32_t tempreg;
    uint32_t apbClock;

    // Disable UART
    huart->CR1 &= ~USART_CR1_UE;
    k_busy_wait(100);

    // Clear all flags
    (void)huart->SR;
    (void)huart->DR;

    // Configure CR1
    tempreg = huart->CR1;
    tempreg &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE);
    tempreg |= USART_CR1_M | USART_CR1_TE | USART_CR1_RE;
    huart->CR1 = tempreg;

    // Set 00 for 1 stop bit
    tempreg = huart->CR2;
    tempreg &= ~USART_CR2_STOP;
    huart->CR2 = tempreg;

    // Reset CR3
    huart->CR3 = 0;

    // Configure APB1 frequency
#if defined(USART2) || defined(USART3) || defined(UART4)
    if (USART2 == huart || USART3 == huart 
#ifdef UART4
        || UART4 == huart
#endif
    ) {
        uint32_t apb1Prescaler = LL_RCC_GetAPB1Prescaler();

        // Get apb1Div from prescaler value
        uint32_t apb1Div = 1;

        switch(apb1Prescaler) {
            case LL_RCC_APB1_DIV_1: {
                apb1Div = 1;
                break;
            }

            case LL_RCC_APB1_DIV_2: {
                apb1Div = 2;
                break;
            }

            case LL_RCC_APB1_DIV_4: {
                apb1Div = 4;
                break;
            }

            case LL_RCC_APB1_DIV_8: {
                apb1Div = 8;
                break;
            }

            case LL_RCC_APB1_DIV_16: {
                apb1Div = 16;
                break;
            }

            default: {
                LOG_WRN("Unexpected APB1 prescaler: 0x%08X", apb1Prescaler);
                // default
                apb1Div = 4;
                break;
            }
        }

        // Get HCLK value and divide it by apb1Prescaler
        uint32_t hclk = SystemCoreClock;
        uint32_t ahbPrescaler = LL_RCC_GetAHBPrescaler();

        // Get ahbDiv from prescaler value
        uint32_t ahbDiv = 1;

        switch(ahbPrescaler) {
            case LL_RCC_SYSCLK_DIV_1: {
                ahbDiv = 1;
                break;
            }

            case LL_RCC_SYSCLK_DIV_2: {
                ahbDiv = 2;
                break;
            }

            case LL_RCC_SYSCLK_DIV_4: {
                ahbDiv = 4; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_8: {
                ahbDiv = 8; 
                break;
            }
            
            case LL_RCC_SYSCLK_DIV_16: {
                ahbDiv = 16; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_64: {
                ahbDiv = 64; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_128: {
                ahbDiv = 128; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_256: {
                ahbDiv = 256; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_512: {
                ahbDiv = 512; 
                break;
            }

            default: {
                ahbDiv = 1; 
                break;
            }
        }

        hclk = SystemCoreClock / ahbDiv;
        apbClock = hclk / apb1Div;

        LOG_INF("SYSCLK=%" PRIu32 " ahbDiv=%" PRIu32 " HCLK=%" PRIu32 " apb1Div=%" PRIu32 " APB1=%" PRIu32 "",
                SystemCoreClock, ahbDiv, hclk, apb1Div, apbClock);
    }
#endif

#ifdef USART1
    else if (USART1 == huart
#ifdef USART6
        || USART6 == huart
#endif
    ) {
        // APB2
        uint32_t apb2Prescaler = LL_RCC_GetAPB2Prescaler();
        uint32_t apb2Div = 1;

        switch(apb2Prescaler) {
            case LL_RCC_APB2_DIV_1: {
                apb2Div = 1;
                break;
            }

            case LL_RCC_APB2_DIV_2: {
                apb2Div = 2;
                break;
            }

            case LL_RCC_APB2_DIV_4: {
                apb2Div = 4;
                break;
            }

            case LL_RCC_APB2_DIV_8: {
                apb2Div = 8;
                break;
            }

            case LL_RCC_APB2_DIV_16: {
                apb2Div = 16;
                break;
            }

            default: {
                apb2Div = 2;
                break;
            }
        }

        uint32_t hclk = SystemCoreClock;
        uint32_t ahbPrescaler = LL_RCC_GetAHBPrescaler();
        uint32_t ahbDiv = 1;

        switch (ahbPrescaler) {
            case LL_RCC_SYSCLK_DIV_1: {
                ahbDiv = 1; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_2: {
                ahbDiv = 2; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_4: {
                ahbDiv = 4; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_8: {
                ahbDiv = 8; 
                break;
            }

            case LL_RCC_SYSCLK_DIV_16: {
                ahbDiv = 16; 
                break;
            }

            default: {
                ahbDiv = 1; 
                break;
            }
        }

        hclk = SystemCoreClock / ahbDiv;
        apbClock = hclk / apb2Div;
    }

#endif
    else {
        LOG_ERR("Unknown UART instance");
        return -ENOTSUP;
    }

    LOG_INF("APB clock: %" PRIu32 " Hz, baudrate: %" PRIu32 "", apbClock, baudrate);

    // Calculate div
    uint32_t usartDiv = (apbClock + (baudrate / 2U)) / baudrate;

    huart->BRR = usartDiv;
    LOG_INF("UART BRR set to: 0x%04X", usartDiv);

    // Re-enable
    huart->CR1 |= USART_CR1_UE;
    k_busy_wait(2000);

    // Clear all flags
    (void)huart->SR;
    (void)huart->DR;

    k_msleep(5);
    return 0;
}

static int ch375_instance_write_u16_timeout(USART_TypeDef *huart, uint16_t data, k_timeout_t timeout) {
    
    if (NULL == huart) {
        return -ENOTSUP;
    }

    int64_t startTime = k_uptime_get();
    int64_t timeoutMs;

    if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
        timeoutMs = INT64_MAX;
    } else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
        timeoutMs = 0;
    } else {
        timeoutMs = timeout.ticks * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    }

    // Wait TXE flag
     while (!(huart->SR & USART_SR_TXE)) {
        if ((k_uptime_get() - startTime) >= timeoutMs) {
            LOG_ERR("TX timeout");
            return -ETIMEDOUT;
        }
        k_busy_wait(10);
    }

    huart->DR = data & 0x01FF;

    // Wait for TC flag
    while (!(huart->SR & USART_SR_TC)) {
        if ((k_uptime_get() - startTime) >= timeoutMs) {
            LOG_ERR("TC timeout");
            return -ETIMEDOUT;
        }
        k_busy_wait(10);
    }

    k_busy_wait(50);
    return 0;
}

static int ch375_instance_read_u16_timeout(USART_TypeDef *huart, uint16_t *data, k_timeout_t timeout) {
    
    if (NULL == huart || NULL == data) {
        return -ENOTSUP;
    }

    int64_t startTime = k_uptime_get();
    int64_t timeoutMs;
    uint32_t attempts = 0;
    uint32_t sr_reg;

    if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
        timeoutMs = INT64_MAX;
    } else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
        timeoutMs = 0;
    } else {
        timeoutMs = timeout.ticks * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    }

    while (1) {
        sr_reg = huart->SR;

        if (sr_reg & USART_SR_RXNE) {
            break;
        }

        if ((k_uptime_get() - startTime) >= timeoutMs) {
            return -ETIMEDOUT;
        }

        attempts++;

        if (sr_reg & USART_SR_ORE) {
            (void)huart->SR;
            (void)huart->DR;
            continue;
        }
        if (sr_reg & USART_SR_FE) {
            (void)huart->DR;
            continue;
        }
        if (sr_reg & USART_SR_NE) {
            (void)huart->DR;
            continue;
        }

        k_busy_wait(10);
    }

    *data = huart->DR & 0x01FF;
    return 0;
}



/* --------------------------------------------------------------------------
 * CH375 Callback functions
 * -------------------------------------------------------------------------*/
static int ch375_write_cmd_cb(struct ch375_Context_t *ctx, uint8_t cmd) {

    ch375_HwContext_t *hw = (ch375_HwContext_t *)ch375_getPriv(ctx);
    uint16_t data = CH375_CMD(cmd);
    int ret = -1;

    if (NULL == hw || NULL == hw->huart) {
        LOG_ERR("ch375_write_cmd_cb: no UART instance");
        return CH375_ERROR;
    }

    ret = ch375_instance_write_u16_timeout(hw->huart, data, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: CMD write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }

    return CH375_SUCCESS;
}

static int ch375_write_data_cb(struct ch375_Context_t *ctx, uint8_t data)
{
    ch375_HwContext_t *hw = (ch375_HwContext_t *)ch375_getPriv(ctx);
    uint16_t val = CH375_DATA(data);
    int ret = -1;

    if (NULL == hw || NULL == hw->huart) {
        LOG_ERR("write_data_cb: no UART instance");
        return CH375_ERROR;
    }

    ret = ch375_instance_write_u16_timeout(hw->huart, val, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("%s: DATA write failed: %d", hw->name, ret);
        return CH375_ERROR;
    }

    return CH375_SUCCESS;
}

static int ch375_read_data_cb(struct ch375_Context_t *ctx, uint8_t *data)
{
    ch375_HwContext_t *hw = (ch375_HwContext_t *)ch375_getPriv(ctx);
    uint16_t val;
    int ret = -1;

    if (NULL == hw || NULL == hw->huart) {
        LOG_ERR("read_data_cb: no UART instance");
        return CH375_ERROR;
    }

    ret = ch375_instance_read_u16_timeout(hw->huart, &val, K_MSEC(50));
    if (ret < 0) {
        if (-ETIMEDOUT== ret) {
            return CH375_TIMEOUT;
        }
        LOG_ERR("%s: READ failed: %d", hw->name, ret);
        return CH375_ERROR;
    }

    *data = (uint8_t)(val & 0xFF);
    return CH375_SUCCESS;
}

static int ch375_query_int_cb(struct ch375_Context_t *ctx)
{
    ch375_HwContext_t *hw = (ch375_HwContext_t *)ch375_getPriv(ctx);

    if (NULL == hw) {
        return 0;
    }

    if (true != device_is_ready(hw->int_gpio.port)) {
        return 0;
    }

    return gpio_pin_get_dt(&hw->int_gpio) == 0 ? 1 : 0;
}
