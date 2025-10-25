#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    int ret = -1;

    static const struct gpio_dt_spec ch375a_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };

    static const struct gpio_dt_spec ch375b_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 14,
        .dt_flags = GPIO_ACTIVE_LOW
    };

    LOG_INF("=================================");
    LOG_INF("Starting GhostHIDe...");
    LOG_INF("=================================");

    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    if (true != device_is_ready(gpioc)) {
        LOG_ERR("GPIO C not ready!");
        return -1;
    }

    while (1) {

        LOG_INF("Cycling in endless loop...");
        k_msleep(1000);
    }

    return 0;
}