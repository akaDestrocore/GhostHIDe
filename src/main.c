/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           main.c
 * @brief          Main program entry point and HID device testing
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Main application demonstrating USB HID device enumeration, descriptor
 * parsing, and mouse report handling. Tests CH375 host functionality with
 * real USB mice and keyboards.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include "ch375.h"
#include "ch375_host.h"
#include "ch375_uart.h"
#include "hid_parser.h"
#include "hid_mouse.h"
#include "usb_hid_proxy.h"
#include "hid_output.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

typedef struct {
    const char *name;
    struct gpio_dt_spec int_gpio;

    struct ch375_Context_t *ch375_ctx;
    struct USB_Device_t usb_dev;
    struct USBHID_Device_t hid_dev;
    struct HID_Mouse_t mouse;

    uint8_t is_connected;
    uint8_t interface_num;

    uint32_t last_report_ts_ms;
    uint32_t report_interval_ms;
} DeviceInput_t;

/* Private variables ---------------------------------------------------------*/
static bool proxyRunning = false;
static DeviceInput_t s_arr_devin[1];

static const char banner[] = 
"                                                                      \n"
" ██████  ██   ██  ██████  ███████ ████████ ██   ██ ██ ██████  ███████ \n"
"██       ██   ██ ██    ██ ██         ██    ██   ██ ██ ██   ██ ██      \n"
"██   ███ ███████ ██    ██ ███████    ██    ███████ ██ ██   ██ █████   \n"
"██    ██ ██   ██ ██    ██      ██    ██    ██   ██ ██ ██   ██ ██      \n"
" ██████  ██   ██  ██████  ███████    ██    ██   ██ ██ ██████  ███████ \n";

/* Private function prototypes -----------------------------------------------*/
static int init_ch375_device(DeviceInput_t *pDevIn, const char *pName, int usart_index, 
                            const struct gpio_dt_spec *pIntGpio, uint8_t interfaceNum);
static int open_device_in(DeviceInput_t *pDevIn);
static int open_mouse(void);
static void handle_device(void);
static int handle_mouse(DeviceInput_t *pDevIn);
static void close_all_devices(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    int ret = -1;

    // GPIO for CH375 INT -> PC13
    static const struct gpio_dt_spec ch375_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };

    // Print banner
    printk("%s%s%s", "\x1b[36m", banner, "\x1b[0m");

    // Initialize GPIOC
    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    if (0 == device_is_ready(gpioc)) {
        LOG_ERR("GPIO C not ready!");
        return -1;
    }
    
    // Initialize USB device stack (STM32 -> PC)
    LOG_INF("Initializing USB HID device stack");
    ret = usbhid_proxyInit();
    if (ret < 0) {
        LOG_ERR("Failed to initialize USB device: %d", ret);
        return ret;
    }
    
    // Wait for USB enumeration with PC
    while (true != usbhid_proxyIsReady()) {
        k_msleep(100);
    }
    LOG_INF("[*] USB device ready!");

    while (1) {
        
        // Initialize CH375
        ret = init_ch375_device(&s_arr_devin[0], "CH375A", CH375_A_USART_INDEX, 
                               &ch375_int, 0);
        if (ret < 0) {
            LOG_ERR("CH375 initialization failed, retrying in 2000ms");
            close_all_devices();
            k_msleep(2000);
            continue;
        }
        
        // Wait for mouse connection
        LOG_INF("Waiting for mouse connection...");
        ret = ch375_hostWaitDeviceConnect(s_arr_devin[0].ch375_ctx, 30000);
        if (CH375_HOST_SUCCESS != ret) {
            LOG_WRN("No device connected, retrying...");
            close_all_devices();
            k_msleep(1000);
            continue;
        }
        LOG_INF("[*] Mouse detected!");
        
        // Enumerate and open device
        ret = open_mouse();
        if (ret < 0) {
            LOG_ERR("Failed to open mouse: %d", ret);
            continue;
        }
        
        LOG_INF("[*] Forwarding input to PC now");
        LOG_INF("  Buttons: %d, Axes: %d-bit, Wheel: %s",
                s_arr_devin[0].mouse.button.count,
                s_arr_devin[0].mouse.orientation.size,
                s_arr_devin[0].mouse.has_wheel ? "Y" : "N");
        
        proxyRunning = true;
        handle_device();
        proxyRunning = false;
        
        // Cleanup and prepare for reconnection
        LOG_WRN("Mouse disconnected, cleaning up...");
        close_all_devices();
        k_msleep(2000);
    }
    
    return 0;
}

/**
 * @brief Initialize CH375 USB host controller
 */
static int init_ch375_device(DeviceInput_t *pDevIn, const char *pName, int usart_index, 
                            const struct gpio_dt_spec *pIntGpio, uint8_t interfaceNum) {
    
    int ret = -1;
    uint8_t version;

    pDevIn->name = pName;
    pDevIn->interface_num = interfaceNum;
    pDevIn->int_gpio = *pIntGpio;
    pDevIn->last_report_ts_ms = 0;
    pDevIn->report_interval_ms = 8;
    pDevIn->is_connected = 0;
    
    LOG_INF("%s: Initializing hardware at 9600 baud", pName);
    ret = ch375_hwInitManual(pName, usart_index, pIntGpio, 9600, &pDevIn->ch375_ctx);
    if (ret < 0) {
        LOG_ERR("%s: Hardware init failed: %d", pName, ret);
        return ret;
    }
    
    LOG_INF("%s: Initializing CH375 host mode", pName);
    ret = ch375_hostInit(pDevIn->ch375_ctx, 115200);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("%s: Host init failed: %d", pName, ret);
        ch375_closeContext(pDevIn->ch375_ctx);
        pDevIn->ch375_ctx = NULL;
        return ret;
    }
    
    // Switch to new baudrate
    LOG_INF("%s: Switching STM32 UART to 115200", pName);
    ret = ch375_hwSetBaudrate(pDevIn->ch375_ctx, 115200);
    if (ret < 0) {
        LOG_ERR("%s: STM32 baudrate switch failed: %d", pName, ret);
        ch375_closeContext(pDevIn->ch375_ctx);
        pDevIn->ch375_ctx = NULL;
        return ret;
    }
    
    // Verify communication at new baudrate
    ret = ch375_getVersion(pDevIn->ch375_ctx, &version);
    if (CH375_SUCCESS == ret) {
        LOG_INF("[*] %s: CH375 ready (version: 0x%02X)", pName, version);
    } else {
        LOG_WRN("%s: Version check failed: %d", pName, ret);
    }
    
    return 0;
}

/**
 * @brief Open device input (USB HID host side)
 */
static int open_device_in(DeviceInput_t *pDevIn) {
    
    int ret = -1;
    
    LOG_INF("%s: Opening USB device", pDevIn->name);
    
    ret = ch375_hostUdevOpen(pDevIn->ch375_ctx, &pDevIn->usb_dev);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("%s: Failed to open USB device: %d", pDevIn->name, ret);
        return ret;
    }
    
    LOG_INF("%s: USB device opened (VID:PID = %04X:%04X)", pDevIn->name, 
                pDevIn->usb_dev.vendor_id, pDevIn->usb_dev.product_id);
    
    // Open HID device
    ret = USBHID_open(&pDevIn->usb_dev, pDevIn->interface_num, &pDevIn->hid_dev);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("%s: Failed to open USBHID: %d", pDevIn->name, ret);
        return ret;
    }

    LOG_INF("[*] %s: HID device opened (type: %s)", pDevIn->name, 
            pDevIn->hid_dev.hid_type == USBHID_TYPE_MOUSE ? "MOUSE" : 
            pDevIn->hid_dev.hid_type == USBHID_TYPE_KEYBOARD ? "KEYBOARD" : "UNKNOWN");

    if (USBHID_TYPE_MOUSE == pDevIn->hid_dev.hid_type) {
        ret = hidMouse_Open(&pDevIn->hid_dev, &pDevIn->mouse);
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("%s: Failed to open mouse: %d", pDevIn->name, ret);
            USBHID_close(&pDevIn->hid_dev);
            ch375_hostUdevClose(&pDevIn->usb_dev);
            return -1;
        }
        LOG_INF("[*] %s: Mouse opened", pDevIn->name);
    } else {
        LOG_ERR("%s: Unsupported HID type: %d", pDevIn->name, pDevIn->hid_dev.hid_type);
        USBHID_close(&pDevIn->hid_dev);
        ch375_hostUdevClose(&pDevIn->usb_dev);
        return -1;
    }
    
    return 0;
}

/**
 * @brief Open mouse device and allocate buffers
 * @return 0 on success, error code otherwise
 */
static int open_mouse(void)
{  
    int ret = open_device_in(&s_arr_devin[0]);
    if (ret < 0) {
        LOG_ERR("%s: Failed to enumerate", s_arr_devin[0].name);
        close_all_devices();
        k_msleep(1000);
        return ret;
    }

    s_arr_devin[0].is_connected = 1;

    return 0;
}

/**
 * @brief Main HID processing loop
 */
static void handle_device(void) {

    int ret = -1;

    LOG_INF("HID processing loop started");

    while (true == proxyRunning) {

        DeviceInput_t *pDevIn = &s_arr_devin[0];

        if (0 == pDevIn->is_connected) {
            continue;
        }

        if (USBHID_TYPE_MOUSE == pDevIn->hid_dev.hid_type) {
            ret = handle_mouse(pDevIn);
            if (USBHID_NO_DEV == ret) {
                LOG_ERR("%s: Device disconnected", pDevIn->name);
                return;
            }
        }
        
        k_msleep(pDevIn->report_interval_ms);
    }
}

/**
 * @brief Forward mouse data to PC
 */
static int handle_mouse(DeviceInput_t *pDevIn) {

    int ret = -1;
    struct HID_Mouse_t *mouse = &pDevIn->mouse;

    // Try to fetch new report
    ret = hidMouse_FetchReport(mouse);
    if (USBHID_NO_DEV == ret) {
        LOG_ERR("%s: Device disconnected", pDevIn->name);
        return USBHID_NO_DEV;
    }

    // Return if we don't have new data
    if (USBHID_SUCCESS != ret) {
        return 0;
    }

    // Send mouse report to PC
    ret = hidOutput_sendMouseReport(mouse);
    if (0 != ret && -EBUSY != ret) {
        LOG_WRN("%s: Failed to send report: %d", pDevIn->name, ret);
    }

    return 0;
}

/**
 * @brief Cleanup all resources
 */
static void close_all_devices(void)
{
    DeviceInput_t *pDevIn = &s_arr_devin[0];

    if (NULL != pDevIn->mouse.hid_dev) {
        hidMouse_Close(&pDevIn->mouse);
        memset(&pDevIn->mouse, 0, sizeof(struct HID_Mouse_t));
    }
    
    if (NULL != pDevIn->hid_dev.pUdev) {
        USBHID_close(&pDevIn->hid_dev);
        memset(&pDevIn->hid_dev, 0, sizeof(struct USBHID_Device_t));
    }
    
    if (NULL != pDevIn->usb_dev.ctx) {
        ch375_hostUdevClose(&pDevIn->usb_dev);
        memset(&pDevIn->usb_dev, 0, sizeof(struct USB_Device_t));
    }
    
    if (NULL != pDevIn->ch375_ctx) {
        ch375_closeContext(pDevIn->ch375_ctx);
        pDevIn->ch375_ctx = NULL;
    }
    
    pDevIn->is_connected = 0;
}