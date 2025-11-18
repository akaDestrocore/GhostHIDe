/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           main.c
 * @brief          Main program entry point and HID device proxy
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Main application demonstrating USB HID device enumeration, descriptor
 * parsing, and real-time input forwarding. Manages dual CH375 USB host
 * controllers for mouse and keyboard passthrough.
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
#include "hid_keyboard.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Defines -------------------------------------------------------------------*/
#define CH375_MODULE_COUNT 2
#define DEFAULT_REPORT_INTERVAL_MS 8
#define MAIN_LOOP_SLEEP_MS 1
#define KEYBOARD_BREAK_TIMEOUT_MS 50
#define ENUMERATION_WAIT_TIMEOUT_MS 10000

/* Type Deffinitions ---------------------------------------------------------*/
typedef struct
{
    const char *name;
    struct gpio_dt_spec intGpio;
    const struct device *usartDev;

    struct ch375_Context_t *ch375Ctx;
    struct USB_Device_t usbDev;
    struct USBHID_Device_t hidDev;
    union {
        struct HID_Mouse_t mouse;
        struct HID_Keyboard_t keyboard;
    };

    bool isConnected;
    uint8_t interfaceNum;

    uint32_t lastReportTimestampMs;
    uint32_t reportIntervalMs;
} DeviceInput_t;

/* Private variables ---------------------------------------------------------*/
static DeviceInput_t gDeviceInputs[CH375_MODULE_COUNT];

static const char banner[] = 
"                                                                      \n"
" ██████  ██   ██  ██████  ███████ ████████ ██   ██ ██ ██████  ███████ \n"
"██       ██   ██ ██    ██ ██         ██    ██   ██ ██ ██   ██ ██      \n"
"██   ███ ███████ ██    ██ ███████    ██    ███████ ██ ██   ██ █████   \n"
"██    ██ ██   ██ ██    ██      ██    ██    ██   ██ ██ ██   ██ ██      \n"
" ██████  ██   ██  ██████  ███████    ██    ██   ██ ██ ██████  ███████ \n";

/* Private function prototypes -----------------------------------------------*/
static int initCh375Device(DeviceInput_t *pDevIn, const char *pName, 
                            int usartIndex, const struct gpio_dt_spec *pIntGpio, 
                            uint8_t interfaceNum);
static int openDeviceInput(DeviceInput_t *pDevIn);
static void waitAllDevicesConnect(void);
static int openAllDeviceInputs(void);
static void loopHandleDevices(void);
static int handleMouseInput(DeviceInput_t *pDevIn);
static int handleKeyboardInput(DeviceInput_t *pDevIn);
static void closeAllDevices(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    int ret = -1;

    // GPIO for MOUSE INT -> PC13
    static const struct gpio_dt_spec ch375aIntGpio = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };

    // GPIO for KEYBOARD INT -> PC14
    static const struct gpio_dt_spec ch375bIntGpio = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 14,
        .dt_flags = GPIO_ACTIVE_LOW
    };

    // Print banner
    printk("%s%s%s", "\x1b[36m", banner, "\x1b[0m");

    // Initialize GPIOC
    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    if (true != device_is_ready(gpioc)) {
        LOG_ERR("GPIO C not ready!");
        return -1;
    }

    // Initialize CH375 USB host controllers
    ret = initCh375Device(&gDeviceInputs[0], "CH375A", 2, &ch375aIntGpio, 0);
    if (0 != ret) {
        return ret;
    }

    ret = initCh375Device(&gDeviceInputs[1], "CH375B", 3, &ch375bIntGpio, 1);
    if (0 != ret) {
        return ret;
    }

    while (1) {
        LOG_INF("Waiting for USB devices...");
        waitAllDevicesConnect();

        LOG_INF("Enumerating devices...");
        ret = openAllDeviceInputs();
        if (ret < 0) {
            LOG_ERR("[ FAILED ] Failed to enumerate devices");
            k_msleep(1000);
            continue;
        }

        LOG_INF("Initializing USB device output...");
        ret = usbhid_proxyInit();
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("[ FAILED ] USB HID proxy initialization failed: %d", ret);
            k_msleep(1000);
            continue;
        }

        LOG_INF("Waiting for USB enumeration...");
        int enumerationAttempts = 0;
        while (true != usbhid_proxyIsReady() && enumerationAttempts < 100) {
            k_msleep(100);
            enumerationAttempts++;
        }

        if (true != usbhid_proxyIsReady()) {
            LOG_ERR("USB enumeration timeout");
            usbhid_proxyCleanup();
            closeAllDevices();
            k_msleep(1000);
            continue;
        }

        LOG_INF("[ OK ] USB ready - starting forwarding");
        loopHandleDevices();

        LOG_WRN("Device disconnected, restarting...");
        usbhid_proxyCleanup();
        closeAllDevices();
        k_msleep(1000);
    }
    
    return 0;
}

/**
 * @brief Initialize CH375 USB host controller
 * @param pDevIn Pointer to device input structure
 * @param pName Device name for logging
 * @param usartIndex Hardware USART index
 * @param pIntGpio GPIO interrupt pin specification
 * @param interfaceNum USB interface number (0=mouse, 1=keyboard)
 * @return 0 on success, negative error code otherwise
 */
static int initCh375Device(DeviceInput_t *pDevIn, const char *pName, int usartIndex, const struct gpio_dt_spec *pIntGpio, uint8_t interfaceNum) {
  
    int ret = -1;

    pDevIn->name = pName;
    pDevIn->interfaceNum = interfaceNum;
    pDevIn->intGpio = *pIntGpio;
    pDevIn->lastReportTimestampMs = 0;
    pDevIn->reportIntervalMs = DEFAULT_REPORT_INTERVAL_MS;
    pDevIn->isConnected = false;

    ret = ch375_hwInitManual(pName, usartIndex, pIntGpio, CH375_DEFAULT_BAUDRATE, &pDevIn->ch375Ctx);
    if (ret < 0) {
        LOG_ERR("[ FAILED ] %s: Hardware init failed: %d", pName, ret);
        return ret;
    }

    ret = ch375_hostInit(pDevIn->ch375Ctx, CH375_WORK_BAUDRATE);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("[ FAILED ] %s: CH375 host init failed: %d", pName, ret);
        return -EIO;
    }

    ret = ch375_hwSetBaudrate(pDevIn->ch375Ctx, CH375_WORK_BAUDRATE);
    if (ret < 0) {
        LOG_ERR("%s: Baudrate switch failed: %d", pName, ret);
        return ret;
    }

    LOG_INF("[ OK ] %s: Initialized successfully!", pName);
    return 0;
}

/**
 * @brief Open and enumerate USB HID device
 * @param pDevIn Device input structure
 * @return 0 on success, negative error code otherwise
 */
static int openDeviceInput(DeviceInput_t *pDevIn)
{
    int ret = -1;

    LOG_INF("%s: Opening USB device...", pDevIn->name);

    ret = ch375_hostUdevOpen(pDevIn->ch375Ctx, &pDevIn->usbDev);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("%s: Failed to open USB device: %d", pDevIn->name, ret);
        return CH375_HOST_ERROR;
    }

    LOG_INF("[ OK ] %s: USB device opened (VID:PID = %04X:%04X)",
            pDevIn->name, pDevIn->usbDev.vendor_id, pDevIn->usbDev.product_id);

    ret = USBHID_open(&pDevIn->usbDev, 0, &pDevIn->hidDev);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("[ FAILED ] %s: Failed to open USBHID: %d", pDevIn->name, ret);
        ch375_hostUdevClose(&pDevIn->usbDev);
        return USBHID_ERROR;
    }

    pDevIn->reportIntervalMs = DEFAULT_REPORT_INTERVAL_MS;

    if (USBHID_TYPE_MOUSE == pDevIn->hidDev.hid_type) {
        ret = hidMouse_Open(&pDevIn->hidDev, &pDevIn->mouse);
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("[ FAILED ] %s: Failed to open mouse: %d", pDevIn->name, ret);
            USBHID_close(&pDevIn->hidDev);
            ch375_hostUdevClose(&pDevIn->usbDev);
            return USBHID_ERROR;
        }
        LOG_INF("[ OK ] %s: Mouse opened", pDevIn->name);
    } 
    else if (USBHID_TYPE_KEYBOARD == pDevIn->hidDev.hid_type) {
        ret = hidKeyboard_Open(&pDevIn->hidDev, &pDevIn->keyboard);
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("[ FAILED ] %s: Failed to open keyboard: %d", pDevIn->name, ret);
            USBHID_close(&pDevIn->hidDev);
            ch375_hostUdevClose(&pDevIn->usbDev);
            return -1;
        }
        LOG_INF("[ OK ] %s: Keyboard opened", pDevIn->name);
    } 
    else {
        LOG_ERR("[ FAILED ] %s: Unsupported HID type: %d", pDevIn->name, pDevIn->hidDev.hid_type);
        USBHID_close(&pDevIn->hidDev);
        ch375_hostUdevClose(&pDevIn->usbDev);
        return -1;
    }

    return 0;
}

/**
 * @brief Wait for all configured USB devices to connect
 * @note Blocks until all devices are connected
 */
static void waitAllDevicesConnect(void)
{
    int ret = -1;
    bool allConnected = false;

    while (1) {
        allConnected = true;

        for (int i = 0; i < CH375_MODULE_COUNT; i++) {
            DeviceInput_t *pDevIn = &gDeviceInputs[i];

            if (true == pDevIn->isConnected) {
                continue;
            }

            // 500ms timeout
            ret = ch375_hostWaitDeviceConnect(pDevIn->ch375Ctx, 500);
            if (CH375_HOST_SUCCESS == ret) {
                LOG_INF("[ OK ] %s: Device connected", pDevIn->name);
                pDevIn->isConnected = true;
            } 
            else if (CH375_HOST_ERROR == ret) {
                LOG_ERR("[ FAILED ] %s: Error waiting for device", pDevIn->name);
                allConnected = false;
            } else {
                allConnected = false;
            }
        }

        if (true == allConnected) {
            break;
        }

        k_msleep(100);
    }

    LOG_INF("[ OK ] All devices connected!");
}

/**
 * @brief Enumerate all connected USB devices
 * @return 0 on success, negative error code on failure
 */
static int openAllDeviceInputs(void)
{
    int ret = -1;

    for (int i = 0; i < CH375_MODULE_COUNT; i++) {
        ret = openDeviceInput(&gDeviceInputs[i]);
        if (ret < 0) {
            LOG_ERR("[ FAILED ] %s: Failed to enumerate", gDeviceInputs[i].name);
            return ret;
        }
        gDeviceInputs[i].isConnected = true;
    }

    return 0;
}

/**
 * @brief Main HID input forwarding loop
 * @note Runs until device disconnection is detected
 */
static void loopHandleDevices(void)
{
    int ret = -1;

    LOG_INF("HID processing loop started");

    while (1) {
        for (int i = 0; i < CH375_MODULE_COUNT; i++) {
            DeviceInput_t *pDevIn = &gDeviceInputs[i];

            if (true != pDevIn->isConnected) {
                continue;
            }

            if (USBHID_TYPE_MOUSE == pDevIn->hidDev.hid_type) {
                ret = handleMouseInput(pDevIn);

                if (USBHID_NO_DEV == ret) {
                    LOG_ERR("%s: Device disconnected", pDevIn->name);
                    return;
                }
            }

            else if (USBHID_TYPE_KEYBOARD == pDevIn->hidDev.hid_type) {
                ret = handleKeyboardInput(pDevIn);

                if (USBHID_NO_DEV == ret) {
                    LOG_ERR("%s: Device disconnected", pDevIn->name);
                    return;
                }
            }
        }

        k_msleep(MAIN_LOOP_SLEEP_MS);
    }
}

/**
 * @brief Handle mouse input and forward to USB output
 * @param pDevIn Device input structure
 * @return 0 on success, USBHID_NO_DEV on disconnection, negative on error
 */
static int handleMouseInput(DeviceInput_t *pDevIn)
{
    int ret= -1;

    // Fetch new report from device
    ret = hidMouse_FetchReport(&pDevIn->mouse);
    if (USBHID_NO_DEV == ret) {
        LOG_ERR("%s: Device disconnected", pDevIn->name);
        return ret;
    }

    // No new data
    if (USBHID_SUCCESS != ret) {
        return 0;
    }

    // Forward report to USB output
    ret = hidOutput_sendMouseReport(&pDevIn->mouse);
    if (0 != ret) {
        LOG_WRN("%s: Failed to send report: %d", pDevIn->name, ret);
    }

    return 0;
}

/**
 * @brief Handle keyboard input and forward to USB output
 * @param pDevIn Device input structure
 * @return 0 on success, USBHID_NO_DEV on disconnection, negative on error
 */
static int handleKeyboardInput(DeviceInput_t *pDevIn)
{
    int ret = -1;
    struct USBHID_Device_t *pHidDev;
    uint8_t *pReportBuff;
    size_t reportLen;

    static uint8_t lastKeyboardReport[8] = {0};
    static uint8_t lastSentReport[8] = {0};
    static uint32_t lastSendTimestampMs = 0;

    uint32_t currentTimeMs;
    bool areKeysPressed;

    pHidDev = pDevIn->keyboard.hid_dev;
    reportLen = pHidDev ? pHidDev->report_len : 0;
    currentTimeMs = k_uptime_get_32();
    areKeysPressed = false;

    // Check if any keys are currently pressed in last sent report
    for (int i = 0; i < reportLen; i++) {
        if (lastSentReport[i] != 0) {
            areKeysPressed = true;
            break;
        }
    }

    // Send break report if keys were pressed and timeout expired
    if (areKeysPressed && (currentTimeMs - lastSendTimestampMs) >= KEYBOARD_BREAK_TIMEOUT_MS) {
        static uint8_t breakReport[8] = {0};
        
        ret = usbhid_proxySendReport(pDevIn->interfaceNum, breakReport, reportLen);
        if (0 == ret) {
            memset(lastSentReport, 0, sizeof(lastSentReport));
            lastSendTimestampMs = currentTimeMs;
        }
    }

    // Fetch new report
    ret = hidKeyboard_FetchReport(&pDevIn->keyboard);

    if (USBHID_NO_DEV == ret) {
        return USBHID_NO_DEV;
    }

    // No new data
    if (USBHID_SUCCESS != ret) {
        return 0;
    }

    // Get report buffer
    ret = USBHID_getReportBuffer(pHidDev, &pReportBuff, NULL, false);
    if (USBHID_SUCCESS != ret || NULL == pReportBuff) {
        return 0;
    }

    // Skip if no chnages
    if (memcmp(pReportBuff, lastKeyboardReport, reportLen) == 0) {
        return 0;
    }

    memcpy(lastKeyboardReport, pReportBuff, reportLen);

    // Forward to USB output
    ret = usbhid_proxySendReport(pDevIn->interfaceNum, pReportBuff, reportLen);

    if (0 == ret) {
        memcpy(lastSentReport, pReportBuff, reportLen);
        lastSendTimestampMs = currentTimeMs;
    }  else {
        LOG_ERR("Keyboard send failed: %d", ret);
    }

    return 0;
}

/**
 * @brief Close all open USB devices and free resources
 */
static void closeAllDevices(void)
{
    for (int i = 0; i < CH375_MODULE_COUNT; i++) {
        DeviceInput_t *pDevIn = &gDeviceInputs[i];

        if (USBHID_TYPE_MOUSE == pDevIn->hidDev.hid_type) {
            hidMouse_Close(&pDevIn->mouse);
        } 
        
        else if (USBHID_TYPE_KEYBOARD == pDevIn->hidDev.hid_type) {
            hidKeyboard_Close(&pDevIn->keyboard);
        }

        USBHID_close(&pDevIn->hidDev);
        ch375_hostUdevClose(&pDevIn->usbDev);

        pDevIn->isConnected = false;
    }
}