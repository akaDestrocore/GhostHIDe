/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           usb_hid_proxy.h
 * @brief          USB HID proxy implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * This module uses Zephyr's legacy USB device stack (usb_device.h, usb/class/usb_hid.h)
 * which is marked as @deprecated in favor of the new USBD stack (usbd.h, usbd_hid.h).
 * 
 * The legacy stack is intentionally retained because:
 *  1. Dynamic reconnection: This application requires USB re-initialization after
 *     device disconnection/reconnection cycles. The new USBD stack does not support
 *     calling usbd_init() multiple times (returns -EALREADY on subsequent calls).
 * 
 *  2. Simplified lifecycle: The legacy stack's usb_enable()/usb_disable() pattern
 *     works seamlessly with hot-plug scenarios, while USBD requires persistent
 *     initialization and complex state management across reconnection events.
 * 
 *  3. Minimal devicetree overhead: USBD requires extensive devicetree configuration
 *     for composite devices, while the legacy stack handles this programmatically.
 *
 * Migration to USBD would require architectural
 * changes incompatible with the dynamic device proxy model.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "usb_hid_proxy.h"

LOG_MODULE_REGISTER(usb_hid_proxy, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static void mouse_int_in_ready(const struct device *pDev);
static void kbd_int_in_ready(const struct device *pDev);
void usb_status_cb(enum usb_dc_status_code status, const uint8_t *pParam);

/* Private variables ---------------------------------------------------------*/
static const struct device *pHidDevMouse = NULL;
static const struct device *pHidDevKbd = NULL;
static K_SEM_DEFINE(mouseSem, 1, 1);
static K_SEM_DEFINE(kbdSem, 1, 1);
static volatile enum usb_dc_status_code gUsbStatus = USB_DC_UNKNOWN;
static volatile bool isUsbConfigured = false;
static const struct hid_ops mouseOps = {
    .int_in_ready = mouse_int_in_ready,
};

static const struct hid_ops kbdOps = {
    .int_in_ready = kbd_int_in_ready,
};

/**
 * @brief Generic HID Mouse Report Descriptor
 */
static const uint8_t hidMouseReportDesc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)

    0x05, 0x09,        //    Usage Page (Buttons)
    0x19, 0x01,        //    Usage Minimum (Button 1)
    0x29, 0x08,        //    Usage Maximum (Button 8)
    0x15, 0x00,        //    Logical Minimum (0)
    0x25, 0x01,        //    Logical Maximum (1)
    0x95, 0x08,        //    Report Count (8)
    0x75, 0x01,        //    Report Size (1 bit)
    0x81, 0x02,        //    Input (Data, Variable, Absolute)

    0x05, 0x01,        //    Usage Page (Generic Desktop)
    0x09, 0x30,        //    Usage (X)
    0x09, 0x31,        //    Usage (Y)
    0x16, 0x00, 0x80,  //    Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //    Logical Maximum (32767)
    0x75, 0x10,        //    Report Size (16 bits)
    0x95, 0x02,        //    Report Count (2)
    0x81, 0x06,        //    Input (Data, Variable, Relative)

    0x09, 0x38,        //    Usage (Wheel)
    0x15, 0x81,        //    Logical Minimum (-127)
    0x25, 0x7F,        //    Logical Maximum (127)
    0x75, 0x08,        //    Report Size (8 bits)
    0x95, 0x01,        //    Report Count (1)
    0x81, 0x06,        //    Input (Data, Variable, Relative)

    0xC0               // End Collection
};

static const uint8_t hidKbdReportDesc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Constant)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data, Array)
    0xC0               // End Collection
};

/**
 * @brief Generic HID Keyboard Report Descriptor
 */

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
  * @brief Initialize USB HID devices
  * @retval 0 on success, error code otherwise
  */
int usbhid_proxyInit(void) {
    
    int ret = -1;

    // Reset first
    isUsbConfigured = false;
    gUsbStatus = USB_DC_UNKNOWN;


    // Get bindings for both devices
    pHidDevMouse = device_get_binding("HID_0");
    if (NULL == pHidDevMouse) {
        LOG_ERR("HID_0 device not found");
        return -ENODEV;
    }

    pHidDevKbd = device_get_binding("HID_1");
    if (NULL == pHidDevKbd) {
        LOG_ERR("HID_1 device not found");
        return -ENODEV;
    }

    // Register descriptors
    usb_hid_register_device(pHidDevMouse, hidMouseReportDesc, sizeof(hidMouseReportDesc), &mouseOps);
    LOG_INF("Mouse descriptor registered (HID_0)");
    usb_hid_register_device(pHidDevKbd, hidKbdReportDesc, sizeof(hidKbdReportDesc), &kbdOps);
    LOG_INF("Keyboard descriptor registered (HID_1)");

    // Initialize each HID device
    ret = usb_hid_init(pHidDevMouse);
    if (0 != ret) {
        LOG_ERR("Failed to initialize HID device: %d", ret);
        return -ENODEV;
    }

    ret = usb_hid_init(pHidDevKbd);
    if (0 != ret) {
        LOG_ERR("Failed to initialize HID device: %d", ret);
        return -ENODEV;
    }

    // Enable USB
    ret = usb_enable(usb_status_cb);
    if (0 != ret) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return ret;
    }

    LOG_INF("Waiting for enumeration.");

    return 0;
}

/**
 * @brief Check if the HID device is ready to be used
 * @return USB status
 */
bool usbhid_proxyIsReady(void)
{
    return isUsbConfigured;
}

/**
 * @brief Send report ot endpoint
 * @param ifaceNum interface number
 * @param pReport pointer to the report descriptor
 * @param len size of the report descriptor
 * @return 0 on success, error code otherwise
 */
int usbhid_proxySendReport(uint8_t ifaceNum, uint8_t *pReport, size_t len) {
    
    int ret = -1;
    static uint32_t sendCount[2] = {0,0};
    const struct device *pDev;
    struct k_sem *semaphore;

    if (NULL == pReport || 0 == len) {
        return -EINVAL;
    }
    
    if (true != isUsbConfigured) {
        static uint32_t not_ready_count = 0;
        if (++not_ready_count % 100 == 0) {
            LOG_WRN("USB not configured (%" PRIu32 " times)", not_ready_count);
        }
        return -EAGAIN;
    }
    
    if (0 == ifaceNum) {
        pDev = pHidDevMouse;
        semaphore = &mouseSem;
    }
    else if (1 == ifaceNum) {
        pDev = pHidDevKbd;
        semaphore = &kbdSem;
    } else {
        return -EINVAL;
    }

    if (NULL == pDev) {
        return -ENODEV;
    }
    
    sendCount[ifaceNum]++;
    
    // Wait for EP to be ready
    if (0 != k_sem_take(semaphore, K_MSEC(100))) {
        static uint32_t busyCount[2] = {0, 0};
        if (++busyCount[ifaceNum] % 50 == 0) {
            LOG_WRN("Interface %d: Semaphore busy (%" PRIu32 " times)", ifaceNum, busyCount[ifaceNum]);
        }
        return -EBUSY;
    }
    
    // Write report
    ret = hid_int_ep_write(pDev, pReport, len, NULL);
    
    if (0 != ret) {
        // Give semaphore back on failure
        k_sem_give(semaphore);
        
        static uint32_t writeFailCount[2] = {0, 0};
        if (++writeFailCount[ifaceNum] % 50 == 0) {
            LOG_ERR("Interface %d: Write failed %" PRIu32 " times (last ret=%d)", ifaceNum, writeFailCount[ifaceNum], ret);
        }
        
        return ret;
    }
    
    // Sample successful sends
    if (sendCount[ifaceNum] % 100 == 0) {
        LOG_DBG("Interface %d: Send #%" PRIu32 " successful", ifaceNum, sendCount[ifaceNum]);
    }
    
    return 0;
}

/**
 * @brief Send report ot endpoint
 * @return 0 on success, error code otherwise
 */
void usbhid_proxyCleanup(void) {
    
    usb_disable();
    isUsbConfigured = false;
    pHidDevMouse = NULL;
    pHidDevKbd = NULL;
    
    // Reset semaphore
    k_sem_reset(&mouseSem);
    k_sem_give(&mouseSem);
    k_sem_reset(&kbdSem);
    k_sem_give(&kbdSem);
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static void mouse_int_in_ready(const struct device *pDev) {
    
    (void)(pDev);
    k_sem_give(&mouseSem);
    LOG_DBG("Mouse endpoint ready");
}

static void kbd_int_in_ready(const struct device *pDev) {
    
    (void)(pDev);
    k_sem_give(&kbdSem);
    LOG_DBG("Keyboard endpoint ready");
}

void usb_status_cb(enum usb_dc_status_code status, const uint8_t *pParam) {
    
    (void)(pParam);
    gUsbStatus = status;
    
    LOG_INF("USB Status Change: 0x%02X", status);
    
    switch (status) {
        case USB_DC_ERROR: {
            LOG_ERR("USB_DC_ERROR");
            isUsbConfigured = false;
            break;
        }
            
        case USB_DC_RESET: {
            LOG_INF("USB_DC_RESET - device being reset");
            isUsbConfigured = false;
            break;
        }
            
        case USB_DC_CONNECTED: {
            LOG_INF("USB_DC_CONNECTED - cable connected");
            break;
        }
            
        case USB_DC_CONFIGURED: {
            isUsbConfigured = true;
            LOG_INF("USB_DC_CONFIGURED - ready for writes!");
            break;
        }
            
        case USB_DC_DISCONNECTED: {
            isUsbConfigured = false;
            LOG_WRN("USB_DC_DISCONNECTED");
            break;
        }
            
        case USB_DC_SUSPEND: {
            LOG_INF("USB_DC_SUSPEND");
            break;
        }
            
        case USB_DC_RESUME: {
            isUsbConfigured = true;
            LOG_INF("USB_DC_RESUME");
            break;
        }
            
        case USB_DC_INTERFACE: {
            LOG_DBG("USB_DC_INTERFACE");
            break;
        }
            
        case USB_DC_SET_HALT: {
            LOG_DBG("USB_DC_SET_HALT");
            break;
        }
            
        case USB_DC_CLEAR_HALT: {
            LOG_DBG("USB_DC_CLEAR_HALT");
            break;
        }
            
        case USB_DC_SOF: {
            // LOG_DBG("USB_DC_SOF");
            break;
        }
            
        case USB_DC_UNKNOWN:
        default: {
            LOG_WRN("USB_DC_UNKNOWN: 0x%02X", status);
            break;
        }
    }
}