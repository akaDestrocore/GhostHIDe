/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           usb_hid_proxy.h
 * @brief          USB HID proxy for composite mouse + keyboard interfaces
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * USB HID proxy implementing a composite HID device exposing separate mouse 
 * and keyboard interfaces. This module:
 *  - Registers standard HID report descriptors for a multi-button mouse 
 *      (8-button, 16-bit X/Y, 8-bit wheel) and a standard 6-key keyboard
 * 
 * - Binds to Zephyr HID device instances ("HID_0" and "HID_1"), registers 
 *      descriptors and initializes the HID class devices.
 * 
 * - Provides a thread-safe send API (USB_HID_proxySendReport) using semaphores
 *      that start at 1 to coordinate endpoint writes and the INT callbacks
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef HID_PROXY_H
#define HID_PROXY_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USB HID
 */
int usbhid_proxyInit(void);

/**
 * @brief Send HID report
 */
int usbhid_proxySendReport(uint8_t ifaceNum, uint8_t *pReport, size_t len);

/**
 * @brief USB disable and reset of globals and semaphopres
 */
void usbhid_proxyCleanup(void);

/**
 * @brief Public API to check if USB is ready for writes
 */
bool usbhid_proxyIsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* HID_PROXY_H */
