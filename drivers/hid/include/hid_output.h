/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_output.h
 * @brief          HID mouse interface
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Provides functions to build and send HID mouse reports in standardized HID format.
 * 
 * @copyright
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef HID_OUTPUT_H
#define HID_OUTPUT_H

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <stdint.h>
#include <stdbool.h>
#include "usb_hid_proxy.h"
#include "hid_mouse.h"


#define HID_OUTPUT_REPORT_SIZE 6

// Build translated report from any mouse format
int hidOutput_buildMouseReport(struct HID_Mouse_t *pMouse, uint8_t *pOutReport);

// Translate and send
int hidOutput_sendMouseReport(struct HID_Mouse_t *pMouse);

#endif /* HID_OUTPUT_H */