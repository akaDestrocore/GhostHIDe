/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_mouse.h
 * @brief          HID mouse device interface
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * High-level interface for USB HID mouse devices. Provides functions to
 * open mouse, fetch reports, and get/set button states and axis values.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef HID_MOUSE_H
#define HID_MOUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>
#include "ch375_host.h"
#include "hid_parser.h"

// define report ID byte used by your mouse if any
#if 1
#define MOUSE_REPORTID_BYTE 0x01
#endif

/**
 * @brief HID Mouse Button Definitions
 */
typedef enum {
    HID_MOUSE_BUTTON_LEFT   = 0,
    HID_MOUSE_BUTTON_RIGHT  = 1,
    HID_MOUSE_BUTTON_MIDDLE = 2,
    HID_MOUSE_BUTTON_4      = 3,
    HID_MOUSE_BUTTON_5      = 4
} hidMouse_Button_e;

/**
 * @brief HID Mouse Axis Definitions
 */
typedef enum {
    HID_MOUSE_AXIS_X        = 0,
    HID_MOUSE_AXIS_Y        = 1,
    HID_MOUSE_AXIS_WHEEL    = 2
} hidMouse_Axis_e;

// Forward declare USB device and HID descriptor
struct USB_Device_t;
struct USB_HID_Descriptor_t;

/**
 * @brief HID Mouse Structure
 */
struct HID_Mouse_t {
    struct USBHID_Device_t *hid_dev;
    uint32_t report_len;
    bool has_report_id_declared;
    uint8_t report_id_offset;
    struct HID_DataDescriptor_t button;
    struct HID_DataDescriptor_t orientation;
    struct HID_DataDescriptor_t wheel;
    bool has_wheel;
};

/**
 * @brief HID Mouse Functions
 */
int hidMouse_Open(struct USBHID_Device_t *pHIDDev, struct HID_Mouse_t *pMouse);
void hidMouse_Close(struct HID_Mouse_t *pMouse);
int hidMouse_FetchReport(struct HID_Mouse_t *pMouse);

/**
 * @brief Button functions
 */
int hidMouse_GetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t *pValue, bool isLast);
int hidMouse_SetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t value, bool isLast);

/**
 * @brief Orientation functions
 */
int hidMouse_GetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t *pValue, bool isLast);
int hidMouse_SetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t value, bool isLast);

#ifdef __cplusplus
}
#endif

#endif /* HID_MOUSE_H */