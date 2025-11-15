/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_keyboard.h
 * @brief          HID keyboard device interface
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * High-level interface for USB HID keyboard devices. Provides functions to
 * open keyboard, fetch reports, and get/set modifier keys and key codes.
 * Supports standard boot protocol keyboards. So, things like RGB lighting
 * might not work if they are using a vendor specific protocol.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef HID_KEYBOARD_H
#define HID_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ch375_host.h"
#include "hid_parser.h"

/**
 * @brief Standard HID keyboard report layout
 */
#define HID_KBD_REPORT_SIZE     8
#define HID_KBD_MODIFIER_OFFSET 0
#define HID_KBD_RESERVED_OFFSET 1
#define HID_KBD_KEYS_OFFSET     2
#define HID_KBD_MAX_KEYS        6

/**
 * @brief HID Keyboard key code macros
 */
#define HID_KBD_LETTER(x) (((x) >= 'a' && (x) <= 'z') ? ((x) - 'a' + 4) : \
                          ((x) >= 'A' && (x) <= 'Z') ? ((x) - 'A' + 4) : 0)
#define HID_KBD_NUMBER(x) (((x) >= '1' && (x) <= '9') ? ((x) - '1' + 30) : \
                          ((x) == '0') ? 39 : 0)

/**
 * @brief HID Keyboard Structure
 */
struct HID_Keyboard_t {
    struct USBHID_Device_t *hid_dev;
    uint32_t report_length;
    struct HID_DataDescriptor_t modifier;
    struct HID_DataDescriptor_t keys;
};

/**
 * @brief HID keyboard functions
 */
int hidKeyboard_Open(struct USBHID_Device_t *pHIDDev, struct HID_Keyboard_t *pKbd);
void hidKeyboard_Close(struct HID_Keyboard_t *pKbd);
int hidKeyboard_FetchReport(struct HID_Keyboard_t *pKbd);

/**
 * @brief Key functions
 */
int hidKeyboard_GetKey(struct HID_Keyboard_t *pKbd, uint32_t keyCode, uint32_t *pVal, bool isLast);
int hidKeyboard_SetKey(struct HID_Keyboard_t *pKbd, uint32_t keyCode, uint32_t value, bool isLast);

/**
 * @brief Modifier key functions
 */
int hidKeyboard_GetModifier(struct HID_Keyboard_t *pKbd, uint32_t modNum, uint32_t *pValue, bool isLast);
int hidKeyboard_SetModifier(struct HID_Keyboard_t *pKbd, uint32_t modNum, uint32_t value, bool isLast);

#ifdef __cplusplus
}
#endif

#endif /* HID_KEYBOARD_H */