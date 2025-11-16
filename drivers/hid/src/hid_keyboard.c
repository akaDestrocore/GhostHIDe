/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_keyboard.c
 * @brief          HID keyboard device implementation with dynamic parsing
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements keyboard-specific HID report parsing including modifier keys. 
 * This includes functions open keyboard, fetch reports, and get/set modifier 
 * keys and key codes.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "hid_keyboard.h"

LOG_MODULE_REGISTER(hid_keyboard, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static int parse_hid_report(struct HID_Keyboard_t *pKbd, uint8_t *pReport, uint16_t len);


/**
 * @brief HID Keyboard open function
 * @param pHIDDev Pointer to the USB HID Device structure
 * @param pKbd Pointer to the HID keyboard structure
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_Open(struct USBHID_Device_t *pHIDDev, struct HID_Keyboard_t *pKbd) {

    int ret = -1;

    if(NULL == pKbd || NULL == pHIDDev) {
        LOG_ERR("Invalid parameters");
        return USBHID_PARAM_INVALID;
    }

    if (USBHID_TYPE_KEYBOARD != pHIDDev->hid_type) {
        LOG_ERR("Not a keyboard device");
        return USBHID_NOT_SUPPORT;
    }

    memset(pKbd, 0x00, sizeof(struct HID_Keyboard_t));
    pKbd->hid_dev = pHIDDev;

    ret = parse_hid_report(pKbd, pHIDDev->raw_hid_report_desc, pHIDDev->raw_hid_report_desc_len);
    if (ret < 0) {
        LOG_ERR("Failed to parse HID report");
        return USBHID_NOT_SUPPORT;
    }

    if (0 == pKbd->report_length) {
        LOG_ERR("Invalid report length");
        return USBHID_ERROR;
    }

    ret = USBHID_allocReportBuffer(pHIDDev, pKbd->report_length);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to allocate report buffer");
        return USBHID_ALLOC_FAILED;
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Keyboard wrapper for `USBHID_freeReportBuffer()`
 * @param pHIDDev Pointer to the USB HID Device structure
 * @return None
 */
void hidKeyboard_Close(struct HID_Keyboard_t *pKbd) {

    if (NULL == pKbd) {
        return;
    }

    USBHID_freeReportBuffer(pKbd->hid_dev);
    memset(pKbd, 0x00, sizeof(struct HID_Keyboard_t));
}

/**
 * @brief Keyboard wrapper for `USBHID_getReport()`
 * @param pKbd Pointer to the HID keyboard structure
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_FetchReport(struct HID_Keyboard_t *pKbd) {

    if (NULL == pKbd) {
        return USBHID_PARAM_INVALID;
    }

    return USBHID_fetchReport(pKbd->hid_dev);
}

/**
 * @brief Get specific key code
 * @param pKbd Pointer to the HID keyboard structure
 * @param keyCode Key code to get
 * @param pVal Pointer to the value of the key code
 * @param isLast Flag indicating whether this is the last report
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_GetKey(struct HID_Keyboard_t *pKbd, uint32_t keyCode, uint32_t *pVal, bool isLast) {

    int ret = -1;
    uint8_t *pReportBuff;
    uint8_t *pKeysField;

    if (NULL == pKbd || NULL == pVal) {
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pKbd->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pKeysField = pReportBuff + pKbd->keys.report_buf_off;

    // Go through keys array to find the key code
    *pVal = 0;
    for (int i = 0; i < HID_KBD_MAX_KEYS; i++) {
        if (pKeysField[i] == keyCode) {
            *pVal = 1;
            break;
        }
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Set a specific key on the keyboard.
 * @param pKbd Pointer to the HID keyboard structure
 * @param keyCode Key code to set
 * @param pVal Pointer to the value of the key code
 * @param isLast Flag indicating whether this is the last report
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_SetKey(struct HID_Keyboard_t *pKbd, uint32_t keyCode, uint32_t value, bool isLast) {

    int ret = -1;
    uint8_t *pReportBuff;
    uint8_t *pKeysField;

    if (NULL == pKbd) {
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pKbd->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pKeysField = pReportBuff + pKbd->keys.report_buf_off;

    if (0 != value) {
        // Add a key if not present already
        for (int i = 0; i < HID_KBD_MAX_KEYS; i++) {
            if (0 == pKeysField[i]) {
                pKeysField[i] = keyCode;
                break;
            }
            else if (pKeysField[i] == keyCode) {
                // Already present
                break;
            }
        }
    } else {
        // Delete key
        for (int i = 0; i < HID_KBD_MAX_KEYS; i++) {
            if (pKeysField[i] == keyCode) {
                pKeysField[i] = 0;
                // Need to shift left remaining keys to fill the gap
                for (int j = i; j < (HID_KBD_MAX_KEYS - 1); j++) {
                    pKeysField[j] = pKeysField[j + 1];
                }
                pKeysField[HID_KBD_MAX_KEYS - 1] = 0;
                break;
            }
        }
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Get modifier key state
 * @param pKbd Pointer to the HID keyboard structure
 * @param modNum Modifier key number
 * @param pValue Pointer to the value variable
 * @param isLast True if this is the last key in the report
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_GetModifier(struct HID_Keyboard_t *pKbd, uint32_t modNum, uint32_t *pValue, bool isLast) {

    int ret = -1;
    uint8_t *pReportBuff;
    uint8_t *pModField;

    if (NULL == pKbd || NULL == pValue) {
        return USBHID_PARAM_INVALID;
    }

    // Mod should be a bit position
    if (modNum > 7) {
        LOG_ERR("Invalid modifier: %d", modNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pKbd->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pModField = pReportBuff + pKbd->modifier.report_buf_off;
    *pValue = (*pModField & (1 << (modNum & 0x07))) ? 1: 0;

    return USBHID_SUCCESS;
}

/**
 * @brief Set a modifier key on the keyboard
 * @param pKbd Pointer to the HID keyboard structure
 * @param modNum Modifier key number
 * @param pValue Pointer to the value variable
 * @param isLast True if this is the last key in the report
 * @return 0 on success, error code otherwise
 */
int hidKeyboard_SetModifier(struct HID_Keyboard_t *pKbd, uint32_t modNum, uint32_t value, bool isLast) {

    int ret = -1;
    uint8_t *pReportBuff;
    uint8_t *pModField;

    if (NULL == pKbd) {
        return USBHID_PARAM_INVALID;
    }

    // Mod should be a bit position
    if (modNum > 7) {
        LOG_ERR("Invalid modifier: %d", modNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pKbd->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pModField = pReportBuff + pKbd->modifier.report_buf_off;

    if (0 != value) {
        *pModField |= (1 << (modNum & 0x07));
    } else {
        *pModField &= ~(1 << (modNum & 0x07));
    }

    return USBHID_SUCCESS;
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static int parse_hid_report(struct HID_Keyboard_t *pKbd, uint8_t *pReport, uint16_t len) 
{
    struct HID_DataDescriptor_t *pKey = &pKbd->keys;
    struct HID_DataDescriptor_t *pMod = &pKbd->modifier;

    // Form standard HID keyboard report
    pKbd->report_length = HID_KBD_REPORT_SIZE;

    pMod->physical_minimum = 0;
    pMod->physical_maximum = 8;
    pMod->logical_minimum = 0;
    pMod->logical_maximum = 1;
    pMod->size = 8;
    pMod->count = 1;
    // Should be 0 in most keyboards
    pMod->report_buf_off = HID_KBD_MODIFIER_OFFSET;

    pKey->physical_minimum = 0;
    pKey->physical_maximum = 255;
    pKey->logical_minimum = 0;
    pKey->logical_maximum = 255;
    // 1 byte per key
    pKey->size = 8;
    // Up to 6 simultaneous keys
    pKey->count = HID_KBD_MAX_KEYS;
    pKey->report_buf_off = HID_KBD_KEYS_OFFSET;
    
    return 0;
}