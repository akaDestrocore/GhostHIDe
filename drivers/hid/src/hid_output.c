/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_output.c
 * @brief          HID mouse translation layer implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements translation layer between the input HID devices and the USB 
 * device output. This includes translation of variable-format mouse reports 
 * into a standardized output format.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "hid_output.h"

LOG_MODULE_REGISTER(hid_output, LOG_LEVEL_DBG);


/**
 * @brief Build a standardized HID mouse report from the input data
 * @param pMouse Pointer to the HID device structure
 * @param pOutReport Pointer to the output report
 * @return 0 on success, error code otherwise
 */
int hidOutput_buildMouseReport(struct HID_Mouse_t *pMouse, uint8_t *pOutReport) {

    int ret = -1;

    if (NULL == pMouse || NULL == pOutReport) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }

    if (true == pMouse->has_report_id_declared && 1 == pMouse->report_id_offset) {
        uint8_t *pInputBuff = NULL;
        uint32_t inputLen = 0;

        // Get input buffer from the mouse device
        ret = USBHID_getReportBuffer(pMouse->hid_dev, &pInputBuff, &inputLen, false);
        if (NULL == pInputBuff || USBHID_SUCCESS != ret) {
            LOG_ERR("Failed to get input buffer for Report ID check");
            return -EIO;
        }

        // Check 0th byte of input buffer
        uint8_t reportID = pInputBuff[0];
        if (MOUSE_REPORTID_BYTE != reportID) {
            // Ignore all other reports that are not related to movement/button click
            static uint32_t ignoredCount = 0;
            if (++ignoredCount <= 5) {
                LOG_DBG("Ignoring non-movement Report ID: 0x%02X", reportID);
            }
            return -EAGAIN;
        }
    }

    memset(pOutReport, 0x00, HID_OUTPUT_REPORT_SIZE);

    // Buttons [0]
    uint8_t buttons = 0;

    for (uint32_t i = 0; i < 8 && i < pMouse->button.count; i++) {
        uint32_t value;
        ret = hidMouse_GetButton(pMouse, i, &value, false);
        if (0 != value && USBHID_SUCCESS == ret) {
            buttons |= (1 << i);
        }
    }
    pOutReport[0] = buttons;

    // X/Y axes [1:4]
    int32_t xValue = 0;
    int32_t yValue = 0;

    ret = hidMouse_GetOrientation(pMouse, HID_MOUSE_AXIS_X, &xValue, false);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to get X axis: %d", ret);
        return -EIO;
    }

    ret = hidMouse_GetOrientation(pMouse, HID_MOUSE_AXIS_Y, &yValue, false);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to get Y axis: %d", ret);
        return -EIO;
    }

    // Convert to LE
    sys_put_le16((uint16_t)xValue, &pOutReport[1]);
    sys_put_le16((uint16_t)yValue, &pOutReport[3]);

    // Wheel [5]
    int32_t wheelValue = 0;

    if (true == pMouse->has_wheel) {
        ret = hidMouse_GetOrientation(pMouse, HID_MOUSE_AXIS_WHEEL, &wheelValue, false);
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("Failed to get wheel: %d", ret);
            return -EIO;
        }

        // Clamp to int8_t
        if (wheelValue > INT8_MAX) wheelValue = INT8_MAX;
        if (wheelValue < INT8_MIN) wheelValue = INT8_MIN;
    }

    pOutReport[5] = (uint8_t)(int8_t)wheelValue;

    static int sampleCount = 0;
    sampleCount++;
    
    if (sampleCount <= 10 || 0 != xValue || 0 != yValue || 0 != wheelValue || 0 != buttons) {
        LOG_DBG("Output: BTN=0x%02X X=%d Y=%d WHEEL=%d", 
                pOutReport[0], (int16_t)xValue, (int16_t)yValue, 
                (int8_t)pOutReport[5]);
    } 

    return 0;
}

/**
 * @brief Send a mouse report to the host
 * @param pMouse Pointer to the HID device structure
 * @return 0 on success, error code otherwise
 */
int hidOutput_sendMouseReport(struct HID_Mouse_t *pMouse) {
    
    int ret = -1;
    uint8_t pReportBuff[HID_OUTPUT_REPORT_SIZE];

    if (NULL == pMouse) {
        LOG_ERR("Invalid mouse structure pointer");
        return -EINVAL;
    }

    ret = hidOutput_buildMouseReport(pMouse, pReportBuff);
    if (-EAGAIN == ret) {
        // Probably just a non-movement report that got filtered
        return 0;
    }

    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to build output report: %d", ret);
        return ret;
    }

    ret = usbhid_proxySendReport(0, pReportBuff, HID_OUTPUT_REPORT_SIZE);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    return 0;
}