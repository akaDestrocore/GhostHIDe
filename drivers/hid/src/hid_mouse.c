/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_mouse.c
 * @brief          HID mouse device implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements mouse-specific HID report parsing including button field
 * extraction, orientation (X/Y) axis handling, and wheel support.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "hid_mouse.h"

LOG_MODULE_REGISTER(hid_mouse, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static int parse_hid_report(struct HID_Mouse_t *pMouse, uint8_t *pReport, uint16_t len);

/**
 * @brief HID Mouse Open
 * @param pHIDDev Pointer to the USB HID Device structure
 * @param pMouse Pointer to the HID Mouse structure
 * @return 0 on success, error code otherwise
 */
int hidMouse_Open(struct USBHID_Device_t *pHIDDev, struct HID_Mouse_t *pMouse) {

    int ret = -1;

    if (NULL == pMouse || NULL == pHIDDev) {
        LOG_ERR("Invalid parameters");
        return USBHID_PARAM_INVALID;
    }

    if (USBHID_TYPE_MOUSE != pHIDDev->hid_type) {
        LOG_ERR("Not a mouse device");
        return USBHID_NOT_SUPPORT;
    }

    memset(pMouse, 0x00, sizeof(struct HID_Mouse_t));
    pMouse->hid_dev = pHIDDev;
    pMouse->has_wheel = false;

    ret = parse_hid_report(pMouse, pHIDDev->raw_hid_report_desc, pHIDDev->raw_hid_report_desc_len);

    if (ret < 0) {
        LOG_ERR("Failed to parse HID report");
        return USBHID_NOT_SUPPORT;
    }

    if (0 == pMouse->report_len) {
        LOG_ERR("Invalid report length");
        return USBHID_ERROR;
    }

    ret = USBHID_allocReportBuffer(pHIDDev, pMouse->report_len);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to allocate report buffer");
        return USBHID_ALLOC_FAILED;
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Mouse wrapper for `USBHID_freeReportBuffer()`
 * @param pMouse Pointer to the HID device structure
 * @return None
 */
void hidMouse_Close(struct HID_Mouse_t *pMouse) {
    
    if (NULL == pMouse) {
        return;
    }

    USBHID_freeReportBuffer(pMouse->hid_dev);
    memset(pMouse, 0x00, sizeof(struct HID_Mouse_t));
}

/**
 * @brief Fetch report from the device and parse it into a HID Mouse structure
 * @param pMouse Pointer to the HID device structure
 * @return 0 on success, error code otherwise
 */
int hidMouse_FetchReport(struct HID_Mouse_t *pMouse) {

    int ret = USBHID_PARAM_INVALID;

    if (NULL == pMouse) {
        return ret;
    }

    ret = USBHID_fetchReport(pMouse->hid_dev);
    
    // Only run detection when Report ID exists in descriptor
    if (USBHID_SUCCESS == ret && true == pMouse->has_report_id_declared && 0 == pMouse->report_id_offset) {
        uint8_t *pReportBuff;
        uint32_t actualReportLen;
        uint8_t byte0 = 0;
        uint8_t byte1 = 0;
        bool isReportID = (1 == byte0);
        bool isButtonsMaybe = (byte0 <= 0x1F);
        bool isButtons = (byte1 <= 0x1F);
        
        if (USBHID_SUCCESS != USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, &actualReportLen, false)) {
            return ret;
        }
        
        byte0 = pReportBuff[0];
        byte1 = pReportBuff[1];
        
        LOG_DBG("Report ID detection: byte0=0x%02X byte1=0x%02X", byte0, byte1);
    
        // If byte 0 is equal to a constant consistently, that's the Report ID
        isReportID = (byte0 == 1);
        isButtonsMaybe = (byte0 <= 0x1F);
        isButtons = (byte1 <= 0x1F);
        
        if (true == isReportID && true  == isButtons) {
            // Report ID is not stripped by CH375
            // byte0 = Report ID, byte1 = actual button data
            pMouse->report_id_offset = 1;
            pMouse->button.report_buf_off += 1;
            pMouse->orientation.report_buf_off += 1;
            if (true == pMouse->has_wheel) {
                pMouse->wheel.report_buf_off += 1;
            }
        
            LOG_INF("  Adjusted offsets: BTN=%d ORIENT=%d WHEEL=%d", pMouse->button.report_buf_off, 
                                pMouse->orientation.report_buf_off, pMouse->wheel.report_buf_off);
        } else {
            // Assume Report ID stripped by CH375
            pMouse->report_id_offset = 0;
        }
    }
    return ret;
}

/**
 * @brief Obtain button data from the mouse
 * @param pMouse Pointer to the HID device structure
 * @param buttonNum Button number to get
 * @param pValue Pointer to the value variable where the button state will be stored
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_GetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t *pValue, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pButtonDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t byteOff = buttonNum / 8;
    uint8_t bitOff = buttonNum % 8;

    if (NULL == pMouse || NULL == pValue) {
        return USBHID_PARAM_INVALID;
    }

    if (buttonNum >= pMouse->button.count) {
        LOG_ERR("Invalid button number: %d", buttonNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pButtonDesc = &pMouse->button;
    pFieldBuff = pReportBuff + pButtonDesc->report_buf_off;
    *pValue = (pFieldBuff[byteOff] & (0x01 << bitOff)) ? 1 : 0;

    return USBHID_SUCCESS;
}

/**
 * @brief Set button state
 * @param pMouse Pointer to the HID device structure
 * @param buttonNum Button number to get
 * @param value Button value to set
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_SetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t value, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pButtonDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t byteOff = buttonNum / 8;
    uint8_t bitOff = buttonNum % 8;

    if (NULL == pMouse) {
        return USBHID_PARAM_INVALID;
    }

    if (buttonNum >= pMouse->button.count) {
        LOG_ERR("Invalid button number: %d", buttonNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pButtonDesc = &pMouse->button;
    pFieldBuff = pReportBuff + pButtonDesc->report_buf_off;

    if (0 != value) {
        pFieldBuff[byteOff] |= (0x01 << bitOff);
    } else {
        pFieldBuff[byteOff] &= ~(0x01 << bitOff);
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Get X and Y axis values from the report buffer
 * @param pMouse Pointer to the HID device structure
 * @param axisNum Number of mouse orientation axes
 * @param pValue Pointer to the value variable
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_GetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t *pValue, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t valueByteSize;

    if (NULL == pMouse || NULL == pValue) {
        return USBHID_PARAM_INVALID;
    }
    
    // Wheel is stored separately
    if (2 == axisNum && true == pMouse->has_wheel) {
        pDesc = &pMouse->wheel;
        
        ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
        if (USBHID_SUCCESS != ret) {
            return ret;
        }
        
        pFieldBuff = pReportBuff + pDesc->report_buf_off;
        valueByteSize = pDesc->size / 8;
        
        // Wheel is typically 8-bit signed
        if (1 == valueByteSize) {
            *pValue = ((int8_t *)pFieldBuff)[0];
        } else if (2 == valueByteSize) {
            uint16_t raw;
            memcpy(&raw, pFieldBuff, sizeof(raw));
            raw = sys_le16_to_cpu(raw);
            *pValue = (int32_t)(int16_t)raw;
        } else {
            *pValue = 0;
        }
        
        return USBHID_SUCCESS;
    }

    // X/Y axes (0 and 1)
    if (axisNum >= pMouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d (max=%d)", axisNum, pMouse->orientation.count);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pDesc = &pMouse->orientation;
    pFieldBuff = pReportBuff + pDesc->report_buf_off;
    valueByteSize = pDesc->size / 8;

    if (0 == valueByteSize) {
        LOG_ERR("Invalid value size: size=%d bits", pDesc->size);
        return USBHID_ERROR;
    }

    switch (valueByteSize) {
        case 1: {
            *pValue = (int32_t)((int8_t)pFieldBuff[axisNum]);
            break;
        }

        case 2: {
            // Read 16-bit value as little-endian, then sign-extend to 32-bit
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 2);
            uint16_t raw;
            memcpy(&raw, pAxisStart, sizeof(raw));
            raw = sys_le16_to_cpu(raw);
            // Cast to int16_t for sign extension
            *pValue = (int32_t)(int16_t)raw;
            break;
        }

        case 4: {
            // Read 32-bit value as little-endian
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 4);
            uint32_t raw;
            memcpy(&raw, pAxisStart, sizeof(raw));
            *pValue = (int32_t)sys_le32_to_cpu(raw);
            break;
        }

        default: {
            LOG_ERR("Unexpected value size: %d", valueByteSize);
            return USBHID_ERROR;
        }
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Set desired value for mouse orientation axes
 * @param pMouse Pointer to the HID device structure
 * @param axisNum Number of mouse orientation axes
 * @param value Value to be set to the mouse orientation axes
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_SetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t value, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pOrientDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t valueByteSize;

    if (NULL == pMouse) {
        return USBHID_PARAM_INVALID;
    }

    if (axisNum >= pMouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d (max=%d)", axisNum, pMouse->orientation.count);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pOrientDesc = &pMouse->orientation;
    pFieldBuff = pReportBuff + pOrientDesc->report_buf_off;
    valueByteSize = pOrientDesc->size / 8;

    if (0 == valueByteSize) {
        LOG_ERR("Invalid value size: size=%d bits", pOrientDesc->size);
        return USBHID_ERROR;
    }

    switch (valueByteSize) {
        case 1: {
            ((int8_t *)pFieldBuff)[axisNum] = (int8_t)value;
            break;
        }

        case 2: {
            // Write as little-endian 16-bit
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 2);
            uint16_t raw = sys_cpu_to_le16((uint16_t)(int16_t)value);
            memcpy(pAxisStart, &raw, sizeof(raw));
            break;
        }

        case 4: {
            // Write as little-endian 32-bit
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 4);
            uint32_t raw = sys_cpu_to_le32((uint32_t)value);
            memcpy(pAxisStart, &raw, sizeof(raw));
            break;
        }

        default: {
            LOG_ERR("Unexpected value size: %d", valueByteSize);
            return USBHID_ERROR;
        }
    }

    return USBHID_SUCCESS;
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static int parse_hid_report(struct HID_Mouse_t *pMouse, uint8_t *pReport, uint16_t len) 
{
    struct HID_Item_t item;
    struct HID_DataDescriptor_t *pBtn = &pMouse->button;
    struct HID_DataDescriptor_t *pOrient = &pMouse->orientation;
    struct HID_DataDescriptor_t *pWheel = &pMouse->wheel;
    uint8_t *pHIDRep = pReport;
    uint8_t *pHIDRepEnd = pReport + len;
    uint32_t usagePage = 0;
    uint32_t usage = 0;
    uint32_t usages[8];
    uint32_t usageCount = 0;
    int32_t logicalMin = 0;
    int32_t logicalMax = 0;
    uint32_t reportSize = 0;
    uint32_t reportCount = 0;
    uint32_t reportOffset = 0;
    bool foundButtons = false;
    bool foundOrientation = false;
    bool foundWheel = false;
    bool hasReportId = false;
    uint8_t reportIdValue = 0;
    bool inMouseCollection = false;
    int collectionDepth = 0;

    while (pHIDRep < pHIDRepEnd) {
        uint8_t *oldPtr = pHIDRep;
        pHIDRep = HID_fetchItem(pHIDRep, pHIDRepEnd, &item);
        if (NULL == pHIDRep) {
            LOG_ERR("Failed to fetch HID item at offset %d", (int)(oldPtr - pReport));
            break;
        }

        switch(item.type) {
            case HID_ITEM_TYPE_GLOBAL: {
                switch(item.tag) {
                    case HID_GLOBAL_ITEM_TAG_USAGE_PAGE: {
                        usagePage = item.data.u32 << 16;
                        LOG_INF("  Usage Page: 0x%08X", usagePage);
                        break;
                    }
                    case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM: {
                        if (1 == item.size) {
                            logicalMin = (int8_t)item.data.u8;
                        } else if (2 == item.size) {
                            logicalMin = (int16_t)item.data.u16;
                        } else {
                            logicalMin = item.data.s32;
                        }
                        LOG_DBG("  Logical Min: %d", logicalMin);
                        break;
                    }
                    case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM: {
                        if (1 == item.size) {
                            logicalMax = (int8_t)item.data.u8;
                        } else if (2 == item.size) {
                            logicalMax = (int16_t)item.data.u16;
                        } else {
                            logicalMax = item.data.s32;
                        }
                        LOG_DBG("  Logical Max: %d", logicalMax);
                        break;
                    }
                    case HID_GLOBAL_ITEM_TAG_REPORT_SIZE: {
                        reportSize = item.data.u32;
                        LOG_DBG("  Report Size: %d bits", reportSize);
                        break;
                    }
                    case HID_GLOBAL_ITEM_TAG_REPORT_COUNT: {
                        reportCount = item.data.u32;
                        LOG_DBG("  Report Count: %d", reportCount);
                        break;
                    }
                    case HID_GLOBAL_ITEM_TAG_REPORT_ID: {
                        // Report ID actually exists but CH375 strips it from data, so we don't add to reportOffset
                        hasReportId = true;
                        reportIdValue = item.data.u8;
                        break;
                    }
                }
                break;
            }

            case HID_ITEM_TYPE_LOCAL: {
                if (HID_LOCAL_ITEM_TAG_USAGE == item.tag) {
                    usage = usagePage | item.data.u32;
                    LOG_DBG("  Usage: 0x%08X", usage);
                    
                    // Store usage in array
                    if (usageCount < 8) {
                        usages[usageCount++] = usage;
                    }
                }
                break;
            }

            case HID_ITEM_TYPE_MAIN: {
                if (HID_MAIN_ITEM_TAG_BEGIN_COLLECTION == item.tag) {
                    collectionDepth++;
                    // Detect mouse collection
                    if ((HID_GD_MOUSE == usage || HID_GD_POINTER == usage) && true != inMouseCollection) {
                        inMouseCollection = true;
                        LOG_INF("Mouse collection found at depth %d (Report ID %s)",
                                collectionDepth,
                                hasReportId ? "present but stripped" : "not present");
                        
                        foundButtons = false;
                        foundOrientation = false;
                        foundWheel = false;
                        reportOffset = 0;
                    }
                }
                else if (HID_MAIN_ITEM_TAG_END_COLLECTION == item.tag) {
                    if (collectionDepth > 0) {
                        collectionDepth--;
                    }
                    
                    if (true == inMouseCollection && 0 == collectionDepth) {
                        LOG_INF("Mouse collection ended at bit offset %d", reportOffset);
                        inMouseCollection = false;
                        
                        if (true == foundButtons && true == foundOrientation) {
                            LOG_INF("All required fields found, stopping parse");
                            if (true != foundButtons || true != foundOrientation) {
                                LOG_ERR("Failed to parse mouse fields: buttons=%d orientation=%d", foundButtons, foundOrientation);
                                return -1;
                            }

                            pMouse->report_len = (reportOffset + 7) / 8;
                            pMouse->has_report_id_declared = hasReportId;
                            
                            if (true == foundWheel) {
                                LOG_INF("  Wheel at byte %d", pWheel->report_buf_off);
                            }

                            return 0;
                        }
                    }
                }
                else if (HID_MAIN_ITEM_TAG_INPUT == item.tag) {
                    LOG_DBG("INPUT: offset=%d size=%d count=%d usageCount=%d inMouse=%d", 
                            reportOffset, reportSize, reportCount, usageCount, inMouseCollection);
                    
                    if (true != inMouseCollection) {
                        LOG_DBG("  Skipping INPUT (not in mouse collection)");
                        reportOffset += reportSize * reportCount;
                        usageCount = 0;
                        usage = 0;
                        break;
                    }

                    // Parse BUTTONS
                    if (HID_UP_BUTTON == usagePage && true != foundButtons) {
                        pBtn->logical_minimum = logicalMin;
                        pBtn->logical_maximum = logicalMax;
                        pBtn->size = reportSize;
                        pBtn->count = reportCount;
                        pBtn->report_buf_off = reportOffset / 8;
                        foundButtons = true;
                        LOG_INF("    -> BUTTONS: byte=%d size=%d count=%d", 
                                pBtn->report_buf_off, pBtn->size, pBtn->count);
                    }
                    // Parse X/Y/Wheel
                    else if (HID_UP_GENDESK == usagePage) {
                        bool hasX = false;
                        bool hasY = false;
                        bool hasWheel = false;
                        
                        for (uint32_t i = 0; i < usageCount; i++) {
                            if (usages[i] == HID_GD_X) hasX = true;
                            if (usages[i] == HID_GD_Y) hasY = true;
                            if (usages[i] == HID_GD_WHEEL) hasWheel = true;
                        }
                        
                        // X and Y together
                        if (true == hasX && true == hasY && true != foundOrientation) {
                            pOrient->logical_minimum = logicalMin;
                            pOrient->logical_maximum = logicalMax;
                            pOrient->size = reportSize;
                            pOrient->count = 2;
                            pOrient->report_buf_off = reportOffset / 8;
                            foundOrientation = true;
                            LOG_INF("  -> ORIENTATION: byte=%d size=%d count=%d", 
                                    pOrient->report_buf_off, pOrient->size, pOrient->count);
                            
                            // If wheel is included in same INPUT item
                            if (true == hasWheel && reportCount >= 3) {
                                pWheel->logical_minimum = logicalMin;
                                pWheel->logical_maximum = logicalMax;
                                pWheel->size = reportSize;
                                pWheel->count = 1;
                                pWheel->report_buf_off = (reportOffset + (2 * reportSize)) / 8;
                                foundWheel = true;
                                pMouse->has_wheel = true;
                                LOG_INF("  -> WHEEL: byte=%d size=%d count=%d", 
                                        pWheel->report_buf_off, pWheel->size, pWheel->count);
                            }
                        }
                        // Standalone wheel
                        else if (true == hasWheel && true != hasX && true != hasY && true != foundWheel) {
                            pWheel->logical_minimum = logicalMin;
                            pWheel->logical_maximum = logicalMax;
                            pWheel->size = reportSize;
                            pWheel->count = reportCount;
                            pWheel->report_buf_off = reportOffset / 8;
                            foundWheel = true;
                            pMouse->has_wheel = true;
                            LOG_INF("  -> WHEEL: byte=%d size=%d count=%d", 
                                    pWheel->report_buf_off, pWheel->size, pWheel->count);
                        }
                    }

                    reportOffset += reportSize * reportCount;
                    usageCount = 0;
                    // Clear local state
                    usage = 0;
                }
                break;
            }
        }
    }

    if (true != foundButtons || true != foundOrientation) {
        LOG_ERR("Failed to parse mouse fields: buttons=%d orientation=%d", 
                foundButtons, foundOrientation);
        return -1;
    }

    pMouse->report_len = (reportOffset + 7) / 8;
    pMouse->has_report_id_declared = hasReportId;
    
    if (true == foundWheel) {
        LOG_INF("  Wheel at byte %d", pWheel->report_buf_off);
    }

    return 0;
}