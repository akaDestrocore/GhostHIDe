/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_hid_parser.c
 * @brief          USB HID report descriptor parser unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for USB mouse and keyboard descriptor parsing. Tests various
 * report descriptor types.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <zephyr/ztest.h>
#include "mock_ch375_hw.h"
#include "hid_parser.h"

static struct ch375_Context_t *pCtx;
static struct USB_Device_t udev;

/**
 * @brief Sample HID Report Descriptor for a generic 3-button mouse
 */
static const uint8_t HidMouseReportDesc[] = {
    0x05, 0x01,                 // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                 // USAGE (Mouse)
    0xa1, 0x01,                 // COLLECTION (Application)
    0x09, 0x01,                 //   USAGE (Pointer)
    0xa1, 0x00,                 //   COLLECTION (Physical)
    0x05, 0x09,                 //     USAGE_PAGE (Button)
    0x19, 0x01,                 //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                 //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                 //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                 //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                 //     REPORT_COUNT (3)
    0x75, 0x01,                 //     REPORT_SIZE (1)
    0x81, 0x02,                 //     INPUT (Data,Var,Abs)
    0x95, 0x01,                 //     REPORT_COUNT (1)
    0x75, 0x05,                 //     REPORT_SIZE (5)
    0x81, 0x03,                 //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                 //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                 //     USAGE (X)
    0x09, 0x31,                 //     USAGE (Y)
    0x15, 0x81,                 //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                 //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                 //     REPORT_SIZE (8)
    0x95, 0x02,                 //     REPORT_COUNT (2)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel)
    0xc0,                       //   END_COLLECTION
    0xc0                        // END_COLLECTION
};

/**
 * @brief Keyboard descriptor
 */
static const uint8_t HidKeyboardReportDesc[] = {
    0x05, 0x01,                 // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                 // USAGE (Keyboard)
    0xA1, 0x01,                 // COLLECTION (Application)
    0x75, 0x01,                 // REPORT_SIZE (1)
    0x95, 0x08,                 // REPORT_COUNT (8)

    0x05, 0x07,                 // USAGE_PAGE (Key Codes)
    0x19, 0xE0,                 // USAGE_MINIMUM (224)
    0x29, 0xE7,                 // USAGE_MAXIMUM (231)
    0x15, 0x00,                 // LOGICAL_MINIMUM (0)
    0x25, 0x01,                 // LOGICAL_MAXIMUM (1)

    0x81, 0x02,                 // INPUT (Data,Var,Abs)

    0x95, 0x01,                 // REPORT_COUNT (1)
    0x75, 0x08,                 // REPORT_SIZE (8)
    0x81, 0x03,                 // INPUT (Cnst)

    0x95, 0x05,                 // REPORT_COUNT (5)

    0x75, 0x01,                 // REPORT_SIZE (1)
    0x05, 0x08,                 // USAGE_PAGE (LEDs)
    0x19, 0x01,                 // USAGE_MINIMUM (1)
    0x29, 0x05,                 // USAGE_MAXIMUM (5)
    0x91, 0x02,                 // OUTPUT (Data,Var,Abs)

    0x95, 0x01,                 // REPORT_COUNT (1)
    0x75, 0x03,                 // REPORT_SIZE (3)
    0x91, 0x03,                 // OUTPUT (Cnst)

    0x95, 0x06,                 // REPORT_COUNT (6)
    0x75, 0x08,                 // REPORT_SIZE (8)

    0x15, 0x00,                 // LOGICAL_MINIMUM (0)
    0x25, 0x68,                 // LOGICAL_MAXIMUM (104)
    0x05, 0x07,                 // USAGE_PAGE (Key Codes)
    0x19, 0x00,                 // USAGE_MINIMUM (0)
    0x29, 0x68,                 // USAGE_MAXIMUM (104)

    0x81, 0x00,                 // INPUT (Data, Array)
    0xc0                        // END_COLLECTION
};

/**
 * @brief Report ID descriptor
 */
static const uint8_t HidMouseWithReportID[] = {
    0x05, 0x01,                 // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                 // USAGE (Mouse)
    0xA1, 0x01,                 // COLLECTION (Application)
    0x85, 0x01,                 //   REPORT_ID (1)
    0x09, 0x01,                 //   USAGE (Pointer)
    0xA1, 0x00,                 //   COLLECTION (Physical)
    0x05, 0x09,                 //     USAGE_PAGE (Button)
    0x19, 0x01,                 //     USAGE_MINIMUM (Button 1)
    0x29, 0x05,                 //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,                 //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                 //     LOGICAL_MAXIMUM (1)
    0x35, 0x00,                 //     PHYSICAL_MINIMUM (0)
    0x45, 0x01,                 //     PHYSICAL_MAXIMUM (1)
    0x65, 0x00,                 //     UNIT (None)
    0x55, 0x00,                 //     UNIT_EXPONENT (0)
    0x75, 0x01,                 //     REPORT_SIZE (1)
    0x95, 0x05,                 //     REPORT_COUNT (5)
    0x81, 0x02,                 //     INPUT (Data,Var,Abs)
    0x95, 0x03,                 //     REPORT_COUNT (3)
    0x81, 0x03,                 //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                 //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                 //     USAGE (X)
    0x26, 0xFF, 0x7F,           //     LOGICAL_MAXIMUM (32767)
    0x45, 0x00,                 //     PHYSICAL_MAXIMUM (0)
    0x75, 0x10,                 //     REPORT_SIZE (16)
    0x95, 0x01,                 //     REPORT_COUNT (1)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel)
    0x09, 0x31,                 //     USAGE (Y)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel)
    0x09, 0x38,                 //     USAGE (Wheel)
    0x25, 0x7F,                 //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                 //     REPORT_SIZE (8)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel)
    0x05, 0x0C,                 //     USAGE_PAGE (Consumer Devices)
    0x0A, 0x38, 0x02,           //     USAGE (AC Pan)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel)
    0xC1, 0x00,                 //   END_COLLECTION
    0xC1, 0x00                  // END_COLLECTION
};

/**
 * @brief Mouse descriptor with wheel
 */
static const uint8_t MouseWheelReportDesc[] = {
    0x05, 0x01,                 // USAGE_PAGE (Generic Desktop Ctrls)
    0x09, 0x02,                 // USAGE (Mouse)
    0xA1, 0x01,                 // COLLECTION (Application)
    0x09, 0x01,                 //   USAGE (Pointer)
    0xA1, 0x00,                 //   COLLECTION (Physical)
    0x05, 0x09,                 //     USAGE_PAGE (Button)
    0x19, 0x01,                 //     USAGE_MINIMUM (0x01)
    0x29, 0x03,                 //     USAGE_MAXIMUM(0x03)
    0x15, 0x00,                 //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                 //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                 //     REPORT_SIZE (1)
    0x95, 0x03,                 //     REPORT_COUNT (3)
    0x81, 0x02,                 //     INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x05,                 //     REPORT_SIZE (5)
    0x95, 0x01,                 //     REPORT_COUNT (1)
    0x81, 0x01,                 //     INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,                 //     USAGE_PAGE (Generic Desktop Ctrls)
    0x09, 0x30,                 //     USAGE (X)
    0x09, 0x31,                 //     USAGE (Y)
    0x09, 0x38,                 //     USAGE (Wheel)
    0x15, 0x81,                 //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                 //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                 //     REPORT_SIZE (8)
    0x95, 0x03,                 //     REPORT_COUNT (3)
    0x81, 0x06,                 //     INPUT (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                       //   END_COLLECTION
    0xC0,                       // END_COLLECTION
};

static void test_setup(void *f) {
    
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
    
    memset(&udev, 0x00, sizeof(udev));
    udev.ctx = pCtx;
}

static void test_teardown(void *f) {
    
    if (NULL != pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: HID Item Parsing
 * ======================================================================== */
ZTEST(hid_parser, test_fetch_item_short_0byte) {
    // No pData
    uint8_t pData[] = {0x00};
    struct HID_Item_t item;
    
    uint8_t *pNext = HID_fetchItem(pData, pData + sizeof(pData), &item);
    
    zassert_not_null(pNext);
    zassert_equal(item.format, HID_ITEM_FORMAT_SHORT);
    zassert_equal(item.size, 0);
    zassert_equal(pNext - pData, 1);
}

ZTEST(hid_parser, test_fetch_item_short_1byte) {
    // Generic Desktop
    uint8_t pData[] = {0x05, 0x01};
    struct HID_Item_t item;
    
    uint8_t *pNext = HID_fetchItem(pData, pData + sizeof(pData), &item);
    
    zassert_not_null(pNext);
    zassert_equal(item.format, HID_ITEM_FORMAT_SHORT);
    zassert_equal(item.type, HID_ITEM_TYPE_GLOBAL);
    zassert_equal(item.tag, HID_GLOBAL_ITEM_TAG_USAGE_PAGE);
    zassert_equal(item.size, 1);
    zassert_equal(item.data.u8, 0x01);
}

ZTEST(hid_parser, test_fetch_item_short_2byte) {
    // Logical Minimum (-127)
    uint8_t pData[] = {0x16, 0x81, 0xFF};
    struct HID_Item_t item;
    
    uint8_t *pNext = HID_fetchItem(pData, pData + sizeof(pData), &item);
    
    zassert_not_null(pNext);
    zassert_equal(item.format, HID_ITEM_FORMAT_SHORT);
    zassert_equal(item.size, 2);
    zassert_equal(item.data.s16, -127);
}

ZTEST(hid_parser, test_fetch_item_short_4byte) {
    // 4 bytes
    uint8_t pData[] = {0x27, 0xFF, 0x00, 0x00, 0x00, 0x00};
    struct HID_Item_t item;
    
    uint8_t *pNext = HID_fetchItem(pData, pData + sizeof(pData), &item);
    
    zassert_not_null(pNext);
    zassert_equal(item.format, HID_ITEM_FORMAT_SHORT);
    zassert_equal(item.size, 4);
    zassert_equal(item.data.u32, 0x000000FF);
}

ZTEST(hid_parser, test_fetch_item_buffer_overflow) {
    // Incomplete item
    uint8_t pData[] = {0x05};
    struct HID_Item_t item;
    
    uint8_t *pNext = HID_fetchItem(pData, pData + sizeof(pData), &item);
    
    zassert_is_null(pNext, "Should detect buffer overflow");
}

/* ========================================================================
 * Test: Report Descriptor Type Detection
 * ======================================================================== */
ZTEST(hid_parser, test_parse_mouse_descriptor) {
    
    uint8_t detectedType = 0;
    
    int ret = HID_parseReportDescriptor(
        (uint8_t *)HidMouseReportDesc,
        sizeof(HidMouseReportDesc),
        &detectedType
    );
    
    zassert_equal(ret, 0);
    zassert_equal(detectedType, USBHID_TYPE_MOUSE);
}

ZTEST(hid_parser, test_parse_keyboard_descriptor) {
    
    uint8_t detectedType = 0;
    
    int ret = HID_parseReportDescriptor(
        (uint8_t *)HidKeyboardReportDesc,
        sizeof(HidKeyboardReportDesc),
        &detectedType
    );
    
    zassert_equal(ret, 0);
    zassert_equal(detectedType, USBHID_TYPE_KEYBOARD);
}

ZTEST(hid_parser, test_parse_mouse_with_wheel) {
    
    uint8_t detectedType = 0;
    
    int ret = HID_parseReportDescriptor(
        (uint8_t *)MouseWheelReportDesc,
        sizeof(MouseWheelReportDesc),
        &detectedType
    );
    
    zassert_equal(ret, 0);
    zassert_equal(detectedType, USBHID_TYPE_MOUSE);
}

ZTEST(hid_parser, test_parse_invalid_descriptor) {
    
    uint8_t pInvalid[] = {0xC0, 0xFF, 0xEE};
    uint8_t detectedType = 0;
    
    int ret = HID_parseReportDescriptor(pInvalid, sizeof(pInvalid), &detectedType);
    
    zassert_not_equal(ret, 0, "Should fail on invalid descriptor");
}

/* ========================================================================
 * Test: Report Buffer Management
 * ======================================================================== */
ZTEST(hid_parser, test_alloc_report_buffer) {
    
    struct USBHID_Device_t hid_dev;
    memset(&hid_dev, 0x00, sizeof(hid_dev));
    
    int ret = USBHID_allocReportBuffer(&hid_dev, 8);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_not_null(hid_dev.report_buffer);
    zassert_equal(hid_dev.report_len, 8);
    // Double buffered
    zassert_equal(hid_dev.report_buff_len, 16);  
    
    USBHID_freeReportBuffer(&hid_dev);
}

ZTEST(hid_parser, test_get_report_buffer_current) {
    
    struct USBHID_Device_t hid_dev;
    memset(&hid_dev, 0x00, sizeof(hid_dev));
    
    USBHID_allocReportBuffer(&hid_dev, 4);
    
    uint8_t *pBuffer;
    uint32_t len;
    int ret = USBHID_getReportBuffer(&hid_dev, &pBuffer, &len, false);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_not_null(pBuffer);
    zassert_equal(len, 4);
    
    USBHID_freeReportBuffer(&hid_dev);
}

ZTEST(hid_parser, test_get_report_buffer_last) {
    
    struct USBHID_Device_t hid_dev;
    memset(&hid_dev, 0x00, sizeof(hid_dev));
    
    USBHID_allocReportBuffer(&hid_dev, 4);
    
    uint8_t *pBuffer;
    uint32_t len;
    int ret = USBHID_getReportBuffer(&hid_dev, &pBuffer, &len, true);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_not_null(pBuffer);
    zassert_equal(len, 4);
    
    USBHID_freeReportBuffer(&hid_dev);
}

ZTEST(hid_parser, test_double_alloc_fails) {
    
    struct USBHID_Device_t hid_dev;
    memset(&hid_dev, 0x00, sizeof(hid_dev));
    
    USBHID_allocReportBuffer(&hid_dev, 4);
    int ret = USBHID_allocReportBuffer(&hid_dev, 4);
    
    zassert_equal(ret, USBHID_ERROR, "Should fail on double allocation");
    
    USBHID_freeReportBuffer(&hid_dev);
}

/* ========================================================================
 * Test: Report ID Detection
 * ======================================================================== */
ZTEST(hid_parser, test_report_id_detection) {
    
    uint8_t detectedType = 0;
    
    // Parse descriptor with Report ID
    int ret = HID_parseReportDescriptor(
        (uint8_t *)HidMouseWithReportID,
        sizeof(HidMouseWithReportID),
        &detectedType
    );
    
    zassert_equal(ret, 0);
    zassert_equal(detectedType, USBHID_TYPE_MOUSE);
    
    // Verify Report ID item is present in descriptor
    struct HID_Item_t item;
    uint8_t *pCur = (uint8_t *)HidMouseWithReportID;
    uint8_t *pEnd = pCur + sizeof(HidMouseWithReportID);
    bool isReportIDFound = false;
    
    while (pCur < pEnd) {
        pCur = HID_fetchItem(pCur, pEnd, &item);
        if (NULL == pCur) {
            break;
        }
        
        if (HID_ITEM_TYPE_GLOBAL == item.type && HID_GLOBAL_ITEM_TAG_REPORT_ID == item.tag) {
            isReportIDFound = true;
            zassert_equal(item.data.u8, 1, "Report ID should be 1");
            break;
        }
    }
    
    zassert_true(isReportIDFound, "Should detect Report ID tag");
}

/* ========================================================================
 * Test: Collection Depth Tracking
 * ======================================================================== */
ZTEST(hid_parser, test_collection_nesting) {
    
    struct HID_Item_t item;
    uint8_t *pCur = (uint8_t *)HidMouseReportDesc;
    uint8_t *pEnd = pCur + sizeof(HidMouseReportDesc);
    int collectionDepth = 0;
    int maxDepth = 0;
    
    while (pCur < pEnd) {
        pCur = HID_fetchItem(pCur, pEnd, &item);
        if (NULL == pCur) {
            break;
        }
        
        if (HID_ITEM_TYPE_MAIN == item.type) {
            if (HID_MAIN_ITEM_TAG_BEGIN_COLLECTION == item.tag) {
                collectionDepth++;
                if (collectionDepth > maxDepth) {
                    maxDepth = collectionDepth;
                }
            } else if (HID_MAIN_ITEM_TAG_END_COLLECTION == item.tag) {
                collectionDepth--;
            }
        }
    }
    
    zassert_equal(collectionDepth, 0, "Collections should be balanced");
    zassert_equal(maxDepth, 2, "Mouse descriptor has 2 nested collections");
}

/* ========================================================================
 * Test: Edge Cases
 * ======================================================================== */
ZTEST(hid_parser, test_null_parameters) {
    
    struct HID_Item_t item;
    uint8_t pData[] = {0x05, 0x01};
    
    // Null start pointer
    zassert_is_null(HID_fetchItem(NULL, pData + 2, &item));
    
    // Null end pointer
    zassert_is_null(HID_fetchItem(pData, NULL, &item));
    
    // Null item pointer
    zassert_is_null(HID_fetchItem(pData, pData + 2, NULL));
}

ZTEST(hid_parser, test_empty_buffer) {
    
    struct HID_Item_t item;
    uint8_t pData[] = {0x00};
    
    uint8_t *pResult = HID_fetchItem(pData, pData, &item);
    
    zassert_is_null(pResult, "Should handle empty buffer");
}

ZTEST(hid_parser, test_report_buffer_not_allocated) {
    
    struct USBHID_Device_t hid_dev;
    memset(&hid_dev, 0x00, sizeof(hid_dev));
    
    uint8_t *pBuffer;
    uint32_t len;
    int ret = USBHID_getReportBuffer(&hid_dev, &pBuffer, &len, false);
    
    zassert_equal(ret, USBHID_BUFFER_NOT_ALLOC);
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(hid_parser, NULL, NULL, test_setup, test_teardown, NULL);