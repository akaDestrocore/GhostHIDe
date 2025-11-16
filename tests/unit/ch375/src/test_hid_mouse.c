/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_hid_mouse.c
 * @brief          USB HID mouse device unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for USB mouse-specific HID report parsing. Tests various
 * mice descriptors.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <zephyr/ztest.h>
#include "usb_stubs.h"
#include "hid_parser.h"
#include "hid_mouse.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *pCtx;
static struct USB_Device_t gUdev;
static struct USBHID_Device_t gHidDev;

/**
 * @brief ZOWIE FK2 - 6 buttons, 16-bit X/Y, wheel
 */
static const uint8_t ZOWIE_FK2[69] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x06,        //     Usage Maximum (0x06)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x06,        //     Report Count (6)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x02,        //     Report Count (2)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x01,        //     Input (Const,Array,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xA1, 0x00,        //   Collection (Physical)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x08,        //     Report Size (8)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection
};

/**
 * @brief Razer Viper Ultimate - 5 buttons, 16-bit X/Y, wheel
 */
static const uint8_t RAZER_VIPER_ULTIMATE_1[94] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x05,        //     Report Count (5)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x03,        //     Input (Const,Var,Abs)
    0x06, 0x00, 0xFF,  //     Usage Page (Vendor Defined 0xFF00)
    0x09, 0x40,        //     Usage (0x40)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x00, 0x80,  //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x02,        //   Usage (0x02)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x5A,        //   Report Count (90)
    0xB1, 0x01,        //   Feature (Const,Array,Abs)
    0xC0,              // End Collection
};

/**
 * @brief Raspberry Pi Mouse - 3 buttons, 8-bit X/Y/Wheel
 */
static const uint8_t RASPBERRY_PI[52] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x75, 0x05,        //     Report Size (5)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x01,        //     Input (Const,Array,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection
};

/**
 * @brief Logitech G305 - 16 buttons, 16-bit X/Y, wheel, pan
 */
static const uint8_t LOGITECH_G305_2[148] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x10,        //     Usage Maximum (0x10)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x10,        //     Report Count (16)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x09, 0x38,        //     Usage (Wheel)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x02,        //   Report Count (2)
    0x15, 0x01,        //   Logical Minimum (1)
    0x26, 0xFF, 0x02,  //   Logical Maximum (767)
    0x19, 0x01,        //   Usage Minimum (Consumer Control)
    0x2A, 0xFF, 0x02,  //   Usage Maximum (0x02FF)
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0xC0,              // End Collection
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x80,        // Usage (Sys Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x04,        //   Report ID (4)
    0x75, 0x02,        //   Report Size (2)
    0x95, 0x01,        //   Report Count (1)
    0x15, 0x01,        //   Logical Minimum (1)
    0x25, 0x03,        //   Logical Maximum (3)
    0x09, 0x82,        //   Usage (Sys Sleep)
    0x09, 0x81,        //   Usage (Sys Power Down)
    0x09, 0x83,        //   Usage (Sys Wake Up)
    0x81, 0x60,        //   Input (Data,Array,Abs)
    0x75, 0x06,        //   Report Size (6)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0xC0,              // End Collection
    0x06, 0xBC, 0xFF,  // Usage Page (Vendor Defined 0xFFBC)
    0x09, 0x88,        // Usage (0x88)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x08,        //   Report ID (8)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0xFF,        //   Usage Maximum (0xFF)
    0x15, 0x01,        //   Logical Minimum (1)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0xC0,              // End Collection
};

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
    
    memset(&gUdev, 0x00, sizeof(gUdev));
    gUdev.ctx = pCtx;
    
    memset(&gHidDev, 0x00, sizeof(gHidDev));
}

static void test_teardown(void *f)
{
    if (NULL != gHidDev.report_buffer) {
        USBHID_freeReportBuffer(&gHidDev);
    }
    
    if (NULL != pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: Basic Mouse Opening - ZOWIE FK2
 * ======================================================================== */
ZTEST(hid_mouse, test_open_zowie_fk2)
{
    struct HID_Mouse_t mouse;
    uint32_t btnValue = 0;
    int32_t axisValue = 0;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test NULL mouse pointer
    ret = hidMouse_GetButton(NULL, HID_MOUSE_BUTTON_LEFT, &btnValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetButton(NULL, HID_MOUSE_BUTTON_LEFT, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_GetOrientation(NULL, HID_MOUSE_AXIS_X, &axisValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetOrientation(NULL, HID_MOUSE_AXIS_X, 10, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test NULL value pointer
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test invalid button number
    ret = hidMouse_GetButton(&mouse, 10, &btnValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetButton(&mouse, 10, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test invalid axis number
    ret = hidMouse_GetOrientation(&mouse, 5, &axisValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetOrientation(&mouse, 5, 10, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Close NULL Mouse
 * ======================================================================== */
ZTEST(hid_mouse, test_close_null_mouse)
{
    // Should not crash
    hidMouse_Close(NULL);
}

/* ========================================================================
 * Test: Report Buffer Management
 * ======================================================================== */
ZTEST(hid_mouse, test_report_buffer_allocation)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Verify report buffer was allocated
    zassert_not_null(gHidDev.report_buffer, "Report buffer should be allocated");
    zassert_true(gHidDev.report_len > 0, "Report length should be positive");
    
    // Verify buffer is double-buffered
    zassert_equal(gHidDev.report_buff_len, gHidDev.report_len * 2);
    
    hidMouse_Close(&mouse);
    
    // Verify buffer was freed
    zassert_is_null(gHidDev.report_buffer, "Report buffer should be freed");
}

/* ========================================================================
 * Test: Button Bit Positions
 * ======================================================================== */
ZTEST(hid_mouse, test_button_bit_positions)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set each button individually and verify only that button is set
    for (uint32_t btn = 0; btn < mouse.button.count; btn++) {
        // Clear all buttons
        uint8_t *pBuff;
        uint32_t len;
        USBHID_getReportBuffer(&gHidDev, &pBuff, &len, false);
        memset(pBuff, 0, len);
        
        // Set specific button
        hidMouse_SetButton(&mouse, btn, 1, false);
        
        // Verify only this button is set
        for (uint32_t checkBtn = 0; checkBtn < mouse.button.count; checkBtn++) {
            uint32_t value;
            hidMouse_GetButton(&mouse, checkBtn, &value, false);
            
            if (checkBtn == btn) {
                zassert_equal(value, 1, "Button %d should be set", btn);
            } else {
                zassert_equal(value, 0, "Button %d should not be set", checkBtn);
            }
        }
    }
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Axis Byte Order (LE)
 * ======================================================================== */
ZTEST(hid_mouse, test_axis_byte_order)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set a value that will show byte order issues if wrong
    // 0x1234 = 4660
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 0x1234, false);
    
    // Get raw buffer and check byte order (should be LE)
    uint8_t *pBuff;
    uint32_t len;
    USBHID_getReportBuffer(&gHidDev, &pBuff, &len, false);
    
    uint8_t *pAxisData = pBuff + mouse.orientation.report_buf_off;
    
    // Little endian: LSB first
    zassert_equal(pAxisData[0], 0x34, "LSB should be first");
    zassert_equal(pAxisData[1], 0x12, "MSB should be second");
    
    // Verify we can read it back correctly
    int32_t value;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, 0x1234);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Negative Values in Signed Fields
 * ======================================================================== */
ZTEST(hid_mouse, test_negative_values)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test negative values
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, -50, false);
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_Y, -127, false);
    
    int32_t value;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, -50);
    
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &value, false);
    zassert_equal(value, -127);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Boundary Values
 * ======================================================================== */
ZTEST(hid_mouse, test_boundary_values_8bit)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test min value
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, -127, false);
    int32_t value;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, -127);
    
    // Test max value
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 127, false);
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, 127);
    
    // Test zero
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 0, false);
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, 0);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Boundary Values 16-bit
 * ======================================================================== */
ZTEST(hid_mouse, test_boundary_values_16bit)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test min value
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, -32767, false);
    int32_t value;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, -32767);
    
    // Test max value
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 32767, false);
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, 32767);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Report Offset Calculations
 * ======================================================================== */
ZTEST(hid_mouse, test_report_offset_calculations)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Byte 0: 3 buttons (bits 0-2) + 5 bits padding
    // Bytes 1-3: X, Y, Wheel (8-bit each)
    
    zassert_equal(mouse.button.report_buf_off, 0, "Buttons should start at byte 0");
    zassert_equal(mouse.orientation.report_buf_off, 1, "Orientation should start at byte 1");
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Report Offset with 16-bit Axes
 * ======================================================================== */
ZTEST(hid_mouse, test_report_offset_16bit_axes)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Byte 0: 6 buttons + 2 bits padding
    // Bytes 1-4: X, Y (16-bit each)
    // Byte 5: Wheel
    
    zassert_equal(mouse.button.report_buf_off, 0);
    zassert_equal(mouse.orientation.report_buf_off, 1);
    
    if (mouse.has_wheel) {
        zassert_equal(mouse.wheel.report_buf_off, 5);
    }
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Zero Report Length
 * ======================================================================== */
ZTEST(hid_mouse, test_zero_report_length)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    zassert_true(mouse.report_len > 0, "Report length should be positive");
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: All Buttons Off by Default
 * ======================================================================== */
ZTEST(hid_mouse, test_default_button_state)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // All buttons should be 0 initially
    for (uint32_t btn = 0; btn < mouse.button.count; btn++) {
        uint32_t value;
        hidMouse_GetButton(&mouse, btn, &value, false);
        zassert_equal(value, 0, "Button %d should be 0 by default", btn);
    }
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Default Axis Values
 * ======================================================================== */
ZTEST(hid_mouse, test_default_axis_values)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Axes should be 0 initially
    int32_t value;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(value, 0);
    
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &value, false);
    zassert_equal(value, 0);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Multiple Open/Close Cycles
 * ======================================================================== */
ZTEST(hid_mouse, test_multiple_open_close)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    for (int i = 0; i < 3; i++) {
        int ret = hidMouse_Open(&gHidDev, &mouse);
        zassert_equal(ret, USBHID_SUCCESS);
        
        // Use the mouse
        hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 1, false);
        uint32_t value;
        hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &value, false);
        zassert_equal(value, 1);
        
        hidMouse_Close(&mouse);
        
        // Verify buffer was freed
        zassert_is_null(gHidDev.report_buffer);
    }
}

/* ========================================================================
 * Test: Basic Mouse Opening - Razer Viper
 * ======================================================================== */
ZTEST(hid_mouse, test_open_razer_viper)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RAZER_VIPER_ULTIMATE_1;
    gHidDev.raw_hid_report_desc_len = sizeof(RAZER_VIPER_ULTIMATE_1);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(mouse.button.count, 5, "Should have 5 buttons");
    zassert_equal(mouse.orientation.count, 2, "Should have 2 axes");
    zassert_equal(mouse.orientation.size, 16, "Should be 16-bit axes");
    zassert_true(mouse.has_wheel, "Should have wheel");
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Simple Mouse - Raspberry Pi
 * ======================================================================== */
ZTEST(hid_mouse, test_open_raspberry_pi)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(mouse.button.count, 3, "Should have 3 buttons");
    zassert_equal(mouse.orientation.count, 2, "Should have 2 axes (X, Y)");
    zassert_equal(mouse.orientation.size, 8, "Should be 8-bit axes");
    zassert_true(mouse.has_wheel, "Should have wheel");
    zassert_equal(mouse.wheel.size, 8, "Wheel should be 8-bit");
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Mouse with Report ID - Logitech G305
 * ======================================================================== */
ZTEST(hid_mouse, test_open_logitech_g305_with_report_id)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)LOGITECH_G305_2;
    gHidDev.raw_hid_report_desc_len = sizeof(LOGITECH_G305_2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(mouse.button.count, 16, "Should have 16 buttons");
    zassert_equal(mouse.orientation.count, 2, "Should have 2 axes");
    zassert_equal(mouse.orientation.size, 16, "Should be 16-bit axes");
    zassert_true(mouse.has_report_id_declared, "Should have Report ID declared");
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Button Operations
 * ======================================================================== */
ZTEST(hid_mouse, test_button_get_set)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set left button
    ret = hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Get left button
    uint32_t value = 0;
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1);
    
    // Set right button
    ret = hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1);
    
    // Clear left button
    ret = hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 0, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 0);
    
    // Right button should still be set
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Multiple Buttons
 * ======================================================================== */
ZTEST(hid_mouse, test_multiple_buttons)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set multiple buttons
    hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 1, false);
    hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_MIDDLE, 1, false);
    // Button 6
    hidMouse_SetButton(&mouse, 5, 1, false);
    
    // Verify all buttons
    uint32_t value;
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &value, false);
    zassert_equal(value, 1);
    
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, &value, false);
    zassert_equal(value, 0);
    
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_MIDDLE, &value, false);
    zassert_equal(value, 1);
    
    hidMouse_GetButton(&mouse, 5, &value, false);
    zassert_equal(value, 1);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Orientation Operations - 8-bit
 * ======================================================================== */
ZTEST(hid_mouse, test_orientation_8bit)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set X axis
    ret = hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 10, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set Y axis
    ret = hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_Y, -15, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Get X axis
    int32_t value;
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 10);
    
    // Get Y axis
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, -15);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Orientation Operations - 16-bit
 * ======================================================================== */
ZTEST(hid_mouse, test_orientation_16bit)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)ZOWIE_FK2;
    gHidDev.raw_hid_report_desc_len = sizeof(ZOWIE_FK2);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test large 16-bit values
    ret = hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 1000, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_Y, -2000, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    int32_t value;
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1000);
    
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, -2000);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Combined Button and Orientation
 * ======================================================================== */
ZTEST(hid_mouse, test_combined_button_orientation)
{
    struct HID_Mouse_t mouse;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    int ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set buttons
    hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 1, false);
    hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, 1, false);
    
    // Set orientation
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 5, false);
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_Y, -3, false);
    
    // Verify buttons
    uint32_t btnValue;
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &btnValue, false);
    zassert_equal(btnValue, 1);
    
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, &btnValue, false);
    zassert_equal(btnValue, 1);
    
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_MIDDLE, &btnValue, false);
    zassert_equal(btnValue, 0);
    
    // Verify orientation
    int32_t axisValue;
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &axisValue, false);
    zassert_equal(axisValue, 5);
    
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &axisValue, false);
    zassert_equal(axisValue, -3);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Invalid Parameters
 * ======================================================================== */
ZTEST(hid_mouse, test_invalid_parameters)
{
    struct HID_Mouse_t mouse;
    uint32_t btnValue = 0;
    int32_t axisValue = 0;
    int ret;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_MOUSE;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    ret = hidMouse_Open(&gHidDev, &mouse);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test NULL mouse pointer
    ret = hidMouse_GetButton(NULL, HID_MOUSE_BUTTON_LEFT, &btnValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetButton(NULL, HID_MOUSE_BUTTON_LEFT, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_GetOrientation(NULL, HID_MOUSE_AXIS_X, &axisValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetOrientation(NULL, HID_MOUSE_AXIS_X, 10, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test NULL value pointer
    ret = hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test invalid button number
    ret = hidMouse_GetButton(&mouse, 10, &btnValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetButton(&mouse, 10, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test invalid axis number
    ret = hidMouse_GetOrientation(&mouse, 5, &axisValue, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidMouse_SetOrientation(&mouse, 5, 10, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    hidMouse_Close(&mouse);
}

/* ========================================================================
 * Test: Open Non-Mouse Device
 * ======================================================================== */
ZTEST(hid_mouse, test_open_non_mouse_device)
{
    struct HID_Mouse_t mouse;
    int ret;
    
    gHidDev.pUdev = &gUdev;
    gHidDev.hid_type = USBHID_TYPE_KEYBOARD;
    gHidDev.raw_hid_report_desc = (uint8_t *)RASPBERRY_PI;
    gHidDev.raw_hid_report_desc_len = sizeof(RASPBERRY_PI);
    
    ret = hidMouse_Open(&gHidDev, &mouse);
    
    zassert_equal(ret, USBHID_NOT_SUPPORT, "Should reject non-mouse device");
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(hid_mouse, NULL, NULL, test_setup, test_teardown, NULL);