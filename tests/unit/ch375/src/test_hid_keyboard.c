/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_hid_keyboard.c
 * @brief          USB HID keyboard device unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for USB keyboard-specific HID report parsing. Tests standard
 * boot protocol keyboard functionality including modifier keys, key arrays,
 * and report buffer management.
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
#include "hid_keyboard.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *pCtx;
static struct USB_Device_t gUdev;
static struct USBHID_Device_t hidDev;

/**
 * @brief Standard Boot Protocol Keyboard Descriptor
 */
const uint8_t COOLERMASTER_MASTERKEYS_S_1[64] = {
		0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
		0x09, 0x06,        // Usage (Keyboard)
		0xA1, 0x01,        // Collection (Application)
		0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
		0x19, 0xE0,        //   Usage Minimum (0xE0)
		0x29, 0xE7,        //   Usage Maximum (0xE7)
		0x15, 0x00,        //   Logical Minimum (0)
		0x25, 0x01,        //   Logical Maximum (1)
		0x75, 0x01,        //   Report Size (1)
		0x95, 0x08,        //   Report Count (8)
		0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x95, 0x01,        //   Report Count (1)
		0x75, 0x08,        //   Report Size (8)
		0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x95, 0x03,        //   Report Count (3)
		0x75, 0x01,        //   Report Size (1)
		0x05, 0x08,        //   Usage Page (LEDs)
		0x19, 0x01,        //   Usage Minimum (Num Lock)
		0x29, 0x03,        //   Usage Maximum (Scroll Lock)
		0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		0x95, 0x01,        //   Report Count (1)
		0x75, 0x05,        //   Report Size (5)
		0x91, 0x03,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
		0x95, 0x06,        //   Report Count (6)
		0x75, 0x08,        //   Report Size (8)
		0x15, 0x00,        //   Logical Minimum (0)
		0x26, 0xA4, 0x00,  //   Logical Maximum (164)
		0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
		0x19, 0x00,        //   Usage Minimum (0x00)
		0x29, 0xA4,        //   Usage Maximum (0xA4)
		0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,              // End Collection
	};

/**
 * @brief Gaming Keyboard with extra keys
 */
const uint8_t RAZER_BLACKWIDOW_V4_KEYBOARD[177] = {
		0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
		0x09, 0x06,        // Usage (Keyboard)
		0xA1, 0x01,        // Collection (Application)
		0x85, 0x01,        //   Report ID (1)
		0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
		0x19, 0xE0,        //   Usage Minimum (0xE0)
		0x29, 0xE7,        //   Usage Maximum (0xE7)
		0x15, 0x00,        //   Logical Minimum (0)
		0x25, 0x01,        //   Logical Maximum (1)
		0x75, 0x01,        //   Report Size (1)
		0x95, 0x08,        //   Report Count (8)
		0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x19, 0x00,        //   Usage Minimum (0x00)
		0x2A, 0xFF, 0x00,  //   Usage Maximum (0xFF)
		0x15, 0x00,        //   Logical Minimum (0)
		0x26, 0xFF, 0x00,  //   Logical Maximum (255)
		0x75, 0x08,        //   Report Size (8)
		0x95, 0x0E,        //   Report Count (14)
		0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,              // End Collection
		0x05, 0x0C,        // Usage Page (Consumer)
		0x09, 0x01,        // Usage (Consumer Control)
		0xA1, 0x01,        // Collection (Application)
		0x85, 0x02,        //   Report ID (2)
		0x19, 0x00,        //   Usage Minimum (0x00)
		0x2A, 0x3C, 0x02,  //   Usage Maximum (0x023C)
		0x15, 0x00,        //   Logical Minimum (0)
		0x26, 0x3C, 0x02,  //   Logical Maximum (572)
		0x95, 0x01,        //   Report Count (1)
		0x75, 0x10,        //   Report Size (16)
		0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x75, 0x08,        //   Report Size (8)
		0x95, 0x0D,        //   Report Count (13)
		0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,              //   End Collection
		0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
		0x09, 0x80,        //   Usage (Sys Control)
		0xA1, 0x01,        //   Collection (Application)
		0x85, 0x03,        //       Report ID (3)
		0x19, 0x81,        //       Usage Minimum (Sys Power Down)
		0x29, 0x83,        //       Usage Maximum (Sys Wake Up)
		0x15, 0x00,        //       Logical Minimum (0)
		0x25, 0x01,        //       Logical Maximum (1)
		0x75, 0x01,        //       Report Size (1)
		0x95, 0x03,        //       Report Count (3)
		0x81, 0x02,        //       Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x95, 0x05,        //       Report Count (5)
		0x81, 0x01,        //       Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0x75, 0x08,        //       Report Size (8)
		0x95, 0x0E,        //       Report Count (14)
		0x81, 0x01,        //       Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,              //   End Collection
		0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
		0x09, 0x00,        // Usage (Undefined)
		0xA1, 0x01,        // Collection (Application)
		0x85, 0x04,        //   Report ID (4)
		0x09, 0x03,        //   Usage (Undefined)
		0x15, 0x00,        //   Logical Minimum (0)
		0x26, 0xFF, 0x00,  //   Logical Maximum (255)
		0x35, 0x00,        //   Physical Minimum (0)
		0x46, 0xFF, 0x00,  //   Physical Maximum (255)
		0x75, 0x08,        //   Report Size (8)
		0x95, 0x0F,        //   Report Count (15)
		0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0,              // End Collection
		0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
		0x09, 0x00,        // Usage (Undefined)
		0xA1, 0x01,        // Collection (Application)
		0x85, 0x05,        //   Report ID (5)
		0x09, 0x03,        //   Usage (Undefined)
		0x15, 0x00,        //   Logical Minimum (0)
		0x26, 0xFF, 0x00,  //   Logical Maximum (255)
		0x35, 0x00,        //   Physical Minimum (0)
		0x46, 0xFF, 0x00,  //   Physical Maximum (255)
		0x75, 0x08,        //   Report Size (8)
		0x95, 0x0F,        //   Report Count (15)
		0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
		0xC0               // End Collection
	};

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
    
    memset(&gUdev, 0x00, sizeof(gUdev));
    gUdev.ctx = pCtx;
    
    memset(&hidDev, 0x00, sizeof(hidDev));
}

static void test_teardown(void *f)
{
    if (NULL != hidDev.report_buffer) {
        USBHID_freeReportBuffer(&hidDev);
    }
    
    if (NULL != pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: Basic Keyboard Opening
 * ======================================================================== */
ZTEST(hid_keyboard, test_open_standard_keyboard)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(kbd.report_length, HID_KBD_REPORT_SIZE, "Report should be 8 bytes");
    zassert_equal(kbd.modifier.report_buf_off, 0, "Modifier offset should be 0");    
    zassert_equal(kbd.keys.report_buf_off, HID_KBD_KEYS_OFFSET, "Keys offset should be %d but got %d", 
                                        HID_KBD_KEYS_OFFSET, kbd.keys.report_buf_off);
    zassert_equal(kbd.keys.count, HID_KBD_MAX_KEYS, "Should support 6 keys");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Open Non-keyboard Device
 * ======================================================================== */
ZTEST(hid_keyboard, test_open_non_keyboard_device)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_MOUSE;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    
    zassert_equal(ret, USBHID_NOT_SUPPORT, "Should reject non-keyboard device");
}

/* ========================================================================
 * Test: NULL Parameter Validation
 * ======================================================================== */
ZTEST(hid_keyboard, test_null_parameters)
{
    struct HID_Keyboard_t kbd;
    uint32_t value = 0;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test NULL keyboard pointer
    ret = hidKeyboard_GetKey(NULL, HID_KBD_LETTER('a'), &value, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidKeyboard_SetKey(NULL, HID_KBD_LETTER('a'), 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidKeyboard_GetModifier(NULL, 0, &value, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidKeyboard_SetModifier(NULL, 0, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    // Test NULL value pointer
    ret = hidKeyboard_GetKey(&kbd, HID_KBD_LETTER('a'), NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidKeyboard_GetModifier(&kbd, 0, NULL, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Close NULL Keyboard
 * ======================================================================== */
ZTEST(hid_keyboard, test_close_null_keyboard)
{
    // Should not crash
    hidKeyboard_Close(NULL);
}

/* ========================================================================
 * Test: Report Buffer Allocation
 * ======================================================================== */
ZTEST(hid_keyboard, test_report_buffer_allocation)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Verify report buffer was allocated
    zassert_not_null(hidDev.report_buffer, "Report buffer should be allocated");
    zassert_equal(hidDev.report_len, HID_KBD_REPORT_SIZE);
    
    // Verify buffer is double buffered
    zassert_equal(hidDev.report_buff_len, hidDev.report_len * 2);
    
    hidKeyboard_Close(&kbd);
    
    // Verify buffer was freed
    zassert_is_null(hidDev.report_buffer, "Report buffer should be freed");
}

/* ========================================================================
 * Test: Modifier Key Operations
 * ======================================================================== */
ZTEST(hid_keyboard, test_modifier_keys)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set left ctrl - bit 0
    ret = hidKeyboard_SetModifier(&kbd, 0, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint32_t value;
    ret = hidKeyboard_GetModifier(&kbd, 0, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Left Control should be set");
    
    // Set left shift - bit 1
    ret = hidKeyboard_SetModifier(&kbd, 1, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidKeyboard_GetModifier(&kbd, 1, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Left Shift should be set");
    
    // Verify left ctrl still set
    ret = hidKeyboard_GetModifier(&kbd, 0, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Left Control should still be set");
    
    // Clear left ctrl
    ret = hidKeyboard_SetModifier(&kbd, 0, 0, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidKeyboard_GetModifier(&kbd, 0, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 0, "Left Control should be cleared");
    
    // Verify left shift still set
    ret = hidKeyboard_GetModifier(&kbd, 1, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Left Shift should still be set");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: All Modifier Keys
 * ======================================================================== */
ZTEST(hid_keyboard, test_all_modifier_keys)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set all 8 mod bits
    for (uint32_t i = 0; i < 8; i++) {
        ret = hidKeyboard_SetModifier(&kbd, i, 1, false);
        zassert_equal(ret, USBHID_SUCCESS);
    }
    
    // Verify all are set
    for (uint32_t i = 0; i < 8; i++) {
        uint32_t value;
        ret = hidKeyboard_GetModifier(&kbd, i, &value, false);
        zassert_equal(ret, USBHID_SUCCESS);
        zassert_equal(value, 1, "Modifier bit %d should be set", i);
    }
    
    // Check raw buffer value should be 0xFF
    uint8_t *pBuff;
    uint32_t len;
    USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(pBuff[HID_KBD_MODIFIER_OFFSET], 0xFF, 
                  "All modifier bits should be set");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Invalid Modifier Number
 * ======================================================================== */
ZTEST(hid_keyboard, test_invalid_modifier_number)
{
    struct HID_Keyboard_t kbd;
    uint32_t value;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Test mod bigger than 7
    ret = hidKeyboard_GetModifier(&kbd, 8, &value, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    ret = hidKeyboard_SetModifier(&kbd, 8, 1, false);
    zassert_equal(ret, USBHID_PARAM_INVALID);
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Key Code Operations
 * ======================================================================== */
ZTEST(hid_keyboard, test_key_operations)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set key 'A'
    uint8_t keyA = HID_KBD_LETTER('A');
    ret = hidKeyboard_SetKey(&kbd, keyA, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint32_t value;
    ret = hidKeyboard_GetKey(&kbd, keyA, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Key 'A' should be pressed");
    
    // Set key 'B'
    uint8_t keyB = HID_KBD_LETTER('B');
    ret = hidKeyboard_SetKey(&kbd, keyB, 1, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidKeyboard_GetKey(&kbd, keyB, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Key 'B' should be pressed");
    
    // Verify 'A' still pressed
    ret = hidKeyboard_GetKey(&kbd, keyA, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Key 'A' should still be pressed");
    
    // Release key 'A'
    ret = hidKeyboard_SetKey(&kbd, keyA, 0, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    ret = hidKeyboard_GetKey(&kbd, keyA, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 0, "Key 'A' should be released");
    
    // Verify 'B' still pressed
    ret = hidKeyboard_GetKey(&kbd, keyB, &value, false);
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(value, 1, "Key 'B' should still be pressed");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Maximum Keys (6-Key Rollover)
 * ======================================================================== */
ZTEST(hid_keyboard, test_six_key_rollover)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Press 6 keys at a time
    uint8_t keys[6] = {
        HID_KBD_LETTER('a'),
        HID_KBD_LETTER('b'),
        HID_KBD_LETTER('c'),
        HID_KBD_LETTER('d'),
        HID_KBD_LETTER('e'),
        HID_KBD_LETTER('f')
    };
    
    for (int i = 0; i < 6; i++) {
        ret = hidKeyboard_SetKey(&kbd, keys[i], 1, false);
        zassert_equal(ret, USBHID_SUCCESS);
    }
    
    // Verify all 6 keys are pressed
    for (int i = 0; i < 6; i++) {
        uint32_t value;
        ret = hidKeyboard_GetKey(&kbd, keys[i], &value, false);
        zassert_equal(ret, USBHID_SUCCESS);
        zassert_equal(value, 1, "Key %d should be pressed", i);
    }
    
    // Verify raw buffer
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    for (int i = 0; i < 6; i++) {
        zassert_equal(pBuff[kbd.keys.report_buf_off + i], keys[i],
                      "Key slot %d should contain correct key code", i);
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Key array shifting on release
 * ======================================================================== */
ZTEST(hid_keyboard, test_key_array_shifting)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint8_t keyA = HID_KBD_LETTER('a');
    uint8_t keyB = HID_KBD_LETTER('b');
    uint8_t keyC = HID_KBD_LETTER('c');
    
    // Press A, B, C in order
    hidKeyboard_SetKey(&kbd, keyA, 1, false);
    hidKeyboard_SetKey(&kbd, keyB, 1, false);
    hidKeyboard_SetKey(&kbd, keyC, 1, false);
    
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Verify order is the same
    zassert_equal(pBuff[kbd.keys.report_buf_off + 0], keyA);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 1], keyB);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 2], keyC);
    
    // Release B
    hidKeyboard_SetKey(&kbd, keyB, 0, false);
    
    // Verify if array actually shifted
    zassert_equal(pBuff[kbd.keys.report_buf_off + 0], keyA);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 1], keyC);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 2], 0);
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Duplicate Key Prevention
 * ======================================================================== */
ZTEST(hid_keyboard, test_duplicate_key_prevention)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint8_t keyA = HID_KBD_LETTER('a');
    
    // Press 'A' multiple times
    hidKeyboard_SetKey(&kbd, keyA, 1, false);
    hidKeyboard_SetKey(&kbd, keyA, 1, false);
    hidKeyboard_SetKey(&kbd, keyA, 1, false);
    
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Count occurrences of keyA
    int count = 0;
    for (int i = 0; i < HID_KBD_MAX_KEYS; i++) {
        if (pBuff[kbd.keys.report_buf_off + i] == keyA) {
            count++;
        }
    }
    
    zassert_equal(count, 1, "Key 'A' should appear only once");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Combined modifiers and keys
 * ======================================================================== */
ZTEST(hid_keyboard, test_combined_modifiers_and_keys)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Simulate L ctrl + L shift + A
    hidKeyboard_SetModifier(&kbd, 0, 1, false);
    hidKeyboard_SetModifier(&kbd, 1, 1, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('A'), 1, false);
    
    // Verify mods
    uint32_t value;
    hidKeyboard_GetModifier(&kbd, 0, &value, false);
    zassert_equal(value, 1, "Left Control should be set");
    
    hidKeyboard_GetModifier(&kbd, 1, &value, false);
    zassert_equal(value, 1, "Left Shift should be set");
    
    // Verify key
    hidKeyboard_GetKey(&kbd, HID_KBD_LETTER('A'), &value, false);
    zassert_equal(value, 1, "Key 'A' should be pressed");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Letter Key Macros
 * ======================================================================== */
ZTEST(hid_keyboard, test_letter_key_macros)
{
    // Test lowercase
    zassert_equal(HID_KBD_LETTER('a'), 4);
    zassert_equal(HID_KBD_LETTER('z'), 29);
    
    // Test uppercase
    zassert_equal(HID_KBD_LETTER('A'), 4);
    zassert_equal(HID_KBD_LETTER('Z'), 29);
    
    // Test weird
    zassert_equal(HID_KBD_LETTER('0'), 0);
    zassert_equal(HID_KBD_LETTER('!'), 0);
}

/* ========================================================================
 * Test: Number Key Macros
 * ======================================================================== */
ZTEST(hid_keyboard, test_number_key_macros)
{
    // Test numbers 1-9
    zassert_equal(HID_KBD_NUMBER('1'), 30);
    zassert_equal(HID_KBD_NUMBER('9'), 38);
    
    // Test zero
    zassert_equal(HID_KBD_NUMBER('0'), 39);
    
    // Test invalid
    zassert_equal(HID_KBD_NUMBER('a'), 0);
}

/* ========================================================================
 * Test: Default Key State
 * ======================================================================== */
ZTEST(hid_keyboard, test_default_key_state)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // All keys should be 0 initially
    uint32_t value;
    for (uint8_t i = 1; i < 104; i++) {
        ret = hidKeyboard_GetKey(&kbd, i, &value, false);
        zassert_equal(ret, USBHID_SUCCESS);
        zassert_equal(value, 0, "Key %d should be 0 by default", i);
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Default Modifier State
 * ======================================================================== */
ZTEST(hid_keyboard, test_default_modifier_state)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // All modifiers should be 0 initially
    for (uint32_t i = 0; i < 8; i++) {
        uint32_t value;
        ret = hidKeyboard_GetModifier(&kbd, i, &value, false);
        zassert_equal(ret, USBHID_SUCCESS);
        zassert_equal(value, 0, "Modifier %d should be 0 by default", i);
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Report Structure
 * ======================================================================== */
ZTEST(hid_keyboard, test_report_structure)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);    
    zassert_equal(len, HID_KBD_REPORT_SIZE);
    
    // Initially all should be zero
    for (int i = 0; i < HID_KBD_REPORT_SIZE; i++) {
        zassert_equal(pBuff[i], 0, "Byte %d should be 0", i);
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Multiple Open/Close Cycles
 * ======================================================================== */
ZTEST(hid_keyboard, test_multiple_open_close)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    for (int i = 0; i < 3; i++) {
        int ret = hidKeyboard_Open(&hidDev, &kbd);
        zassert_equal(ret, USBHID_SUCCESS);
        
        // Use keyboard
        hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a'), 1, false);
        uint32_t value;
        hidKeyboard_GetKey(&kbd, HID_KBD_LETTER('a'), &value, false);
        zassert_equal(value, 1);
        
        hidKeyboard_Close(&kbd);
        
        // Verify buffer freed
        zassert_is_null(hidDev.report_buffer);
    }
}

/* ========================================================================
 * Test: Gaming keyboard with report ID
 * ======================================================================== */
ZTEST(hid_keyboard, test_gaming_keyboard_with_report_id)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)RAZER_BLACKWIDOW_V4_KEYBOARD;
    hidDev.raw_hid_report_desc_len = sizeof(RAZER_BLACKWIDOW_V4_KEYBOARD);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    
    zassert_equal(ret, USBHID_SUCCESS);
    zassert_equal(kbd.report_length, HID_KBD_REPORT_SIZE);
    zassert_equal(kbd.keys.count, HID_KBD_MAX_KEYS);
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Key Release When Array Full
 * ======================================================================== */
ZTEST(hid_keyboard, test_key_release_from_full_array)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Fill array with 6 keys
    uint8_t keys[6] = {
        HID_KBD_LETTER('a'),
        HID_KBD_LETTER('b'),
        HID_KBD_LETTER('c'),
        HID_KBD_LETTER('d'),
        HID_KBD_LETTER('e'),
        HID_KBD_LETTER('f')
    };
    
    for (int i = 0; i < 6; i++) {
        hidKeyboard_SetKey(&kbd, keys[i], 1, false);
    }
    
    // Release first key
    hidKeyboard_SetKey(&kbd, keys[0], 0, false);
    
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Verify remaining keys shifted
    zassert_equal(pBuff[kbd.keys.report_buf_off + 0], keys[1]);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 1], keys[2]);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 2], keys[3]);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 3], keys[4]);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 4], keys[5]);
    zassert_equal(pBuff[kbd.keys.report_buf_off + 5], 0);
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Modifier Bit Isolation
 * ======================================================================== */
ZTEST(hid_keyboard, test_modifier_bit_isolation)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set each mod individually and verify only that bit is set
    for (uint32_t bit = 0; bit < 8; bit++) {
        // Clear all mods
        uint8_t *pBuff;
        uint32_t len;
        ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
        zassert_equal(ret, USBHID_SUCCESS);
        pBuff[kbd.modifier.report_buf_off] = 0;
        
        // Set specific mod
        hidKeyboard_SetModifier(&kbd, bit, 1, false);
        
        // Verify only this bit is set
        for (uint32_t checkBit = 0; checkBit < 8; checkBit++) {
            uint32_t value;
            hidKeyboard_GetModifier(&kbd, checkBit, &value, false);
            
            if (checkBit == bit) {
                zassert_equal(value, 1, "Modifier bit %d should be set", bit);
            } else {
                zassert_equal(value, 0, "Modifier bit %d should not be set", checkBit);
            }
        }
        
        // Verify raw value
        zassert_equal(pBuff[kbd.modifier.report_buf_off], (1 << bit),
                      "Raw modifier byte should be 0x%02X", (1 << bit));
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Reserved Byte Unchanged
 * ======================================================================== */
ZTEST(hid_keyboard, test_reserved_byte_unchanged)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Set reserved byte to test value
    pBuff[1] = 0xAA;
    
    hidKeyboard_SetModifier(&kbd, 0, 1, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a'), 1, false);
    
    // Verify reserved byte unchanged
    zassert_equal(pBuff[1], 0xAA,
                  "Reserved byte should remain unchanged");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Clear All Keys
 * ======================================================================== */
ZTEST(hid_keyboard, test_clear_all_keys)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Press multiple keys
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a'), 1, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('b'), 1, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('c'), 1, false);
    
    // Release all keys
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a'), 0, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('b'), 0, false);
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('c'), 0, false);
    
    // Verify all key slots are 0
    uint8_t *pBuff;
    uint32_t len;
    ret = USBHID_getReportBuffer(&hidDev, &pBuff, &len, false);
    zassert_equal(ret, USBHID_SUCCESS);
    
    for (int i = 0; i < HID_KBD_MAX_KEYS; i++) {
        zassert_equal(pBuff[kbd.keys.report_buf_off + i], 0,
                      "Key slot %d should be empty", i);
    }
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Release Non-Existent Key
 * ======================================================================== */
ZTEST(hid_keyboard, test_release_non_existent_key)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Press key 'A'
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a'), 1, false);
    
    // Try to release key 'B' (not pressed)
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('b'), 0, false);
    
    // Verify 'A' still pressed
    uint32_t value;
    hidKeyboard_GetKey(&kbd, HID_KBD_LETTER('a'), &value, false);
    zassert_equal(value, 1, "Key 'A' should still be pressed");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Phantom Key Overflow
 * ======================================================================== */
ZTEST(hid_keyboard, test_phantom_key_prevention)
{
    struct HID_Keyboard_t kbd;
    
    hidDev.pUdev = &gUdev;
    hidDev.hid_type = USBHID_TYPE_KEYBOARD;
    hidDev.raw_hid_report_desc = (uint8_t *)COOLERMASTER_MASTERKEYS_S_1;
    hidDev.raw_hid_report_desc_len = sizeof(COOLERMASTER_MASTERKEYS_S_1);
    
    int ret = hidKeyboard_Open(&hidDev, &kbd);
    zassert_equal(ret, USBHID_SUCCESS);
    
    // Fill array with 6 keys
    for (uint8_t i = 0; i < 6; i++) {
        hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('a') + i, 1, false);
    }
    
    // Try to add 7th key
    hidKeyboard_SetKey(&kbd, HID_KBD_LETTER('g'), 1, false);
    
    // Verify 7th key not added
    uint32_t value;
    hidKeyboard_GetKey(&kbd, HID_KBD_LETTER('g'), &value, false);
    zassert_equal(value, 0, "7th key should not be added");
    
    hidKeyboard_Close(&kbd);
}

/* ========================================================================
 * Test: Keyboard Constants
 * ======================================================================== */
ZTEST(hid_keyboard, test_keyboard_constants)
{
    zassert_equal(HID_KBD_REPORT_SIZE, 8);
    zassert_equal(HID_KBD_MODIFIER_OFFSET, 0);
    zassert_equal(HID_KBD_RESERVED_OFFSET, 1);
    zassert_equal(HID_KBD_KEYS_OFFSET, 2);
    zassert_equal(HID_KBD_MAX_KEYS, 6);
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(hid_keyboard, NULL, NULL, test_setup, test_teardown, NULL);