/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_ch375_core.c
 * @brief          CH375 core protocol unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for CH375 core functionality including existence check,
 * version query, USB mode setting, status retrieval, device connection,
 * token sending, and block data transfers. Depends on mock hardware layer.
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
#include "ch375.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *pCtx;

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
}

static void test_teardown(void *f)
{
    if (pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: CH375 Existence Check
 * ======================================================================== */
ZTEST(ch375_core, test_checkexist_success)
{
    // Expected response: ~0x65 = 0x9A
    mock_ch375QueueResponse(CH375_CHECK_EXIST_DATA2);
    
    int ret = ch375_checkExist(pCtx);
    
    zassert_equal(ret, CH375_SUCCESS, "Check exist should succeed");
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_CHECK_EXIST), "CHECK_EXIST command should be sent");
    zassert_equal(mock_ch375GetLastData(), CH375_CHECK_EXIST_DATA1, "Should send 0x65 as data");
}

ZTEST(ch375_core, test_checkexist_wrong_response)
{
    // Queue wrong response
    mock_ch375QueueResponse(0x42);
    
    int ret = ch375_checkExist(pCtx);
    
    zassert_equal(ret, CH375_NO_EXIST, "Should fail with wrong response");
}

ZTEST(ch375_core, test_checkexist_timeout)
{
    // Don't queue any response
    int ret = ch375_checkExist(pCtx);
    
    zassert_equal(ret, CH375_READ_DATA_FAILED, "Should timeout without response");
}

/* ========================================================================
 * Test: Get Version
 * ======================================================================== */
ZTEST(ch375_core, test_get_version_success)
{
    uint8_t version = 0;
    
    // CH375 version is in lower 6 bits
    mock_ch375QueueResponse(0x43);
    
    int ret = ch375_getVersion(pCtx, &version);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(version, 0x03, "Should mask to lower 6 bits");
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_GET_IC_VER));
}

ZTEST(ch375_core, test_get_version_null_param)
{
    int ret = ch375_getVersion(pCtx, NULL);
    
    zassert_equal(ret, CH375_PARAM_INVALID);
}

/* ========================================================================
 * Test: Set USB Mode
 * ======================================================================== */
ZTEST(ch375_core, test_set_usb_mode_host)
{
    mock_ch375QueueResponse(CH375_CMD_RET_SUCCESS);
    
    int ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_SOF_AUTO);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_SET_USB_MODE));
}

ZTEST(ch375_core, test_set_usb_mode_failure)
{
    mock_ch375QueueResponse(CH375_CMD_RET_FAILED);
    
    int ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_SOF_AUTO);
    
    zassert_equal(ret, CH375_ERROR, "Should fail with error response");
}

/* ========================================================================
 * Test: Get Status
 * ======================================================================== */
ZTEST(ch375_core, test_get_status)
{
    uint8_t status = 0;
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_getStatus(pCtx, &status);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(status, CH375_USB_INT_SUCCESS);
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_GET_STATUS));
}

/* ========================================================================
 * Test: Test Connect
 * ======================================================================== */
ZTEST(ch375_core, test_connect_device_connected)
{
    uint8_t connStatus = 0;
    mock_ch375QueueResponse(CH375_USB_INT_CONNECT);
    
    int ret = ch375_testConnect(pCtx, &connStatus);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(connStatus, CH375_USB_INT_CONNECT);
}

ZTEST(ch375_core, test_connect_device_disconnected)
{
    uint8_t connStatus = 0;
    
    // First response for testConnect
    mock_ch375QueueResponse(CH375_USB_INT_DISCONNECT);
    // Second response for getStatus called internally
    mock_ch375QueueStatus(0x00);
    
    int ret = ch375_testConnect(pCtx, &connStatus);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(connStatus, CH375_USB_INT_DISCONNECT);
}

/* ========================================================================
 * Test: Wait for INT
 * ======================================================================== */
ZTEST(ch375_core, test_wait_int_immediate)
{
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_waitInt(pCtx, 1000);
    
    zassert_equal(ret, CH375_SUCCESS);
}

ZTEST(ch375_core, test_wait_int_after_polling)
{
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_waitInt(pCtx, 1000);
    
    zassert_equal(ret, CH375_SUCCESS);
}

ZTEST(ch375_core, test_wait_int_timeout)
{
    // Set default status to other value
    mock_ch375SetDefaultStatus(0x00);
    
    int ret = ch375_waitInt(pCtx, 10);
    
    zassert_equal(ret, CH375_TIMEOUT);
}

/* ========================================================================
 * Test: Send Token
 * ======================================================================== */
ZTEST(ch375_core, test_send_token_setup)
{
    uint8_t status;

    // Queue statuses for ch375_waitInt() polling
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    // Final getStatus call
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_sendToken(pCtx, 0, false, USB_PID_SETUP, &status);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(status, CH375_USB_INT_SUCCESS);
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_ISSUE_TKN_X));
}

ZTEST(ch375_core, test_send_token_in)
{
    uint8_t status;
    
    // Queue statuses for polling
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_sendToken(pCtx, 0, true, USB_PID_IN, &status);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(status, CH375_USB_INT_SUCCESS);
    
    // Verify toggle bit set (0xC0)
    uint8_t history[10];
    int count;
    mock_ch375GetDataHistory(history, &count, 10);
    zassert_true(count >= 2);
    zassert_equal(history[count-2], 0xC0, "DATA1 toggle should be 0xC0");
}

/* ========================================================================
 * Test: Block Data Read/Write
 * ======================================================================== */
ZTEST(ch375_core, test_write_block_data)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    
    int ret = ch375_writeBlockData(pCtx, data, sizeof(data));
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_WR_USB_DATA7));
    
    // Verify data was written
    uint8_t history[10];
    int count;
    mock_ch375GetDataHistory(history, &count, 10);
    zassert_equal(count, 5, "Should write length + 4 bytes");
    zassert_equal(history[0], 4, "First byte should be length");
    zassert_mem_equal(&history[1], data, 4, "Data should match");
}

ZTEST(ch375_core, test_read_block_data)
{
    uint8_t buffer[10];
    uint8_t actualLen;
    
    // Queue response: length + data
    uint8_t response[] = {4, 0xAA, 0xBB, 0xCC, 0xDD};
    mock_ch375QueueResponses(response, sizeof(response));
    
    int ret = ch375_readBlockData(pCtx, buffer, sizeof(buffer), &actualLen);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(actualLen, 4);
    zassert_equal(buffer[0], 0xAA);
    zassert_equal(buffer[3], 0xDD);
}

ZTEST(ch375_core, test_read_block_data_short_packet)
{
    uint8_t buffer[10];
    uint8_t actualLen;
    
    // Queue: length says 10, but only 3 bytes available
    mock_ch375QueueResponse(10);
    mock_ch375QueueResponse(0x11);
    mock_ch375QueueResponse(0x22);
    mock_ch375QueueResponse(0x33);
    // No more data should cause timeout
    
    int ret = ch375_readBlockData(pCtx, buffer, sizeof(buffer), &actualLen);
    
    zassert_equal(ret, CH375_SUCCESS);
    zassert_equal(actualLen, 3, "Should return actual bytes read");
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(ch375_core, NULL, NULL, test_setup, test_teardown, NULL);