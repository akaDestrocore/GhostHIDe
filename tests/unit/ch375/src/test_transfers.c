/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_transfers.c
 * @brief          USB transfer protocol unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for USB control and bulk transfers including GET_DESCRIPTOR,
 * SET_ADDRESS, STALL handling, disconnect detection, multi-packet data
 * phase, NAK retry logic, and clear stall endpoint recovery.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <zephyr/ztest.h>
#include <zephyr/usb/usb_ch9.h>
#include "usb_stubs.h"
#include "ch375_host.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *gCtx;
static struct USB_Device_t udev;

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&gCtx), CH375_SUCCESS);
    
    // Initialize minimal USB device structure
    memset(&udev, 0, sizeof(udev));
    udev.ctx = gCtx;
    udev.ep0_max_packet = 64;
    udev.connected = true;
}

static void test_teardown(void *f)
{
    if (gCtx) {
        ch375_closeContext(gCtx);
        gCtx = NULL;
    }
}

/* ========================================================================
 * Helper: Queue Control Transfer Success Responses
 * ======================================================================== */

static void queue_control_success_responses(void)
{
    // SETUP stage success
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    // Final getStatus call after waitInt returns
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
}

static void queue_control_data_in_responses(const uint8_t *pData, size_t len)
{
    // DATA IN stage success
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    // Queue data with length prefix
    mock_ch375QueueResponse(len);
    mock_ch375QueueResponses(pData, len);
    
    // STATUS OUT stage
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
}

static void queue_control_status_in_success(void)
{
    // STATUS IN stage
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
}

/* ========================================================================
 * Test: Control Transfer - Get Descriptor (IN)
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_get_device_descriptor)
{
    uint8_t buffer[18];
    int actualLen = 0;
    
    // Simulate device descriptor response
    uint8_t pDeviceDesc[] = {
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
        0x5E, 0x04, 0x3B, 0x00, 0x20, 0x01, 0x01, 0x02,
        0x00, 0x01
    };
    
    queue_control_success_responses();
    queue_control_data_in_responses(pDeviceDesc, sizeof(pDeviceDesc));
    
    // Execute: GET_DESCRIPTOR (Device)
    int ret = ch375_hostControlTransfer(&udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                USB_SREQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0, buffer, sizeof(buffer), &actualLen, 5000);
    
    // Verify
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actualLen, 18);
    zassert_mem_equal(buffer, pDeviceDesc, 18);
    
    // Verify command sequence
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_WR_USB_DATA7),
                 "Should write SETUP packet");
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_ISSUE_TKN_X),
                 "Should issue tokens");
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_RD_USB_DATA),
                 "Should read data");
}

/* ========================================================================
 * Test: Control Transfer - Set Address (OUT)
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_set_address)
{
    queue_control_success_responses();
    queue_control_status_in_success();
    
    // Execute: SET_ADDRESS
    int ret = ch375_hostControlTransfer( &udev, USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                                                                USB_SREQ_SET_ADDRESS, 5, 0, NULL, 0, NULL, 5000);
    
    // Verify
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_true(mock_ch375VerifyCmdSent(CH375_CMD_ISSUE_TKN_X));
}

/* ========================================================================
 * Test: Control Transfer - STALL Handling
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_stall)
{
    uint8_t buffer[8];
    
    // SETUP stage success
    queue_control_success_responses();
    
    // Simulate STALL on DATA stage
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_STALL));
    mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_STALL));
    
    int ret = ch375_hostControlTransfer(&udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                        USB_SREQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0, buffer, sizeof(buffer), NULL, 5000 );
    
    zassert_equal(ret, CH375_HOST_STALL, "Should detect STALL");
}

/* ========================================================================
 * Test: Control Transfer - Device Disconnect During Transfer
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_disconnect)
{
    uint8_t buffer[8];
    
    // SETUP stage success
    queue_control_success_responses();
    
    // Simulate disconnect on DATA stage
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_DISCONNECT);
    mock_ch375QueueStatus(CH375_USB_INT_DISCONNECT);
    
    int ret = ch375_hostControlTransfer( &udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                        USB_SREQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0, buffer, sizeof(buffer), NULL, 5000 );
    
    zassert_equal(ret, CH375_HOST_DEV_DISCONNECT);
}

/* ========================================================================
 * Test: Control Transfer - Multi-Packet Data Phase
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_multi_packet)
{
    uint8_t buffer[128];
    int actualLen = 0;
    
    // First packet: full 64 bytes
    uint8_t packet1[64];
    memset(packet1, 0xAA, sizeof(packet1));
    
    // Second packet: 32 bytes
    uint8_t packet2[32];
    memset(packet2, 0xBB, sizeof(packet2));
    
    // SETUP stage
    queue_control_success_responses();
    
    // First DATA IN packet: full 64 bytes
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(64);
    mock_ch375QueueResponses(packet1, 64);
    
    // Second DATA IN packet
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(32);
    mock_ch375QueueResponses(packet2, 32);
    
    // STATUS OUT stage
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    // Execute
    int ret = ch375_hostControlTransfer( &udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
            USB_SREQ_GET_DESCRIPTOR, USB_DESC_CONFIGURATION << 8, 0, buffer, sizeof(buffer), &actualLen, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actualLen, 96, "Should receive 64 + 32 bytes");
    zassert_mem_equal(buffer, packet1, 64);
    zassert_mem_equal(buffer + 64, packet2, 32);
}

/* ========================================================================
 * Test: Bulk Transfer - Basic IN Transfer
 * ======================================================================== */
ZTEST(ch375_transfers, test_bulk_transfer_in_basic)
{
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x81;
    udev.interfaces[0].endpoints[0].max_packet = 64;
    udev.interfaces[0].endpoints[0].attributes = 0x02;
    udev.interfaces[0].endpoints[0].data_toggle = false;
    
    uint8_t buffer[64];
    int actualLen = 0;
    uint8_t testData[64];
    memset(testData, 0xCC, sizeof(testData));
    
    // Simulate successful IN transfer
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(64);
    mock_ch375QueueResponses(testData, 64);
    
    int ret = ch375_hostBulkTransfer(&udev, 0x81, buffer, sizeof(buffer), &actualLen, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actualLen, 64);
    zassert_mem_equal(buffer, testData, 64);
    zassert_true(udev.interfaces[0].endpoints[0].data_toggle);
}

/* ========================================================================
 * Test: Bulk Transfer - NAK Handling
 * ======================================================================== */
ZTEST(ch375_transfers, test_bulk_transfer_nak_then_success)
{
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x81;
    udev.interfaces[0].endpoints[0].max_packet = 64;
    udev.interfaces[0].endpoints[0].attributes = 0x02;
    udev.interfaces[0].endpoints[0].data_toggle = false;
    
    uint8_t buffer[4];
    int actualLen = 0;
    uint8_t testData[4] = {0x11, 0x22, 0x33, 0x44};
    
    // First attempt: NAK
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_NAK));
    mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_NAK));
    
    // Second attempt: Success
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(4);
    mock_ch375QueueResponses(testData, 4);
    
    int ret = ch375_hostBulkTransfer(&udev, 0x81, buffer, sizeof(buffer), &actualLen, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actualLen, 4);
    zassert_mem_equal(buffer, testData, 4);
}

/* ========================================================================
 * Test: Bulk Transfer - NAK Timeout
 * ======================================================================== */
ZTEST(ch375_transfers, test_bulk_transfer_nak_timeout)
{
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x81;
    udev.interfaces[0].endpoints[0].max_packet = 64;
    udev.interfaces[0].endpoints[0].attributes = 0x02;
    udev.interfaces[0].endpoints[0].data_toggle = false;
    
    uint8_t buffer[64];
    int actualLen = 0;
    
    // Keep responding with NAK
    for (int attempt = 0; attempt < 10; attempt++) {
        mock_ch375QueueStatus(0x00);
        mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_NAK));
        mock_ch375QueueStatus(CH375_PID2STATUS(USB_PID_NAK));
    }
    
    int ret = ch375_hostBulkTransfer(&udev, 0x81, buffer, sizeof(buffer), &actualLen, 5);
    
    zassert_equal(ret, CH375_HOST_TIMEOUT);
    zassert_equal(actualLen, 0);
}

/* ========================================================================
 * Test: Bulk Transfer - OUT Direction
 * ======================================================================== */
ZTEST(ch375_transfers, test_bulk_transfer_out)
{
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x01;
    udev.interfaces[0].endpoints[0].max_packet = 64;
    udev.interfaces[0].endpoints[0].attributes = 0x02;
    udev.interfaces[0].endpoints[0].data_toggle = false;
    
    uint8_t data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    int actualLen = 0;
    
    // Successful OUT
    mock_ch375QueueStatus(0x00);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    mock_ch375QueueStatus(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_hostBulkTransfer(&udev, 0x01, data, sizeof(data), &actualLen, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actualLen, 4);
}

/* ========================================================================
 * Test: Clear Stall
 * ======================================================================== */
ZTEST(ch375_transfers, test_clear_stall)
{
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x81;
    udev.interfaces[0].endpoints[0].data_toggle = true;
    
    // Simulate CLEAR_FEATURE control transfer
    queue_control_success_responses();
    queue_control_status_in_success();
    
    int ret = ch375_hostClearStall(&udev, 0x81);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_false(udev.interfaces[0].endpoints[0].data_toggle,
                  "Toggle should be reset to DATA0");
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(ch375_transfers, NULL, NULL, test_setup, test_teardown, NULL);