#include <zephyr/ztest.h>
#include <zephyr/usb/usb_ch9.h>
#include "usb_stubs.h"
#include "ch375_host.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *ctx;
static struct USB_Device_t udev;

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&ctx), CH375_SUCCESS);
    
    // Initialize minimal USB device structure
    memset(&udev, 0, sizeof(udev));
    udev.ctx = ctx;
    udev.ep0_max_packet = 64;
    udev.connected = true;
}

static void test_teardown(void *f)
{
    if (ctx) {
        ch375_closeContext(ctx);
        ctx = NULL;
    }
}

/* ========================================================================
 * Helper: Queue Control Transfer Success Responses
 * ======================================================================== */
static void queue_control_success_responses(void)
{
    // SETUP stage success
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
}

static void queue_control_data_in_responses(const uint8_t *data, size_t len)
{
    // DATA IN stage success
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    
    // Queue data with length prefix
    mock_ch375QueueResponse(len);
    mock_ch375QueueResponses(data, len);
    
    // STATUS OUT stage success
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
}

static void queue_control_status_in_success(void)
{
    // STATUS IN stage success
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
}

/* ========================================================================
 * Test: Control Transfer - Get Descriptor (IN)
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_get_device_descriptor)
{
    uint8_t buffer[18];
    int actual_len = 0;
    
    // Simulate device descriptor response
    uint8_t device_desc[] = {
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
        0x5E, 0x04, 0x3B, 0x00, 0x20, 0x01, 0x01, 0x02,
        0x00, 0x01
    };
    
    queue_control_success_responses();
    queue_control_data_in_responses(device_desc, sizeof(device_desc));
    
    // Execute: GET_DESCRIPTOR (Device)
    int ret = ch375_hostControlTransfer(&udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                USB_SREQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0, buffer, sizeof(buffer), &actual_len, 5000);
    
    // Verify
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actual_len, 18);
    zassert_mem_equal(buffer, device_desc, 18);
    
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
    queue_control_success_responses();
    
    // Simulate STALL on DATA stage
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_PID2STATUS(USB_PID_STALL));
    
    uint8_t buffer[8];
    int ret = ch375_hostControlTransfer(&udev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                        USB_SREQ_GET_DESCRIPTOR, USB_DESC_DEVICE << 8, 0, buffer, sizeof(buffer), NULL, 5000 );
    
    zassert_equal(ret, CH375_HOST_STALL, "Should detect STALL");
}

/* ========================================================================
 * Test: Control Transfer - Device Disconnect During Transfer
 * ======================================================================== */
ZTEST(ch375_transfers, test_control_transfer_disconnect)
{
    queue_control_success_responses();
    
    // Simulate disconnect on DATA stage
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_DISCONNECT);
    
    uint8_t buffer[8];
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
    int actual_len = 0;
    
    // First packet: full 64 bytes
    uint8_t packet1[64];
    memset(packet1, 0xAA, sizeof(packet1));
    
    // Second packet: full 64 bytes
    uint8_t packet2[64];
    memset(packet2, 0xBB, sizeof(packet2));
    
    // SETUP stage
    queue_control_success_responses();
    
    // First DATA IN packet
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(64);
    mock_ch375QueueResponses(packet1, 64);
    
    // Second DATA IN packet
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(32);
    mock_ch375QueueResponses(packet2, 32);
    
    // STATUS OUT stage
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    
    // Execute
    int ret = ch375_hostControlTransfer(
        &udev,
        USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_GET_DESCRIPTOR,
        USB_DESC_CONFIGURATION << 8,
        0,
        buffer,
        sizeof(buffer),
        &actual_len,
        5000
    );
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actual_len, 96, "Should receive 64 + 32 bytes");
    zassert_mem_equal(buffer, packet1, 64);
    zassert_mem_equal(buffer + 64, packet2, 32);
}

/* ========================================================================
 * Test: Bulk Transfer - Basic IN Transfer
 * ======================================================================== */
ZTEST(ch375_transfers, test_bulk_transfer_in_basic)
{
    // Setup device with one interface and one IN endpoint
    udev.interface_count = 1;
    udev.interfaces[0].endpoint_count = 1;
    udev.interfaces[0].endpoints[0].ep_addr = 0x81;
    udev.interfaces[0].endpoints[0].max_packet = 64;
    udev.interfaces[0].endpoints[0].attributes = 0x02;
    udev.interfaces[0].endpoints[0].data_toggle = false;
    
    uint8_t buffer[64];
    int actual_len = 0;
    uint8_t test_data[64];
    memset(test_data, 0xCC, sizeof(test_data));
    
    // Simulate successful IN transfer
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(64);
    mock_ch375QueueResponses(test_data, 64);
    
    int ret = ch375_hostBulkTransfer(
        &udev,
        0x81,
        buffer,
        sizeof(buffer),
        &actual_len,
        5000
    );
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actual_len, 64);
    zassert_mem_equal(buffer, test_data, 64);
    zassert_true(udev.interfaces[0].endpoints[0].data_toggle, "Toggle should flip");
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
    
    uint8_t buffer[4];  // Match the expected data size
    int actual_len = 0;
    uint8_t test_data[4] = {0x11, 0x22, 0x33, 0x44};
    
    // First attempt: NAK
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_PID2STATUS(USB_PID_NAK));
    
    // Second attempt: Success
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    mock_ch375QueueResponse(4);
    mock_ch375QueueResponses(test_data, 4);
    
    int ret = ch375_hostBulkTransfer(&udev, 0x81, buffer, sizeof(buffer), &actual_len, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actual_len, 4);
    zassert_mem_equal(buffer, test_data, 4);
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
    int actual_len = 0;
    
    // Keep responding with NAK
    for (int i = 0; i < 10; i++) {
        mock_ch375SetIntState(true);
        mock_ch375QueueResponse(CH375_PID2STATUS(USB_PID_NAK));
    }
    
    // Short timeout for test speed
    int ret = ch375_hostBulkTransfer(
        &udev,
        0x81,
        buffer,
        sizeof(buffer),
        &actual_len,
        5
    );
    
    zassert_equal(ret, CH375_HOST_TIMEOUT);
    zassert_equal(actual_len, 0, "No data should be transferred");
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
    int actual_len = 0;
    
    // Successful OUT
    mock_ch375SetIntState(true);
    mock_ch375QueueResponse(CH375_USB_INT_SUCCESS);
    
    int ret = ch375_hostBulkTransfer(&udev, 0x01, data, sizeof(data), &actual_len, 5000);
    
    zassert_equal(ret, CH375_HOST_SUCCESS);
    zassert_equal(actual_len, 4);
    
    // Verify data was written
    uint8_t history[20];
    int count;
    mock_ch375GetDataHistory(history, &count, 20);
    
    // Should have: length byte + 4 data bytes + toggle + ep/pid
    zassert_true(count >= 6);
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