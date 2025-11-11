/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           test_descriptors.c
 * @brief          USB descriptor parsing unit tests
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Unit tests for USB descriptor parsing including device, configuration,
 * interface, endpoint, and HID descriptors. Tests multi-interface configs,
 * malformed descriptor detection, and endpoint type/direction parsing.
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

static struct ch375_Context_t *pCtx;

/**
 * @brief Sample USB Device Descriptor
 */
static const uint8_t sampleDeviceDesc[] = {
    0x12,                       // bLength
    0x01,                       // bDescriptorType: DEVICE
    0x00, 0x02,                 // bcdUSB: 2.0 (LE)
    0x00,                       // bDeviceClass
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    0x40,                       // bMaxPacketSize0: 64
    0x5E, 0x04,                 // idVendor: 0x045E (Microsoft)
    0x3B, 0x00,                 // idProduct: 0x003B
    0x20, 0x01,                 // bcdDevice: 1.20
    0x01,                       // iManufacturer
    0x02,                       // iProduct
    0x00,                       // iSerialNumber
    0x01                        // bNumConfigurations
};

/**
 * @brief Sample USB Mouse Configuration Descriptor
 */
static const uint8_t sampleMouseConfig[] = {
    // Config Descriptor
    0x09,                       // bLength
    0x02,                       // bDescriptorType: CONFIGURATION
    0x22, 0x00,                 // wTotalLength: 34 bytes
    0x01,                       // bNumInterfaces
    0x01,                       // bConfigurationValue
    0x00,                       // iConfiguration
    0xA0,                       // bmAttributes: Remote wakeup
    0x32,                       // bMaxPower: 100mA

    // Interface Descriptor
    0x09,                       // bLength
    0x04,                       // bDescriptorType: INTERFACE
    0x00,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x01,                       // bNumpEndpoints
    0x03,                       // bInterfaceClass: HID
    0x01,                       // bInterfaceSubClass: Boot
    0x02,                       // bInterfaceProtocol: Mouse
    0x00,                       // iInterface

    // HID Descriptor
    0x09,                       // bLength
    0x21,                       // bDescriptorType: HID
    0x11, 0x01,                 // bcdHID: 1.11
    0x00,                       // bCountryCode
    0x01,                       // bNumDescriptors
    0x22,                       // bDescriptorType: Report
    0x34, 0x00,                 // wDescriptorLength: 52

    // pEndpoint Descriptor
    0x07,                       // bLength
    0x05,                       // bDescriptorType: pEndPOINT
    0x81,                       // bEndpointAddress: IN pEndpoint 1
    0x03,                       // bmAttributes: Interrupt
    0x04, 0x00,                 // wMaxPacketSize: 4
    0x0A                        // bInterval: 10ms
};

/**
 * @brief Sample USB Keyboard Configuration with multiple pEndpoints
 */
static const uint8_t sampleKeyboardConfig[] = {
    // Configuration descriptor
    0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32,
    // Interface descriptor, keyboard
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
    // HID class descriptor
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3e, 0x00,
    // Endpoint descriptor
    0x07, 0x05, 0x84, 0x03, 0x08, 0x00, 0x0a,
    // Interface descriptor, media/consumer
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
    // HID class descriptor
    0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00,
    // Endpoint descriptor
    0x07, 0x05, 0x87, 0x03, 0x04, 0x00, 0x0a
};


static const struct usb_desc_header *find_descriptor_of_type(
    const uint8_t *pBuf, size_t buflen, uint8_t descType, unsigned int nth) {
    
    const uint8_t *pCur = pBuf;
    const uint8_t *pEnd = pBuf + buflen;
    unsigned int found = 0;

    while (pCur + sizeof(struct usb_desc_header) <= pEnd) {
        const struct usb_desc_header *pHdr = (const struct usb_desc_header *)pCur;

        if (0 == pHdr->bLength) {
            return NULL;
        }
        if (pCur + pHdr->bLength > pEnd) {
            return NULL;
        }

        if (pHdr->bDescriptorType == descType) {
            if (found == nth) {
                return pHdr;
            }
            found++;
        }

        pCur += pHdr->bLength;
    }

    return NULL;
}

static const struct usb_desc_header *find_descriptor_from_offset(
    const uint8_t *pBuff, size_t buflen, size_t offset, uint8_t descType)
{
    if (offset >= buflen) {
        return NULL;
    }
    return find_descriptor_of_type(pBuff + offset, buflen - offset, descType, 0);
}

static void test_setup(void *f) {

    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
}

static void test_teardown(void *f) {

    if (NULL != pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: Device Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_device_descriptor_basic) {

    const struct usb_device_descriptor *pDesc = (const struct usb_device_descriptor *)sampleDeviceDesc;

    zassert_equal(pDesc->bLength, 18);
    zassert_equal(pDesc->bDescriptorType, USB_DESC_DEVICE);
    zassert_true(8 == pDesc->bMaxPacketSize0 || 16 == pDesc->bMaxPacketSize0 ||
                32 == pDesc->bMaxPacketSize0 || 64 == pDesc->bMaxPacketSize0);

    zassert_equal(sys_le16_to_cpu(pDesc->bcdUSB), 0x0200);
    zassert_equal(pDesc->bMaxPacketSize0, 64);
    zassert_equal(sys_le16_to_cpu(pDesc->idVendor), 0x045E);
    zassert_equal(sys_le16_to_cpu(pDesc->idProduct), 0x003B);
}

 /* ========================================================================
 * Test: Configuration Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_config_descriptor_basic) {

    const struct usb_cfg_descriptor *pCfg = (const struct usb_cfg_descriptor *)sampleMouseConfig;

    zassert_equal(pCfg->bLength, 9);
    zassert_equal(pCfg->bDescriptorType, USB_DESC_CONFIGURATION);
    zassert_equal(sys_le16_to_cpu(pCfg->wTotalLength), 34);
    zassert_equal(pCfg->bNumInterfaces, 1);
    zassert_equal(pCfg->bConfigurationValue, 1);

    zassert_true(sys_le16_to_cpu(pCfg->wTotalLength) <= sizeof(sampleMouseConfig));
}

/* ========================================================================
 * Test: Interface Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_interface_descriptor) {

    const struct usb_if_descriptor *pIfDesc = (const struct usb_if_descriptor *)(sampleMouseConfig + 9);

    zassert_equal(pIfDesc->bLength, 9);
    zassert_equal(pIfDesc->bDescriptorType, USB_DESC_INTERFACE);
    zassert_equal(pIfDesc->bInterfaceNumber, 0);
    zassert_equal(pIfDesc->bNumEndpoints, 1);
    zassert_equal(pIfDesc->bInterfaceClass, 0x03, "Should be HID class");
    zassert_equal(pIfDesc->bInterfaceSubClass, 0x01, "Should be Boot subclass");
    zassert_equal(pIfDesc->bInterfaceProtocol, 0x02, "Should be Mouse protocol");
}

/* ========================================================================
 * Test: Endpoint Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_endpoint_descriptor) {

    const struct usb_ep_descriptor *pEP = (const struct usb_ep_descriptor *)(sampleMouseConfig + 27);

    zassert_equal(pEP->bLength, 7);
    zassert_equal(pEP->bDescriptorType, USB_DESC_ENDPOINT);
    zassert_equal(pEP->bEndpointAddress, 0x81, "Should be IN endpoint 1");
    zassert_equal(pEP->bmAttributes & 0x03, 0x03, "Should be Interrupt type");
    zassert_equal(sys_le16_to_cpu(pEP->wMaxPacketSize), 4);
    zassert_equal(pEP->bInterval, 10);
}

/* ========================================================================
 * Test: Full Configuration Parsing (Mouse)
 * ======================================================================== */
ZTEST(ch375_descriptors, test_full_mouse_config_walk) {

    const uint8_t *pCur = sampleMouseConfig;
    const uint8_t *pEnd = sampleMouseConfig + sizeof(sampleMouseConfig);
    int descriptor_count = 0;
    int interface_count = 0;
    int endpoint_count = 0;

    while (pCur + sizeof(struct usb_desc_header) <= pEnd) {
        
        const struct usb_desc_header *pHdr = (const struct usb_desc_header *)pCur;

        zassert_not_equal(pHdr->bLength, 0, "Descriptor length cannot be 0");
        zassert_true(pCur + pHdr->bLength <= pEnd, "Descriptor exceeds buffer");

        descriptor_count++;

        switch (pHdr->bDescriptorType) {
        case USB_DESC_CONFIGURATION:
            zassert_equal(descriptor_count, 1, "First descriptor must be configuration");
            break;

        case USB_DESC_INTERFACE:
            interface_count++;
            break;

        case USB_DESC_ENDPOINT:
            endpoint_count++;
            break;

        default:
            break;
        }

        pCur += pHdr->bLength;
    }

    zassert_equal(interface_count, 1, "Mouse should have 1 interface");
    zassert_equal(endpoint_count, 1, "Mouse should have 1 endpoint");
}

/* ========================================================================
 * Test: Multi-Interface Configuration (Keyboard)
 * ======================================================================== */
ZTEST(ch375_descriptors, test_keyboard_multi_interface) {

    const uint8_t *pCur = sampleKeyboardConfig;
    const uint8_t *pEnd = sampleKeyboardConfig + sizeof(sampleKeyboardConfig);
    int interface_count = 0;
    int endpoint_count = 0;
    int current_interface = -1;
    int ep_per_interface[2] = {0, 0};

    while (pCur + sizeof(struct usb_desc_header) <= pEnd) {
        const struct usb_desc_header *pHdr = (const struct usb_desc_header *)pCur;

        if (0 == pHdr->bLength || pCur + pHdr->bLength > pEnd) {
            break;
        }

        if (USB_DESC_INTERFACE == pHdr->bDescriptorType) {
            const struct usb_if_descriptor *iface = (const struct usb_if_descriptor *)pCur;
            current_interface = iface->bInterfaceNumber;
            interface_count++;

            if (0 == current_interface) {
                zassert_equal(iface->bInterfaceClass, 0x03);
                zassert_equal(iface->bInterfaceProtocol, 0x01, "Interface 0 should be keyboard");
            }
        } else if (USB_DESC_ENDPOINT == pHdr->bDescriptorType) {
            endpoint_count++;
            if (current_interface >= 0 && current_interface < 2) {
                ep_per_interface[current_interface]++;
            }
        }

        pCur += pHdr->bLength;
    }

    zassert_equal(interface_count, 2, "Keyboard should have 2 interfaces");
    zassert_equal(endpoint_count, 2, "Should have 2 endpoints total");
    zassert_equal(ep_per_interface[0], 1, "Interface 0 should have 1 EP");
    zassert_equal(ep_per_interface[1], 1, "Interface 1 should have 1 EP");
}

/* ========================================================================
 * Test: Malformed Descriptor Handling
 * ======================================================================== */
ZTEST(ch375_descriptors, test_malformed_zero_length) {

    uint8_t pBadDesc[] = {
        0x09, 0x02, 0x09, 0x00, 0x01, 0x01, 0x00, 0xA0, 0x32,
        0x00
    };

    const uint8_t *pCur = pBadDesc;
    const uint8_t *pEnd = pBadDesc + sizeof(pBadDesc);
    int descCount = 0;

    while (pCur + sizeof(struct usb_desc_header) <= pEnd) {
        const struct usb_desc_header *pHdr = (const struct usb_desc_header *)pCur;

        if (0 == pHdr->bLength) {
            break;
        }

        descCount++;
        pCur += pHdr->bLength;
    }

    zassert_equal(descCount, 1, "Should stop at zero-length descriptor");
}

ZTEST(ch375_descriptors, test_malformed_length_exceeds) {

    uint8_t pBadDesc[] = {
        0x09, 0x02, 0x0F, 0x00, 0x01, 0x01, 0x00, 0xA0, 0x32,
        0xFF, 0x04
    };

    const uint8_t *pCur = pBadDesc;
    const uint8_t *pEnd = pBadDesc + sizeof(pBadDesc);
    bool detected_overflow = false;
    int descCount = 0;

    while (pCur + sizeof(struct usb_desc_header) <= pEnd) {
        const struct usb_desc_header *pHdr = (const struct usb_desc_header *)pCur;

        if (0 == pHdr->bLength || pCur + pHdr->bLength > pEnd) {
            detected_overflow = true;
            break;
        }

        descCount++;
        pCur += pHdr->bLength;
    }

    zassert_true(detected_overflow, "Should detect length overflow");
}

/* ========================================================================
 * Test: HID Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_hid_descriptor) {

    const struct usb_desc_header *pHidHdr = find_descriptor_from_offset(sampleMouseConfig, 
        sizeof(sampleMouseConfig), 18, 0x21);

    zassert_not_null(pHidHdr, "HID descriptor not found");

    const uint8_t *pHid = (const uint8_t *)pHidHdr;

    zassert_equal(pHid[0], 9, "HID bLength");
    zassert_equal(pHid[1], 0x21, "HID bDescriptorType");

    uint16_t bcdHID = pHid[2] | (pHid[3] << 8);
    zassert_equal(bcdHID, 0x0111, "HID version 1.11 expected");

    zassert_equal(pHid[5], 0x01, "bNumDescriptors should be 1");
    zassert_equal(pHid[6], 0x22, "class descriptor type should be Report (0x22)");

    uint16_t report_len = pHid[7] | (pHid[8] << 8);
    zassert_equal(report_len, 52, "Report descriptor length should be 52");
}

/* ========================================================================
 * Test: Endpoint Address & Type Detection
 * ======================================================================== */
ZTEST(ch375_descriptors, test_endpoint_direction_detection) {

    uint8_t epInAddr = 0x81;
    zassert_true(epInAddr & 0x80, "Should detect IN direction");

    uint8_t epOutAddr = 0x01;
    zassert_false(epOutAddr & 0x80, "Should detect OUT direction");
}

ZTEST(ch375_descriptors, test_endpoint_type_detection) {

    uint8_t attrInt = 0x03;
    zassert_equal(attrInt & 0x03, 0x03, "Should be Interrupt");

    uint8_t attrBulk = 0x02;
    zassert_equal(attrBulk & 0x03, 0x02, "Should be Bulk");

    uint8_t attrIso = 0x01;
    zassert_equal(attrIso & 0x03, 0x01, "Should be Isochronous");
}

/* -------------------------------------------------------------------------
 * Test suite registration
 * ------------------------------------------------------------------------- */
ZTEST_SUITE(ch375_descriptors, NULL, NULL, test_setup, test_teardown, NULL);
