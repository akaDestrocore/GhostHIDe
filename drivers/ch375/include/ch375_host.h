/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           ch375_host.h
 * @brief          CH375 USB host layer interface
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * High-level USB host operations including device enumeration, descriptor
 * parsing, control transfers, bulk transfers, and endpoint management.
 * Provides USB device structure and interface definitions
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef CH375_HOST_H
#define CH375_HOST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ch375.h"
#include "usb.h"

// USB Control Setup Size
#define CONTROL_SETUP_SIZE 8
#define USB_MAX_ENDPOINTS 4
#define USB_MAX_INTERFACES 4

#define RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS 1000
#define TRANSFER_TIMEOUT 5000

#define USB_DEFAULT_ADDRESS 1
#define USB_DEFAULT_EP0_MAX_PACKSIZE 8

/**
 * @brief CH375 Host Error Codes
 */
typedef enum {
    CH375_HOST_SUCCESS          = 0,
    CH375_HOST_ERROR            = -1,
    CH375_HOST_PARAM_INVALID    = -2,
    CH375_HOST_TIMEOUT          = -3,
    CH375_HOST_DEV_DISCONNECT   = -4,
    CH375_HOST_STALL            = -5,
    CH375_HOST_IO_ERROR         = -6,
    CH375_HOST_NOT_SUPPORT      = -7,
    CH375_HOST_ALLOC_FAILED     = -8,
} ch375_HostErrNo_e;

// USB Request type
typedef enum {
    USB_RECIP_DEVICE    =   0x00,
    USB_RECIP_INTERFACE =   0x01,
    USB_RECIP_ENDPOINT  =   0x02,
    USB_DIR_IN          =   0x80,
    USB_DIR_OUT         =   0x00,
    USB_TYPE_STANDARD   =   0x00,
    USB_TYPE_CLASS      =   0x20,
    USB_TYPE_VENDOR     =   0x40
} USB_REQ_t;

#define USB_REQ_TYPE(dir, type, recip) ((dir) | (type) | (recip))
#define SETUP_IN(x) ((x) & 0x80)
#define EP_IN(x) ((x) & 0x80)

/**
 * @brief USB EP Structure
 */
struct USB_Endpoint_t {
    uint8_t ep_addr;
    uint8_t attributes;
    uint16_t max_packet;
    uint8_t interval;
    bool data_toggle;
};

/**
 * @brief USB Interface Stucture
 */
struct USB_Interface_t {
    uint8_t interface_number;
    uint8_t interface_class;
    uint8_t interface_subclass;
    uint8_t interface_protocol;
    uint8_t endpoint_count;
    struct USB_Endpoint_t endpoints[USB_MAX_ENDPOINTS];
};

/**
 * @brief USB Device Structure
 */
struct USB_Device_t {
    struct ch375_Context_t *ctx;

    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t speed;
    uint8_t ep0_max_packet;
    uint8_t config_value;

    struct usb_device_descriptor raw_dev_desc;
    uint8_t *raw_conf_desc;
    size_t  raw_conf_desc_len;
    
    uint8_t interface_count;
    struct USB_Interface_t interfaces[USB_MAX_INTERFACES];

    bool connected;
    bool configured; 
};

/**
 * @brief USB HID Descriptor
 */
struct USB_HID_Descriptor_t {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bClassDescriptorType;
    uint16_t wClassDescriptorLength;
} __packed;

/**
 * @brief Function prototypes
 */
int ch375_hostInit(struct ch375_Context_t *pCtx, uint32_t baudrate);
int ch375_hostWaitDeviceConnect(struct ch375_Context_t *pCtx, uint32_t timeout);
int ch375_hostUdevOpen(struct ch375_Context_t *pCtx, struct USB_Device_t *pUdev);
void ch375_hostUdevClose(struct USB_Device_t *pUdev);
int ch375_hostResetDev(struct USB_Device_t *pUdev);

/**
 * @brief Transfer functions
 */
int ch375_hostControlTransfer(struct USB_Device_t *pUdev, uint8_t reqType, uint8_t bRequest, 
    uint16_t wValue, uint16_t wIndex, uint8_t *pData, uint16_t wLength, int *pActualLen, uint32_t timeout);
int ch375_hostBulkTransfer(struct USB_Device_t *pUdev, uint8_t ep, uint8_t *pData, int len, 
                                                                        int *pActualLen, uint32_t timeout);
int ch375_hostClearStall(struct USB_Device_t *pUdev, uint8_t ep);
int ch375_hostSetConfiguration(struct USB_Device_t *pUdev, uint8_t config);

#ifdef __cplusplus
}
#endif

#endif /* CH375_HOST_H */