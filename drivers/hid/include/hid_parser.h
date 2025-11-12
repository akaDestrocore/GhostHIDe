/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_parser.h
 * @brief          USB HID report descriptor parser interface
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Parses USB HID report descriptors to extract button, axis, and other
 * input field definitions. Identifies mouse, keyboard, and other HID
 * device types. Manages report buffers and data access.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef HID_PARSER_H
#define HID_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "ch375_host.h"

#define HID_ITEM_TAG_LONG 15
#define USB_CLASS_HID 0x03

/**
 * @brief USBHID Device Types
 */
enum usbHid_Type_e {
    USBHID_TYPE_NONE = 0,
    USBHID_TYPE_MOUSE,
    USBHID_TYPE_KEYBOARD,
    USBHID_TYPE_JOYSTICK,
};

/**
 * @brief HID Mouse Axis Definitions
 */
typedef enum {
    USBHID_SUCCESS          = 0,
    USBHID_ERROR            = -1,
    USBHID_PARAM_INVALID    = -2,
    USBHID_NO_DEV           = -3,
    USBHID_IO_ERROR         = -4,
    USBHID_NOT_SUPPORT      = -5,
    USBHID_NOT_HID_DEV      = -6,
    USBHID_BUFFER_NOT_ALLOC = -7,
    USBHID_ALLOC_FAILED     = -8
} usbHid_ErrNo_e;

/**
 * @brief USBHID Device Structure
 */
struct USBHID_Device_t {
    struct USB_Device_t *pUdev;
    uint8_t interface_num;
    uint8_t endpoint_in;
    struct USB_Endpoint_t *endpoint;
    
    struct USB_HID_Descriptor_t *hid_desc;
    uint8_t *raw_hid_report_desc;
    uint16_t raw_hid_report_desc_len;

    enum usbHid_Type_e hid_type;

    uint8_t *report_buffer;
    uint32_t report_len;
    uint32_t report_buff_len;
    uint32_t report_buffer_last_offset;
};

/**
 * @brief HID Report Item Format
 */
typedef enum {
    HID_ITEM_FORMAT_SHORT,
    HID_ITEM_FORMAT_LONG
} HID_ItemFormat_e;

/**
 * @brief HID Request Codes
 */
typedef enum {
    HID_GET_REPORT      = 0x01,
    HID_GET_IDLE        = 0x02,
    HID_GET_PROTOCOL    = 0x03,
    HID_SET_REPORT      = 0x09,
    HID_SET_IDLE        = 0x0A,
    HID_SET_PROTOCOL    = 0x0B    
} HID_ReqCode_e;

/**
 * @brief HID Report Types
 */
typedef enum {
    HID_REPORT_TYPE_INPUT   = 0x01,
    HID_REPORT_TYPE_OUTPUT  = 0x02,
    HID_REPORT_TYPE_FEATURE = 0x03
} HID_ReportType_e;

/**
 * @brief HID Main Item Tags
 */
typedef enum {
    HID_MAIN_ITEM_TAG_INPUT             = 8,
    HID_MAIN_ITEM_TAG_OUTPUT            = 9,
    HID_MAIN_ITEM_TAG_FEATURE           = 11,
    HID_MAIN_ITEM_TAG_BEGIN_COLLECTION  = 10,
    HID_MAIN_ITEM_TAG_END_COLLECTION    = 12
} HID_MainItemTag_e;

/**
 * @brief HID Global Item Tags
 */
typedef enum {
    HID_GLOBAL_ITEM_TAG_USAGE_PAGE       =  0,
    HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM  =  1,
    HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM  =  2,
    HID_GLOBAL_ITEM_TAG_PHYSICAL_MINIMUM =  3,
    HID_GLOBAL_ITEM_TAG_PHYSICAL_MAXIMUM =  4,
    HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT    =  5,
    HID_GLOBAL_ITEM_TAG_UNIT             =  6,
    HID_GLOBAL_ITEM_TAG_REPORT_SIZE      =  7,
    HID_GLOBAL_ITEM_TAG_REPORT_ID        =  8,
    HID_GLOBAL_ITEM_TAG_REPORT_COUNT     =  9,
    HID_GLOBAL_ITEM_TAG_PUSH             =  10,
    HID_GLOBAL_ITEM_TAG_POP              =  11
} HID_GlobalItemTag_e;

/**
 * @brief HID Local Item Tags
 */
typedef enum {
    HID_LOCAL_ITEM_TAG_USAGE            = 0,
    HID_LOCAL_ITEM_TAG_USAGE_MINIMUM    = 1,
    HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM    = 2
} HID_LocalItemTag_e;

/* HID Usage Pages */
#define HID_UP_GENDESK          0x00010000
#define HID_UP_KEYBOARD         0x00070000
#define HID_UP_BUTTON           0x00090000

/* HID Usage IDs */
#define HID_GD_POINTER          0x00010001
#define HID_GD_MOUSE            0x00010002
#define HID_GD_KEYBOARD         0x00010006
#define HID_GD_X                0x00010030
#define HID_GD_Y                0x00010031
#define HID_GD_Z                0x00010032
#define HID_GD_WHEEL            0x00010038

/**
 * @brief HID Item Structure
 */
struct HID_Item_t {
    uint8_t format;
    uint8_t size;
    uint8_t type;
    uint8_t tag;
    union {
        uint8_t u8;
        int8_t s8;
        uint16_t u16;
        int16_t s16;
        uint32_t u32;
        int32_t s32;
        uint8_t *longdata;
    } data;
};

/**
 * @brief HID Data Descriptor
 */
struct HID_DataDescriptor_t {
    int32_t physical_minimum;
    int32_t physical_maximum;
    int32_t logical_minimum;
    int32_t logical_maximum;
    uint32_t size;
    uint32_t count;
    uint32_t report_buf_off;
};

/**
 * @brief Parsing functions
 */
uint8_t *HID_fetchItem(uint8_t *pStart, uint8_t *pEnd, struct HID_Item_t *pItem);
int HID_parseReportDescriptor(uint8_t *pReport, uint16_t len, uint8_t *pType);

/**
 * @brief USB HID Core Functions
 */
int USBHID_open(struct USB_Device_t *pUdev, uint8_t interface_num,
               struct USBHID_Device_t *pDev);
void USBHID_close(struct USBHID_Device_t *pDev);
void USBHID_freeReportBuffer(struct USBHID_Device_t *pDev);
int USBHID_fetchReport(struct USBHID_Device_t *pDev);
int USBHID_getReportBuffer(struct USBHID_Device_t *pDev, uint8_t **ppBuff,
                            uint32_t *pLen, bool isLast);
int USBHID_allocReportBuffer(struct USBHID_Device_t *pDev, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* HID_PARSER_H */