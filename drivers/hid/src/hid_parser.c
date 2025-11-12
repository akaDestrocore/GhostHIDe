/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           hid_parser.c
 * @brief          USB HID report descriptor parser implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements HID item fetching, report descriptor parsing, device type
 * detection (mouse/keyboard), and report buffer management. Handles
 * GET_DESCRIPTOR, SET_IDLE, and other HID-specific control requests.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "hid_parser.h"

LOG_MODULE_REGISTER(hid_parser, LOG_LEVEL_DBG);

/* Private function prototypes -----------------------------------------------*/
static int get_hid_descriptor(struct USB_Device_t *pUdev, uint8_t interfaceNum, 
                                        struct USB_HID_Descriptor_t **ppHID_Desc);
static void set_idle(struct USB_Device_t *pUdev, uint8_t interfaceNum, 
                                            uint8_t duration, uint8_t reportID);
static int get_ep_in(struct USB_Device_t *pUdev, uint8_t interfaceNum, uint8_t *pEP);
static int hid_get_class_descriptor(struct USB_Device_t *pUdev, uint8_t interfaceNum, 
                                        uint8_t type, uint8_t *pBuff, uint16_t len);
static int set_report(struct USB_Device_t *udev, uint8_t interfaceNum, 
                                            uint8_t reportType, uint8_t reportID);
static inline uint8_t *get_report_buffer(struct USBHID_Device_t *pDev, bool isLast);
static int usbhid_read(struct USBHID_Device_t *pDev, uint8_t *pBuff, int len, int *pActualLen);

/**
 * @brief Fetch a single HID report descriptor item from a buffer.
 * @param pStart Pointer to the start of the input buffer (pointer to the first byte to parse).
 * @param pEnd Pointer one past the last valid byte in the input buffer (exclusive end).
 * @param pItem  Pointer to the HID_Item_t structure to be filled.
 * @return Pointer to the next unread byte after the parsed item on success and NULL pointer
 * on error.
 */
uint8_t *HID_fetchItem(uint8_t *pStart, uint8_t *pEnd, struct HID_Item_t *pItem) {

    uint8_t firstByte;
    uint16_t u16Val;
    uint32_t u32Val;

    if (NULL == pStart || NULL == pEnd || NULL == pItem) {
        return NULL;
    }

    if ((pEnd - pStart) <= 0) {
        return NULL;
    }

    firstByte = *pStart++;
    pItem->type = (firstByte >> 2) & 0x03;
    pItem->tag = (firstByte >> 4) & 0x0F;
    pItem->data.u32 = 0;

    if (HID_ITEM_TAG_LONG == pItem->tag) {
        if ((pEnd - pStart) < 2) {
            return NULL;
        }

        pItem->format = HID_ITEM_FORMAT_LONG;
        pItem->size = *pStart++;
        pItem->tag = *pStart++;

        if ((pEnd - pStart) < pItem->size) {
            return NULL;
        }

        pItem->data.longdata = pStart;
        pStart += pItem->size;
        return pStart;
    }

    pItem->format = HID_ITEM_FORMAT_SHORT;
    pItem->size = firstByte & 0x03;

    switch (pItem->size) {
        case 0: {
            pItem->data.u32 = 0;
            return pStart;
        }

        case 1: {
            if ((pEnd - pStart) < 1) {
                return NULL;
            }

            pItem->data.u8 = *pStart++;
            return pStart;
        }

        case 2: {
            if ((pEnd - pStart) < 2) {
                return NULL;
            }

            memcpy(&u16Val, pStart, sizeof(u16Val));
            pItem->data.u16 = sys_le16_to_cpu(u16Val);
            pStart += 2;
            return pStart;
        }

        case 3: {
            if ((pEnd - pStart) < 4) {
                return NULL;
            }

            memcpy(&u32Val, pStart, sizeof(u32Val));
            pItem->data.u32 = sys_le32_to_cpu(u32Val);
            pStart += 4;
            pItem->size = 4;
            return pStart;
        }
    }

    return NULL;
}

/**
 * @brief Parse a report descriptor and return the type of the report descriptor.
 * @param pReport pointer to the report descriptor
 * @param len length of the report descriptor
 * @param pType pointer to the type handle of the report
 * @return 0 on success, error code otherwise
 * on error.
 */
int HID_parseReportDescriptor(uint8_t *pReport, uint16_t len, uint8_t *pType)
{
    struct HID_Item_t item = {0};
    uint8_t *pEnd = pReport + len;
    uint8_t *pCur = pReport;
    uint32_t usagePage = 0;
    uint32_t usage = 0;
    bool deviceTypeFound = false;
    
    if (NULL == pReport || NULL == pType || len < 2) {
        return -EINVAL;
    }
    
    *pType = 0;
    
    // Iterate through the entire descriptor
    while (pCur < pEnd && true != deviceTypeFound) {
        pCur = HID_fetchItem(pCur, pEnd, &item);
        if (!pCur) break;
        
        if (HID_ITEM_TYPE_GLOBAL == item.type && HID_GLOBAL_ITEM_TAG_USAGE_PAGE == item.tag) {
            usagePage = item.data.u32 << 16;
        }
        else if (HID_ITEM_TYPE_LOCAL == item.type && HID_LOCAL_ITEM_TAG_USAGE == item.tag) {
            // Store usage
            usage = usagePage | item.data.u32;
        }
        else if (HID_ITEM_TYPE_MAIN == item.type && HID_MAIN_ITEM_TAG_BEGIN_COLLECTION == item.tag) {
            // Check if this is a standard HID app collection
            if (HID_GD_MOUSE == usage) {
                *pType = USBHID_TYPE_MOUSE;
                deviceTypeFound = true;
                LOG_INF("Detected HID Mouse (usage=0x%08X)", usage);
            }
            else if (HID_GD_KEYBOARD == usage) {
                *pType = USBHID_TYPE_KEYBOARD;
                deviceTypeFound = true;
                LOG_INF("Detected HID Keyboard (usage=0x%08X)", usage);
            }
        }
    }
    
    if (true != deviceTypeFound) {
        pCur = pReport;
        bool hasInput = false;
        bool hasOutput = false;
        int inputCount = 0;
        
        while (pCur < pEnd) {
            pCur = HID_fetchItem(pCur, pEnd, &item);
            if (!pCur) break;
            
            if (HID_ITEM_TYPE_MAIN == item.type) {
                if (HID_MAIN_ITEM_TAG_INPUT == item.tag) {
                    hasInput = true;
                    inputCount++;
                } else if (HID_MAIN_ITEM_TAG_OUTPUT == item.tag) {
                    hasOutput = true;
                }
            }
        }
        
        if (true == hasInput && true == hasOutput) {
            *pType = USBHID_TYPE_KEYBOARD;
            LOG_INF("Detected HID Device - likely a keyboard");
            return 0;
        }
        else if (true == hasInput) {
            *pType = USBHID_TYPE_MOUSE;
            LOG_INF("Detected HID device - likely a mouse");
            return 0;
        }
        
        LOG_ERR("Unknown HID device type");
        return -ENOTSUP;
    }
    
    return 0;
}

/**
 * @brief Open a HID device
 * @param pUdev Pointer to the USB device
 * @param interface_num Interface number of the HID device
 * @param pDev Pointer to the USBHID structure
 * @return 0 on success, error code otherwise
 */
int USBHID_open(struct USB_Device_t *pUdev, uint8_t interface_num, struct USBHID_Device_t *pDev) {

    int ret = -1;
    struct USB_HID_Descriptor_t *pHID_Desc = NULL;
    uint8_t *pRawHIDReportDesc = NULL;
    uint16_t rawHIDReportDescLen = 0;
    struct HID_Item_t item;
    uint8_t hidType = USBHID_TYPE_NONE;
    uint8_t epIN;
    uint8_t *pCur;
    uint8_t *pEnd;
    
    ret = get_hid_descriptor(pUdev, interface_num, &pHID_Desc);
    if ( ret < 0) {
        LOG_ERR("Cannot find HID descriptor for interface %d", interface_num);
        return USBHID_NOT_HID_DEV;
    }

    LOG_INF("HID descriptor found: version=0x%04X, country=0x%02X", 
                sys_le16_to_cpu(pHID_Desc->bcdHID), pHID_Desc->bCountryCode);
    
    if ( pHID_Desc->bNumDescriptors > 1) {
        LOG_ERR("Multiple descriptors not supported: %d", pHID_Desc->bNumDescriptors);
        return USBHID_NOT_SUPPORT;
    }

    pCur = pRawHIDReportDesc;
    pEnd = pCur + rawHIDReportDescLen;
    memset(&item, 0x00, sizeof(struct HID_Item_t));
    
    while (pCur < pEnd) {
        pCur = HID_fetchItem(pCur, pEnd, &item);
        if (NULL == pCur) {
            break;
        }
        
        if (HID_ITEM_TYPE_GLOBAL == item.type && HID_GLOBAL_ITEM_TAG_REPORT_ID == item.tag) {
            break;
        }
    }

    memset(pDev, 0x00, sizeof(struct USBHID_Device_t));
    set_idle(pUdev, interface_num, 0, 0);
    
    rawHIDReportDescLen = sys_le16_to_cpu(pHID_Desc->wClassDescriptorLength);
    pRawHIDReportDesc = k_malloc(rawHIDReportDescLen);
    if (NULL == pRawHIDReportDesc) {
        LOG_ERR("Failed to allocate HID report buffer (len=%d)", rawHIDReportDescLen);
        return USBHID_ALLOC_FAILED;
    }
    memset(pRawHIDReportDesc, 0x00, rawHIDReportDescLen);

    ret = get_ep_in(pUdev, interface_num, &epIN);
    if (ret < 0) {
        LOG_ERR("Get endpoint failed for interface %d", interface_num);
        k_free(pRawHIDReportDesc);
        return USBHID_NOT_SUPPORT;
    }

    // Cache the endpoint struct pointer
    struct USB_Endpoint_t *cachedEP = NULL;
    if (interface_num < pUdev->interface_count) {
        struct USB_Interface_t *iface = &pUdev->interfaces[interface_num];
        if (iface->endpoint_count > 0) {
            cachedEP = &iface->endpoints[0];
            LOG_INF("Cached endpoint: ep_addr=0x%02X max_packet=%d", cachedEP->ep_addr, cachedEP->max_packet);
        }
    }

    if (NULL == cachedEP) {
        LOG_ERR("Failed to cache endpoint pointer");
        k_free(pRawHIDReportDesc);
        return USBHID_ERROR;
    }

    // Get HID report descriptor
    ret = hid_get_class_descriptor(pUdev, interface_num, 0x22, pRawHIDReportDesc, rawHIDReportDescLen);
    if (ret < 0) {
        LOG_ERR("Parse HID report failed");
        k_free(pRawHIDReportDesc);
        return USBHID_NOT_SUPPORT;
    }

    // Parse report descriptor to determine device type
    ret = HID_parseReportDescriptor(pRawHIDReportDesc, rawHIDReportDescLen, &hidType);
    if (ret < 0) {
        LOG_WRN("Failed to parse report descriptor, trying interface protocol fallback");
        hidType = USBHID_TYPE_NONE;
    }

    // Fallback to interface protocol if parsing failed
    if (USBHID_TYPE_NONE == hidType && interface_num < pUdev->interface_count) {
        uint8_t protocol = pUdev->interfaces[interface_num].interface_protocol;
        if (1 == protocol) {
            hidType = USBHID_TYPE_KEYBOARD;
            LOG_INF("Detected KEYBOARD by interface protocol");
        } else if (2 == protocol) {
            hidType = USBHID_TYPE_MOUSE;
            LOG_INF("Detected MOUSE by interface protocol");
        }
    }

    if (USBHID_TYPE_KEYBOARD == hidType) {
        ret = set_report(pUdev, interface_num, HID_REPORT_TYPE_OUTPUT, 0);
        if (USBHID_SUCCESS != ret) {
            LOG_ERR("Set report failed");
            k_free(pRawHIDReportDesc);
            return USBHID_IO_ERROR;
        }
    }

    pDev->pUdev = pUdev;
    pDev->interface_num = interface_num;
    pDev->endpoint_in = epIN;
    pDev->raw_hid_report_desc = pRawHIDReportDesc;
    pDev->raw_hid_report_desc_len = rawHIDReportDescLen;
    pDev->hid_desc = pHID_Desc;
    pDev->hid_type = hidType;
    pDev->endpoint = cachedEP;

    return USBHID_SUCCESS;
}

/**
 * @brief Close the HID device
 * @param pDev Pointer to the HID device
 * @return 0 on success, error code otherwise
 */
void USBHID_close(struct USBHID_Device_t *pDev) {

    if (NULL == pDev) {
        return;
    }

    if (NULL != pDev->raw_hid_report_desc) {
        k_free(pDev->raw_hid_report_desc);
        pDev->raw_hid_report_desc = NULL;
    }

    USBHID_freeReportBuffer(pDev);
    memset(pDev, 0x00, sizeof(struct USBHID_Device_t));
}

/**
 * @brief Free the report buffer
 * @param pDev Pointer to the HID device
 * @return 0 on success, error code otherwise
 */
void USBHID_freeReportBuffer(struct USBHID_Device_t *pDev) {

    if (NULL == pDev) {
        return;
    }

    if (NULL != pDev->report_buffer) {
        k_free(pDev->report_buffer);
        pDev->report_buffer = NULL;
    }

    pDev->report_len = 0;
    pDev->report_buff_len = 0;
    pDev->report_buffer_last_offset = 0;
}

int USBHID_fetchReport(struct USBHID_Device_t *pDev) {

    int ret = -1;
    uint8_t *pLastReportBuff;
    int actualLen = 0;

    if (NULL == pDev) {
        return USBHID_PARAM_INVALID;
    }

    pLastReportBuff = get_report_buffer(pDev, true);
    if (NULL == pLastReportBuff) {
        LOG_ERR("Report buffer not allocated");
        return USBHID_BUFFER_NOT_ALLOC;
    }

    ret = usbhid_read(pDev, pLastReportBuff, pDev->report_len, &actualLen);

    if (USBHID_SUCCESS == ret) {
        if (0 != pDev->report_buffer_last_offset) {
            pDev->report_buffer_last_offset = 0;
        } else {
            pDev->report_buffer_last_offset = pDev->report_len;
        }

        return USBHID_SUCCESS;
    }

    if (-EAGAIN == ret) {
        return -EAGAIN;
    }

    // For other possible return codes return as is
    return ret;
}

/**
 * @brief Get the report buffer
 * @param pDev Pointer to the device
 * @param ppBuff Pointer to the report buffer
 * @param pLen Pointer to the report length
 * @param isLast True for last report
 * @return 0 on success, error code otherwise
 */
int USBHID_getReportBuffer(struct USBHID_Device_t *pDev, uint8_t **ppBuff, 
                                            uint32_t *pLen, bool isLast) {
    
    uint8_t *pReportBuf;

    if (NULL == pDev || NULL == ppBuff) {
        return USBHID_PARAM_INVALID;
    }

    pReportBuf = get_report_buffer(pDev, isLast);
    if (NULL == pReportBuf) {
        LOG_ERR("Report buffer not allocated");
        return USBHID_BUFFER_NOT_ALLOC;
    }

    if (NULL != ppBuff) {
        *ppBuff = pReportBuf;
    }

    if (NULL != pLen) {
        *pLen = pDev->report_len;
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Allocate memory for report buffer
 * @param pDev Pointer to device
 * @param len Length of the report
 * @return 0 on success, error code otherwise
 */
int USBHID_allocReportBuffer(struct USBHID_Device_t *pDev, uint32_t len) {

    if (NULL == pDev) {
        LOG_ERR("Invalid device pointer");
        return USBHID_PARAM_INVALID;
    }

    uint8_t *pBuff;
    uint32_t buffLen= 0;

    if (NULL != pDev->report_buffer) {
        LOG_ERR("Report buffer already allocated!");
        return USBHID_ERROR;
    }

    // Twice the size of original buffer
    buffLen = len * 2;
    pBuff = k_malloc(buffLen);

    if (NULL == pBuff) {
        LOG_ERR("Failed to allocate report buffer (size=%d)", buffLen);
        return USBHID_ALLOC_FAILED;
    }

    memset(pBuff, 0x00, buffLen);

    pDev->report_len = len;
    pDev->report_buffer = pBuff;
    pDev->report_buff_len = buffLen;
    pDev->report_buffer_last_offset = 0;

    return USBHID_SUCCESS;
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static int get_hid_descriptor(struct USB_Device_t *pUdev, uint8_t interfaceNum, 
                                        struct USB_HID_Descriptor_t **ppHID_Desc) {
    
    if (NULL == pUdev || NULL == pUdev->raw_conf_desc) {
        return -1;
    }

    struct usb_desc_header *pDesc = (struct usb_desc_header *)pUdev->raw_conf_desc;
    void *pRawConfDescEnd = (uint8_t *)pUdev->raw_conf_desc + pUdev->raw_conf_desc_len;
    uint8_t curInterfaceNum = 0;

    while ((void *)pDesc < pRawConfDescEnd) {
        if (0 == pDesc->bLength) {
            LOG_ERR("Descriptor with zero length encountered");
            return -1;
        }

        switch (pDesc->bDescriptorType) {
            
            case USB_DESC_INTERFACE: {
                curInterfaceNum = ((struct usb_if_descriptor *)pDesc)->bInterfaceNumber;
                break;
            }

            case USB_DESC_HID: {
                if (curInterfaceNum == interfaceNum) {
                    *ppHID_Desc = (struct USB_HID_Descriptor_t *)pDesc;
                    return 0;
                }

                break;
            }

            default: {
                break;
            }

        }

        pDesc = (struct usb_desc_header *)((uint8_t *)pDesc + pDesc->bLength);
    }

    return -1;
}

static void set_idle(struct USB_Device_t *pUdev, uint8_t interfaceNum, uint8_t duration, uint8_t reportID) {

    int ret = -1;

    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_OUT, 
        USB_TYPE_CLASS, USB_RECIP_INTERFACE), HID_SET_IDLE, (duration << 8) | 
        reportID, interfaceNum, NULL, 0, NULL, TRANSFER_TIMEOUT);

    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set idle failed: %d", ret);
    }
}

static int get_ep_in(struct USB_Device_t *pUdev, uint8_t interfaceNum, uint8_t *pEP) {
    
    if (interfaceNum >= pUdev->interface_count) {
        return -1;
    }

    struct USB_Interface_t *iface = &pUdev->interfaces[interfaceNum];

    if (iface->endpoint_count < 1) {
        LOG_ERR("Interface %d has no endpoints", interfaceNum);
        return -1;
    }

    *pEP = iface->endpoints[0].ep_addr;
    return 0;
}

static int hid_get_class_descriptor(struct USB_Device_t *pUdev, uint8_t interfaceNum, 
                                    uint8_t type, uint8_t *pBuff, uint16_t len) {
    
    int ret = -1;
    int actualLen = 0;
    
    if (len > 64) {
        actualLen = 0;
        ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_INTERFACE), 
                USB_SREQ_GET_DESCRIPTOR, (type << 8) | 0, interfaceNum, pBuff, 64, &actualLen, TRANSFER_TIMEOUT);
        
        if (CH375_HOST_SUCCESS == ret && actualLen > 0) {
            LOG_INF("Initial read got %d bytes", actualLen);
            
            // If we got less than requested then - that's all 
            if (actualLen < 64) {
                LOG_INF(" Complete descriptor received: %d bytes", actualLen);
                return USBHID_SUCCESS;
            }
            
            // Try to get the rest
            if (len > actualLen) {
                int remainingLen = len - actualLen;
                int additionalLen = 0;
                
                LOG_DBG("Attempting to read remaining %d bytes", remainingLen);
                ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_INTERFACE),
                USB_SREQ_GET_DESCRIPTOR, (type << 8) | 0, interfaceNum, pBuff + actualLen, remainingLen, 
                                                                                    &additionalLen, TRANSFER_TIMEOUT);
                
                if (CH375_HOST_SUCCESS == ret && additionalLen > 0) {
                    LOG_INF(" Got additional %d bytes, total %d", additionalLen, actualLen + additionalLen);
                    return USBHID_SUCCESS;
                }
            }
            
            // Any data is success
            LOG_INF(" Partial descriptor: %d bytes", actualLen);
            return USBHID_SUCCESS;
        }
    }
    
    actualLen = 0;
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_INTERFACE), 
            USB_SREQ_GET_DESCRIPTOR, (type << 8) | 0, interfaceNum, pBuff, len, &actualLen, TRANSFER_TIMEOUT);
    
    if (CH375_HOST_SUCCESS == ret && actualLen > 0) {
        LOG_INF(" STANDARD/INTERFACE succeeded: %d bytes", actualLen);
        return USBHID_SUCCESS;
    }
    LOG_DBG(" STANDARD/INTERFACE failed: ret=%d actual=%d", ret, actualLen);
    
    actualLen = 0;
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_CLASS, USB_RECIP_INTERFACE), 
    USB_SREQ_GET_DESCRIPTOR, (type << 8) | 0, interfaceNum, pBuff, len, &actualLen, TRANSFER_TIMEOUT);
    
    if (CH375_HOST_SUCCESS == ret && actualLen > 0) {
        return USBHID_SUCCESS;
    }
    
    actualLen = 0;
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_CLASS, USB_RECIP_INTERFACE),0x06, 
                                (0x22 << 8) | 0, interfaceNum, pBuff, len, &actualLen, TRANSFER_TIMEOUT);
    
    if (CH375_HOST_SUCCESS == ret && actualLen > 0) {
        LOG_INF(" Explicit request succeeded: %d bytes", actualLen);
        return USBHID_SUCCESS;
    }
    
    actualLen = 0;
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_CLASS, USB_RECIP_INTERFACE), HID_GET_REPORT, 
                (HID_REPORT_TYPE_INPUT << 8) | 0, interfaceNum, pBuff, len > 64 ? 64 : len, &actualLen, TRANSFER_TIMEOUT);
    
    if (CH375_HOST_SUCCESS == ret && actualLen > 0) {
        LOG_INF(" GET_REPORT succeeded: %d bytes", actualLen);
        return USBHID_SUCCESS;
    }
    
    LOG_ERR("All descriptor retrieval methods failed for interface %d", interfaceNum);
    return USBHID_ERROR;
}

static int set_report(struct USB_Device_t *pUdev, uint8_t interfaceNum, uint8_t reportType, 
                                                                        uint8_t reportID) {
    
    int ret = -1;
    int actualLen = 0;
    uint8_t dataFragment = 0x00;

    // 0x21 | CLASS | INTERFACE
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_CLASS, USB_RECIP_INTERFACE),
    HID_SET_REPORT, (reportType << 8) | reportID, interfaceNum, &dataFragment, sizeof(dataFragment), 
                                                                            &actualLen, TRANSFER_TIMEOUT);
        
    if (CH375_HOST_SUCCESS != ret) {
        LOG_WRN("Set report failed (this may be normal for some devices): %d", ret);
        return USBHID_SUCCESS;
    }

    return USBHID_SUCCESS;
}

static inline uint8_t *get_report_buffer(struct USBHID_Device_t *pDev, bool isLast) {
    
    if (NULL == pDev || NULL == pDev->report_buffer) {
        return NULL;
    }

    if (true == isLast) {
        return pDev->report_buffer + pDev->report_buffer_last_offset;
    } else {
        uint32_t offset = pDev->report_buffer_last_offset ? 0 : pDev->report_len;
        return pDev->report_buffer + offset;
    }
}

static int usbhid_read(struct USBHID_Device_t *pDev, uint8_t *pBuff, int len, int *pActualLen) {

    int ret = -1;
    struct USB_Device_t *pUdev = pDev->pUdev;
    struct ch375_Context_t *pCtx = pUdev->ctx;
    struct USB_Endpoint_t *pEP = pDev->endpoint;
    
    uint8_t status;

    if (NULL == pEP) {
        LOG_ERR("No cached endpoint!");
        return USBHID_ERROR;
    }
    
    // Set retry mode for INT transfers
    ret = ch375_setRetry(pCtx, CH375_RETRY_TIMES_ZERO);
    if (CH375_SUCCESS != ret) {
        return USBHID_IO_ERROR;
    }


    // Send IN token
    ret = ch375_sendToken(pCtx, pEP->ep_addr, pEP->data_toggle, USB_PID_IN, &status);
    if (CH375_SUCCESS != ret) {
        return USBHID_IO_ERROR;
    }

    // Check status
    if (CH375_USB_INT_SUCCESS == status) {
        uint8_t readLen;
        ret = ch375_readBlockData(pCtx, pBuff, len, &readLen);
        if (CH375_SUCCESS != ret) {
            return USBHID_IO_ERROR;
        }
        
        pEP->data_toggle = !pEP->data_toggle;

        if (NULL != pActualLen) {
            *pActualLen = readLen;
        }

        return USBHID_SUCCESS;
    }

    if (CH375_PID2STATUS(USB_PID_NAK) == status) {
        if(NULL != pActualLen) {
            *pActualLen = 0;
        }
        return -EAGAIN;
    }

    if (CH375_USB_INT_DISCONNECT == status) {
        return USBHID_NO_DEV;
    }

    return USBHID_IO_ERROR;
}