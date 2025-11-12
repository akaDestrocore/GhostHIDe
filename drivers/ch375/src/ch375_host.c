/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           ch375_host.c
 * @brief          CH375 USB host layer implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements USB host protocol stack including device reset, address
 * assignment, configuration, control transfers with SETUP/DATA/STATUS
 * stages, and bulk transfers with NAK handling and retry logic.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "ch375_host.h"

LOG_MODULE_REGISTER(ch375_host, LOG_LEVEL_DBG);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static int set_dev_address(struct USB_Device_t *pUdev, uint8_t addr);
static int get_config_descriptor(struct USB_Device_t *pUdev, uint8_t *pBuff, uint16_t len);
static int parse_config_descriptor(struct USB_Device_t *pUdev);
static void parse_interface_descriptor(struct USB_Device_t *pUdev, struct usb_if_descriptor *pDesc);
static void parse_endpoint_descriptor(struct USB_Interface_t *pIfc, struct usb_ep_descriptor *pDesc);
static int reset_dev(struct ch375_Context_t *pCtx);
static int get_endpoint(struct USB_Device_t *pUdev, uint8_t epAddr, struct USB_Endpoint_t **ppEP);

/**
  * @brief Initialize the CH375 in host mode
  * @param pCtx Pointer to the context
  * @param baudrate Baud rate to communicate with baudrate
  * @retval 0 on success, error code otherwise
  */
int ch375_hostInit(struct ch375_Context_t *pCtx, uint32_t baudrate) {
    
    int ret = -1;

    if ( 9600 != baudrate && 115200 != baudrate) {
        LOG_ERR("Invalid baudrate value: %" PRIu32 "", baudrate);
        return CH375_HOST_PARAM_INVALID;
    }

    ret = ch375_checkExist(pCtx);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("CH375 doesn't exist: %d", ret);
        return CH375_HOST_ERROR;
    }

    ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_SOF_AUTO);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Set USB mode failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    LOG_INF("Set USB mode to Host with SOF");

    k_msleep(20);

    ret = ch375_setBaudrate(pCtx, baudrate);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Set baudrate failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    k_msleep(1);
    return CH375_HOST_SUCCESS;
}

/**
  * @brief Try to connect to the CH375
  * @param pCtx Pointer to the context
  * @param timeout Timeout in milliseconds
  * @retval 0 on success, error code otherwise
  */
int ch375_hostWaitDeviceConnect(struct ch375_Context_t *pCtx, uint32_t timeout) {

    int ret = -1;
    uint8_t conn_status;
    uint32_t cnt;

    for (cnt = 0; cnt < timeout; cnt++) {
        ret = ch375_testConnect(pCtx, &conn_status);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Test connect failed: %d", ret);
            return CH375_HOST_ERROR;
        }

        if (CH375_USB_INT_DISCONNECT != conn_status) {
            return CH375_HOST_SUCCESS;
        }
    }

    return CH375_HOST_TIMEOUT;
}

/**
  * @brief Open a CH375 device
  * @param pCtx Pointer to the context
  * @param pUdev Pointer to the device structure
  * @retval 0 on success, error code otherwise
  */
int ch375_hostUdevOpen(struct ch375_Context_t *pCtx, struct USB_Device_t *pUdev) {

    int ret = -1;
    int i = 0;
    uint8_t ep_cnt = 0;
    uint16_t conf_total_len = 0;
    struct usb_cfg_descriptor conf_desc = {0};
    
    if (NULL == pUdev) {
        LOG_ERR("Invalid device pointer");
        return CH375_HOST_PARAM_INVALID;
    }

    memset(pUdev, 0x00, sizeof(struct USB_Device_t));
    pUdev->ctx = pCtx;
    pUdev->ep0_max_packet = USB_DEFAULT_EP0_MAX_PACKSIZE;

    ret = ch375_hostResetDev(pUdev);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Reset device failed: %d", ret);
        return ret;
    }

    LOG_INF("Getting device descriptor");
    // get first 8 bytes to read bMaxPacketSize0
    ret = ch375_hostControlTransfer(pUdev, 
            USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
            USB_SREQ_GET_DESCRIPTOR,
            USB_DESC_DEVICE << 8, 0,
            (uint8_t *)&pUdev->raw_dev_desc, 8, NULL, TRANSFER_TIMEOUT);

    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Get device descriptor (8 bytes) failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    LOG_INF("Device descriptor: bMaxPacketSize0=%" PRIu32 ", VID:PID=%04X:%04X",
        pUdev->raw_dev_desc.bMaxPacketSize0,
        sys_le16_to_cpu(pUdev->raw_dev_desc.idVendor),
        sys_le16_to_cpu(pUdev->raw_dev_desc.idProduct));

    // Now we know the real max packet size
    pUdev->ep0_max_packet = pUdev->raw_dev_desc.bMaxPacketSize0;
    LOG_INF("EP0 max packet size = %d", pUdev->ep0_max_packet);

    // Get full device descriptor with correct packet size
    LOG_INF("Getting full device descriptor");
    ret = ch375_hostControlTransfer(pUdev,
        USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE),
        USB_SREQ_GET_DESCRIPTOR,
        USB_DESC_DEVICE << 8, 0,
        (uint8_t *)&pUdev->raw_dev_desc, sizeof(struct usb_device_descriptor), NULL, TRANSFER_TIMEOUT);
        
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Get device descriptor failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    pUdev->vendor_id = sys_le16_to_cpu(pUdev->raw_dev_desc.idVendor);
    pUdev->product_id = sys_le16_to_cpu(pUdev->raw_dev_desc.idProduct);

    LOG_INF("Device VID:PID = %04X:%04X", pUdev->vendor_id, pUdev->product_id);
    LOG_INF("EP0 max packet size = %d", pUdev->ep0_max_packet);

    LOG_INF("Setting device address");
    ret = set_dev_address(pUdev, USB_DEFAULT_ADDRESS);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set device address failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    LOG_INF("Getting config descriptor");
    ret = get_config_descriptor(pUdev, (uint8_t *)&conf_desc, sizeof(conf_desc));
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Get short config descriptor failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    conf_total_len = sys_le16_to_cpu(conf_desc.wTotalLength);
    pUdev->config_value = conf_desc.bConfigurationValue;
    pUdev->raw_conf_desc_len = conf_total_len;
    LOG_INF("Config total length = %d", conf_total_len);

    pUdev->raw_conf_desc = k_malloc(conf_total_len);
    if (NULL == pUdev->raw_conf_desc) {
        LOG_ERR("Allocate config descriptor buffer failed");
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    memset(pUdev->raw_conf_desc, 0x00, conf_total_len);

    ret = get_config_descriptor(pUdev, pUdev->raw_conf_desc, conf_total_len);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Get full config descriptor failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    ret = parse_config_descriptor(pUdev);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Parse config descriptor failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }

    LOG_INF("Short config: total_len=%" PRIu32 ", configuration_value=%" PRIu32 "",
        conf_total_len, conf_desc.bConfigurationValue);

    LOG_INF("Parsed config: interfaces=%d", pUdev->interface_count);
    for (int i = 0; i < pUdev->interface_count; ++i) {
        LOG_INF(" Interface %d: endpoints=%d class=0x%02X", i,
                pUdev->interfaces[i].endpoint_count, pUdev->interfaces[i].interface_class);
        for (int j = 0; j < pUdev->interfaces[i].endpoint_count; ++j) {
            LOG_INF("  EP[%d] addr=0x%02X attr=0x%02X maxpack=%d interval=%d tog=%d",
                    j,
                    pUdev->interfaces[i].endpoints[j].ep_addr,
                    pUdev->interfaces[i].endpoints[j].attributes,
                    pUdev->interfaces[i].endpoints[j].max_packet,
                    pUdev->interfaces[i].endpoints[j].interval,
                    pUdev->interfaces[i].endpoints[j].data_toggle);
        }
    }

    for (i = 0; i < pUdev->interface_count; i++) {
        ep_cnt += pUdev->interfaces[i].endpoint_count;
    }
    LOG_INF("Device has %d interfaces, %d endpoints", pUdev->interface_count, ep_cnt);
    
    ret = ch375_hostSetConfiguration(pUdev, pUdev->config_value);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set configuration failed: %d", ret);
        if (pUdev->raw_conf_desc) {
            k_free(pUdev->raw_conf_desc);
        }
        memset(pUdev, 0x00, sizeof(struct USB_Device_t));
        return CH375_HOST_ERROR;
    }
    
    LOG_INF("Set configuration %d success", pUdev->config_value);
    pUdev->connected = true;

    return CH375_HOST_SUCCESS;
}

/**
  * @brief Close the Ch376 device
  * @param pCtx Pointer to the context
  * @param pUdev Pointer to the device
  * @retval 0 on success, error code otherwise
  */
void ch375_hostUdevClose(struct USB_Device_t *pUdev) {

    if (NULL == pUdev) {
        return;
    }
    
    if (pUdev->raw_conf_desc) {
        k_free(pUdev->raw_conf_desc);
        pUdev->raw_conf_desc = NULL;
    }
    
    memset(pUdev, 0x00, sizeof(struct USB_Device_t));
}

/**
  * @brief Reset device connected to CH375
  * @param pUdev Pointer to the device
  * @retval 0 on success, error code otherwise
  */
int ch375_hostResetDev(struct USB_Device_t *pUdev) {
    
    int ret = -1;
    uint8_t conn_status;
    uint8_t speed;
    struct ch375_Context_t *pCtx;

    if (NULL == pUdev || NULL == pUdev->ctx) {
        LOG_ERR("Invalid device or context");
        return CH375_HOST_PARAM_INVALID;
    }

    pCtx = pUdev->ctx;
    pUdev->configured = false;
    
    ret = ch375_testConnect(pCtx, &conn_status);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Device connection failed: %d", ret);
        pUdev->connected = false;
        pUdev->configured = false;
        return CH375_HOST_ERROR;
    }

    if (CH375_USB_INT_DISCONNECT == conn_status) {
        LOG_ERR("Device disconnected");
        pUdev->connected = false;
        pUdev->configured = false;
        return CH375_HOST_DEV_DISCONNECT;
    }

    ret = ch375_getDevSpeed(pCtx, &speed);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Failed obtaining device speed info: %d", ret);
        pUdev->connected = false;
        pUdev->configured = false;
        return CH375_HOST_ERROR;
    }

    pUdev->speed = speed;
    switch(speed) {
        case USB_SPEED_SPEED_LS: {
            LOG_INF("Device speed: LOW");
            break;
        }

        case USB_SPEED_SPEED_FS: {
            LOG_INF("Device speed: FULL");
            break;
        }

        case USB_SPEED_UNKNOWN:
        default: {
            LOG_ERR("Unknown device speed");
            break;
        }
    }

    ret = reset_dev(pCtx);
    if ( CH375_HOST_DEV_DISCONNECT == ret) {
        pUdev->connected = false;
        pUdev->configured = false;
        return CH375_HOST_DEV_DISCONNECT;
    }

    if ( USB_SPEED_SPEED_LS == speed) {
        ret = ch375_setDevSpeed(pCtx, speed);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Failed to set device speed: %d", ret);
            pUdev->connected = false;
            pUdev->configured = false;
            return CH375_HOST_DEV_DISCONNECT;
        }
    }

    pUdev->connected = true;
    pUdev->configured = true;
    return CH375_HOST_SUCCESS;
}

/* --------------------------------------------------------------------------
 * Transfer functions
 * -------------------------------------------------------------------------*/
/**
  * @brief USB control transfer function
  * @param pUdev Pointer to the device
  * @param reqType Transfer type
  * @param bRequest Request code
  * @param wValue Request value
  * @param wIndex Request index
  * @param pData Pointer to data buffer
  * @param wLength Request length
  * @param pActualLen Actual length of data
  * @param timeout Timeout in milliseconds
  * @retval 0 on success, error code otherwise
  */
int ch375_hostControlTransfer(struct USB_Device_t *pUdev, uint8_t reqType, uint8_t bRequest, 
    uint16_t wValue, uint16_t wIndex, uint8_t *pData, uint16_t wLength, int *pActualLen, uint32_t timeout) {

    int ret = -1;
    struct ch375_Context_t *pCtx;
    uint8_t setupBuff[CONTROL_SETUP_SIZE];
    int totalReceived = 0;
    bool toggle = false;
    uint8_t status;

    if (NULL == pUdev || NULL == pUdev->ctx) {
        LOG_ERR("Invalid device or context");
        return CH375_HOST_PARAM_INVALID;
    }

    if (NULL == pData && 0 != wLength) {
        LOG_ERR("Invalid data/length parameters.");
        return CH375_HOST_PARAM_INVALID;
    }

    pCtx = pUdev->ctx;

    if (USB_SREQ_GET_DESCRIPTOR == bRequest) {
        ret = ch375_setRetry(pCtx, CH375_RETRY_TIMES_2MS);
        LOG_DBG("Using tolerant retry for GET_DESCRIPTOR");
    } else {
        ret = ch375_setRetry(pCtx, CH375_RETRY_TIMES_INFINITY);
    }
    if ( CH375_SUCCESS != ret) {
        LOG_ERR("Set retry failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    // Setup stage
    struct usb_setup_packet *setup = (struct usb_setup_packet *)setupBuff;
    setup->bmRequestType = reqType;
    setup->bRequest = bRequest;
    setup->wValue = sys_cpu_to_le16(wValue);
    setup->wIndex = sys_cpu_to_le16(wIndex);
    setup->wLength = sys_cpu_to_le16(wLength);

    ret = ch375_writeBlockData(pCtx, setupBuff, CONTROL_SETUP_SIZE);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Write SETUP packet failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    k_busy_wait(200);

    ret = ch375_sendToken(pCtx, 0, toggle, USB_PID_SETUP, &status);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Send SETUP token failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    if (CH375_USB_INT_SUCCESS != status) {
        LOG_ERR("SETUP failed, status: 0x%02X", status);
        if (CH375_USB_INT_DISCONNECT == status) {
            return CH375_HOST_DEV_DISCONNECT;
        }
        if (status == CH375_PID2STATUS(USB_PID_STALL)) {
            return CH375_HOST_STALL;
        }
        LOG_ERR("Unhandled status: 0x%02X", status);
        return CH375_HOST_ERROR;
    }

    toggle = true;

    if (wLength > 0) {
        if (SETUP_IN(reqType)) {
            uint32_t naks = 0;
            
            while (totalReceived < wLength) {
                uint8_t packetLen = 0;
                
                ret = ch375_sendToken(pCtx, 0x00, toggle, USB_PID_IN, &status);
                if (CH375_SUCCESS != ret) {
                    LOG_ERR("Send IN token failed: %d", ret);
                    return CH375_HOST_ERROR;
                }

                if (CH375_USB_INT_SUCCESS != status) {
                    // Handle NAK
                    if (status == CH375_PID2STATUS(USB_PID_NAK)) {
                        naks++;
                        if (naks % 100 == 0) {
                            LOG_DBG("NAK count: %d (received so far: %d/%d)", 
                                   naks, totalReceived, wLength);
                        }
                        
                        // For large descriptors wait longer
                        if (totalReceived > 0) {
                            k_busy_wait(500);
                        } else {
                            k_busy_wait(100);
                        }
                        
                        // Don't toggle on NAK
                        continue;
                    }
                    
                    // Handle STALL
                    if (status == CH375_PID2STATUS(USB_PID_STALL)) {
                        return CH375_HOST_STALL;
                    }
                    
                    LOG_ERR("IN token failed, status: 0x%02X (received: %d/%d)", 
                           status, totalReceived, wLength);
                    
                    if (CH375_USB_INT_DISCONNECT == status) {
                        return CH375_HOST_DEV_DISCONNECT;
                    }
                    
                    // Return data if any 
                    if (totalReceived > 0) {
                        LOG_WRN("Partial data transfer, returning %d bytes", totalReceived);
                        break;
                    }
                    
                    return CH375_HOST_ERROR;
                }

                // Read data
                ret = ch375_readBlockData(pCtx, pData + totalReceived, 
                                        wLength - totalReceived, &packetLen);
                
                if (CH375_SUCCESS != ret) {
                    LOG_ERR("Read data failed: %d (received: %d/%d)", 
                           ret, totalReceived, wLength);
                    
                    // Return any data we got so far
                    if (totalReceived > 0) {
                        LOG_WRN("Partial read, returning %d bytes", totalReceived);
                        break;
                    }
                    return CH375_HOST_ERROR;
                }

                if (packetLen > 0) {
                    totalReceived += packetLen;
                    toggle = !toggle;
                    
                    LOG_DBG("Received packet: len=%d, total=%d/%d, toggle=%d", 
                           packetLen, totalReceived, wLength, toggle);
                }

                // Short packet indicates EOT
                if (packetLen < pUdev->ep0_max_packet) {
                    LOG_DBG("Short packet (%d < %d), transfer complete at %d/%d bytes",
                           packetLen, pUdev->ep0_max_packet, totalReceived, wLength);
                    break;
                }

                if (totalReceived < wLength) {
                    k_busy_wait(100);
                }
            }
            
            if (naks > 0) {
                LOG_DBG("Transfer complete after %d NAKs", naks);
            }
            
        } else {
            while (totalReceived < wLength) {
                uint8_t toSend = wLength - totalReceived;
                if (toSend > pUdev->ep0_max_packet) {
                    toSend = pUdev->ep0_max_packet;
                }

                ret = ch375_writeBlockData(pCtx, pData + totalReceived, toSend);
                if (CH375_SUCCESS != ret) {
                    LOG_ERR("Write data failed: %d", ret);
                    return CH375_HOST_ERROR;
                }

                ret = ch375_sendToken(pCtx, 0x00, toggle, USB_PID_OUT, &status);
                if (CH375_SUCCESS != ret) {
                    LOG_ERR("Send OUT token failed: %d", ret);
                    return CH375_HOST_ERROR;
                }

                if (CH375_USB_INT_SUCCESS != status) {
                    LOG_ERR("OUT token failed, status: 0x%02X", status);
                    if (CH375_USB_INT_DISCONNECT == status) {
                        return CH375_HOST_DEV_DISCONNECT;
                    }
                    if (status == CH375_PID2STATUS(USB_PID_STALL)) {
                        return CH375_HOST_STALL;
                    }
                    return CH375_HOST_ERROR;
                }
                
                totalReceived += toSend;
                toggle = !toggle;
            }
        }
    }

    // Status stage
    if (SETUP_IN(reqType)) {
        uint8_t dummy[1] = {0};
        ret = ch375_writeBlockData(pCtx, dummy, 0);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Write status OUT failed: %d", ret);
            
            if (totalReceived > 0) {
                LOG_WRN("Status write failed but data received, treating as success");
                if (NULL != pActualLen) {
                    *pActualLen = totalReceived;
                }
                return CH375_HOST_SUCCESS;
            }
            return CH375_HOST_ERROR;
        }

        ret = ch375_sendToken(pCtx, 0x00, 0x01, USB_PID_OUT, &status);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Send status OUT token failed: %d", ret);
            
            if (totalReceived > 0) {
                LOG_WRN("Status token failed but data received, treating as success");
                if (NULL != pActualLen) {
                    *pActualLen = totalReceived;
                }
                return CH375_HOST_SUCCESS;
            }
            return CH375_HOST_ERROR;
        }

        if (status != CH375_USB_INT_SUCCESS) {
            LOG_ERR("Status OUT failed: 0x%02X", status);
            
            if (totalReceived > 0) {
                LOG_WRN("Status stage failed (0x%02X) but %d bytes received successfully, ignoring error",
                       status, totalReceived);
                if (NULL != pActualLen) {
                    *pActualLen = totalReceived;
                }
                return CH375_HOST_SUCCESS;
            }
            
            if (CH375_USB_INT_DISCONNECT == status) {
                return CH375_HOST_DEV_DISCONNECT;
            }
            if (status == CH375_PID2STATUS(USB_PID_STALL)) {
                return CH375_HOST_STALL;
            }
            LOG_ERR("Unhandled status: 0x%02X", status);
            return CH375_HOST_ERROR;
        }
    } else {
        ret = ch375_sendToken(pCtx, 0x00, 0x01, USB_PID_IN, &status);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Send status IN token failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        
        if (CH375_USB_INT_SUCCESS != status) {
            LOG_ERR("Status IN failed: 0x%02X", status);
            if (CH375_USB_INT_DISCONNECT == status) {
                return CH375_HOST_DEV_DISCONNECT;
            }
            if (status == CH375_PID2STATUS(USB_PID_STALL)) {
                return CH375_HOST_STALL;
            }
            LOG_ERR("Unhandled status: 0x%02X", status);
            return CH375_HOST_ERROR;
        }
    }

    if (NULL != pActualLen) {
        *pActualLen = totalReceived;
    }

    return CH375_HOST_SUCCESS;
}

/**
  * @brief Bulk transfer to a device
  * @param pUdev Pointer to the device
  * @param ep Endpoint number
  * @param pData Pointer to data buffer
  * @param len Length of the data buffer
  * @param pActualLen Actual length of the data successfully transferred
  * @param timeout Timeout in milliseconds
  * @retval 0 on success, error code otherwise
  */
int ch375_hostBulkTransfer(struct USB_Device_t *pUdev, uint8_t ep, uint8_t *pData, int len, int *pActualLen, uint32_t timeout) {
    int ret = -1;
    struct ch375_Context_t *pCtx;
    struct USB_Endpoint_t *endpoint = NULL;
    int resiLen = len;
    int offset = 0;
    uint8_t status;
    static uint32_t transferCount = 0;
    uint32_t thisTransfer = ++transferCount;

    if (NULL == pData && 0 != len) {
        LOG_ERR("Invalid data/length parameters");
        return CH375_HOST_PARAM_INVALID;
    }

    pCtx = pUdev->ctx;

    ret = get_endpoint(pUdev, ep, &endpoint);
    if (ret < 0) {
        LOG_ERR("Endpoint 0x%02X not found", ep);
        return CH375_HOST_PARAM_INVALID;
    }

    ret = ch375_setRetry(pCtx, CH375_RETRY_TIMES_ZERO);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Set retry dailed: %d", ret);
        return CH375_HOST_ERROR;
    }

    uint32_t nakCount = 0;
    uint32_t loopCount = 0;

    while (resiLen > 0) {
        loopCount++;
        uint8_t len = resiLen > endpoint->max_packet ? endpoint->max_packet : resiLen;
        uint8_t actualLen = 0;
        
        if (EP_IN(ep)) {
            ret = ch375_sendToken(pCtx, ep, endpoint->data_toggle, USB_PID_IN, &status);
            if (CH375_SUCCESS != ret) {
                LOG_ERR("[#%" PRIu32 "] Send IN token failed: %d", thisTransfer, ret);
                return CH375_HOST_ERROR;
            }
            
            if (loopCount <= 3 || status != CH375_PID2STATUS(USB_PID_NAK)) {
                LOG_DBG("[#%" PRIu32 "] IN token response: status=0x%02X (loop=%" PRIu32 ")",
                        thisTransfer, status, loopCount);
            }
            
            if (CH375_USB_INT_SUCCESS == status) {
                ret = ch375_readBlockData(pCtx, pData + offset, len, &actualLen);
                if (CH375_SUCCESS != ret) {
                    LOG_ERR("[#%" PRIu32 "] Read data failed: %d", thisTransfer, ret);
                    return CH375_HOST_ERROR;
                }
                LOG_DBG("[#%" PRIu32 "] Read SUCCESS: actual_len=%d", thisTransfer, actualLen);
            }
        } else {
            ret = ch375_writeBlockData(pCtx, pData + offset, len);
            if (CH375_SUCCESS != ret) {
                LOG_ERR("[#%" PRIu32 "] Write data failed: %d", thisTransfer, ret);
                return CH375_HOST_ERROR;
            }
            
            ret = ch375_sendToken(pCtx, ep, endpoint->data_toggle, USB_PID_OUT, &status);
            if (CH375_SUCCESS != ret) {
                LOG_ERR("[#%" PRIu32 "] Send OUT token failed: %d", thisTransfer, ret);
                return CH375_HOST_ERROR;
            }
            
            if (CH375_USB_INT_SUCCESS == status) {
                actualLen = len;
            }
        }

        if (CH375_USB_INT_SUCCESS == status) {
            LOG_DBG("[#%" PRIu32 "] Transfer successful: offset=%d->%d residue=%d->%d tog=%d->%d",
                    thisTransfer, offset, offset + actualLen, 
                    resiLen, resiLen - actualLen,
                    endpoint->data_toggle, !endpoint->data_toggle);
            
            endpoint->data_toggle = !endpoint->data_toggle;
            offset += actualLen;
            resiLen -= actualLen;
            continue;
        }
        
        if (status == CH375_PID2STATUS(USB_PID_NAK)){
            nakCount++;

            if (nakCount <= 5 || nakCount % 100 == 0) {
                LOG_DBG("[#%" PRIu32 "] NAK received (count=%" PRIu32 ", timeout=%" PRIu32 ", offset=%d)",
                        thisTransfer, nakCount, timeout, offset);
            }
            
            if (timeout == 0) {
                LOG_DBG("[#%" PRIu32 "] NAK with timeout=0: returning TIMEOUT (offset=%d, NAKs=%" PRIu32 ")",
                        thisTransfer, offset, nakCount);
                
                if (NULL != pActualLen) {
                    *pActualLen = offset;
                }
                return CH375_HOST_TIMEOUT;
            }
            timeout--;
            k_msleep(1);
        } else {
            LOG_ERR("[#%" PRIu32 "] Transfer failed, status: 0x%02X (offset=%d)",
                    thisTransfer, status, offset);
            if (CH375_USB_INT_DISCONNECT == status) {
                return CH375_HOST_DEV_DISCONNECT;
            }
            if (status == CH375_PID2STATUS(USB_PID_STALL)) {
                return CH375_HOST_STALL;
            }
            LOG_ERR("[#%" PRIu32 "] Unhandled status: 0x%02X", thisTransfer, status);
            return CH375_HOST_ERROR;
        }
    }

    if (NULL != pActualLen) {
        *pActualLen = offset;
    }

    return CH375_HOST_SUCCESS;
}

/**
  * @brief Clear the stall condition on a given endpoint
  * @param pUdev Pointer to the device
  * @param ep Endpoint number
  * @retval 0 on success, error code otherwise
  */
int ch375_hostClearStall(struct USB_Device_t *pUdev, uint8_t ep) {
    int ret = -1;
    struct USB_Endpoint_t *endpoint = NULL;

    if (0 != ep) {
        ret = get_endpoint(pUdev, ep, &endpoint);
        if (ret < 0) {
            LOG_ERR("Endpoint 0x%02X not found", ep);
            return CH375_HOST_PARAM_INVALID;
        }
    }

    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_ENDPOINT), 
                                            USB_SREQ_CLEAR_FEATURE, 0, ep, NULL, 0, NULL, TRANSFER_TIMEOUT);
    
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Clear feature failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    if ( NULL != endpoint) {
        endpoint->data_toggle = false;
    }

    return CH375_HOST_SUCCESS;
}

/**
  * @brief Set configuration of the device
  * @param pUdev Pointer to the device
  * @param config device configuration
  * @retval 0 on success, error code otherwise
  */
int ch375_hostSetConfiguration(struct USB_Device_t *pUdev, uint8_t config) {
    
    // 0x00 | STANDARD | DEVICE
    int ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                                    USB_SREQ_SET_CONFIGURATION, config, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
        
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set configuration failed: %d", ret);
        return CH375_HOST_ERROR;
    }
    
    return CH375_HOST_SUCCESS;
}

/* --------------------------------------------------------------------------
 * INSTANCE helpers
 * -------------------------------------------------------------------------*/
static int set_dev_address(struct USB_Device_t *pUdev, uint8_t addr) {
    
    int ret = -1;

    // REQ_TYPE: 0x00 | STARNDARD | DEVICE
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_OUT, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
                                        USB_SREQ_SET_ADDRESS, addr, 0, NULL, 0, NULL, TRANSFER_TIMEOUT);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set address failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    // Send the newly assigned addr to device
    ret = ch375_setUSBAddr(pUdev->ctx, addr);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Set CH375 USB addr failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    return CH375_HOST_SUCCESS;
}

static int get_config_descriptor(struct USB_Device_t *pUdev, uint8_t *pBuff, uint16_t len) {
    
    int ret = -1;
    int actual_len = 0;
    
    // REQ_TYPE: 0x80 | STARNDARD | DEVICE
    ret = ch375_hostControlTransfer(pUdev, USB_REQ_TYPE(USB_DIR_IN, USB_TYPE_STANDARD, USB_RECIP_DEVICE), 
    USB_SREQ_GET_DESCRIPTOR, USB_DESC_CONFIGURATION << 8, 0, pBuff, len, &actual_len, TRANSFER_TIMEOUT);

    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Get config descriptor failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    if (actual_len < len) {
        LOG_ERR("Config descriptor too short!");
        return CH375_HOST_ERROR;
    }

    return CH375_HOST_SUCCESS;
}

static int parse_config_descriptor(struct USB_Device_t *pUdev) {
    
    if ( NULL == pUdev || NULL == pUdev->raw_conf_desc || 0 == pUdev->raw_conf_desc_len) {
        return CH375_HOST_ERROR;
    }

    uint8_t *pDescStart = (uint8_t *)pUdev->raw_conf_desc;
    uint8_t *pDescEnd = pDescStart + pUdev->raw_conf_desc_len;

    while (pDescStart + sizeof(struct usb_desc_header) <= pDescEnd) {
        struct usb_desc_header *pHdr = (struct usb_desc_header *)pDescStart;

        if (0 == pHdr->bLength) {
            LOG_ERR("Descriptor parsing error: %d length", pHdr->bLength);
            return CH375_HOST_ERROR;
        }

        if (pDescStart + pHdr->bLength > pDescEnd) {
            LOG_ERR("Descriptor parsing error: %" PRIu32 " length exceed descriptor end. %" PRIu32 " bytes truncated", pHdr->bLength, (unsigned)(pDescEnd - pDescStart));
            return CH375_HOST_ERROR;
        }

        switch (pHdr->bDescriptorType) {
            case USB_DESC_INTERFACE: {
                parse_interface_descriptor(pUdev, (struct usb_if_descriptor *)pHdr);
                break;
            }
            
            case USB_DESC_ENDPOINT: {
                parse_endpoint_descriptor(&pUdev->interfaces[pUdev->interface_count - 1], (struct usb_ep_descriptor  *)pHdr);
                break;
            }

            default: {
                break;
            }
        }

        pDescStart += pHdr->bLength;
    }

    return CH375_HOST_SUCCESS;
}

static void parse_interface_descriptor(struct USB_Device_t *pUdev, struct usb_if_descriptor *pDesc) {
    
    struct USB_Interface_t *interface = &pUdev->interfaces[pUdev->interface_count];

    interface->interface_number = pDesc->bInterfaceNumber;
    interface->interface_class = pDesc->bInterfaceClass;
    interface->interface_subclass = pDesc->bInterfaceSubClass;
    interface->interface_protocol = pDesc->bInterfaceProtocol;

    pUdev->interface_count++;
}

static void parse_endpoint_descriptor(struct USB_Interface_t *pIfc, struct usb_ep_descriptor *pDesc) {

    struct USB_Endpoint_t *pEP = &pIfc->endpoints[pIfc->endpoint_count];

    pEP->ep_addr = pDesc->bEndpointAddress;
    pEP->data_toggle = false;
    pEP->max_packet = sys_le16_to_cpu(pDesc->wMaxPacketSize);
    pEP->attributes = pDesc->bmAttributes;
    pEP->interval = pDesc->bInterval;
    
    pIfc->endpoint_count++;
}

static int reset_dev(struct ch375_Context_t *pCtx) {
    
    int ret = -1;

    ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_RESET);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("USB reset failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    k_msleep(20);

    ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_SOF_AUTO);
    if (CH375_SUCCESS != ret) {
        LOG_ERR("Set USB SOF mode failed: %d", ret);
        return CH375_HOST_ERROR;
    }

    k_msleep(20);

    ret = ch375_hostWaitDeviceConnect(pCtx, RESET_WAIT_DEVICE_RECONNECT_TIMEOUT_MS);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Wait device reconnect failed: %d", ret);
        ret = ch375_setUSBMode(pCtx, CH375_USB_MODE_SOF_AUTO);
        if (CH375_SUCCESS != ret) {
            LOG_ERR("Set USB SOF mode failed: %d", ret);
            return CH375_HOST_ERROR;
        }
        return CH375_HOST_DEV_DISCONNECT;
    }

    k_msleep(40);
    return CH375_HOST_SUCCESS;
}

static int get_endpoint(struct USB_Device_t *pUdev, uint8_t epAddr, struct USB_Endpoint_t **ppEP) {
    
    int i, j;

    if (NULL == pUdev || NULL == ppEP || 0x00 == epAddr) {
        return -1;
    }

    for (i = 0; i < pUdev->interface_count; i++) {
        struct USB_Interface_t *interface = &pUdev->interfaces[i];
        for (j = 0; j < interface->endpoint_count; j++) {
            if (interface->endpoints[j].ep_addr == epAddr) {
                *ppEP = &interface->endpoints[j];
                LOG_DBG("Found EP 0x%02X: interface=%d idx=%d maxpack=%d", 
                                        epAddr, i, j, (*ppEP)->max_packet);
                return 0;
            }
        }
    }

    LOG_ERR("Endpoint 0x%02X not found in device", epAddr);
    return -1;
}