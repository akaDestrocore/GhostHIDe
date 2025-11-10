#include "hid_mouse.h"

LOG_MODULE_REGISTER(hid_mouse, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static int parse_hid_report(struct HID_Mouse_t *pMouse, uint8_t *pReport, uint16_t len);

/**
 * @brief HID Mouse Open
 * @param pHIDDev Pointer to the USB HID Device structure
 * @param pMouse Pointer to the HID Mouse structure
 * @return 0 on success, error code otherwise
 */
int hidMouse_Open(struct USBHID_Device_t *pHIDDev, struct HID_Mouse_t *pMouse) {

    int ret = -1;

    if (NULL == pMouse || NULL == pHIDDev) {
        LOG_ERR("Invalid parameters");
        return USBHID_PARAM_INVALID;
    }

    if (USBHID_TYPE_MOUSE != pHIDDev->hid_type) {
        LOG_ERR("Not a mouse device");
        return USBHID_NOT_SUPPORT;
    }

    memset(pMouse, 0x00, sizeof(struct HID_Mouse_t));
    pMouse->hid_dev = pHIDDev;
    pMouse->has_wheel = false;

    ret = parse_hid_report(pMouse, pHIDDev->raw_hid_report_desc, pHIDDev->raw_hid_report_desc_len);

    if (ret < 0) {
        LOG_ERR("Failed to parse HID report");
        return USBHID_NOT_SUPPORT;
    }

    if (0 == pMouse->report_len) {
        LOG_ERR("Invalid report length");
        return USBHID_ERROR;
    }

    ret = USBHID_allocReportBuffer(pHIDDev, pMouse->report_len);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to allocate report buffer");
        return USBHID_ALLOC_FAILED;
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Mouse wrapper for `USBHID_freeReportBuffer()`
 * @param pMouse Pointer to the HID device structure
 * @return None
 */
void hidMouse_Close(struct HID_Mouse_t *pMouse) {
    
    if (NULL == pMouse) {
        return;
    }

    USBHID_freeReportBuffer(pMouse->hid_dev);
    memset(pMouse, 0x00, sizeof(struct HID_Mouse_t));
}

/**
 * @brief Fetch report from the device and parse it into a HID Mouse structure
 * @param pMouse Pointer to the HID device structure
 * @return 0 on success, error code otherwise
 */
int hidMouse_FetchReport(struct HID_Mouse_t *pMouse) {

    int ret = USBHID_PARAM_INVALID;

    if (NULL == pMouse) {
        return ret;
    }

        ret = USBHID_fetchReport(pMouse->hid_dev);
    
    // Only run detection when Report ID exists in descriptor
    if (USBHID_SUCCESS == ret && true == pMouse->has_report_id_declared && 0 == pMouse->report_id_offset) {
        uint8_t *pReportBuff;
        uint32_t actualReportLen;
        uint8_t byte0 = 0;
        uint8_t byte1 = 0;
        bool isReportID = (1 == byte0);
        bool isButtonsMaybe = (byte0 <= 0x1F);
        bool isButtons = (byte1 <= 0x1F);
        
        if (USBHID_SUCCESS != USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, &actualReportLen, false)) {
            return ret;
        }
        
        byte0 = pReportBuff[0];
        byte1 = pReportBuff[1];
        
        LOG_DBG("Report ID detection: byte0=0x%02X byte1=0x%02X", byte0, byte1);
    
        // If byte 0 is equal to a constant consistently, that's the Report ID
        isReportID = (byte0 == 1);
        isButtonsMaybe = (byte0 <= 0x1F);
        isButtons = (byte1 <= 0x1F);
        
        if (true == isReportID && true  == isButtons) {
            // Report ID is not stripped by CH375
            // byte0 = Report ID, byte1 = actual button data
            pMouse->report_id_offset = 1;
            pMouse->button.report_buf_off += 1;
            pMouse->orientation.report_buf_off += 1;
            if (true == pMouse->has_wheel) {
                pMouse->wheel.report_buf_off += 1;
            }
        
            LOG_INF("  Adjusted offsets: BTN=%d ORIENT=%d WHEEL=%d", pMouse->button.report_buf_off, 
                                pMouse->orientation.report_buf_off, pMouse->wheel.report_buf_off);
        } else {
            // Assume Report ID stripped by CH375
            pMouse->report_id_offset = 0;
        }
    }
    return ret;
}

/**
 * @brief Obtain button data from the mouse
 * @param pMouse Pointer to the HID device structure
 * @param buttonNum Button number to get
 * @param pValue Pointer to the value variable where the button state will be stored
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_GetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t *pValue, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pButtonDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t byteOff = buttonNum / 8;
    uint8_t bitOff = buttonNum % 8;

    if (NULL == pMouse || NULL == pValue) {
        return USBHID_PARAM_INVALID;
    }

    if (buttonNum >= pMouse->button.count) {
        LOG_ERR("Invalid button number: %d", buttonNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pButtonDesc = &pMouse->button;
    pFieldBuff = pReportBuff + pButtonDesc->report_buf_off;
    *pValue = (pFieldBuff[byteOff] & (0x01 << bitOff)) ? 1 : 0;

    return USBHID_SUCCESS;
}

/**
 * @brief Set button state
 * @param pMouse Pointer to the HID device structure
 * @param buttonNum Button number to get
 * @param value Button value to set
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_SetButton(struct HID_Mouse_t *pMouse, uint32_t buttonNum, uint32_t value, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pButtonDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t byteOff = buttonNum / 8;
    uint8_t bitOff = buttonNum % 8;

    if (NULL == pMouse) {
        return USBHID_PARAM_INVALID;
    }

    if (buttonNum >= pMouse->button.count) {
        LOG_ERR("Invalid button number: %d", buttonNum);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pButtonDesc = &pMouse->button;
    pFieldBuff = pReportBuff + pButtonDesc->report_buf_off;

    if (0 != value) {
        pFieldBuff[byteOff] |= (0x01 << bitOff);
    } else {
        pFieldBuff[byteOff] &= ~(0x01 << bitOff);
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Get X and Y axis values from the report buffer
 * @param pMouse Pointer to the HID device structure
 * @param axisNum Number of mouse orientation axes
 * @param pValue Pointer to the value variable
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_GetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t *pValue, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t valueByteSize;

    if (NULL == pMouse || NULL == pValue) {
        return USBHID_PARAM_INVALID;
    }
    
    // Wheel is stored separately
    if (2 == axisNum && true == pMouse->has_wheel) {
        pDesc = &pMouse->wheel;
        
        ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
        if (USBHID_SUCCESS != ret) {
            return ret;
        }
        
        pFieldBuff = pReportBuff + pDesc->report_buf_off;
        valueByteSize = pDesc->size / 8;
        
        // Wheel is typically 8-bit signed
        if (valueByteSize == 1) {
            *pValue = ((int8_t *)pFieldBuff)[0];
        } else if (valueByteSize == 2) {
            int16_t temp;
            memcpy(&temp, pFieldBuff, sizeof(temp));
            *pValue = (int32_t)sys_le16_to_cpu((uint16_t)temp);
        } else {
            *pValue = 0;
        }
        
        return USBHID_SUCCESS;
    }

    // X/Y axes (0 and 1)
    if (axisNum >= pMouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d (max=%d)", axisNum, pMouse->orientation.count);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pDesc = &pMouse->orientation;
    pFieldBuff = pReportBuff + pDesc->report_buf_off;
    valueByteSize = pDesc->size / 8;

    if (0 == valueByteSize) {
        LOG_ERR("Invalid value size: size=%d bits", pDesc->size);
        return USBHID_ERROR;
    }

    switch (valueByteSize) {
        case 1: {
            *pValue = (int32_t)((int8_t)pFieldBuff[axisNum]);
            break;
        }

        case 2: {
            int16_t temp;
            // X is at pFieldBuff[0:1], Y is at pFieldBuff[2:3]
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 2);
            memcpy(&temp, pAxisStart, sizeof(temp));
            *pValue = (int32_t)sys_le16_to_cpu((uint16_t)temp);
            break;
        }

        case 4: {
            int32_t temp;
            // 32-bit: axes are at [0:3] and [4:7]
            uint8_t *pAxisStart = pFieldBuff + (axisNum * 4);
            memcpy(&temp, pAxisStart, sizeof(temp));
            *pValue = sys_le32_to_cpu((uint32_t)temp);
            break;
        }

        default: {
            LOG_ERR("Unexpected value size: %d", valueByteSize);
            return USBHID_ERROR;
        }
    }

    return USBHID_SUCCESS;
}

/**
 * @brief Set desired value for mouse orientation axes
 * @param pMouse Pointer to the HID device structure
 * @param axisNum Number of mouse orientation axes
 * @param value Value to be set to the mouse orientation axes
 * @param isLast Flag indicating if this is the last report
 * @return 0 on success, error code otherwise
 */
int hidMouse_SetOrientation(struct HID_Mouse_t *pMouse, uint32_t axisNum, int32_t value, bool isLast) {

    int ret = -1;
    struct HID_DataDescriptor_t *pOrientDesc;
    uint8_t *pReportBuff;
    uint8_t *pFieldBuff;
    uint8_t valueByteSize;

    if (NULL == pMouse) {
        return USBHID_PARAM_INVALID;
    }

    if (axisNum >= pMouse->orientation.count) {
        LOG_ERR("Invalid axis number: %d (max=%d)", axisNum, pMouse->orientation.count);
        return USBHID_PARAM_INVALID;
    }

    ret = USBHID_getReportBuffer(pMouse->hid_dev, &pReportBuff, NULL, isLast);
    if (USBHID_SUCCESS != ret) {
        return ret;
    }

    pOrientDesc = &pMouse->orientation;
    pFieldBuff = pReportBuff + pOrientDesc->report_buf_off;
    valueByteSize = pOrientDesc->size / 8;

    if (0 == valueByteSize) {
        LOG_ERR("Invalid value size: size=%d bits", pOrientDesc->size);
        return USBHID_ERROR;
    }

    switch (valueByteSize) {
        case 1: {
            ((int8_t *)pFieldBuff)[axisNum] = (int8_t)value;
            break;
        }

        case 2: {
            int16_t temp = (int16_t)value;
            uint16_t le_val = sys_cpu_to_le16((uint16_t)temp);
            memcpy(&((int16_t *)pFieldBuff)[axisNum], &le_val, sizeof(le_val));
            break;
        }

        case 4: {
            uint32_t le_val = sys_cpu_to_le32((uint32_t)value);
            memcpy(&((int32_t *)pFieldBuff)[axisNum], &le_val, sizeof(le_val));
            break;
        }

        default: {
            LOG_ERR("Unexpected value size: %d", valueByteSize);
            return USBHID_ERROR;
        }
    }

    return USBHID_SUCCESS;
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static int parse_hid_report(struct HID_Mouse_t *pMouse, uint8_t *pReport, uint16_t len) {

    struct HID_Item_t item;
    struct HID_DataDescriptor_t *pBtn = &pMouse->button;
    struct HID_DataDescriptor_t *pOrient = &pMouse->orientation;
    struct HID_DataDescriptor_t *pWheel = &pMouse->wheel;

    uint8_t *pHIDRep = pReport;
    uint8_t *pHIDRepEnd = pReport + len;

    uint32_t usagePage = 0;
    uint32_t usage = 0;
    uint32_t usageMin = 0;
    uint32_t usageMax = 0;
    int32_t logicalMin = 0;
    int32_t logicalMax = 0;
    uint32_t reportSize = 0;
    uint32_t reportCount = 0;
    uint32_t reportOffset = 0;

    bool foundButtons = false;
    bool foundOrientation = false;
    bool foundWheel = false;
    bool hasReportId = false;
    uint8_t reportIdValue = 0;
    bool inMouseCollection = false;
    int collectionDepth = 0;

    while (pHIDRep < pHIDRepEnd) {
        uint8_t *oldPtr = pHIDRep;
        pHIDRep = HID_fetchItem(pHIDRep, pHIDRepEnd, &item);
        if (NULL == pHIDRep) {
            LOG_ERR("Failed to fetch HID item at offset %d", (int)(oldPtr - pReport));
            break;
        }

        switch(item.type) {

            case HID_ITEM_TYPE_GLOBAL: {
                switch(item.tag) {

                    case HID_GLOBAL_ITEM_TAG_USAGE_PAGE: {
                        usagePage = item.data.u32 << 16;
                        LOG_INF("  Usage Page: 0x%08X", usagePage);
                        break;
                    }

                    case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM: {
                        if (1 == item.size) {
                            logicalMin = (int8_t)item.data.u8;
                        } else if (2 == item.size) {
                            logicalMin = (int16_t)item.data.u16;
                        } else {
                            logicalMin = item.data.s32;
                        }
                        LOG_DBG("  Logical Min: %d", logicalMin);
                        break;
                    }

                    case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM: {
                        if (1 == item.size) {
                            logicalMax = (int8_t)item.data.u8;
                        } else if (2 == item.size) {
                            logicalMax = (int16_t)item.data.u16;
                        } else {
                            logicalMax = item.data.s32;
                        }
                        LOG_DBG("  Logical Max: %d", logicalMax);
                        break;
                    }

                    case HID_GLOBAL_ITEM_TAG_REPORT_SIZE: {
                        reportSize = item.data.u32;
                        LOG_DBG("  Report Size: %d bits", reportSize);
                        break;
                    }

                    case HID_GLOBAL_ITEM_TAG_REPORT_COUNT: {
                        reportCount = item.data.u32;
                        LOG_DBG("  Report Count: %d", reportCount);
                        break;
                    }

                    case HID_GLOBAL_ITEM_TAG_REPORT_ID: {
                        // Report ID actually exists but CH375 strips it from data, so we don't add to reportOffset
                        hasReportId = true;
                        reportIdValue = item.data.u8;
                        break;
                    }
                }

                break;
            }

            case HID_ITEM_TYPE_LOCAL: {
                if (HID_LOCAL_ITEM_TAG_USAGE == item.tag) {
                    usage = usagePage | item.data.u32;
                    LOG_DBG("  Usage: 0x%08X", usage);
                }
                break;
            }

            case HID_ITEM_TYPE_MAIN: {
                if (HID_MAIN_ITEM_TAG_BEGIN_COLLECTION == item.tag) {
                    collectionDepth++;
                    
                    // Detect mouse collection
                    if ((HID_GD_MOUSE  == usage || HID_GD_POINTER == usage) && true != inMouseCollection) {
                        inMouseCollection = true;
                        LOG_INF("Mouse collection found at depth %d (Report ID %s)",
                                collectionDepth,
                                hasReportId ? "present but stripped" : "not present");
                        
                        // Reset field tracking
                        foundButtons = false;
                        foundOrientation = false;
                        foundWheel = false;
                        // Report ID already stripped
                        reportOffset = 0;
                    }
                }
                else if (HID_MAIN_ITEM_TAG_END_COLLECTION == item.tag) {
                    if (collectionDepth > 0) {
                        collectionDepth--;
                    }
                    
                    if (true == inMouseCollection && 0 == collectionDepth) {
                        LOG_INF("Mouse collection ended at bit offset %d", reportOffset);
                        inMouseCollection = false;
                        
                        if (true == foundButtons && true == foundOrientation) {
                            LOG_INF("All required fields found, stopping parse");
                            goto parse_done;
                        }
                    }
                }
                else if (HID_MAIN_ITEM_TAG_INPUT == item.tag) {
                    LOG_DBG("INPUT: offset=%d size=%d count=%d usage=0x%08X inMouse=%d", 
                            reportOffset, reportSize, reportCount, usage, inMouseCollection);
                    
                    if (true != inMouseCollection) {
                        LOG_DBG("  Skipping INPUT (not in mouse collection)");
                        reportOffset += reportSize * reportCount;
                        usage = 0;
                        break;
                    }

                    // Parse BUTTONS
                    if (HID_UP_BUTTON == usagePage && true != foundButtons) {
                        pBtn->logical_minimum = logicalMin;
                        pBtn->logical_maximum = logicalMax;
                        pBtn->size = reportSize;
                        pBtn->count = reportCount;
                        pBtn->report_buf_off = reportOffset / 8;

                        foundButtons = true;
                        LOG_INF("    -> BUTTONS: byte=%d size=%d count=%d", pBtn->report_buf_off, pBtn->size, pBtn->count);

                    }

                    // Parse X/Y/Wheel
                    else if (HID_UP_GENDESK == usagePage) {
                        bool isXAxis = (HID_GD_X == usage);
                        bool isYAxis = (HID_GD_Y == usage);
                        bool isWheel = (HID_GD_WHEEL == usage);
                        
                        // X/Y orientation
                        if ((true == isXAxis || true == isYAxis) && true != foundOrientation && (8 == reportSize || 16 == reportSize) && 2 == reportCount) {
                            pOrient->logical_minimum = logicalMin;
                            pOrient->logical_maximum = logicalMax;
                            pOrient->size = reportSize;
                            pOrient->count = reportCount;
                            pOrient->report_buf_off = reportOffset / 8;

                            foundOrientation = true;
                            LOG_INF("  -> ORIENTATION: byte=%d size=%d count=%d", 
                                    pOrient->report_buf_off, pOrient->size, pOrient->count);
                        }
                        // Wheel (separate field)
                        else if (true == isWheel && true != foundWheel) {
                            pWheel->logical_minimum = logicalMin;
                            pWheel->logical_maximum = logicalMax;
                            pWheel->size = reportSize;
                            pWheel->count = reportCount;
                            pWheel->report_buf_off = reportOffset / 8;

                            foundWheel = true;
                            pMouse->has_wheel = true;
                            LOG_INF("  -> WHEEL: byte=%d size=%d count=%d", pWheel->report_buf_off, pWheel->size, pWheel->count);
                        }
                    }

                    // Advance offset by the size of this field
                    reportOffset += reportSize * reportCount;

                    // Clear local state
                    usage = 0;
                }
                break;
            }
        }
    }

    LOG_INF("Parse complete: buttons=%d orientation=%d total_bits=%d", foundButtons, foundOrientation, reportOffset);

parse_done:
    if (true != foundButtons || true != foundOrientation) {
        LOG_ERR("Failed to parse mouse fields: buttons=%d orientation=%d", foundButtons, foundOrientation);
        return -1;
    }

    // Calculate report data length
    pMouse->report_len = (reportOffset + 7) / 8;
    
    // Store whether Report ID exists
    pMouse->has_report_id_declared = hasReportId;
    if (true == foundWheel) {
        LOG_INF("  Wheel at byte %d", pWheel->report_buf_off);
    }

    return 0;
}