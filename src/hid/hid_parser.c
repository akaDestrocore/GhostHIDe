#include "hid_parser.h"

LOG_MODULE_REGISTER(hid_parser, LOG_LEVEL_DBG);

/**
 * @brief  Fetch a single HID report descriptor item from a buffer.
 * @param  pStart Pointer to the start of the input buffer (pointer to the first byte to parse).
 * @param  pEnd   Pointer one past the last valid byte in the input buffer (exclusive end).
 * @param  pItem  Pointer to the HID_Item_t structure to be filled.
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
 * @brief  Parse a report descriptor and return the type of the report descriptor.
 * @param pReport pointer to the report descriptor
 * @param len length of the report descriptor
 * @param pType pointer to the type of the report
 * @return 0 on success, error code otherwise
 * on error.
 */
int HID_parseReportDescriptor(uint8_t *pReport, uint16_t len, uint8_t *pType)
{
    struct HID_Item_t item = {0};
    uint8_t *end = pReport + len;
    uint8_t *cur = pReport;
    uint32_t usage_page = 0;
    uint32_t usage = 0;
    bool found_device_type = false;
    
    if (!pReport || !pType || len < 2) {
        return -EINVAL;
    }
    
    *pType = 0;
    
    /* Iterate through the entire descriptor looking for standard HID collections */
    while (cur < end && !found_device_type) {
        cur = HID_fetchItem(cur, end, &item);
        if (!cur) break;
        
        if (item.type == HID_ITEM_TYPE_GLOBAL && 
            item.tag == HID_GLOBAL_ITEM_TAG_USAGE_PAGE) {
            /* Store usage page (can be 1, 2, or 4 bytes) */
            usage_page = item.data.u32 << 16;
        }
        else if (item.type == HID_ITEM_TYPE_LOCAL && 
                 item.tag == HID_LOCAL_ITEM_TAG_USAGE) {
            /* Store usage */
            usage = usage_page | item.data.u32;
        }
        else if (item.type == HID_ITEM_TYPE_MAIN && 
                 item.tag == HID_MAIN_ITEM_TAG_BEGIN_COLLECTION) {
            /* Check if this is a standard HID application collection */
            
            if (usage == HID_GD_MOUSE) {
                *pType = USBHID_TYPE_MOUSE;
                found_device_type = true;
                LOG_INF("Detected HID Mouse (usage=0x%08X)", usage);
            }
            else if (usage == HID_GD_KEYBOARD) {
                *pType = USBHID_TYPE_KEYBOARD;
                found_device_type = true;
                LOG_INF("Detected HID Keyboard (usage=0x%08X)", usage);
            }
            /* Add other device types as needed */
        }
    }
    
    if (!found_device_type) {
        /* Fallback: analyze interface protocol from USB interface descriptor
         * This should have been passed in or checked elsewhere, but we can
         * make an educated guess based on descriptor structure */
        
        cur = pReport;
        bool has_input = false;
        bool has_output = false;
        int input_count = 0;
        
        while (cur < end) {
            cur = HID_fetchItem(cur, end, &item);
            if (!cur) break;
            
            if (item.type == HID_ITEM_TYPE_MAIN) {
                if (item.tag == HID_MAIN_ITEM_TAG_INPUT) {
                    has_input = true;
                    input_count++;
                } else if (item.tag == HID_MAIN_ITEM_TAG_OUTPUT) {
                    has_output = true;
                }
            }
        }
        
        /* Keyboards typically have OUTPUT for LEDs, mice don't */
        if (has_input && has_output) {
            *pType = USBHID_TYPE_KEYBOARD;
            LOG_INF("Detected HID Keyboard (by OUTPUT presence - likely vendor-specific)");
            return 0;
        }
        else if (has_input) {
            *pType = USBHID_TYPE_MOUSE;
            LOG_INF("Detected HID Mouse (by INPUT only - likely vendor-specific)");
            return 0;
        }
        
        LOG_ERR("Unknown HID device type");
        return -ENOTSUP;
    }
    
    return 0;
}