#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ch375.h"
#include "ch375_host.h"
#include "ch375_uart.h"
#include "hid_parser.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static void dump_hid_items(uint8_t *ppReport, uint16_t len);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    struct ch375_Context_t *pCtx;
    struct USB_Device_t udev;
    int ret;
    uint8_t *pDesc, *pDescEnd;
    
    // GPIO for CH375 INT
    static const struct gpio_dt_spec ch375_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };
    
    LOG_INF("=================== HID Parser Test ===================");
    
    LOG_INF("Initializing CH375.");
    ret = ch375_hwInitManual("CH375", 2, &ch375_int, 9600, &pCtx);
    if (ret < 0) {
        LOG_ERR("Failed to init CH375: %d", ret);
        return ret;
    }
    
    ret = ch375_hostInit(pCtx, 115200);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Failed to init host mode: %d", ret);
        return ret;
    }
    
    ret = ch375_hwSetBaudrate(pCtx, 115200);
    if (ret < 0) {
        LOG_ERR("Failed to set baudrate: %d", ret);
        return ret;
    }
    
    LOG_INF("CH375 initialized successfully");
    
    // Wait
    LOG_INF("Waiting for USB device...");
    ret = ch375_hostWaitDeviceConnect(pCtx, 5000);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("No device connected");
        return ret;
    }
    
    LOG_INF("Device connected!");
    
    LOG_INF("Enumerating device...");
    ret = ch375_hostUdevOpen(pCtx, &udev);
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Failed to enumerate: %d", ret);
        return ret;
    }
    
    // Find all HID interfaces and print info
    LOG_INF("Device enumerated:");
    LOG_INF("  VID:PID = %04X:%04X", udev.vendor_id, udev.product_id);
    LOG_INF("  Interfaces = %d", udev.interface_count);

    int hidInterface = -1;
    int mouseInterface = -1;
    int keyboardInterface = -1;

    for (int i = 0; i < udev.interface_count; i++) {
        if (USB_CLASS_HID == udev.interfaces[i].interface_class) {
            const char *pProtocolStr = "None";
            if (udev.interfaces[i].interface_protocol == 1) {
                pProtocolStr = "Keyboard";
                if (keyboardInterface < 0) keyboardInterface = i;
            } else if (2 == udev.interfaces[i].interface_protocol) {
                pProtocolStr = "Mouse";
                if (mouseInterface < 0) mouseInterface = i;
            }
            
            LOG_INF("  Interface %d: HID (subclass=0x%02X protocol=%d - %s)",
                    i, udev.interfaces[i].interface_subclass, 
                    udev.interfaces[i].interface_protocol,
                    pProtocolStr);
        }
    }

    // Prefer mouse interface, then keyboard, then first HID interface
    if (mouseInterface >= 0) {
        hidInterface = mouseInterface;
        LOG_INF("Selected mouse interface %d", hidInterface);
    } else if (keyboardInterface >= 0) {
        hidInterface = keyboardInterface;
        LOG_INF("Selected keyboard interface %d", hidInterface);
    } else {
        // Fall back to first HID interface
        for (int i = 0; i < udev.interface_count; i++) {
            if (udev.interfaces[i].interface_class == USB_CLASS_HID) {
                hidInterface = i;
                LOG_INF("Selected first HID interface %d", hidInterface);
                break;
            }
        }
    }

    if (hidInterface < 0) {
        LOG_ERR("No HID interface found!");
        return -1;
    }
    
    // Get HID descriptor from config descriptor
    LOG_INF("\n=================== Parsing Configuration Descriptor ===================");
    LOG_HEXDUMP_INF(udev.raw_conf_desc, udev.raw_conf_desc_len, "Config Descriptor");
    
    // Find HID class
    pDesc = udev.raw_conf_desc;
    pDescEnd = pDesc + udev.raw_conf_desc_len;
    struct USB_HID_Descriptor_t *pHidDesc = NULL;
    uint8_t currentInterface = 0;
    
    while (pDesc < pDescEnd) {
        struct usb_desc_header *pHdr = (struct usb_desc_header *)pDesc;
        
        if (0 == pHdr->bLength) break;
        
        if (USB_DESC_INTERFACE == pHdr->bDescriptorType) {
            currentInterface = ((struct usb_if_descriptor *)pDesc)->bInterfaceNumber;
        } else if (pHdr->bDescriptorType == 0x21 && currentInterface == hidInterface) {
            pHidDesc = (struct USB_HID_Descriptor_t *)pDesc;
            break;
        }
        
        pDesc += pHdr->bLength;
    }
    
    if (NULL == pHidDesc) {
        LOG_ERR("HID descriptor not found!");
        return -1;
    }
    
    LOG_INF("\n=================== HID Descriptor ===================");
    LOG_INF("  bcdHID = 0x%04X", sys_le16_to_cpu(pHidDesc->bcdHID));
    LOG_INF("  bCountryCode = 0x%02X", pHidDesc->bCountryCode);
    LOG_INF("  bNumDescriptors = %d", pHidDesc->bNumDescriptors);
    LOG_INF("  bDescriptorType = 0x%02X", pHidDesc->bClassDescriptorType);
    LOG_INF("  wDescriptorLength = %d", 
            sys_le16_to_cpu(pHidDesc->wClassDescriptorLength));
    
    // Get HID report descriptor
    uint16_t reportDescLen = sys_le16_to_cpu(pHidDesc->wClassDescriptorLength);
    uint8_t *pReportDesc = k_malloc(reportDescLen);
    if (NULL == pReportDesc) {
        LOG_ERR("Failed to allocate report descriptor buffer");
        return -ENOMEM;
    }
    
    LOG_INF("\n=================== Fetching HID Report Descriptor ===================");
    ret = ch375_hostControlTransfer(&udev, 0x81, USB_SREQ_GET_DESCRIPTOR, 
        0x22 << 8, hidInterface, pReportDesc, reportDescLen, NULL, 5000);
    
    if (CH375_HOST_SUCCESS != ret) {
        LOG_ERR("Failed to get report descriptor: %d", ret);
        k_free(pReportDesc);
        return ret;
    }
    
    LOG_INF("Report descriptor fetched successfully!");
    LOG_HEXDUMP_INF(pReportDesc, reportDescLen, "Raw Report Descriptor");
    
    LOG_INF("\n=================== TEST 1: Parse HID Items ===================");
    dump_hid_items(pReportDesc, reportDescLen);
    
    LOG_INF("\n=================== TEST 2: Determine Device Type ===================");
    uint8_t deviceType = 0;
    ret = HID_parseReportDescriptor(pReportDesc, reportDescLen, &deviceType);
    if (ret < 0) {
        LOG_ERR("Failed to parse report descriptor: %d", ret);
    } else {
        const char *pTypeStr = "Unknown";
        switch (deviceType) {
            case 1: {
                pTypeStr = "MOUSE"; 
                break;
            }

            case 2: {
                pTypeStr = "KEYBOARD"; 
                break;
            }

            case 3: {
                pTypeStr = "JOYSTICK"; 
                break;
            }
        }
        LOG_INF("Device Type: %s (by report descriptor)", pTypeStr);
    }

    if (0 == deviceType) {
        if (1 == udev.interfaces[hidInterface].interface_protocol) {
            deviceType = 2;
            LOG_INF("Fallback: KEYBOARD (by USB interface protocol)");
        } else if (2 == udev.interfaces[hidInterface].interface_protocol) {
            deviceType = 1;
            LOG_INF("Fallback: MOUSE (by USB interface protocol)");
        }
    }

    LOG_INF("\n=================== TEST 3: Find Key Report Fields ===================");

    struct HID_Item_t item;
    pDesc = pReportDesc;
    pDescEnd = pReportDesc + reportDescLen;

    uint32_t usage_page = 0;
    uint32_t report_size = 0;
    uint32_t report_count = 0;
    uint32_t report_offset = 0;
    int32_t logical_min = 0;
    int32_t logical_max = 0;

    while (pDesc < pDescEnd) {
        pDesc = HID_fetchItem(pDesc, pDescEnd, &item);
        if (NULL == pDesc) break;
        
        if (HID_ITEM_TYPE_GLOBAL == item.type) {
            switch (item.tag) {
                case HID_GLOBAL_ITEM_TAG_USAGE_PAGE: {
                    usage_page = item.data.u32 << 16;
                    break;
                }
                    
                case HID_GLOBAL_ITEM_TAG_REPORT_SIZE: {
                    report_size = item.data.u32;
                    break;
                }
                    
                case HID_GLOBAL_ITEM_TAG_REPORT_COUNT: {
                    report_count = item.data.u32;
                    break;
                }
                    
                case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM: {
                    if (1 == item.size) {
                        logical_min = (int8_t)item.data.u8;
                    } else if (2 == item.size) {
                        logical_min = (int16_t)item.data.u16;
                    } else {
                        logical_min = item.data.s32;
                    }
                    break;
                }
                    
                case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM: {
                    if (1 == item.size) {
                        logical_max = (int8_t)item.data.u8;
                    } else if (2 == item.size) {
                        logical_max = (int16_t)item.data.u16;
                    } else {
                        logical_max = item.data.s32;
                    }
                    break;
                }
            }

        } else if (HID_ITEM_TYPE_MAIN == item.type && HID_MAIN_ITEM_TAG_INPUT == item.tag) {
            // Prevent overflow
            if (report_size > 0 && report_size <= 32 && report_count > 0 && report_count <= 256) {
                
                LOG_INF("  INPUT: offset=%d size=%d count=%d usage_page=0x%08X log_min=%d log_max=%d",
                    report_offset, report_size, report_count, usage_page, logical_min, logical_max);
                
                if (HID_UP_BUTTON == usage_page) {
                    LOG_INF("    -> Buttons field (byte offset=%d)", 
                            report_offset / 8);
                } else if (HID_UP_GENDESK == usage_page) {
                    LOG_INF("    -> Generic Desktop field (X/Y/Wheel, byte offset=%d)", 
                            report_offset / 8);
                }
                
                report_offset += report_size * report_count;
            } else {
                LOG_WRN("  INPUT with invalid size/count: size=%d count=%d (usage_page=0x%08X) - SKIPPED",
                        report_size, report_count, usage_page);
            }
        }
    }

    LOG_INF("Total report length: %d bytes", (report_offset + 7) / 8);
    
    k_free(pReportDesc);
    ch375_hostUdevClose(&udev);
    
    while (1) {
        k_msleep(1000);
    }
    
    return 0;
}

static void dump_hid_items(uint8_t *pReport, uint16_t len) {
    struct HID_Item_t item;
    uint8_t *pItemCur = pReport;
    uint8_t *pItemEnd = pReport + len;
    int itemCount = 0;
    
    LOG_INF("=================== HID Item Dump ===================");
    
    while (pItemCur < pItemEnd) {
        pItemCur = HID_fetchItem(pItemCur, pItemEnd, &item);
        if (NULL == pItemCur) {
            LOG_ERR("Failed to parse item #%d", itemCount);
            break;
        }
        
        // Get item type
        const char *pTypeStr = "?";
        switch (item.type) {
            case HID_ITEM_TYPE_MAIN: {
                pTypeStr = "MAIN"; 
                break;
            }

            case HID_ITEM_TYPE_GLOBAL: {
                pTypeStr = "GLOBAL"; 
                break;
            }
            
            case HID_ITEM_TYPE_LOCAL: {
                pTypeStr = "LOCAL"; 
                break;
            }
        }
        
        // Decode tag based on type
        const char *pTagStr = "?";
        if (HID_ITEM_TYPE_MAIN == item.type) {
            switch (item.tag) {
                case HID_MAIN_ITEM_TAG_INPUT: {
                    pTagStr = "Input"; 
                    break;
                }

                case HID_MAIN_ITEM_TAG_OUTPUT: {
                    pTagStr = "Output"; 
                    break;
                }

                case HID_MAIN_ITEM_TAG_FEATURE: {
                    pTagStr = "Feature"; 
                    break;
                }

                case HID_MAIN_ITEM_TAG_BEGIN_COLLECTION: {
                    pTagStr = "BeginCollection"; 
                    break;
                }

                case HID_MAIN_ITEM_TAG_END_COLLECTION: {
                    pTagStr = "EndCollection"; 
                    break;
                }
            }
        } else if (HID_ITEM_TYPE_GLOBAL == item.type) {
            switch (item.tag) {
                case HID_GLOBAL_ITEM_TAG_USAGE_PAGE: {
                    pTagStr = "UsagePage";
                    break;
                }

                case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM: {
                    pTagStr = "LogicalMin"; 
                    break;
                }

                case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM: {
                    pTagStr = "LogicalMax"; 
                    break;
                }

                case HID_GLOBAL_ITEM_TAG_REPORT_SIZE: {
                    pTagStr = "ReportSize";
                    break;
                }

                case HID_GLOBAL_ITEM_TAG_REPORT_COUNT: {
                    pTagStr = "ReportCount";
                    break;
                }

                case HID_GLOBAL_ITEM_TAG_REPORT_ID: {
                    pTagStr = "ReportID";
                    break;
                }
            }
        } else if (HID_ITEM_TYPE_LOCAL == item.type) {
            switch (item.tag) {
                case HID_LOCAL_ITEM_TAG_USAGE: {
                    pTagStr = "Usage"; 
                    break;
                }

                case HID_LOCAL_ITEM_TAG_USAGE_MINIMUM: {
                    pTagStr = "UsageMin"; 
                    break;
                }

                case HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM: {
                    pTagStr = "UsageMax"; 
                    break;
                }
            }
        }
        
        LOG_INF("Item #%d: %s/%s size=%d data=0x%08X",
                itemCount, pTypeStr, pTagStr, item.size, item.data.u32);
        
        // Get specific useful values
        if (HID_ITEM_TYPE_GLOBAL == item.type) {
            if (HID_GLOBAL_ITEM_TAG_USAGE_PAGE == item.tag) {
                
                uint32_t page = item.data.u32 << 16;
                if (HID_UP_GENDESK == page) LOG_INF("  -> Generic Desktop");
                else if (HID_UP_KEYBOARD == page) LOG_INF("  -> Keyboard");
                else if (HID_UP_BUTTON == page) LOG_INF("  -> Button");
            }
        } else if (HID_ITEM_TYPE_LOCAL == item.type) {
            if (HID_LOCAL_ITEM_TAG_USAGE == item.tag) {
                
                uint32_t usage = item.data.u32;
                
                if (usage == (HID_GD_MOUSE & 0xFF)) {
                    LOG_INF("  -> Mouse");
                }
                else if (usage == (HID_GD_KEYBOARD & 0xFF)) {
                    LOG_INF("  -> Keyboard");
                }
                
                else if (usage == (HID_GD_X & 0xFF)) {
                    LOG_INF("  -> X axis");
                }

                else if (usage == (HID_GD_Y & 0xFF)) {
                    LOG_INF("  -> Y axis");
                }

                else if (usage == (HID_GD_WHEEL & 0xFF)) {
                    LOG_INF("  -> Wheel");
                }
            }
        }
        
        itemCount++;
    }
    
    LOG_INF("Total items parsed: %d", itemCount);
}