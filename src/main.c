#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ch375.h"
#include "ch375_host.h"
#include "ch375_uart.h"
#include "hid_parser.h"
#include "hid_mouse.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Private function prototypes -----------------------------------------------*/
static void dump_hid_items(uint8_t *ppReport, uint16_t len);
static int test_mouse(struct USBHID_Device_t *pHIDDev);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    struct ch375_Context_t *pCtx;
    struct USB_Device_t udev;
    struct USBHID_Device_t hidDev;
    int ret;
    uint8_t *pDesc, *pDescEnd;
    
    // GPIO for CH375 INT
    static const struct gpio_dt_spec ch375_int = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpioc)),
        .pin = 13,
        .dt_flags = GPIO_ACTIVE_LOW
    };
    
    LOG_INF("=================== HID Mouse Test ===================");
    
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
        ch375_hostUdevClose(&udev);
        return -1;
    }
    
    // Open HID device
    LOG_INF("\n=================== Opening HID Device ===================");
    ret = USBHID_open(&udev, hidInterface, &hidDev);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to open HID device: %d", ret);
        ch375_hostUdevClose(&udev);
        return ret;
    }
    
    LOG_INF("HID device opened successfully!");
    LOG_INF("  Device Type: %s", 
            hidDev.hid_type == USBHID_TYPE_MOUSE ? "MOUSE" :
            hidDev.hid_type == USBHID_TYPE_KEYBOARD ? "KEYBOARD" : "UNKNOWN");
    LOG_INF("  Report Descriptor Length: %d", hidDev.raw_hid_report_desc_len);
    
    // Test mouse functionality if it's a mouse
    if (USBHID_TYPE_MOUSE == hidDev.hid_type) {
        LOG_INF("\n=================== Testing Mouse Functionality ===================");
        ret = test_mouse(&hidDev);
        if (ret < 0) {
            LOG_ERR("Mouse functionality test failed: %d", ret);
        }
    } else {
        LOG_WRN("Device is not a mouse, skipping mouse tests");
    }

    // Cleanup
    USBHID_close(&hidDev);
    ch375_hostUdevClose(&udev);
    
    while (1) {
        k_msleep(1000);
    }
    
    return 0;
}


/**
 * @brief Dump the contents of a report descriptor
 * @param pReport Pointer to the report descriptor
 * @param len Length of the report descriptor
 * @return None
 */
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

/**
 * @brief Test mouse functionality
 * @param pHIDDev Pointer to the HID device
 * @return 0 on success, error code otherwise
 */
static int test_mouse(struct USBHID_Device_t *pHIDDev) {
    
    struct HID_Mouse_t mouse;
    int ret;
    
    // Open mouse
    LOG_INF("Opening mouse device...");
    ret = hidMouse_Open(pHIDDev, &mouse);
    if (USBHID_SUCCESS != ret) {
        LOG_ERR("Failed to open mouse: %d", ret);
        return ret;
    }
    
    LOG_INF("Mouse opened successfully!");
    LOG_INF("  Report Length: %d bytes", mouse.report_len);
    LOG_INF("  Button Field: offset=%d size=%d count=%d", 
            mouse.button.report_buf_off, mouse.button.size, mouse.button.count);
    LOG_INF("  Orientation Field: offset=%d size=%d count=%d", 
            mouse.orientation.report_buf_off, mouse.orientation.size, mouse.orientation.count);
    
    k_msleep(1500);

    // Test reading
    LOG_INF("\n--- Reading Mouse Reports (move mouse now!) ---");
    int successCount = 0;
    int timeoutCount = 0;
    
    for (int i = 0; i < 50; i++) {
        ret = hidMouse_FetchReport(&mouse);
        
        if (USBHID_SUCCESS == ret) {
            uint32_t leftBtn = 0, rightBtn = 0, middleBtn = 0;
            int32_t x = 0, y = 0;
            
            // Get button states
            hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &leftBtn, false);
            hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_RIGHT, &rightBtn, false);
            hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_MIDDLE, &middleBtn, false);
            
            // Get orientation
            hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &x, false);
            hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &y, false);
            
            LOG_INF("Sample %d: L=%d R=%d M=%d X=%d Y=%d", 
                    i, leftBtn, rightBtn, middleBtn, x, y);
            
            successCount++;
            
            // Dump raw report
            if (successCount <= 3) {
                uint8_t *reportBuf;
                uint32_t reportLen;
                ret = USBHID_getReportBuffer(pHIDDev, &reportBuf, &reportLen, false);
                if (USBHID_SUCCESS == ret) {
                    LOG_HEXDUMP_INF(reportBuf, reportLen, "Raw Report");
                }
            }
        } else if (ret == -EAGAIN) {
            // No data available - this is normal when mouse is idle
            timeoutCount++;
            if (timeoutCount % 10 == 0) {
                LOG_DBG("Waiting for mouse input... (%d timeouts)", timeoutCount);
            }
        } else {
            LOG_ERR("Fetch report failed: %d", ret);
            break;
        }
        
        // Poll every 50ms
        k_msleep(50);
    }
    
    LOG_INF("Capture complete: %d successful reads, %d timeouts", successCount, timeoutCount);
    
    // Test write
    LOG_INF("\n--- Testing Write Operations ---");
    hidMouse_SetButton(&mouse, HID_MOUSE_BUTTON_LEFT, 1, false);
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_X, 10, false);
    hidMouse_SetOrientation(&mouse, HID_MOUSE_AXIS_Y, -5, false);
    
    uint32_t testBtn;
    int32_t testX, testY;
    hidMouse_GetButton(&mouse, HID_MOUSE_BUTTON_LEFT, &testBtn, false);
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_X, &testX, false);
    hidMouse_GetOrientation(&mouse, HID_MOUSE_AXIS_Y, &testY, false);
    
    LOG_INF("Modified report: L=%d X=%d Y=%d", testBtn, testX, testY);
    
    // Show raw buffer
    uint8_t *reportBuf;
    uint32_t reportLen;
    ret = USBHID_getReportBuffer(pHIDDev, &reportBuf, &reportLen, false);
    if (USBHID_SUCCESS == ret) {
        LOG_HEXDUMP_INF(reportBuf, reportLen, "Modified Raw Report");
    }
    
    // Close mouse
    hidMouse_Close(&mouse);
    LOG_INF("Mouse closed successfully");
    
    return 0;
}
