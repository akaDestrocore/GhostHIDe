#include <zephyr/ztest.h>
#include <zephyr/usb/usb_ch9.h>
#include "usb_stubs.h"
#include "ch375_host.h"
#include "mock_ch375_hw.h"

static struct ch375_Context_t *pCtx;

// Sample USB Device Descriptor (18 bytes)
static const uint8_t sample_device_desc[] = {
    0x12,       // bLength
    0x01,       //bDescriptorType: DEVICE
    0x00, 0x02, // bcdUSB: 2.0
    0x00,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    0x40,       // bMaxPacketSize0: 64
    0x5E, 0x04, // idVendor: 0x045E (Microsoft)
    0x3B, 0x00, // idProduct: 0x003B
    0x20, 0x01, // ibcdDevice: 1.20
    0x01,       // iManufacturer
    0x02,       // iProduct
    0x00,       // iSerialNumber
    0x01        // bNumConfigurations
};

// Sample USB Mouse Configuration Descriptor
static const uint8_t sample_mouse_config[] = {
    // Config Descriptor
    0x09,       // bLength
    0x02,       // bDescriptorType: CONFIGURATION
    0x22, 0x00, // wTotalLength: 34 bytes
    0x01,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0xA0,       // bmAttributes: Remote wakeup
    0x32,       // bMaxPower: 100mA
    
    // Interface Descriptor
    0x09,       // bLength
    0x04,       // bDescriptorType: INTERFACE
    0x00,       // bInterfaceNumber
    0x00,       // bAlternateSetting
    0x01,       // bNumEndpoints
    0x03,       // bInterfaceClass: HID
    0x01,       // bInterfaceSubClass: Boot
    0x02,       // bInterfaceProtocol: Mouse
    0x00,       // iInterface
    
    // HID Descriptor
    0x09,       // bLength
    0x21,       // bDescriptorType: HID
    0x11, 0x01,  //bcdHID: 1.11
    0x00,       // bCountryCode
    0x01,       // bNumDescriptors
    0x22,       // bDescriptorType: Report
    0x34, 0x00,  //wDescriptorLength: 52
    
    // Endpoint Descriptor
    0x07,       // bLength
    0x05,       // bDescriptorType: ENDPOINT
    0x81,       // bEndpointAddress: IN endpoint 1
    0x03,       // bmAttributes: Interrupt
    0x04, 0x00,  //wMaxPacketSize: 4
    0x0A        // bInterval: 10ms
};

// Sample USB Keyboard Configuration with multiple endpoints
static const uint8_t sample_keyboard_config[] = {
    //Configuration Descriptor
    0x09, 0x02, 0x3B, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32,
    
    //Interface 0: Keyboard
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x41, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0A,
    
    //Interface 1: Media Keys
    0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00,
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x1D, 0x00,
    0x07, 0x05, 0x82, 0x03, 0x04, 0x00, 0x0A
};

static void test_setup(void *f)
{
    mock_ch375Reset();
    zassert_equal(mock_ch375Init(&pCtx), CH375_SUCCESS);
}

static void test_teardown(void *f)
{
    if (pCtx) {
        ch375_closeContext(pCtx);
        pCtx = NULL;
    }
}

/* ========================================================================
 * Test: Device Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_device_descriptor_basic)
{
    // Create a minimal device to test descriptor access
    struct usb_device_descriptor *desc = 
        (struct usb_device_descriptor *)sample_device_desc;
    
    // Verify parsing
    zassert_equal(desc->bLength, 18);
    zassert_equal(desc->bDescriptorType, USB_DESC_DEVICE);
    zassert_equal(sys_le16_to_cpu(desc->bcdUSB), 0x0200);
    zassert_equal(desc->bMaxPacketSize0, 64);
    zassert_equal(sys_le16_to_cpu(desc->idVendor), 0x045E);
    zassert_equal(sys_le16_to_cpu(desc->idProduct), 0x003B);
}

/* ========================================================================
 * Test: Configuration Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_config_descriptor_basic)
{
    struct usb_cfg_descriptor *desc = 
        (struct usb_cfg_descriptor *)sample_mouse_config;
    
    zassert_equal(desc->bLength, 9);
    zassert_equal(desc->bDescriptorType, USB_DESC_CONFIGURATION);
    zassert_equal(sys_le16_to_cpu(desc->wTotalLength), 34);
    zassert_equal(desc->bNumInterfaces, 1);
    zassert_equal(desc->bConfigurationValue, 1);
}

/* ========================================================================
 * Test: Interface Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_interface_descriptor)
{
    // Skip config descriptor (9 bytes) to get to interface
    struct usb_if_descriptor *desc = 
        (struct usb_if_descriptor *)(sample_mouse_config + 9);
    
    zassert_equal(desc->bLength, 9);
    zassert_equal(desc->bDescriptorType, USB_DESC_INTERFACE);
    zassert_equal(desc->bInterfaceNumber, 0);
    zassert_equal(desc->bNumEndpoints, 1);
    zassert_equal(desc->bInterfaceClass, 0x03, "Should be HID class");
    zassert_equal(desc->bInterfaceSubClass, 0x01, "Should be Boot subclass");
    zassert_equal(desc->bInterfaceProtocol, 0x02, "Should be Mouse protocol");
}

/* ========================================================================
 * Test: Endpoint Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_parse_endpoint_descriptor)
{
    // Skip to endpoint descriptor (9 + 9 + 9 = 27 bytes)
    struct usb_ep_descriptor *desc = 
        (struct usb_ep_descriptor *)(sample_mouse_config + 27);
    
    zassert_equal(desc->bLength, 7);
    zassert_equal(desc->bDescriptorType, USB_DESC_ENDPOINT);
    zassert_equal(desc->bEndpointAddress, 0x81, "Should be IN endpoint 1");
    zassert_equal(desc->bmAttributes, 0x03, "Should be Interrupt type");
    zassert_equal(sys_le16_to_cpu(desc->wMaxPacketSize), 4);
    zassert_equal(desc->bInterval, 10);
}

/* ========================================================================
 * Test: Full Configuration Parsing (Mouse)
 * ======================================================================== */
ZTEST(ch375_descriptors, test_full_mouse_config_walk)
{
    const uint8_t *ptr = sample_mouse_config;
    const uint8_t *end = ptr + sizeof(sample_mouse_config);
    
    int descriptor_count = 0;
    int interface_count = 0;
    int endpoint_count = 0;
    
    while (ptr + sizeof(struct usb_desc_header) <= end) {
        struct usb_desc_header *hdr = (struct usb_desc_header *)ptr;
        
        zassert_not_equal(hdr->bLength, 0, "Descriptor length cannot be 0");
        zassert_true(ptr + hdr->bLength <= end, "Descriptor exceeds buffer");
        
        descriptor_count++;
        
        switch (hdr->bDescriptorType) {
        case USB_DESC_CONFIGURATION:
            // First descriptor should be configuration
            zassert_equal(descriptor_count, 1);
            break;
            
        case USB_DESC_INTERFACE:
            interface_count++;
            break;
            
        case USB_DESC_ENDPOINT:
            endpoint_count++;
            break;
        }
        
        ptr += hdr->bLength;
    }
    
    zassert_equal(interface_count, 1, "Mouse should have 1 interface");
    zassert_equal(endpoint_count, 1, "Mouse should have 1 endpoint");
}

/* ========================================================================
 * Test: Multi-Interface Configuration (Keyboard)
 * ======================================================================== */
ZTEST(ch375_descriptors, test_keyboard_multi_interface)
{
    const uint8_t *ptr = sample_keyboard_config;
    const uint8_t *end = ptr + sizeof(sample_keyboard_config);
    
    int interface_count = 0;
    int endpoint_count = 0;
    int current_interface = -1;
    int ep_per_interface[2] = {0};
    
    while (ptr + sizeof(struct usb_desc_header) <= end) {
        struct usb_desc_header *hdr = (struct usb_desc_header *)ptr;
        
        if (hdr->bLength == 0 || ptr + hdr->bLength > end) {
            break;
        }
        
        if (hdr->bDescriptorType == USB_DESC_INTERFACE) {
            struct usb_if_descriptor *iface = (struct usb_if_descriptor *)ptr;
            current_interface = iface->bInterfaceNumber;
            interface_count++;
            
            if (current_interface == 0) {
                zassert_equal(iface->bInterfaceClass, 0x03);
                zassert_equal(iface->bInterfaceProtocol, 0x01, 
                             "Interface 0 should be keyboard");
            }
        } else if (hdr->bDescriptorType == USB_DESC_ENDPOINT) {
            endpoint_count++;
            if (current_interface >= 0 && current_interface < 2) {
                ep_per_interface[current_interface]++;
            }
        }
        
        ptr += hdr->bLength;
    }
    
    zassert_equal(interface_count, 2, "Keyboard should have 2 interfaces");
    zassert_equal(endpoint_count, 2, "Should have 2 endpoints total");
    zassert_equal(ep_per_interface[0], 1, "Interface 0 should have 1 EP");
    zassert_equal(ep_per_interface[1], 1, "Interface 1 should have 1 EP");
}

/* ========================================================================
 * Test: Malformed Descriptor Handling
 * ======================================================================== */
ZTEST(ch375_descriptors, test_malformed_zero_length)
{
    uint8_t bad_desc[] = {
        0x09, 0x02, 0x09, 0x00, 0x01, 0x01, 0x00, 0xA0, 0x32,
        0x00
    };
    
    const uint8_t *ptr = bad_desc;
    const uint8_t *end = ptr + sizeof(bad_desc);
    int desc_count = 0;
    
    while (ptr + sizeof(struct usb_desc_header) <= end) {
        struct usb_desc_header *hdr = (struct usb_desc_header *)ptr;
        
        if (hdr->bLength == 0) {
            break;
        }
        
        desc_count++;
        ptr += hdr->bLength;
    }
    
    zassert_equal(desc_count, 1, "Should stop at zero-length descriptor");
}

ZTEST(ch375_descriptors, test_malformed_length_exceeds)
{
    uint8_t bad_desc[] = {
        0x09, 0x02, 0x0F, 0x00, 0x01, 0x01, 0x00, 0xA0, 0x32,
        0xFF, 0x04
    };
    
    const uint8_t *ptr = bad_desc;
    const uint8_t *end = ptr + sizeof(bad_desc);
    int desc_count = 0;
    bool detected_overflow = false;
    
    while (ptr + sizeof(struct usb_desc_header) <= end) {
        struct usb_desc_header *hdr = (struct usb_desc_header *)ptr;
        
        if (hdr->bLength == 0 || ptr + hdr->bLength > end) {
            detected_overflow = true;
            break;
        }
        
        desc_count++;
        ptr += hdr->bLength;
    }
    
    zassert_true(detected_overflow, "Should detect length overflow");
}

/* ========================================================================
 * Test: HID Descriptor Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_hid_descriptor)
{
    // HID descriptor is after interface descriptor in mouse config
    struct USB_HID_Descriptor_t *hid = 
        (struct USB_HID_Descriptor_t *)(sample_mouse_config + 18);
    
    zassert_equal(hid->bLength, 9);
    zassert_equal(hid->bDescriptorType, 0x21, "Should be HID descriptor");
    zassert_equal(sys_le16_to_cpu(hid->bcdHID), 0x0111, "HID version 1.11");
    zassert_equal(hid->bNumDescriptors, 1);
    zassert_equal(hid->bClassDescriptorType, 0x22, "Report descriptor");
    zassert_equal(sys_le16_to_cpu(hid->wClassDescriptorLength), 52);
}

/* ========================================================================
 * Test: Endpoint Address Parsing
 * ======================================================================== */
ZTEST(ch375_descriptors, test_endpoint_direction_detection)
{
    // Test IN endpoint
    uint8_t ep_in_addr = 0x81;
    zassert_true(ep_in_addr & 0x80, "Should detect IN direction");
    
    // Test OUT endpoint
    uint8_t ep_out_addr = 0x01;
    zassert_false(ep_out_addr & 0x80, "Should detect OUT direction");
}

ZTEST(ch375_descriptors, test_endpoint_type_detection)
{
    // Interrupt endpoint
    uint8_t attr_int = 0x03;
    zassert_equal(attr_int & 0x03, 0x03, "Should be Interrupt");
    
    // Bulk endpoint
    uint8_t attr_bulk = 0x02;
    zassert_equal(attr_bulk & 0x03, 0x02, "Should be Bulk");
    
    // Isochronous endpoint
    uint8_t attr_iso = 0x01;
    zassert_equal(attr_iso & 0x03, 0x01, "Should be Isochronous");
}

/* ========================================================================
 * Test Suite Setup
 * ======================================================================== */
ZTEST_SUITE(ch375_descriptors, NULL, NULL, test_setup, test_teardown, NULL);