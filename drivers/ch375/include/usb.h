#ifndef _USB_PID_H
#define _USB_PID_H

#include <zephyr/usb/usb_ch9.h>
#include <stdint.h>

typedef enum {
    USB_PID_NULL   = 0x00,
    USB_PID_SOF    = 0x05,
    USB_PID_SETUP  = 0x0D,
    USB_PID_IN     = 0x09,
    USB_PID_OUT    = 0x01,
    USB_PID_ACK    = 0x02,
    USB_PID_NAK    = 0x0A,
    USB_PID_STALL  = 0x0E,
    USB_PID_DATA0  = 0x03,
    USB_PID_DATA1  = 0x0B,
    USB_PID_PRE    = 0x0C
} USB_PID_e;

#endif /* _USB_PID_H */
