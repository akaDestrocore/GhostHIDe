#ifndef CH375_H
#define CH375_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdlib.h>
#include "usb.h"

#define WAIT_INT_TIMEOUT_MS 2000
#define CH375_CHECK_EXIST_DATA1 0x65
#define CH375_CHECK_EXIST_DATA2 ((uint8_t)~CH375_CHECK_EXIST_DATA1)

/**
 * @brief CH375 commands
 */
typedef enum {
    CH375_CMD_GET_IC_VER        =   0x01,
    CH375_CMD_SET_BAUDRATE      =   0x02,
    CH375_CMD_SET_USB_SPEED     =   0x04,
    CH375_CMD_CHECK_EXIST       =   0x06,
    CH375_CMD_GET_DEV_RATE      =   0x0A,
    CH375_CMD_SET_RETRY         =   0x0B,
    CH375_CMD_SET_USB_ADDR      =   0x13,
    CH375_CMD_SET_USB_MODE      =   0x15,
    CH375_CMD_TEST_CONNECT      =   0x16,
    CH375_CMD_ABORT_NAK         =   0x17,
    CH375_CMD_SET_ENDP6         =   0x1C,
    CH375_CMD_SET_ENDP7         =   0x1D,
    CH375_CMD_GET_STATUS        =   0x22,
    CH375_CMD_UNLOCK_USB        =   0x23,
    CH375_CMD_RD_USB_DATA0      =   0x27,
    CH375_CMD_RD_USB_DATA       =   0x28,
    CH375_CMD_WR_USB_DATA7      =   0x2B,
    CH375_CMD_GET_DESC          =   0x46,
    CH375_CMD_ISSUE_TKN_X       =   0x4E,
    CH375_CMD_ISSUE_TOKEN       =   0x4F,
    /* State codes*/
    CH375_CMD_RET_SUCCESS       =   0x51,
    CH375_CMD_RET_FAILED        =   0x5F
} ch375_CMD_e;

/**
 * @brief CH375 USB host modes
 */
typedef enum {
    CH375_USB_MODE_INVALID      =   0x04,
    CH375_USB_MODE_NO_SOF       =   0x05,
    CH375_USB_MODE_SOF_AUTO     =   0x06,
    CH375_USB_MODE_RESET        =   0x07
} ch375_USBHostMode_e;

/**
 * @brief CH375 USB host interrupt states
 */
typedef enum {
    CH375_USB_INT_SUCCESS       =   0x14,
    CH375_USB_INT_CONNECT       =   0x15,
    CH375_USB_INT_DISCONNECT    =   0x16,
    CH375_USB_INT_BUF_OVER      =   0x17,
    CH375_USB_INT_USB_READY     =   0x18
} ch375_USBHostInt_e;

/* Macros for command/data differentiation in 9-bit mode */
#define CH375_CMD(x)  ((uint16_t)((x) | 0x0100))
#define CH375_DATA(x) ((uint16_t)((x) | 0x0000))
#define CH375_PID2STATUS(x) ((x) | 0x20)

/**
 * @brief CH375 error codes
 */
typedef enum {
    CH375_SUCCESS               =   0,
    CH375_ERROR                 =   -1,
    CH375_PARAM_INVALID         =   -2,
    CH375_WRITE_CMD_FAILED      =   -3,
    CH375_READ_DATA_FAILED      =   -4,
    CH375_NO_EXIST              =   -5,
    CH375_TIMEOUT               =   -6,
    CH375_NOT_FOUND             =   -7,
} ch375_ErrNo;

/**
 * @brief CH375 retry
 */
typedef enum {
    CH375_RETRY_TIMES_ZERO      =   0x00,
    CH375_RETRY_TIMES_2MS       =   0x01,
    CH375_RETRY_TIMES_INFINITY  =   0x02
} ch375_Retry_e;

/* Default Baudrates */
#define CH375_DEFAULT_BAUDRATE  9600
#define CH375_WORK_BAUDRATE     115200

// Forward declration of CH375 context structure
struct ch375_Context_t;

// Function pointer types for hardware abstraction
typedef int (*ch375_writeCmdFn_t)(struct ch375_Context_t *pCtx, uint8_t cmd);
typedef int (*ch375_writeDataFn_t)(struct ch375_Context_t *pCtx, uint8_t data);
typedef int (*ch375_readDataFn_t)(struct ch375_Context_t *pCtx, uint8_t *data);
typedef int (*ch375_queryIntFn_t)(struct ch375_Context_t *pCtx);

/**
 * @brief CH375 Context structure
 */
struct ch375_Context_t {
    void *priv;
    ch375_writeCmdFn_t write_cmd;
    ch375_writeDataFn_t write_data;
    ch375_readDataFn_t read_data;
    ch375_queryIntFn_t query_int;
    struct k_mutex lock;
};

/**
 * @brief CH375 core functions
 */
int ch375_openContext(struct ch375_Context_t **ppCtx,
                       ch375_writeCmdFn_t write_cmd,
                       ch375_writeDataFn_t write_data,
                       ch375_readDataFn_t read_data,
                       ch375_queryIntFn_t query_int,
                       void *priv);
int ch375_closeContext(struct ch375_Context_t *pCtx);
void *ch375_getPriv(struct ch375_Context_t *pCtx);

/**
 * @brief Transfer commands
 */
int ch375_checkExist(struct ch375_Context_t *pCtx);
int ch375_getVersion(struct ch375_Context_t *pCtx, uint8_t *pVersion);
int ch375_setBaudrate(struct ch375_Context_t *pCtx, uint32_t baudrate);
int ch375_setUSBMode(struct ch375_Context_t *pCtx, uint8_t mode);
int ch375_getStatus(struct ch375_Context_t *pCtx, uint8_t *pStatus);
int ch375_abortNAK(struct ch375_Context_t *pCtx);
int ch375_queryInt(struct ch375_Context_t *pCtx);
int ch375_waitInt(struct ch375_Context_t *pCtx, uint32_t timeout_ms);

/**
 * @brief Host commands
 */
int ch375_testConnect(struct ch375_Context_t *pCtx, uint8_t *pConnStatus);
int ch375_getDevSpeed(struct ch375_Context_t *pCtx, uint8_t *pSpeed);
int ch375_setDevSpeed(struct ch375_Context_t *pCtx, uint8_t speed);
int ch375_setUSBAddr(struct ch375_Context_t *pCtx, uint8_t addr);
int ch375_setRetry(struct ch375_Context_t *pCtx, uint8_t times);
int ch375_sendToken(struct ch375_Context_t *pCtx, uint8_t ep, uint8_t tog,
                    uint8_t pid, uint8_t *pStatus);

/**
 * @brief Data transfer commands
 */
int ch375_writeCmd(struct ch375_Context_t *pCtx, uint8_t cmd);
int ch375_writeData(struct ch375_Context_t *pCtx, uint8_t data);
int ch375_readData(struct ch375_Context_t *pCtx, uint8_t *pData);

#endif /* CH375_H */