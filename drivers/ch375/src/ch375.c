#include "ch375.h"

LOG_MODULE_REGISTER(ch375, LOG_LEVEL_DBG);

/**
 * @brief CH375 core functions
 */

 
/**
  * @brief Initialize the context
  * @param ppCtx The context to initialize
  * @param write_cmd The function that will be called when a command needs to be written to the device.
  * @param write_data The function that will be called when data needs to be written to the device.
  * @param read_data The function that will be called when data is received from the device.
  * @param query_int The function that will be called when the device is in interrupt mode.
  * @retval 0 on success, error code otherwise
  */
int ch375_openContext(struct Ch375_Context_t **ppCtx, ch375_writeCmdFn_t write_cmd,
                       ch375_writeDataFn_t write_data, ch375_readDataFn_t read_data,
                       ch375_queryIntFn_t query_int, void *priv) {
	struct Ch375_Context_t *new_ctx;

	if (NULL == ppCtx || NULL == write_cmd || NULL == write_data || NULL == read_data || NULL == query_int) {
		LOG_ERR("Invlid parameters!");
		return CH375_PARAM_INVALID;           
	}

	new_ctx = k_malloc(sizeof(struct Ch375_Context_t));
	if (NULL == new_ctx) {
		LOG_ERR("Failed to allocate memory for context!");
		return CH375_ERROR;
	}

	memset(new_ctx, 0x00, sizeof(struct Ch375_Context_t));
	k_mutex_init(&new_ctx->lock);

	new_ctx->priv = priv;
	new_ctx->write_cmd = write_cmd;
	new_ctx->write_data = write_data;
	new_ctx->read_data = read_data;
	new_ctx->query_int = query_int;

	*ppCtx = new_ctx;

    return CH375_SUCCESS;
}

/**
  * @brief Closes a context and frees all resources associated with it
  * @param pCtx The context to close
  * @retval 0 on success, -2 otherwise
  */
int ch375_closeContext(struct Ch375_Context_t *pCtx) {
	
	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	k_free(pCtx);
	return CH375_SUCCESS;
}

/**
  * @brief Gets the private data associated with a context
  * @param pCtx The context to get the private data from
  * @retval None
  */
void *ch375_getPriv(struct Ch375_Context_t *pCtx) {

	if (NULL == pCtx) {
		return NULL;
	}

	return pCtx->priv;
}

/**
 * @brief Transfer commands
 */

/**
  * @brief Checks if a device exists
  * @param pCtx The context to check
  * @retval 0 on success, error code otherwise
  */
int ch375_checkExist(struct Ch375_Context_t *pCtx) {

	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	uint8_t recvBuff = 0;
	int ret = -1;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_CHECK_EXIST);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, CH375_CHECK_EXIST_DATA1);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_readData(pCtx, &recvBuff);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);

	if ( CH375_CHECK_EXIST_DATA2 != recvBuff) {
		LOG_ERR("Expected 0x%02X, but got 0x%02X!", CH375_CHECK_EXIST_DATA2, recvBuff);
		return CH375_NO_EXIST;
	}

	return CH375_SUCCESS;
}

/**
  * @brief Get device version
  * @param pCtx The context to check
  * @param pVersion pointer to buffer for version
  * @retval 0 on success, error code otherwise
  */
int ch375_getVersion(struct Ch375_Context_t *pCtx, uint8_t *pVersion) {
	
	if (NULL == pCtx || NULL == pVersion) {
		LOG_ERR("Invalid parameters!");
		return CH375_PARAM_INVALID;
	}

	uint8_t ver = 0;
	int ret = -1;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_GET_IC_VER);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_readData(pCtx, &ver);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}

	// only lower 6 bits is version
	*pVersion = ver & 0x3F;

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Set CH375 baud rate
  * @param pCtx The context
  * @param baudrate Desired baud rate
  * @retval 0 on success, error code otherwise
  */
int ch375_setBaudrate(struct Ch375_Context_t *pCtx, uint32_t baudrate) {

	if ( NULL == pCtx ) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
    uint8_t data1 = 0;
	uint8_t data2 = 0;

	switch(baudrate) {
		case 9600: {
			data1 = 0x02;
			data2 = 0xB2;
			break;
		}

		case 19200: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x02;
			data2 = 0xD9;
			break;
		}

		case 57600: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x03;
			data2 = 0x98;
			break;
		}

		case 115200: {
			data1 = 0x03;
			data2 = 0xCC;
			break;
		}

		case 460800: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x03;
			data2 = 0xF3;
			break;
		}

		case 921600: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x07;
			data2 = 0xF3;
		}

		case 100000: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x03;
			data2 = 0xC4;
			break;
		}

		case 1000000: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x03;
			data2 = 0xFA;
			break;
		}

		case 2000000: {
			LOG_WRN("Suspicious baudrate value selected: %" PRIu32 ".", baudrate);
			data1 = 0x03;
			data2 = 0xFD;
			break;
		}
		default: {
			LOG_ERR("Unsupported baudrate: %" PRIu32 "", baudrate);
			return CH375_PARAM_INVALID;
		}
	}

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_SET_BAUDRATE);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, data1);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, data2);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Set CH375 USB device mode
  * @param pCtx The context
  * @param mode The USB mode to set
  * @retval 0 on success, error code otherwise
  */
int ch375_setUSBMode(struct Ch375_Context_t *pCtx, uint8_t mode) {

	if ( NULL == pCtx ) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t usb_mode = 0;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_SET_USB_MODE);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, mode);
	if ( CH375_SUCCESS != ret ) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_readData(pCtx, &usb_mode);
	if ( CH375_SUCCESS != ret ) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);

	if (CH375_CMD_RET_SUCCESS != usb_mode) {
		LOG_ERR("Set USB mode failed: ret=0x%02X", usb_mode);
		return CH375_ERROR;
	}

	return CH375_SUCCESS;
}

/**
  * @brief Get CH375 interrupt status
  * @param pCtx The context
  * @param pStatus pointer to the interrupt status buffer
  * @retval 0 on success, error code otherwise
  */
int ch375_getStatus(struct Ch375_Context_t *pCtx, uint8_t *pStatus) {

	if (NULL == pCtx || NULL == pStatus) {
		LOG_ERR("Invalid parameters!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t status = -1;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_GET_STATUS);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_readData(pCtx, &status);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}

	*pStatus = status;

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Abort NAK
  * @param pCtx The context
  * @retval 0 on success, error code otherwise
  */
int ch375_abortNAK(struct Ch375_Context_t *pCtx) {

	if ( NULL == pCtx ) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_ABORT_NAK);

	k_mutex_unlock(&pCtx->lock);

	return ret == CH375_SUCCESS ? CH375_SUCCESS : CH375_WRITE_CMD_FAILED;
}

/**
  * @brief Trigger INT pin
  * @param pCtx The context
  * @retval query result (1 if INT is asserted, 0 if not asserted)
  */
int ch375_queryInt(struct Ch375_Context_t *pCtx) {
	
	if ( NULL == pCtx ) {
		LOG_ERR("Invalid context!");
		return 0;
	}

	return pCtx->query_int(pCtx);
}

/**
  * @brief Wait for INT pin
  * @param pCtx The context
  * @param timeout_ms Timeout in ms
  * @retval 0 on success, timeout error otherwise
  */
int ch375_waitInt(struct Ch375_Context_t *pCtx, uint32_t timeout_ms) {

	if ( NULL == pCtx ) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	uint32_t start = k_uptime_get_32();
	
	while((k_uptime_get_32()-start) < timeout_ms) {
		if (0 != ch375_queryInt(pCtx)) {
			return CH375_SUCCESS;
		}
		k_msleep(1);
	}

	return CH375_TIMEOUT;
}

/**
 * @brief Host commands
 */

/**
  * @brief Query the connection status of the current USB device
  * @param pCtx The context
  * @param pConnStatus The connection status
  * @retval 0 on success, error code otherwise
  */
int ch375_testConnect(struct Ch375_Context_t *pCtx, uint8_t *pConnStatus) {

	if (NULL == pCtx || NULL == pConnStatus) {
		LOG_ERR("Invalid parameters!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t status = 0;
	uint8_t buff = 0;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx,CH375_CMD_TEST_CONNECT);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	k_msleep(1);

	ret = ch375_readData(pCtx, &buff);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}
	
	if (CH375_USB_INT_DISCONNECT != buff && CH375_USB_INT_CONNECT != buff &&
		CH375_USB_INT_USB_READY != buff) {
			buff = CH375_USB_INT_DISCONNECT;
	}

	if (CH375_USB_INT_DISCONNECT == buff) {
		ch375_getStatus(pCtx, &status);
	}

	*pConnStatus = buff;
	return CH375_SUCCESS;
}

/**
  * @brief Get the connection status
  * @param pCtx The context
  * @param pSpeed The device operation speed
  * @retval 0 on success, error code otherwise
  */
int ch375_getDevSpeed(struct Ch375_Context_t *pCtx, uint8_t *pSpeed) {
	
	if (NULL == pCtx || NULL == pSpeed) {
		LOG_ERR("Invalid parameters!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t devSpeed;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret  = ch375_writeCmd(pCtx, CH375_CMD_GET_DEV_RATE);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	// get device rate data
	ret = ch375_writeData(pCtx, 0x07);
	if ( ret != CH375_SUCCESS) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_readData(pCtx, &devSpeed);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_READ_DATA_FAILED;
	}

	*pSpeed = (devSpeed & 0x10) ? USB_SPEED_SPEED_LS : USB_SPEED_SPEED_FS;

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Set the device speed
  * @param pCtx The context
  * @param speed The device speed
  * @retval 0 on success, error code otherwise
  */
int ch375_setDevSpeed(struct Ch375_Context_t *pCtx, uint8_t speed) {
	
	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t devSpeed = 0;

	if ( USB_SPEED_SPEED_LS != speed && USB_SPEED_SPEED_FS != speed) {
		LOG_ERR("Invalid speed value: 0x%02X", speed);
		return CH375_PARAM_INVALID;
	}

	devSpeed = (speed == USB_SPEED_SPEED_LS ? 0x02 : 0x00);

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_SET_USB_SPEED);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, devSpeed);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Asign new fixed address for HID
  * @param pCtx The context
  * @param addr The device address
  * @retval 0 on success, error code otherwise
  */
int ch375_setUSBAddr(struct Ch375_Context_t *pCtx, uint8_t addr) {

	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	
	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_SET_USB_ADDR);
	if ( CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, addr);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);
	return CH375_SUCCESS;
}

/**
  * @brief Asign new fixed address for HID
  * @param pCtx The context
  * @param addr The device address
  * @retval 0 on success, error code otherwise
  */
int ch375_setRetry(struct Ch375_Context_t *pCtx, uint8_t times) {

	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t param;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_SET_RETRY);
	if ( CH375_SUCCESS != ret ) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	// Set retry data
	ret = ch375_writeData(pCtx, 0x25);
	if (ret != CH375_SUCCESS) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	if (0 == times) {
		// No retry, NAK all the time
		param = 0x05;
	} else if  (1 == times) {
		// Retry 200ms
		param = 0xC0;
	} else {
		// Infinite retry
		param = 0x85;
	}

	ret = ch375_writeData(pCtx, param);
    if (CH375_SUCCESS != ret) {
        k_mutex_unlock(&pCtx->lock);
        return CH375_WRITE_CMD_FAILED;
    }
    
    k_mutex_unlock(&pCtx->lock);
    return CH375_SUCCESS;
}

/**
  * @brief Send a token to CH375
  * @param pCtx The context
  * @param ep The endpoint number
  * @param tog The toggle value (between DATA0 and DATA1)
  * @param pid The PID
  * @param pStatus The pointer to the variable that will store the status
  * @retval 0 on success, error code otherwise
  */
int ch375_sendToken(struct Ch375_Context_t *pCtx, uint8_t ep, uint8_t tog,
                    uint8_t pid, uint8_t *pStatus) {
	
	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	int ret = -1;
	uint8_t togVal = 0;
	uint8_t epPID = 0;
	uint8_t status = -1;

	if (NULL == pCtx || NULL == pStatus) {
		LOG_ERR("Invalid parameters");
		return CH375_PARAM_INVALID;
	}

	// if tog == 1 -> DATA1, else DATA0
	togVal = tog ? 0xC0 : 0x00;

	// 4 MSBs are EP number and the rest is PID token
	epPID = (ep << 4) | pid;

	k_mutex_lock(&pCtx->lock, K_FOREVER);

	ret = ch375_writeCmd(pCtx, CH375_CMD_ISSUE_TKN_X);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, togVal);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	ret = ch375_writeData(pCtx, epPID);
	if (CH375_SUCCESS != ret) {
		k_mutex_unlock(&pCtx->lock);
		return CH375_WRITE_CMD_FAILED;
	}

	k_mutex_unlock(&pCtx->lock);

	if ( USB_PID_IN != pid) {
		// 500us delay for IN tokens
		k_busy_wait(500);
	}

	// Wait for INT
	ret = ch375_waitInt(pCtx, WAIT_INT_TIMEOUT_MS);
    if (CH375_SUCCESS != ret) {
        return CH375_TIMEOUT;
    }

	ret = ch375_getStatus(pCtx, &status);
	if (CH375_SUCCESS != ret) {
        return CH375_ERROR;
    }

	*pStatus = status;
	return CH375_SUCCESS;
}

/**
 * @brief Data transfer commands
 */

/**
  * @brief Writes a command to the device
  * @param pCtx The context
  * @param cmd The command to write
  * @retval The result of the write
  */
int ch375_writeCmd(struct Ch375_Context_t *pCtx, uint8_t cmd) {
	
	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	return pCtx->write_cmd(pCtx, cmd);
}

/**
  * @brief Writes data to the device
  * @param pCtx The context
  * @param data The data to write
  * @retval The result of the write
  */
int ch375_writeData(struct Ch375_Context_t *pCtx, uint8_t data) {

	if (NULL == pCtx) {
		LOG_ERR("Invalid context!");
		return CH375_PARAM_INVALID;
	}

	return pCtx->write_data(pCtx, data);
}

/**
  * @brief Read data from the device
  * @param pCtx The context
  * @param pData Pointer to 1 byte buffer to store data in 
  * @retval The result of the write
  */
int ch375_readData(struct Ch375_Context_t *pCtx, uint8_t *pData) {
	
	if (NULL == pCtx || NULL == pData) {
		LOG_ERR("Invalid parameters!");
		return CH375_PARAM_INVALID;
	}

	return pCtx->read_data(pCtx, pData);
}