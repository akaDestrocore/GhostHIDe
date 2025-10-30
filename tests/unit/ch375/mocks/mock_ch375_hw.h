#ifndef MOCK_CH375_HW_H
#define MOCK_CH375_HW_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ch375.h"

/**
 * @brief Initialize mock CH375 context
 * @param ppCtx Pointer to context pointer
 * @return 0 on success
 */
int mock_ch375Init(struct ch375_Context_t **ppCtx);

/**
 * @brief Reset mock state
 */
void mock_ch375Reset(void);

/**
 * @brief Queue a response byte
 * @param data Response byte to queue
 */
void mock_ch375QueueResponse(uint8_t data);

/**
 * @brief Queue multiple response bytes
 * @param pData Array of bytes to queue
 * @param len Number of bytes
 */
void mock_ch375QueueResponses(const uint8_t *pData, size_t len);

/**
 * @brief Set INT pin state
 * @param asserted true if INT should be asserted (low)
 */
void mock_ch375SetIntState(bool asserted);

/**
 * @brief Make next write command fail
 * @param fail true to make fail
 */
void mock_ch375SetWriteCmdFail(bool fail);

/**
 * @brief Make next write data fail
 * @param fail true to make fail
 */
void mock_ch375SetWriteDataFail(bool fail);

/**
 * @brief Make next read data fail
 * @param fail true to make fail
 */
void mock_ch375SetReadDataFail(bool fail);

/**
 * @brief Get last command sent
 * @return Last command byte
 */
uint8_t mock_ch375GetLastCmd(void);

/**
 * @brief Get last data sent
 * @return Last data byte
 */
uint8_t mock_ch375GetLastData(void);

/**
 * @brief Verify a command was sent
 * @param cmd Command to check
 * @return true if command was sent
 */
bool mock_ch375VerifyCmdSent(uint8_t cmd);

/**
 * @brief Count how many times a command was sent
 * @param cmd Command to count
 * @return Number of times sent
 */
int mock_ch375GetCmdCount(uint8_t cmd);

/**
 * @brief Get data write history
 * @param pBuff Buffer to store history
 * @param pCount Pointer to store count
 * @param max_count Maximum to copy
 */
void mock_ch375GetDataHistory(uint8_t *pBuff, int *pCount, int max_count);

#endif /* MOCK_CH375_HW_H */