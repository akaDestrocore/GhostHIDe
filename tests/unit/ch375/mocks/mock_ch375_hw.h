/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           mock_ch375_hw.h
 * @brief          CH375 hardware mock for unit testing with polling support
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Mock implementation of CH375 hardware interface for isolated unit
 * testing. Provides response queue, status queue for polling mode,
 * INT pin simulation, and command/data history tracking for test verification.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

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
 * @brief Queue a status value for GET_STATUS polling
 * @param status Status byte to queue
 * @note Use this for control of status polling sequences.
 *       If not used, mock_ch375QueueResponse() will work for both data and status.
 */
void mock_ch375QueueStatus(uint8_t status);

/**
 * @brief Queue multiple status values
 * @param pStatuses Array of status bytes
 * @param len Number of status bytes
 * @note Use for complex status sequences. Regular queue works for most tests.
 */
void mock_ch375QueueStatuses(const uint8_t *pStatuses, size_t len);

/**
 * @brief Set default status returned when status queue is empty
 * @param status Default status byte
 * @note Used as fallback when both queues are empty during GET_STATUS
 */
void mock_ch375SetDefaultStatus(uint8_t status);

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