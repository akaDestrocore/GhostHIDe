/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           mock_ch375_hw.c
 * @brief          CH375 hardware mock implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements mock CH375 hardware callbacks for unit testing including
 * write command/data, read data, and INT pin query. Maintains response
 * queue and command/data history for test assertions.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include "ch375.h"
#include "mock_ch375_hw.h"

#define MOCK_HISTORY_SIZE 128
#define MOCK_STATUS_QUEUE_SIZE 64

static int mockWriteCmdFail = 0;
static uint8_t mockLastCmd = 0;
static int mockCmdHistoryCount = 0;
static uint8_t mockCmdHistory[MOCK_HISTORY_SIZE];
static int mockWriteDataFail = 0;
static uint8_t mockLastData = 0;
static int mockRespHead = 0;
static int mockRespTail = 0;
static uint8_t mockRespQueue[256];
static bool mockIntState = false;
static int mockReadDataFail = 0;
static int mockDataHistoryCount = 0;
static uint8_t mockDataHistory[MOCK_HISTORY_SIZE];
static int mockStatusHead = 0;
static int mockStatusTail = 0;
static uint8_t mockStatusQueue[MOCK_STATUS_QUEUE_SIZE];
static uint8_t mockDefaultStatus = 0x00;

static int mock_writeCmd(struct ch375_Context_t *ctx, uint8_t cmd)
{
    if (mockWriteCmdFail) {
        return CH375_ERROR;
    }
    
    mockLastCmd = cmd;
    if (mockCmdHistoryCount < MOCK_HISTORY_SIZE) {
        mockCmdHistory[mockCmdHistoryCount++] = cmd;
    }
    
    return CH375_SUCCESS;
}

static int mock_writeData(struct ch375_Context_t *ctx, uint8_t data)
{
    if (mockWriteDataFail) {
        return CH375_ERROR;
    }
    
    mockLastData = data;
    if (mockDataHistoryCount < MOCK_HISTORY_SIZE) {
        mockDataHistory[mockDataHistoryCount++] = data;
    }
    
    return CH375_SUCCESS;
}

static int mock_readData(struct ch375_Context_t *ctx, uint8_t *data)
{
    if (mockReadDataFail) {
        return CH375_ERROR;
    }
    
    // Check if this is a GET_STATUS
    if (mockLastCmd == CH375_CMD_GET_STATUS) {
        // Return from status queue if available
        if (mockStatusHead != mockStatusTail) {
            *data = mockStatusQueue[mockStatusTail];
            mockStatusTail = (mockStatusTail + 1) % MOCK_STATUS_QUEUE_SIZE;
            return CH375_SUCCESS;
        }
        // Return default status
        *data = mockDefaultStatus;
        return CH375_SUCCESS;
    }
    
    // Regular response queue for other reads
    if (mockRespHead == mockRespTail) {
        return CH375_TIMEOUT;
    }
    
    *data = mockRespQueue[mockRespTail];
    mockRespTail = (mockRespTail + 1) % 256;
    
    return CH375_SUCCESS;
}

static int mock_queryInt(struct ch375_Context_t *ctx)
{
    return mockIntState ? 1 : 0;
}

int mock_ch375Init(struct ch375_Context_t **ppCtx)
{
    mock_ch375Reset();

    return ch375_openContext(ppCtx, mock_writeCmd, mock_writeData, mock_readData, mock_queryInt, NULL);
}

void mock_ch375Reset(void)
{
    mockRespHead = 0;
    mockRespTail = 0;
    mockLastCmd = 0;
    mockLastData = 0;
    mockIntState = false;
    mockWriteCmdFail = 0;
    mockWriteDataFail = 0;
    mockReadDataFail = 0;
    mockCmdHistoryCount = 0;
    mockDataHistoryCount = 0;
    mockStatusHead = 0;
    mockStatusTail = 0;
    mockDefaultStatus = 0x00;
}

void mock_ch375QueueResponse(uint8_t data)
{
    mockRespQueue[mockRespHead] = data;
    mockRespHead = (mockRespHead + 1) % 256;
}

void mock_ch375QueueResponses(const uint8_t *pData, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        mock_ch375QueueResponse(pData[i]);
    }
}

void mock_ch375QueueStatus(uint8_t status)
{
    if ((mockStatusHead + 1) % MOCK_STATUS_QUEUE_SIZE != mockStatusTail) {
        mockStatusQueue[mockStatusHead] = status;
        mockStatusHead = (mockStatusHead + 1) % MOCK_STATUS_QUEUE_SIZE;
    }
}

void mock_ch375QueueStatuses(const uint8_t *pStatuses, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        mock_ch375QueueStatus(pStatuses[i]);
    }
}

void mock_ch375SetDefaultStatus(uint8_t status)
{
    mockDefaultStatus = status;
}

void mock_ch375SetIntState(bool asserted)
{
    mockIntState = asserted;
}

void mock_ch375SetWriteCmdFail(bool fail)
{
    mockWriteCmdFail = fail ? 1 : 0;
}

void mock_ch375SetWriteDataFail(bool fail)
{
    mockWriteDataFail = fail ? 1 : 0;
}

void mock_ch375SetReadDataFail(bool fail)
{
    mockReadDataFail = fail ? 1 : 0;
}

uint8_t mock_ch375GetLastCmd(void)
{
    return mockLastCmd;
}

uint8_t mock_ch375GetLastData(void)
{
    return mockLastData;
}

bool mock_ch375VerifyCmdSent(uint8_t cmd)
{
    for (int i = 0; i < mockCmdHistoryCount; i++) {
        if (mockCmdHistory[i] == cmd) {
            return true;
        }
    }
    return false;
}

int mock_ch375GetCmdCount(uint8_t cmd)
{
    int count = 0;
    for (int i = 0; i < mockCmdHistoryCount; i++) {
        if (mockCmdHistory[i] == cmd) {
            count++;
        }
    }

    return count;
}

void mock_ch375GetDataHistory(uint8_t *pBuff, int *pCount, int max_count)
{
    int copy_count = mockDataHistoryCount < max_count ? 
                     mockDataHistoryCount : max_count;
    memcpy(pBuff, mockDataHistory, copy_count);
    *pCount = copy_count;
}