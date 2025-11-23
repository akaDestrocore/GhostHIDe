/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           input_patterns.с
 * @brief          Dynamic modification of HID data with specific patterns implementation
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Implements recoil compensation.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "input_patterns.h"

LOG_MODULE_REGISTER(input_patterns, LOG_LEVEL_INF);

/**
 * @brief Soldier 76 recoil pattern
 * {x_offset, y_offset, time_ms}
 */
static const float gRawPreset_OW2_Soldier76[] = {
    +0.00000, +0.00000, 111,
    +0.00000, -1.45500, 111,
    +0.00000, +0.47045, 111,
    +0.00000, -1.36901, 111,
    +0.00000, +0.44265, 111,
    +0.00000, -0.85873, 111,
    +0.00000, +0.41649, 111,
    +0.00000, -0.80798, 111,
    +0.00000, +0.39187, 111,
    +0.00000, -0.38012, 111,
    +0.00000, +0.36871, 111,
    +0.00000, -0.35765, 111,
    +0.00000, +0.34692, 111,
    +0.00000, -0.33651, 111,
    +0.00000, +0.32642, 111,
    +0.00000, -0.18998, 111,
    +0.00000, +0.18428, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.17339, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
    +0.00000, +0.00000, 111,
};

/**
 * @brief Cassidy preset recoil pattern
 */
static const float gRawPreset_OW2_Cassidy[] = {
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
    +0.00000, +0.00000, 50,
    +0.00000, -20.20000, 150,
    +0.00000, +0.00000, 300,
};

static const PresetCollection_t sRecoilCollectArr[RECOIL_COMP_PRESET_COUNT] = {
    
    {
        .pData = NULL,
        .dataLen = 0,
        .firerounds_sampling = 0,
    },

    {
        .pData = gRawPreset_OW2_Soldier76,
        .dataLen = ARRAY_SIZE(gRawPreset_OW2_Soldier76),
        .firerounds_sampling = round(111/USB_REPORT_INTERVAL),
    },

    {
        .pData = gRawPreset_OW2_Cassidy,
        .dataLen = ARRAY_SIZE(gRawPreset_OW2_Cassidy),
        .firerounds_sampling = round(500/USB_REPORT_INTERVAL),
    },
};

/* Private Function Prototypes ------------------------------------------------*/
static void recoilComp_cbFreeLocked(struct RecoilComp_Context_t* pCtx);
static uint32_t recoilComp_cbTimeElapsed(uint32_t start, uint32_t now);
static int recoilComp_cbGenerateDataLocked(struct RecoilComp_Context_t* pCtx);

/**
 * @brief Initialize recoil compensation context
 * @param ppCtx Pointer to context pointer for output
 * @return 0 on success, negative error code otherwise
 */
int recoilComp_open(struct RecoilComp_Context_t** ppCtx) {
    
    struct RecoilComp_Context_t* pNewCtx;

    if (NULL == ppCtx) {
        return -EINVAL;
    }

    pNewCtx = k_malloc(sizeof(struct RecoilComp_Context_t));
    if (NULL == pNewCtx) {
        LOG_ERR("Failed to allocate space for new context");
        return -ENOMEM;
    }

    memset(pNewCtx, 0x00, sizeof(struct RecoilComp_Context_t));
    k_mutex_init(&pNewCtx->lock);

    pNewCtx->coefficient = RECOIL_COMP_DEFAULT_COEFF;
    pNewCtx->sensitivity = RECOIL_COMP_DEFAULT_SENS;
    pNewCtx->stateFlags = RECOIL_COMP_STATE_INITIALIZED;

    *ppCtx = pNewCtx;

    LOG_INF("[ OK ] Recoil compensation context initialized");
    return 0;
}

/**
 * @brief Close recoil compensation context and free resources
 * @param pCtx Pointer to context
 */
void recoilComp_close(struct RecoilComp_Context_t* pCtx) {

    if (NULL == pCtx) {
        return;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);
    recoilComp_cbFreeLocked(pCtx);
    pCtx->stateFlags = RECOIL_COMP_STATE_UNINITIALIZED;
    k_mutex_unlock(&pCtx->lock);

    k_free(pCtx);
}

/**
 * @brief Restart recoil compensation
 * @param pCtx Pointer to context
 * @return 0 on success, negative error code otherwise
 */
int recoilComp_restart(struct RecoilComp_Context_t* pCtx) {

    if (NULL == pCtx) {
        return -EINVAL;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);

    // Not initialized
    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_INITIALIZED)) {
        k_mutex_unlock(&pCtx->lock);
        LOG_ERR("Context not initialized");
        return -EINVAL;
    }

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_PRESET_ACTIVE)) {
        k_mutex_unlock(&pCtx->lock);
        LOG_ERR("Preset not selected.");
        return -EINVAL;
    }

    pCtx->lastTickMs = k_uptime_get_32();
    pCtx->arrIndex = 0;

    k_mutex_unlock(&pCtx->lock);

    return 0;
}

/**
 * @brief Get next compensated movement data
 * @param pCtx Pointer to context
 * @param pData Pointer to output data structure
 * @return 0 if data available, error code otherwise
 */
int recoilComp_getNextData(struct RecoilComp_Context_t* pCtx, struct PatternCompensation_t* pData) {

    int ret = -1;
    uint32_t now = 0;
    uint32_t elapsed = 0;

    if (NULL == pCtx || NULL == pData) {
        return -EINVAL;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_INITIALIZED)) {
        k_mutex_unlock(&pCtx->lock);
        return -EINVAL;
    }

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_ARRAYS_ALLOCATED)) {
        k_mutex_unlock(&pCtx->lock);
        return -EINVAL;
    }

    // Check if sequence is complete
    if (pCtx->arrIndex >= pCtx->arrLen) {
        k_mutex_unlock(&pCtx->lock);
        return -1;
    }

    now = k_uptime_get_32();
    elapsed = recoilComp_cbTimeElapsed(pCtx->lastTickMs, now);

    // Check if time has passed for next shot
    if (elapsed >= (uint32_t)pCtx->pTs[pCtx->arrIndex]) {
        pCtx->lastTickMs += (uint32_t)pCtx->pTs[pCtx->arrIndex];

        pData->x = pCtx->pX[pCtx->arrIndex];
        pData->y = -1 * pCtx->pY[pCtx->arrIndex];

        pCtx->arrIndex++;
        ret = 0;
    } else {
        ret = -1;
    }

    k_mutex_unlock(&pCtx->lock);
    
    return ret;
}

/**
 * @brief Set current preset and load compensation profile
 * @param pCtx Pointer to context
 * @param presetIndex Preset enum value
 * @return 0 on success, error code otherwise
 */
int recoilComp_setPreset(struct RecoilComp_Context_t* pCtx, uint32_t presetIndex) {
    
    int ret = -1;

    if (NULL == pCtx) {
        return -EINVAL;
    }

    if (presetIndex >= RECOIL_COMP_PRESET_COUNT) {
        LOG_ERR("Invalid preset index: %u", presetIndex);
        return -EINVAL;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_INITIALIZED)) {
        k_mutex_unlock(&pCtx->lock);
        LOG_ERR("Context not initialized");
        return -EINVAL;
    }

    // Free old data
    recoilComp_cbFreeLocked(pCtx);

    // Set new collection
    pCtx->pCollect = &sRecoilCollectArr[presetIndex];

    // Generate new data
    ret = recoilComp_cbGenerateDataLocked(pCtx);
    if (ret < 0) {
        pCtx->pCollect = NULL;
        LOG_ERR("Failed to generate recoil data: %d", ret);
        k_mutex_unlock(&pCtx->lock);
        return ret;
    }

    pCtx->stateFlags |= RECOIL_COMP_STATE_PRESET_ACTIVE;
    k_mutex_unlock(&pCtx->lock);

    LOG_INF("Preset active: %u", presetIndex);
    return 0;
}

/**
 * @brief Adjust compensation coefficient
 * @param pCtx Pointer to context
 * @param isAdd true to increase, false to decrease
 * @return 0 on success, error code otherwise
 */
int recoilComp_changeCoefficient(struct RecoilComp_Context_t* pCtx, bool isAdd) {
    
    if (NULL == pCtx) {
        return -EINVAL;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_INITIALIZED)) {
        k_mutex_unlock(&pCtx->lock);
        return -EINVAL;
    }

    pCtx->coefficient = CLAMP(pCtx->coefficient + (isAdd ? RECOIL_COMP_COEFF_STEP : -RECOIL_COMP_COEFF_STEP), 
                                                                RECOIL_COMP_COEFF_MIN, RECOIL_COMP_COEFF_MAX);

    if (pCtx->stateFlags & RECOIL_COMP_STATE_PRESET_ACTIVE) {
        recoilComp_cbFreeLocked(pCtx);
        int ret = recoilComp_cbGenerateDataLocked(pCtx);
        if (ret < 0) {
            LOG_ERR("Failed to regenerate compensation data: %d", ret);
        } else {
            LOG_INF("Compensation data regenerated successfully");
        }
    }

    LOG_INF("Coefficient: %.2f", (double)pCtx->coefficient);
    k_mutex_unlock(&pCtx->lock);

    return 0;
}

/**
 * @brief Adjust compensation sensitivity
 * @param pCtx Pointer to context
 * @param isAdd true to increase, false to decrease
 * @return 0 on success, negative error code otherwise
 */
int recoilComp_changeSensitivity(struct RecoilComp_Context_t* pCtx, bool isAdd) {

    if (NULL == pCtx) {
        return -EINVAL;
    }

    k_mutex_lock(&pCtx->lock, K_FOREVER);

    if (0 == (pCtx->stateFlags & RECOIL_COMP_STATE_INITIALIZED)) {
        k_mutex_unlock(&pCtx->lock);
        return -EINVAL;
    }

    pCtx->sensitivity = CLAMP( pCtx->sensitivity + (isAdd ? RECOIL_COMP_SENS_STEP : -RECOIL_COMP_SENS_STEP), 
                                                                RECOIL_COMP_SENS_MIN, RECOIL_COMP_SENS_MAX);

    if (pCtx->stateFlags & RECOIL_COMP_STATE_PRESET_ACTIVE) {
        recoilComp_cbFreeLocked(pCtx);
        int ret = recoilComp_cbGenerateDataLocked(pCtx);
        if (ret < 0) {
            LOG_ERR("Failed to regenerate compensation data: %d", ret);
        } else {
            LOG_INF("Compensation data regenerated successfully");
        }
    }

    LOG_INF("Sensitivity: %.2f", (double)pCtx->sensitivity);
    k_mutex_unlock(&pCtx->lock);
    
    return 0;
}

/* --------------------------------------------------------------------------
 * HELPER FUNCTIONS
 * -------------------------------------------------------------------------*/
static void recoilComp_cbFreeLocked(struct RecoilComp_Context_t* pCtx) {
    // Lock must be held by caller
    if (NULL != pCtx->pX) {
        k_free(pCtx->pX);
        pCtx->pX = NULL;
    }

    if (NULL != pCtx->pY) {
        k_free(pCtx->pY);
        pCtx->pY = NULL;
    }

    if (NULL != pCtx->pTs) {
        k_free(pCtx->pTs);
        pCtx->pTs = NULL;
    }

    pCtx->arrLen = 0;
    pCtx->arrIndex = 0;
    pCtx->stateFlags &= ~RECOIL_COMP_STATE_ARRAYS_ALLOCATED;
}

static uint32_t recoilComp_cbTimeElapsed(uint32_t start, uint32_t now) {
    
    if (now >= start) {
        return now - start;
    }

    return (0xFFFFFFFF - start) + now + 1;
}

static int recoilComp_cbGenerateDataLocked(struct RecoilComp_Context_t* pCtx) {
    
    int ret = -1;
    const PresetCollection_t* pColl;
    int rawIdx = 0;
    int idx = 0;
    int i = 0;
    int j = 0;
    int groups = 0;
    float x = 0;
    float y = 0;
    float timestamp = 0;
    float sensitivity = 0;
    int32_t sX = 0;
    int32_t sY = 0;
    int32_t sTimestamp = 0; 
    int32_t fixX = 0; 
    int32_t fixY = 0; 
    int32_t fixTs = 0;
    float sumX = 0;
    float sumY = 0;
    float sumTs = 0;
    float sumX0 = 0;
    float sumY0 = 0;
    float sumTs0 = 0;

    if (NULL == pCtx) {
        return -EINVAL;
    }

    pColl = pCtx->pCollect;
    if (NULL == pColl) {
        LOG_ERR("Invalid collection data");
        return -EINVAL;
    }

    if (NULL == pColl->pData) {
        LOG_ERR("Invalid collection data");
        return -EINVAL;
    }

    if ((pColl->dataLen <= 0) || (0 != (pColl->dataLen % RECOIL_COMP_DATA_GROUP_SIZE))) {
        LOG_ERR("Invalid collection data");
        return -EINVAL;
    }

    if (pColl->firerounds_sampling <= 0) {
        LOG_ERR("Invalid collection data");
        return -EINVAL;
    }

    // Clamp parameters
    pCtx->coefficient = CLAMP(pCtx->coefficient, RECOIL_COMP_COEFF_MIN, RECOIL_COMP_COEFF_MAX);
    pCtx->sensitivity = CLAMP(pCtx->sensitivity, RECOIL_COMP_SENS_MIN, RECOIL_COMP_SENS_MAX);

    sensitivity = pCtx->sensitivity;
    groups = pColl->dataLen / RECOIL_COMP_DATA_GROUP_SIZE;
    pCtx->arrLen = pColl->firerounds_sampling * groups;

    // Allocate arrays
    pCtx->pX = k_malloc(sizeof(int32_t) * pCtx->arrLen);
    if (NULL == pCtx->pX) {
        LOG_ERR("Failed to allocate X array");
        return -ENOMEM;
    }

    pCtx->pY = k_malloc(sizeof(int32_t) * pCtx->arrLen);
    if (NULL == pCtx->pY) {
        LOG_ERR("Failed to allocate Y array");
        k_free(pCtx->pX);
        pCtx->pX = NULL;
        return -ENOMEM;
    }

    pCtx->pTs = k_malloc(sizeof(int32_t) * pCtx->arrLen);
    if (NULL == pCtx->pTs) {
        k_free(pCtx->pX);
        k_free(pCtx->pY);
        pCtx->pX = NULL;
        pCtx->pY = NULL;
        LOG_ERR("Failed to allocate timestamp array");
        return -ENOMEM;
    }

    // Generate data with error correction for rounding
    for (i = 0; i < groups; i++) {
        rawIdx = i * RECOIL_COMP_DATA_GROUP_SIZE;

        x = pColl->pData[rawIdx + 0] * pCtx->coefficient / sensitivity;
        y = pColl->pData[rawIdx + 1] * pCtx->coefficient / sensitivity;
        timestamp = pColl->pData[rawIdx + 2];

        sX = (int32_t)floorf( x / (float)pColl->firerounds_sampling);
        sY = (int32_t)floorf( y / (float)pColl->firerounds_sampling);
        sTimestamp = (int32_t)floorf( timestamp / (float)pColl->firerounds_sampling);

        sumX += sX * pColl->firerounds_sampling;
        sumY += sY * pColl->firerounds_sampling;
        sumTs += sTimestamp * pColl->firerounds_sampling;
        sumX0 += x;
        sumY0 += y;
        sumTs0 += timestamp;

        fixX = (int32_t)roundf(sumX0 - sumX);
        fixY = (int32_t)roundf(sumY0 - sumY);
        fixTs = (int32_t)roundf(sumTs0 - sumTs);

        for (j = 0; j < pColl->firerounds_sampling; j++) {
            
            if (idx >= pCtx->arrLen) {
                break;
            }

            pCtx->pX[idx] = sX;
            pCtx->pY[idx] = sY;
            pCtx->pTs[idx] = sTimestamp;

            if (fixX > 0) {
                pCtx->pX[idx]++;
                sumX++;
                fixX--;
            }

            if (fixY > 0) {
                pCtx->pY[idx]++;
                sumY++;
                fixY--;
            }

            if (fixTs > 0) {
                pCtx->pTs[idx]++;
                sumTs++;
                fixTs--;
            }

            idx++;
        }
    }

    pCtx->stateFlags |= RECOIL_COMP_STATE_ARRAYS_ALLOCATED;

    return 0;
}