/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║                          GhostHIDe Project                            ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 * 
 * @file           input_patterns.h
 * @brief          Dynamic modification of HID data with specific patterns
 * 
 * @author         destrocore
 * @date           2025
 * 
 * @details
 * Provides recoil compensation. Each recoil patterns that are compensated through
 * mouse movement adjustments.
 * 
 * @copyright 
 * Copyright (c) 2025 akaDestrocore
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef INPUT_PATTERNS_H
#define INPUT_PATTERNS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Macros -------------------------------------------------------------------*/
#define USB_REPORT_INTERVAL             8
#define RECOIL_COMP_DEFAULT_COEFF       1.0f
#define RECOIL_COMP_DEFAULT_SENS        2.5f
#define RECOIL_COMP_COEFF_MIN           0.1f
#define RECOIL_COMP_COEFF_MAX           10.0f
#define RECOIL_COMP_PRESET_COUNT        3
#define RECOIL_COMP_DATA_GROUP_SIZE     3
#define RECOIL_COMP_SENS_MIN            0.1f
#define RECOIL_COMP_SENS_MAX            100.0f
#define RECOIL_COMP_COEFF_STEP          0.1f
#define RECOIL_COMP_SENS_STEP           0.1f

/* Type Definitions ---------------------------------------------------------*/
/**
 * @brief Pattern Presets
 */
typedef enum {
    TEMPLATE_NONE,
    TEMPLATE_OW2_SOLDIER76,
    TEMPLATE_OW2_CASSIDY
} PatternPreset_e;

/**
 * @brief Context state flags
 */
typedef enum {
    RECOIL_COMP_STATE_UNINITIALIZED     = 0,
    RECOIL_COMP_STATE_INITIALIZED       = 1,
    RECOIL_COMP_STATE_PRESET_ACTIVE     = 2,
    RECOIL_COMP_STATE_ARRAYS_ALLOCATED  = 4
} RecoilCompState_e;

/**
 * @brief Recoil data collection structure
 */
typedef struct {
    const float* pData;
    int dataLen;
    int firerounds_sampling;
} PresetCollection_t;

/**
 * @brief Compensation axes X and Y
 */
struct PatternCompensation_t {
    int32_t x;
    int32_t y;
};

/**
 * @brief Recoil compensation context structure
 */
struct RecoilComp_Context_t {
    float coefficient;
    float sensitivity;
    uint32_t stateFlags;

    // Preset data
    const PresetCollection_t* pCollect;
    int32_t* pX;
    int32_t* pY;
    int32_t* pTs;
    int arrLen;

    // Sequence state
    int arrIndex;
    uint32_t lastTickMs;

    struct k_mutex lock;
};

/**
 * @brief Initialize recoil compensation context
 */
int recoilComp_open(struct RecoilComp_Context_t** ppCtx);

/**
 * @brief Close recoil compensation context
 */
void recoilComp_close(struct RecoilComp_Context_t* pCtx);

/**
 * @brief Restart recoil compensation sequence
 */
int recoilComp_restart(struct RecoilComp_Context_t* pCtx);

/**
 * @brief Get next compensated movement data
 */
int recoilComp_getNextData(struct RecoilComp_Context_t* pCtx, struct PatternCompensation_t* pData);

/**
 * @brief Set current preset and load compensation profile
 */
int recoilComp_setPreset(struct RecoilComp_Context_t* pCtx, uint32_t presetIndex);

/**
 * @brief Adjust compensation strength
 */
int recoilComp_changeCoefficient(struct RecoilComp_Context_t* pCtx, bool isAdd);

/**
 * @brief Adjust compensation sensitivity
 */
int recoilComp_changeSensitivity(struct RecoilComp_Context_t* pCtx, bool isAdd);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_PATTERNS_H */