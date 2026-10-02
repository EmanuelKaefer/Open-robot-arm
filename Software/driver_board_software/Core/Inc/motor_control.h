/*
 * motor_control.h
 *
 *  Created on: 02.10.2026
 *      Author: matteo gassler
 *
 */
/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
 * See LICENSE-SOFTWARE in the project root for details.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"   // Wichtig für uint8_t, TIM_HandleTypeDef, HAL-Makros
#include <math.h>   // Für fabsf, sqrtf

// Das Struct muss hierher, damit beide Dateien wissen, was eine "AxisRamp" ist
typedef struct {
    float    current_freq;
    float    vmax;
    float    accel;
    float    pos;
    float    target;
    uint8_t  direction;
    uint8_t  active;
} AxisRamp;

// Externe Deklaration der Variablen (Speicher wird in der .c reserviert)
extern AxisRamp axisRamp[6];
extern volatile uint32_t move_start_time;
extern volatile uint32_t axis_finish_time[6];
extern volatile uint8_t print_results_flag;

// Öffentliche Funktionen (Prototypen)
void UpdateAllRamps(float dt, uint8_t min_freq);
void StartMove(uint8_t ax, float target_steps, float vmax, float accel);
void StartSyncedMove(float target_steps[6], float accel, float vmax_max, uint8_t axis_count);

#endif /* MOTOR_CONTROL_H */