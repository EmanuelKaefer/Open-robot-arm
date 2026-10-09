/*
 * motion.h - Sollwert-Puffer + Folgeregler (Testversion mit virtuellen Achsen)
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
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

#define N_AX    6
#define FIFO_N  8U               /* Zweierpotenz! */

enum { MC_DISABLED = 0, MC_IDLE = 1, MC_RUNNING = 2, MC_STOPPING = 3, MC_FAULT = 4 };
enum { FL_UNDERRUN = 1, FL_OVERRUN = 2, FL_CFG_OK = 4, FL_JUMP = 8 };

void    MC_Init(void);
int     MC_Config(uint16_t period_ms, const float vmax[N_AX], const float accel[N_AX]);
int     MC_Enable(void);
void    MC_Disable(void);          /* kontrolliert bremsen, dann DISABLED */
void    MC_ClearFault(void);       /* FAULT -> DISABLED, Flags löschen   */
int     MC_SetZero(void);          /* nur im Zustand DISABLED            */
int     MC_Push(const int32_t p[N_AX]);   /* Main-Kontext                */
void    MC_Tick(void);                    /* 1 kHz, aus TIM5-ISR         */

uint8_t MC_State(void);
uint8_t MC_Free(void);
uint8_t MC_Flags(void);
void    MC_GetPos(int32_t out[N_AX]);

#endif
