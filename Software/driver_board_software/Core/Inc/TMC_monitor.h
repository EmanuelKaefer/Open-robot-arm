/*
 * TMC_monitor.h
 *
 *  Created on: 01.11.2026
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

#ifndef INC_TMC_MONITOR_H_
#define INC_TMC_MONITOR_H_

//includes
#include "main.h"
#include "motion.h"
#include <stdint.h>

//flags (1 = TMC hat diesen Fehler gemeldet)
typedef struct {
	uint8_t s2gb;
	uint8_t s2ga;
	uint8_t otpw;
	uint8_t ot;
	uint8_t s2vsb;
	uint8_t s2vsa;
	uint8_t stall;
} monitor;

extern volatile monitor tmc_flags[N_AX];   //eine Flag-Struct pro Achse
extern volatile uint16_t sg_result[N_AX];   //SG_RESULT pro Achse zum Tunen
extern volatile int8_t  sgt[N_AX];       //StallGuard-Schwelle pro Achse, manuell in TMC_monitor.c einstellen
extern volatile float step_freq[N_AX];	//freq
extern volatile int32_t home_offset[N_AX];   //Homing: Schritte von Nullposition bis Anschlag pro Achse


//prototypes
void monitoring_TMC(void);   //DRV_STATUS prüfen, bei Fehler fault_stop()
void stallguard(void);       //SGT schreiben und Stall-Flag lesen

#endif /* INC_TMC_MONITOR_H_ */
