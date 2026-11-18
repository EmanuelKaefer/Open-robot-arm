/*
 * homing.h
 *
 *  Created on: 10.11.2026
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

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

//includes
#include "motion.h"
#include <stdint.h>

//Homing-Status (Reserve-Byte im Zustandsframe T_STATE)
#define HS_HOMED_MASK 0x3FU   // Bit 0..5: Achse gehomed
#define HS_ACTIVE     0x40U   // Bit 6: Homing läuft
#define HS_ERROR      0x80U   // Bit 7: Fehler / abgebrochen

extern volatile int32_t home_offset_pc[N_AX];   //vom PC empfangene Offsets (T_HOMING)

//prototypes
int     Home_Start(const int32_t offsets[N_AX]);  //aus dispatch (T_HOMING)
void    Home_Abort(void);                          //Enable 0 während des Homings
uint8_t Home_Active(void);                         //1 = Homing läuft
uint8_t Home_Status(void);                         //Status-Byte für T_STATE
void    Home_Service(void);                        //Hauptloop, Fahrablauf folgt

#endif /* INC_HOMING_H_ */
