/*
 * TMC_monitor.c
 *
 *  Created on: 01.11.2026
 *      Author: matteo gassler
 *      LICENCED under the GNU GENERAL PUBLIC LICENSE
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

#include "TMC_monitor.h"

//registers for TMC
#define TCOOLTHRS   0x14
#define COOLCONF    0x6D        // write only
#define DRV_STATUS  0x6F        // read only

// TCOOLTHRS (TCOOLTHRS >= TSTEP) gibt nur Stop-on-Stall (sg_stop) und den DIAG-Ausgang frei,
// nicht das StallGuard-Bit 24 im DRV_STATUS (Datenblatt S. 39)
#define TMC_FCLK        12000000UL   // interner Takt TMC5160 (ca. 12 MHz)
#define SG_MIN_STEP_HZ  1000UL       // ab dieser STEP-Frequenz (1/16 Microsteps)
#define TCOOLTHRS_VAL   (TMC_FCLK / (SG_MIN_STEP_HZ * 16UL))   // 1/16 -> 1/256: Faktor 16, ergibt 750

// Bits im DRV_STATUS
#define S2VSA_BIT  12
#define S2VSB_BIT  13
#define STALL_BIT  24
#define OT_BIT     25
#define OTPW_BIT   26
#define S2GA_BIT   27
#define S2GB_BIT   28
#define SG_RESULT_MASK 0x3FF   // Bits 9..0 im DRV_STATUS

// SGT pro Achse, hier manuell anpassen (-64 ... +63, kleiner = empfindlicher)
volatile int8_t sgt[N_AX] = {6, 0, 0, 0, 0, 0};

volatile monitor tmc_flags[N_AX];

volatile uint16_t sg_result[N_AX];   // SG_RESULT pro Achse (0...1023), zum Tunen im Live Watch

// Aus dem Hauptloop aufrufen. Sicherheitsrelevante Fehler -> fault_stop()
void monitoring_TMC(void)
{
  for (uint8_t ax = 0; ax < axis; ax++) {
    uint32_t drv = TMC_ReadReg(ax, DRV_STATUS);

    if (drv & (1UL << S2GA_BIT))  { tmc_flags[ax].s2ga  = 1; fault_stop(); }   // Kurzschluss nach GND Phase A
    if (drv & (1UL << S2GB_BIT))  { tmc_flags[ax].s2gb  = 1; fault_stop(); }   // Kurzschluss nach GND Phase B
    if (drv & (1UL << S2VSA_BIT)) { tmc_flags[ax].s2vsa = 1; fault_stop(); }   // Kurzschluss nach VS Phase A
    if (drv & (1UL << S2VSB_BIT)) { tmc_flags[ax].s2vsb = 1; fault_stop(); }   // Kurzschluss nach VS Phase B
    if (drv & (1UL << OT_BIT))    { tmc_flags[ax].ot    = 1; fault_stop(); }   // Übertemperatur, Treiber aus
    if (drv & (1UL << OTPW_BIT))  { tmc_flags[ax].otpw  = 1; }                 // nur Vorwarnung
  }
}

// Aus dem Hauptloop aufrufen. SGT schreiben und Stall lesen, setzt nur die Flag
void stallguard(void)
{
  for (uint8_t ax = 0; ax < axis; ax++) {
    TMC_WriteReg(ax, TCOOLTHRS, TCOOLTHRS_VAL);                          // nur für sg_stop / DIAG
    TMC_WriteReg(ax, COOLCONF, ((uint32_t)(sgt[ax] & 0x7F)) << 16);      // sgt = Bits 22..16

    uint32_t drv = TMC_ReadReg(ax, DRV_STATUS);
    sg_result[ax] = (uint16_t)(drv & SG_RESULT_MASK);                     // Lastwert, 0 = Stall
    if (drv & (1UL << STALL_BIT)) tmc_flags[ax].stall = 1;                // StallGuard-Bit 24
  }
}
