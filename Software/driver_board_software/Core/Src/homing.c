/*
 * homing.c
 *
 *  Created on: 10.11.2026
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

#include "homing.h"
#include "motion.h"
#include <stdint.h>
#include <string.h>

static volatile uint8_t status;              // HS_* Bits, geht im Zustandsframe an den PC

volatile int32_t home_offset_pc[N_AX];       // vom PC empfangene Offsets, im Live Watch sichtbar

// Aus dispatch (T_HOMING). 0 = angenommen, < 0 = abgelehnt (zählt als rej)
int Home_Start(const int32_t offsets[N_AX])
{
  if (status & HS_ACTIVE) return 0;                  // läuft schon: Wiederholung vom PC ignorieren
  if (MC_State() != MC_DISABLED) return -1;          // nur aus dem Stillstand starten
  if (!(MC_Flags() & FL_DRV_OK)) return -2;          // TMC-Treiber nicht bereit

  for (uint8_t i = 0; i < N_AX; i++) home_offset_pc[i] = offsets[i];
  status = HS_ACTIVE;                                // alte gehomed- und Fehler-Bits löschen
  return 0;
}

// Enable 0 vom PC während des Homings: abbrechen und kontrolliert bremsen
void Home_Abort(void)
{
  if (!(status & HS_ACTIVE)) return;
  status = (uint8_t)((status & ~HS_ACTIVE) | HS_ERROR);   // abgebrochen = nicht fertig
  MC_Disable();
}

uint8_t Home_Active(void) { return (status & HS_ACTIVE) ? 1 : 0; }
uint8_t Home_Status(void) { return status; }

// Aus dem Hauptloop aufrufen
void Home_Service(void)
{
  if (!(status & HS_ACTIVE)) return;
  /* Platzhalter für den Fahrablauf (nächster Schritt):
   * Achse für Achse in den Anschlag, Position = home_offset_pc[ax], zurück auf 0,
   * Bit der Achse in status setzen, am Ende HS_ACTIVE löschen. */
}
