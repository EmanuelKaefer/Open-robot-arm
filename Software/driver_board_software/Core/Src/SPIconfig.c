/*
 * SPIconfig.c
 *
 *  Created on: 27.04.2026
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

#include "SPIconfig.h"
#include "motion.h"   // N_AX, fault_stop()
#include <math.h>     // floorf()

// =====================================================================================
// Motorstrom pro Achse - HIER EINSTELLEN
//
// run_arms:  Strom beim Fahren [A rms pro Phase]
// hold_arms: Haltestrom im Stillstand [A rms pro Phase] (typisch 30...50 % von run_arms)
//
// Die Firmware rechnet daraus GLOBALSCALER, IRUN und IHOLD aus (tmc_current_regs) und
// begrenzt hart auf MODULE_MAX_ARMS. Ein zu großer Wert in der Tabelle kann das Modul also
// nicht überlasten. Es wird immer abgerundet: der echte Strom ist nie größer als eingetragen.
//
// Beispiel 4,2-A-Motor, auf ~70 % begrenzt: {2.9f, 1.45f}
// Die Werte unten entsprechen dem alten Stand (IRUN 16 / IHOLD 10 = 1,63 A / 1,05 A).
// =====================================================================================
typedef struct { float run_arms; float hold_arms; } axis_current_t;
static const axis_current_t axis_current[N_AX] = {
  {1.6f, 1.05f},   // Achse 0 (TMC1)
  {1.6f, 1.05f},   // Achse 1 (TMC2)
  {1.6f, 1.05f},   // Achse 2 (TMC3)
  {1.6f, 1.05f},   // Achse 3 (TMC4)
  {1.6f, 1.05f},   // Achse 4 (TMC5)
  {1.6f, 1.05f},   // Achse 5 (TMC6)
};

#define RSENSE_OHM       0.075f   // BTT TMC5160T Pro V1.0 (Shunt "R075")
#define VFS_V            0.325f   // TMC5160: Vollausschlag-Spannung am Shunt
#define MODULE_MAX_ARMS  3.0f     // Breakout-Pins der Module: nie mehr als das
// Vollausschlag bei GLOBALSCALER 256 und CS 31: 0,325 V / 0,075 Ohm / sqrt(2) = 3,064 A rms
#define I_FS_ARMS        (VFS_V / RSENSE_OHM / 1.41421356f)

#define TPOWERDOWN_VAL       10U   // Stillstand -> Haltestrom nach 10 * 2^18 Takten (~0,22 s)
#define IHOLDDELAY_VAL       8U    // Rampe IRUN -> IHOLD: 8 * 2^18 Takte pro Stufe
#define RUN_CURRENT_DELAY_MS 500U  // nach der Init erst so lange nur Haltestrom, dann IRUN setzen
#define STATUS_CHECK_MS      500U  // so oft GSTAT prüfen (VM weg gewesen?)

// Array für die Chip Select (CS) Pins der SPI-Kommunikation
// (Index 0 = TMC2, Index 1 = TMC1, wie bei den DIR Pins gewünscht)
GPIO_TypeDef* CS_PORTS[6] = {TMC1_CS_GPIO_Port, TMC2_CS_GPIO_Port, TMC3_CS_GPIO_Port, TMC4_CS_GPIO_Port, TMC5_CS_GPIO_Port, TMC6_CS_GPIO_Port};
uint16_t      CS_PINS[6]  = {TMC1_CS_Pin, TMC2_CS_Pin, TMC3_CS_Pin, TMC4_CS_Pin, TMC5_CS_Pin, TMC6_CS_Pin};

//registers for TMC
#define GSTAT 0x01
#define GCONF 0x00
#define IOIN 0x04
#define GLOBALSCALER 0x0B	//write only: 0 = 256 (Vollausschlag), 32...255 = x/256, 1...31 nicht erlaubt
#define IHOLDIRUN 0x10	//write only, returns 0x0 when read
#define TPOWERDOWN 0x11	//write only
#define CHOPCONF 0X6C

// Bits im GSTAT
#define GSTAT_RESET  (1UL << 0)     // Treiber wurde zurückgesetzt (z. B. VM war weg)
#define GSTAT_UV_CP  (1UL << 2)     // Unterspannung der Ladungspumpe, Endstufe aus

#define TMC_VERSION     0x30        // IOIN[31:24] beim TMC5160
#define CHOPCONF_VAL    0x140100C3  // 1/16 microsteps with intpol=1
#define INIT_RETRY_MS   500         // erster Versuch nach 500 ms (Treiber-Hochlauf), dann alle 500 ms




// --- SPI Write Funktion ---
void TMC_WriteReg(uint8_t ax, uint8_t addr, uint32_t value) {
    uint8_t data[5];
    data[0] = addr | 0x80; // MSB auf 1 setzen für Write
    data[1] = (value >> 24) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 8)  & 0xFF;
    data[4] = value & 0xFF;

    // CS LOW, Senden, CS HIGH (Dynamisch über Arrays!)
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_SET);
}

// --- SPI Read Funktion ---
uint32_t TMC_ReadReg(uint8_t ax, uint8_t addr) {
    uint8_t tx_data[5] = {addr & 0x7F, 0, 0, 0, 0}; // MSB auf 0 für Read
    uint8_t rx_data[5] = {0};

    // 1. Transfer: Dem TMC5160 sagen, welches Register wir wollen
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_SET);

    // 2. Transfer: Die eigentlichen Daten abholen
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORTS[ax], CS_PINS[ax], GPIO_PIN_SET);

    // Bytes zusammenbauen (rx_data[0] ist die Dummy-Antwort der Adresse)
    return ((uint32_t)rx_data[1] << 24) | ((uint32_t)rx_data[2] << 16) |
           ((uint32_t)rx_data[3] << 8)  | rx_data[4];
}

// Strom [A rms] -> Register. Reine Rechnung, ohne HAL.
//   I = GLOBALSCALER/256 * (CS+1)/32 * I_FS_ARMS
// IRUN bleibt auf 31 (volle Mikroschritt-Auflösung, Empfehlung von Trinamic), den Maximalstrom
// der Achse legt GLOBALSCALER fest. Alles wird abgerundet -> nie mehr Strom als verlangt.
static void tmc_current_regs(float run_arms, float hold_arms,
                             uint8_t *gs_reg, uint8_t *irun, uint8_t *ihold)
{
  if (!(run_arms > 0.0f))        run_arms = 0.0f;              // auch NaN
  if (run_arms > MODULE_MAX_ARMS) run_arms = MODULE_MAX_ARMS;
  if (run_arms > I_FS_ARMS)       run_arms = I_FS_ARMS;

  uint32_t gs = (uint32_t)floorf(run_arms / I_FS_ARMS * 256.0f);
  uint32_t cs = 31U;
  if (gs < 32U) {
    // GLOBALSCALER 1...31 ist nicht erlaubt: bei 32 bleiben und mit IRUN weiter runter
    gs = 32U;
    const float step32 = I_FS_ARMS * 32.0f / 256.0f / 32.0f;  // A pro CS-Stufe bei GS 32
    const int32_t c = (int32_t)floorf(run_arms / step32) - 1;
    cs = (c < 0) ? 0U : (uint32_t)c;
  }

  const float step = I_FS_ARMS * (float)gs / 256.0f / 32.0f;  // A pro CS-Stufe
  if (!(hold_arms > 0.0f)) hold_arms = 0.0f;
  int32_t h = (int32_t)floorf(hold_arms / step) - 1;
  if (h < 0)             h = 0;
  if (h > (int32_t)cs)   h = (int32_t)cs;                       // Haltestrom nie über Fahrstrom

  *gs_reg = (gs >= 256U) ? 0U : (uint8_t)gs;                    // 0 bedeutet 256
  *irun   = (uint8_t)cs;
  *ihold  = (uint8_t)h;
}

static uint32_t ihold_irun(uint8_t ihold, uint8_t irun)
{
  return ((uint32_t)IHOLDDELAY_VAL << 16) | ((uint32_t)irun << 8) | (uint32_t)ihold;
}

// Alle Treiber konfigurieren und prüfen. Rückgabe: Bitmaske der fehlerhaften Achsen (0 = alle ok)
// Die Endstufe startet nur mit Haltestrom (IRUN = IHOLD): sonst läuft sie nach dem Einschalten
// ~1 s mit IRUN, bis Stillstand erkannt und auf IHOLD abgeregelt ist (Stromspitze bei VM ein).
static uint8_t TMC_InitAll(void)
{
  uint8_t failed = 0;
  for (uint8_t ax = 0; ax < axis; ax++) {
    uint8_t gs, irun, ihold;
    tmc_current_regs(axis_current[ax].run_arms, axis_current[ax].hold_arms, &gs, &irun, &ihold);
    (void)irun;                                          // IRUN kommt erst in TMC_ApplyRunCurrent()

    TMC_WriteReg(ax, GSTAT, 0x00000007);      // reset/drv_err/uv_cp löschen (write 1 to clear)
    TMC_WriteReg(ax, GCONF, 0x00000000);      // disable stealthchop, enable spreadcycle
    TMC_WriteReg(ax, GLOBALSCALER, gs);       // Maximalstrom dieser Achse
    TMC_WriteReg(ax, TPOWERDOWN, TPOWERDOWN_VAL);
    TMC_WriteReg(ax, IHOLDIRUN, ihold_irun(ihold, ihold));   // Start: nur Haltestrom
    TMC_WriteReg(ax, CHOPCONF, CHOPCONF_VAL);  // TOFF > 0: Endstufe ein (muss als Letztes kommen)

    uint8_t ok = 1;
    if ((TMC_ReadReg(ax, IOIN) >> 24) != TMC_VERSION) ok = 0;   // anwser over SPI?
    if (TMC_ReadReg(ax, CHOPCONF) != CHOPCONF_VAL)     ok = 0;   // Config?
    if (TMC_ReadReg(ax, GSTAT) != 0)                   ok = 0;   // Reset/error/Undevoltage
    if (!ok) failed |= (uint8_t)(1u << ax);
  }
  return failed;
}

// Echten Fahrstrom setzen. Erst aufrufen, wenn der Treiber Stillstand erkannt hat: ein neues
// IRUN wirkt dann erst beim nächsten Schritt, es gibt also keine Stromspitze.
static void TMC_ApplyRunCurrent(void)
{
  for (uint8_t ax = 0; ax < axis; ax++) {
    uint8_t gs, irun, ihold;
    tmc_current_regs(axis_current[ax].run_arms, axis_current[ax].hold_arms, &gs, &irun, &ihold);
    TMC_WriteReg(ax, IHOLDIRUN, ihold_irun(ihold, irun));
  }
}

// 1 = mindestens ein Treiber hat seit der Init einen Reset oder Unterspannung gesehen
// (VM war aus). Seine Register stehen dann wieder auf Reset-Werten (TOFF = 0, Motor stromlos).
// drv_err (Bit 1) löst bewusst KEINE Neu-Init aus: ein Kurzschluss soll nicht alle 500 ms wieder
// bestromt werden. Den behandelt monitoring_TMC() über DRV_STATUS.
static uint8_t TMC_LostConfig(void)
{
  for (uint8_t ax = 0; ax < axis; ax++) {
    if (TMC_ReadReg(ax, GSTAT) & (GSTAT_RESET | GSTAT_UV_CP)) return 1;
  }
  return 0;
}

// Aus dem Hauptloop aufrufen, nicht blockierend. Rückgabe 1 = alle Treiber bereit (DRV_OK).
//   INIT:  alle 500 ms Init versuchen (scheitert, solange VM aus ist)
//   SOFT:  Init ok, Endstufe läuft mit Haltestrom. Nach RUN_CURRENT_DELAY_MS IRUN setzen
//   READY: alle STATUS_CHECK_MS GSTAT prüfen. VM war weg -> fault_stop() und neu initialisieren
uint8_t TMC_Service(void)
{
  enum { ST_INIT, ST_SOFT, ST_READY };
  static uint8_t  st = ST_INIT;
  static uint32_t t;                    // Zeitpunkt des letzten Versuchs bzw. Zustandswechsels
  const uint32_t now = HAL_GetTick();

  switch (st) {
  case ST_INIT:
    if (now - t < INIT_RETRY_MS) return 0;
    t = now;
    if (TMC_InitAll() == 0) st = ST_SOFT;
    return 0;

  case ST_SOFT:
    if (now - t < RUN_CURRENT_DELAY_MS) return 0;
    TMC_ApplyRunCurrent();
    t = now;
    st = ST_READY;
    return 1;

  default:  // ST_READY
    if (now - t >= STATUS_CHECK_MS) {
      t = now;
      if (TMC_LostConfig()) {
        fault_stop();                   // fährt der Arm gerade: bremsen -> FAULT (Position unsicher)
        st = ST_INIT;
        return 0;
      }
    }
    return 1;
  }
}
