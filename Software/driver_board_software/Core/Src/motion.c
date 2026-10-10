/*
 * motion.c - Sollwert-Puffer + Folgeregler
 *
 * TESTVERSION: Die Ausgabestufe Out_SetFreq() ist virtuell (gibt nichts an
 * die Timer aus). Im nächsten Schritt wird nur diese eine Funktion durch die
 * echte Timer-Ansteuerung ersetzt.
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
#include "motion.h"
#include "main.h"          
#include <string.h>
#include <math.h>

#define TICK_DT    0.001f      /* 1 kHz-Tick                                  */
#define KP         30.0f       /* 1/s: Positionsfehler wird in ~30 ms abgebaut */
#define PREFILL    3U          /* so viele Sollwerte, bevor die Fahrt startet  */
#define JUMP_FACT  2.5f        /* Sprung darf 2.5 x vmax*T sein                */
#define F_MIN_HZ   1.0f        /* darunter: keine Pulse                        */
#define FIFO_MASK  (FIFO_N - 1U)
#define FIFO_COUNT() ((uint8_t)(f_head - f_tail))

/* ---------- State ---------- */
static volatile uint8_t state      = MC_DISABLED;
static volatile uint8_t after_stop = MC_DISABLED;
static uint8_t  cfg_ok;
static uint8_t  drv_ok;                 /* alle TMC-Treiber initialisiert -> Enable erlaubt */
static uint16_t n_ticks = 10;           /* Ticks pro Sollwert-Periode */
static float    inv_T   = 100.0f;       /* 1 / Periode in s           */
static float    vmax[N_AX], accel[N_AX], max_jump[N_AX];

/* ---------- Sollwert-FIFO (1 write: Main, 1 read: ISR) ---------- */
static int32_t fifo[FIFO_N][N_AX];
static volatile uint8_t f_head, f_tail;   /*  head - tail, FIFO read/write zeiger*/
static int32_t last_pushed[N_AX];

/* ---------- Segment (lineare Interpolation) ---------- */
static int32_t  seg_s[N_AX], seg_e[N_AX];
static uint16_t tick;

/* ---------- Ist-Zustand (aus tatsächlich "ausgegebenen" Schritten) ---------- */
static int32_t pos_i[N_AX];
static float   pos_f[N_AX];
static float   f_cur[N_AX];               /* vorzeichenbehaftet, Schritte/s */
static int8_t  dir[N_AX];

static volatile uint8_t flag_isr;         /* nur ISR setzt   */
static volatile uint8_t flag_main;        /* nur Main setzt  */

/* ---------- Ausgabestufe (virtuell) ---------- */
static float Out_SetFreq(uint8_t ax, float f_abs)
{
    (void)ax;
    return (f_abs >= F_MIN_HZ) ? f_abs : 0.0f;   /* zurück: tatsächlich erzeugte Frequenz */
}

static float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

/* ---------- Öffentliche Funktionen (Main-Kontext) ---------- */
void MC_Init(void)
{
    memset(fifo, 0, sizeof fifo);
    memset(pos_i, 0, sizeof pos_i);
    memset(pos_f, 0, sizeof pos_f);
    memset(f_cur, 0, sizeof f_cur);
    memset(dir, 0, sizeof dir);
    f_head = f_tail = 0;
    flag_isr = flag_main = 0;
    cfg_ok = 0;
    drv_ok = 0;
    state = MC_DISABLED;
}

void MC_SetDriversOk(uint8_t ok)
{
    drv_ok = ok ? 1 : 0;
}

int MC_Config(uint16_t period_ms, const float v[N_AX], const float a[N_AX])
{
    if (state != MC_DISABLED)            return -1;
    if (period_ms < 2 || period_ms > 100) return -2;

    for (uint8_t i = 0; i < N_AX; i++)
        if (!(v[i] > 0.0f) || !(a[i] > 0.0f)) return -3;   /* catches also NaN */

    n_ticks = period_ms;

    inv_T   = 1000.0f / (float)period_ms;
    //set vmax, accel and max stepjump per point
    for (uint8_t i = 0; i < N_AX; i++) {
        vmax[i]     = v[i];
        accel[i]    = a[i];
        max_jump[i] = v[i] * ((float)period_ms / 1000.0f) * JUMP_FACT + 2.0f;
    }

    //config is ok?
    cfg_ok = 1;
    return 0;
}

int MC_Enable(void)
{
    if (!cfg_ok || !drv_ok || state != MC_DISABLED) return -1;
    /* ISR läuft im Zustand DISABLED nichts -> hier ohne Sperre sicher */
    f_head = f_tail = 0;
    for (uint8_t i = 0; i < N_AX; i++) {
        seg_s[i] = seg_e[i] = last_pushed[i] = pos_i[i];
        f_cur[i] = 0.0f;
    }
    flag_isr = 0; flag_main = 0;
    tick = 0;
    state = MC_IDLE;
    return 0;
}

void MC_Disable(void)
{
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    if (state == MC_IDLE || state == MC_RUNNING) {
        after_stop = MC_DISABLED;
        state = MC_STOPPING;
    }
    __set_PRIMASK(pm);
}

static void fault_stop(void)
{
    uint32_t pm = __get_PRIMASK();
     __disable_irq();
    if (state == MC_IDLE || state == MC_RUNNING) {
        after_stop = MC_FAULT;
        state = MC_STOPPING;
    }
    __set_PRIMASK(pm);
}

void MC_ClearFault(void)
{
    uint32_t pm = __get_PRIMASK();
     __disable_irq();
    if (state == MC_FAULT) state = MC_DISABLED;
    flag_isr = 0;
    __set_PRIMASK(pm);
    flag_main = 0;
}

int MC_SetZero(void)
{
    if (state != MC_DISABLED) return -1;
    for (uint8_t i = 0; i < N_AX; i++){
        pos_i[i] = 0; pos_f[i] = 0.0f;
    }
    return 0;
}

int MC_Push(const int32_t p[N_AX])
{
    uint8_t st = state;
    if (st != MC_IDLE && st != MC_RUNNING) return -1;

    if (FIFO_COUNT() >= FIFO_N) { flag_main |= FL_OVERRUN; return -2; }

    for (uint8_t i = 0; i < N_AX; i++) {
        int64_t d = (int64_t)p[i] - (int64_t)last_pushed[i];
        if (d < 0) d = -d;
        if ((float)d > max_jump[i]) {
            flag_main |= FL_JUMP;
            fault_stop();
            return -3;
        }
    }
    memcpy(fifo[f_head & FIFO_MASK], p, sizeof(int32_t) * N_AX);
    memcpy(last_pushed, p, sizeof(int32_t) * N_AX);
    __DMB();
    f_head++;
    return 0;
}

uint8_t MC_State(void) { return state; }
uint8_t MC_Free(void)  { return (uint8_t)(FIFO_N - FIFO_COUNT()); }
uint8_t MC_Flags(void) { return (uint8_t)(flag_isr | flag_main | (cfg_ok ? FL_CFG_OK : 0) | (drv_ok ? FL_DRV_OK : 0)); }

void MC_GetPos(int32_t out[N_AX])
{
    uint32_t pm = __get_PRIMASK();
     __disable_irq();
    memcpy(out, pos_i, sizeof(int32_t) * N_AX);
    __set_PRIMASK(pm);
}

/* ---------- ISR-Kontext ---------- */
static void next_segment(void)
{
    memcpy(seg_s, seg_e, sizeof seg_s);
    if (FIFO_COUNT() > 0) {
        memcpy(seg_e, fifo[f_tail & FIFO_MASK], sizeof seg_e);
        __DMB();
        f_tail++;
    } else {
        flag_isr |= FL_UNDERRUN;      /* Puffer leer -> letzte Position halten */
    }
    tick = 0;
}

void MC_Tick(void)
{
    if (state == MC_DISABLED || state == MC_FAULT) return;

    if (state == MC_IDLE) {
        if (FIFO_COUNT() >= PREFILL) {
            memcpy(seg_e, pos_i, sizeof seg_e);   /* Start = aktuelle Position */
            next_segment();
            state = MC_RUNNING;
        }
    } else if (state == MC_RUNNING && tick >= n_ticks) {
        next_segment();
    }

    uint8_t moving = 0;
    for (uint8_t a = 0; a < N_AX; a++) {
        float f_des = 0.0f;

        if (state == MC_RUNNING) {
            float span = (float)(seg_e[a] - seg_s[a]);
            float err  = (float)(seg_s[a] - pos_i[a])
                       + span * ((float)tick / (float)n_ticks)
                       - pos_f[a];
            f_des = clampf(span * inv_T + KP * err, -vmax[a], vmax[a]);
        }

        /* Beschleunigungsbegrenzung */
        float df = accel[a] * TICK_DT;
        f_cur[a] += clampf(f_des - f_cur[a], -df, df);

        /* Richtungswechsel: einen Tick lang nichts ausgeben, dann DIR umschalten */
        int8_t s = (f_cur[a] > 0.0f) - (f_cur[a] < 0.0f);
        if (s != 0 && s != dir[a]) {
            f_cur[a] = 0.0f;
            dir[a]   = s;             /* später: DIR-Pin setzen */
        }

        float f_real = Out_SetFreq(a, fabsf(f_cur[a]));
        pos_f[a] += (float)dir[a] * f_real * TICK_DT;
        int32_t w = (int32_t)pos_f[a];
        pos_i[a] += w;
        pos_f[a] -= (float)w;

        if (f_real > 0.0f) moving = 1;
    }

    if (state == MC_RUNNING)       tick++;
    else if (state == MC_STOPPING && !moving) state = after_stop;
}
