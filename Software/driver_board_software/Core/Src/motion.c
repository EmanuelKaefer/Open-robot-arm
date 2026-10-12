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
#define FILL_TARGET 4U         /* gewünschter FIFO-Füllstand                   */
#define FILL_BAND   2U         /* erst ab Abweichung >= 2 wird das Tempo angepasst */
#define EXTRAP_MAX  50U        /* Ticks (ms) Extrapolation, danach bremsen     */
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
static uint16_t n_cur = 10;               /* Ticks des laufenden Segments: n_ticks, -1 oder +1 */
static uint8_t  still;                    /* letztes echtes Segment ohne Bewegung (Span 0 auf allen Achsen) */

/* ---------- Geschwindigkeitsschätzung über die letzten V_WIN echten Segmente ----------
 * Ein einzelnes Segment ist zu ungenau: Sollwerte sind ganzzahlig, eine langsame Achse mit
 * 2-3 Schritten pro Segment hätte bis zu ±20 % Fehler. Über 4 Segmente wirkt die Rundung
 * nur noch zu einem Viertel. */
#define V_WIN 4U
static int32_t  vh_e[V_WIN + 1][N_AX];    /* Endpunkte der letzten echten Segmente (Ring) */
static uint16_t vh_n[V_WIN + 1];          /* Ticks vom vorherigen Endpunkt bis zu diesem */
static uint8_t  vh_head, vh_cnt;

/* ---------- Extrapolation bei leerem FIFO ---------- */
static uint8_t  owed;                     /* Zeitschlitze, die ohne Sollwert extrapoliert wurden */
static uint16_t ex_ticks;                 /* Ticks seit Beginn der Extrapolation */
static int32_t  ex_base[N_AX];            /* letzter echter Sollwert: Anker der Extrapolation */
static uint16_t ex_span;                  /* vom Anker bis zum Ende des laufenden Segments extrapolierte Ticks */
static float    v_ex[N_AX];               /* Extrapolationsgeschwindigkeit [Schritte/Tick] */

/* ---------- Ist-Zustand (aus tatsächlich "ausgegebenen" Schritten) ---------- */
static int32_t pos_i[N_AX];
static float   pos_f[N_AX];
static float   f_cur[N_AX];               /* vorzeichenbehaftet, Schritte/s */
static int8_t  dir[N_AX];

static volatile uint8_t flag_isr;         /* nur ISR setzt   */
static volatile uint8_t flag_main;        /* nur Main setzt  */

/* ---------- Ausgabestufe (echt) ---------- */
#define TIM_CLK_HZ   16000000.0f            /* Timertakt aller STEP-Timer               */
#define PSC_FAST     15U                    /* 16 MHz / 16  = 1 MHz Zähltakt             */
#define PSC_SLOW     255U                   /* 16 MHz / 256 = 62,5 kHz Zähltakt          */
#define CLK_FAST     (TIM_CLK_HZ / (PSC_FAST + 1U))
#define CLK_SLOW     (TIM_CLK_HZ / (PSC_SLOW + 1U))
#define F_SPLIT_HZ   200.0f                 /* ab hier schneller Prescaler               */
#define ARR_IDLE     999U                   /* Leerlauf: 1 ms Periode                    */
#define CCR_OFF      0xFFFFU                /* > ARR -> PWM Mode 2 bleibt dauerhaft LOW  */

static TIM_TypeDef * const step_tim[N_AX] = { TIM1, TIM2, TIM3, TIM4, TIM8, TIM10 };
static uint8_t running[N_AX];               /* 1 = Achse gibt gerade Pulse aus           */

/* DIR-Pins der sechs Achsen  */
static GPIO_TypeDef * const dir_port[N_AX] = {
    TMC1_DIR_GPIO_Port, TMC2_DIR_GPIO_Port, TMC3_DIR_GPIO_Port,
    TMC4_DIR_GPIO_Port, TMC5_DIR_GPIO_Port, TMC6_DIR_GPIO_Port };
static const uint16_t dir_pin[N_AX] = {
    TMC1_DIR_Pin, TMC2_DIR_Pin, TMC3_DIR_Pin,
    TMC4_DIR_Pin, TMC5_DIR_Pin, TMC6_DIR_Pin };

/* Bit = 1: Drehrichtung dieser Achse umdrehen (Bit 0 = Achse 0 … Bit 5 = Achse 5) */
#define DIR_INVERT_MASK  0x00U

static float Out_SetFreq(uint8_t ax, float f_abs)
{
    TIM_TypeDef *t = step_tim[ax];

    /* 1. Aus */
    if (f_abs < F_MIN_HZ) {
        if (running[ax]) {
            t->CCR1 = CCR_OFF;              /* keine Flanke mehr                         */
            t->PSC  = PSC_FAST;
            t->ARR  = ARR_IDLE;             /* kurze Periode für den nächsten Start      */
            t->EGR  = TIM_EGR_UG;           /* sofort übernehmen, Zähler auf 0           */
            running[ax] = 0;
        }
        return 0.0f;
    }

    /* 2. Prescaler wählen */
    uint32_t psc;
    float    clk;
    if (f_abs >= F_SPLIT_HZ) { psc = PSC_FAST; clk = CLK_FAST; }
    else                     { psc = PSC_SLOW; clk = CLK_SLOW; }

    /* 3. Periode in Zähltakten, gerundet und begrenzt */
    uint32_t n = (uint32_t)(clk / f_abs + 0.5f);
    if (n < 3U)      n = 3U;
    if (n > 65536U)  n = 65536U;
    uint32_t arr = n - 1U;

    /* 4. + 5. Register schreiben (werden am Periodenende gemeinsam übernommen) */
    t->PSC  = psc;
    t->ARR  = arr;
    t->CCR1 = n / 2U;                       /* Flanke in der Periodenmitte               */

    if (!running[ax]) {                     /* Start aus dem Stillstand:                 */
        t->EGR = TIM_EGR_UG;                /* sofort übernehmen, Zähler bei 0 beginnen  */
        running[ax] = 1;
    }

    /* 6. Tatsächlich erzeugte Frequenz */
    return clk / (float)n;
}

static void Dir_Set(uint8_t ax, int8_t d)
{
    uint8_t level = (d > 0) ? 1U : 0U;              /* +1 → HIGH, −1 → LOW   */
    if (DIR_INVERT_MASK & (1U << ax)) level ^= 1U;  /* Achse invertiert?     */

    if (level){HAL_GPIO_WritePin(dir_port[ax], dir_pin[ax], GPIO_PIN_SET);} /* Pin HIGH  */
    else {HAL_GPIO_WritePin(dir_port[ax], dir_pin[ax], GPIO_PIN_RESET);}    /* Pin LOW  */
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
    memset(running, 0, sizeof(running));
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
    still = 1;
    flag_isr = 0; flag_main = 0;
    tick = 0;
    n_cur = n_ticks;
    owed = 0; ex_ticks = 0;
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
static void fifo_pop(int32_t out[N_AX])
{
    memcpy(out, fifo[f_tail & FIFO_MASK], sizeof(int32_t) * N_AX);
    __DMB();                          /* erst fertig lesen, dann Platz freigeben */
    f_tail++;
}

static void vel_reset(const int32_t start[N_AX])
{
    vh_head = 0;
    vh_cnt  = 1;
    vh_n[0] = 0;
    memcpy(vh_e[0], start, sizeof vh_e[0]);
}

/* Echter Sollwert erreicht: Endpunkt merken. n = Ticks seit dem vorherigen echten Endpunkt */
static void vel_push(const int32_t e[N_AX], uint16_t n)
{
    vh_head = (uint8_t)((vh_head + 1U) % (V_WIN + 1U));
    memcpy(vh_e[vh_head], e, sizeof vh_e[0]);
    vh_n[vh_head] = n;
    if (vh_cnt < V_WIN + 1U) vh_cnt++;
}

/* Mittlere Geschwindigkeit vom ältesten bis zum neuesten gemerkten Endpunkt */
static void vel_estimate(float v[N_AX])
{
    uint8_t  i = vh_head;
    uint32_t sum = 0;
    for (uint8_t k = 1; k < vh_cnt; k++) {
        sum += vh_n[i];
        i = (uint8_t)((i + V_WIN) % (V_WIN + 1U));    /* einen Eintrag zurück */
    }
    for (uint8_t a = 0; a < N_AX; a++)
        v[a] = sum ? (float)(vh_e[vh_head][a] - vh_e[i][a]) / (float)sum : 0.0f;
}

static uint8_t span_zero(void)
{
    for (uint8_t a = 0; a < N_AX; a++)
        if (seg_e[a] != seg_s[a]) return 0;
    return 1;
}

static void next_segment(void)
{
    /* Ticks seit dem letzten echten Endpunkt, falls gerade extrapoliert wird */
    uint16_t gap = owed ? ex_span : 0;

    /* Sollwerte, deren Zeitschlitz schon extrapoliert wurde, sind veraltet */
    while (owed > 0 && FIFO_COUNT() > 0) { f_tail++; owed--; }

    memcpy(seg_s, seg_e, sizeof seg_s);
    uint8_t have = FIFO_COUNT() > 0;
    if (have) {fifo_pop(seg_e);}

    /* Verbrauchstempo am Füllstand anpassen. Alle Achsen teilen sich tick/n_cur -> Gleichlauf bleibt exakt */
    uint8_t cnt = FIFO_COUNT();
    n_cur = n_ticks;
    if (n_ticks >= 4) {
        if (cnt >= FILL_TARGET + FILL_BAND) {n_cur = (uint16_t)(n_ticks - 1);}   /* zu voll  -> schneller */
        else if (cnt + FILL_BAND <= FILL_TARGET)  {n_cur = (uint16_t)(n_ticks + 1);}   /* zu leer  -> langsamer */
    }

    if (have) {
        vel_push(seg_e, (uint16_t)(gap + n_cur));
        still = span_zero();
        ex_ticks = 0;
    } else if (!still) {
        /* Puffer leer während der Fahrt: mit der geschätzten Geschwindigkeit weiter.
         * Immer vom letzten echten Sollwert aus rechnen und nur einmal runden,
         * sonst summieren sich die Rundungsfehler von Segment zu Segment auf. */
        if (owed == 0) {
            memcpy(ex_base, seg_s, sizeof ex_base);
            ex_span = 0;
            vel_estimate(v_ex);
        }
        ex_span = (uint16_t)(ex_span + n_cur);
        for (uint8_t a = 0; a < N_AX; a++)
            seg_e[a] = ex_base[a] + (int32_t)lroundf(v_ex[a] * (float)ex_span);
        owed++;
        flag_isr |= FL_UNDERRUN;
    }
    /* sonst: Stillstand -> einfach halten (seg_e == seg_s), Stream pausiert */
    tick = 0;
}

/* Verspäteter Sollwert für den gerade extrapolierten Zeitschlitz: ersetzt die Schätzung */
static void redirect(void)
{
    while (owed > 1) { f_tail++; owed--; }        /* noch ältere Schlitze: veraltet */
    fifo_pop(seg_e);
    vel_push(seg_e, ex_span);                     /* ex_span enthält das laufende Segment schon */
    still = span_zero();
    owed = 0;
    ex_ticks = 0;
}

/* Alle Achsen mit demselben Faktor k verzögern -> Bahn bleibt erhalten.
 * Jede Achse verlangt k >= 1 - df/|f| (sonst überschreitet sie ihre Beschleunigung);
 * es gilt die strengste dieser Untergrenzen, also die der knappsten Achse. */
static void brake_synced(void)
{
    float k = 0.0f;
    for (uint8_t a = 0; a < N_AX; a++) {
        float fa = fabsf(f_cur[a]);
        if (fa > 0.0f) {
            float ka = 1.0f - accel[a] * TICK_DT / fa;
            if (ka > k) k = ka;
        }
    }
    for (uint8_t a = 0; a < N_AX; a++) f_cur[a] *= k;
}

void MC_Tick(void)
{
    if (state == MC_DISABLED || state == MC_FAULT) return;

    if (state == MC_IDLE) {
        if (FIFO_COUNT() >= PREFILL) {
            memcpy(seg_e, pos_i, sizeof seg_e);   /* Start = aktuelle Position */
            vel_reset(pos_i);
            next_segment();
            state = MC_RUNNING;
        }
    } else if (state == MC_RUNNING) {
        if (tick >= n_cur) {next_segment();}
        else if (owed > 0 && FIFO_COUNT() >= owed)  {redirect();}

        if (owed > 0 && ++ex_ticks > EXTRAP_MAX) {   /* Sollwerte bleiben zu lange aus */
            flag_isr  |= FL_STARVED;
            after_stop = MC_FAULT;
            state      = MC_STOPPING;
        }
    }

    if (state == MC_STOPPING) {brake_synced();}

    uint8_t moving = 0;
    for (uint8_t a = 0; a < N_AX; a++) {
        if (state == MC_RUNNING) {
            /*P-Regler, Vorsteuerung mit Nennperiode (inv_T), nicht mit n_cur: sonst springt sie bei
             * jeder Tempoanpassung um 10 % und stößt an die Beschleunigungsgrenze.
             * Beim Extrapolieren die ungerundete Schätzung: exakt proportional über alle Achsen. */
            float span  = (float)(seg_e[a] - seg_s[a]);
            float err   = (float)(seg_s[a] - pos_i[a]) + span * ((float)tick / (float)n_cur) - pos_f[a];
            float ff    = owed ? v_ex[a] / TICK_DT : span * inv_T;  //feed forward
            float f_des = clampf(ff + KP * err, -vmax[a], vmax[a]);

            /* Beschleunigungsbegrenzung */
            float df = accel[a] * TICK_DT;
            f_cur[a] += clampf(f_des - f_cur[a], -df, df);
        }

        /* Richtungswechsel: einen Tick lang nichts ausgeben, dann DIR umschalten */
        int8_t s = (f_cur[a] > 0.0f) - (f_cur[a] < 0.0f);
        uint8_t dir_changed = 0;
        if (s != 0 && s != dir[a]) {
            f_cur[a] = 0.0f;
            dir[a]   = s;             
            dir_changed=1;
        }
        float f_real = Out_SetFreq(a, fabsf(f_cur[a]));

        if (dir_changed) Dir_Set(a, dir[a]); //Reihenfolge wichtig!!!
        pos_f[a] += (float)dir[a] * f_real * TICK_DT;
        int32_t w = (int32_t)pos_f[a];
        pos_i[a] += w;
        pos_f[a] -= (float)w;

        if (f_real > 0.0f) moving = 1;
    }

    if (state == MC_RUNNING) {tick++;}
    else if (state == MC_STOPPING && !moving) {state = after_stop;}
}
