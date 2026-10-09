 /* uartconfig.c
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

 #include "uartconfig.h"

#include "main.h"
#include "motion.h"
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

#define SOF          0xA5
#define MAX_PAYLOAD  64

/* ---------- Frame-Typen ---------- */
/* PC -> MCU */
#define T_SETPOINT   0x01      /* 6 x int32 Sollposition [Schritte]                   */
#define T_CONFIG     0x02      /* uint16 period_ms, 6 x float vmax, 6 x float accel   */
#define T_ENABLE     0x03      /* uint8: 1 = Enable, 0 = Disable (kontrolliert bremsen) */
#define T_CLEARFAULT 0x04      /* leer */
#define T_SETZERO    0x05      /* leer */
/* MCU -> PC */
#define T_STATE      0x81      /* alle 10 ms  */
#define T_DIAG       0x82      /* alle 100 ms */

/* ---------- Counter variables ---------- */
static volatile uint32_t cnt_ok, cnt_crc, cnt_len, cnt_timeout, cnt_full, cnt_uart;
static uint32_t cnt_bad;       /* unknown Type or wrong datalenght */
static uint32_t cnt_rej;       /* right Command, but not accepted by the  Motion-Modul*/

/* ---------- CRC-16/CCITT-FALSE ---------- */
static uint16_t crc16(const uint8_t *d, uint16_t n)
{
    uint16_t c = 0xFFFF; //startwert
    while (n--) {
        c ^= (uint16_t)(*d++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

/* ---------- Ringpuffer (256 Byte, uint8-Index läuft von selbst über) ---------- */
static uint8_t rx_byte;
static uint8_t rb[256];
static volatile uint8_t rb_head, rb_tail;

void Proto_Start(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    uint8_t next = (uint8_t)(rb_head + 1);
    if (next != rb_tail) {
        rb[rb_head] = rx_byte; rb_head = next; 
    }
    else{
    cnt_full++;
    }

    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    cnt_uart++;
    HAL_UART_AbortReceive(huart);
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

/* ---------- Parser ---------- */
//P_SOF: wait for startbit 0xA5, P_len: reads lenght bit, P_BODY: collect every other bit
typedef enum { P_SOF, P_LEN, P_BODY } pstate_t;
static pstate_t ps = P_SOF;
static uint8_t  fb[MAX_PAYLOAD + 5];   /* len, type, seq, payload, crc_lo, crc_hi */
static uint8_t  fi, need;
static uint32_t t_last;

#define BAD_FRAME 1            /* Rückgabe von dispatch(): Frame passt zu keinem Befehl */

/* Mehrbyte-Werte per memcpy: PC und STM32 sind beide little-endian */
static int dispatch(uint8_t type, const uint8_t *pl, uint8_t len)
{
    switch (type) {
    case T_SETPOINT: {
        if (len != 4 * N_AX) return BAD_FRAME;
        int32_t p[N_AX];
        memcpy(p, pl, sizeof p);
        return MC_Push(p);
    }
    case T_CONFIG: {
        if (len != 2 + 8 * N_AX) return BAD_FRAME;
        uint16_t period;
        float v[N_AX], a[N_AX];
        memcpy(&period, pl, 2);
        memcpy(v, pl + 2, sizeof v);
        memcpy(a, pl + 2 + sizeof v, sizeof a);
        return MC_Config(period, v, a);
    }
    case T_ENABLE:
        if (len != 1) return BAD_FRAME;
        if (pl[0]) return MC_Enable();
        MC_Disable();
        return 0;
    case T_CLEARFAULT:
        if (len != 0) return BAD_FRAME;
        MC_ClearFault();
        return 0;
    case T_SETZERO:
        if (len != 0) return BAD_FRAME;
        return MC_SetZero();
    default:
        return BAD_FRAME;
    }
}

static void on_frame(uint8_t type, uint8_t seq, const uint8_t *pl, uint8_t len)
{
    (void)seq;
    cnt_ok++;
    int r = dispatch(type, pl, len);
    if (r == BAD_FRAME) cnt_bad++;
    else if (r < 0)     cnt_rej++;
}

void Proto_Poll(void)
{
    /* Timeout: haf a frame late -> reset */
    if (ps != P_SOF && (HAL_GetTick() - t_last) > 50){
        ps = P_SOF; cnt_timeout++;
    }

    while (rb_tail != rb_head) {
        uint8_t b = rb[rb_tail++];
        t_last = HAL_GetTick();

        switch (ps) {
        case P_SOF:
            if (b == SOF) ps = P_LEN;
            break;

        case P_LEN:
            if (b > MAX_PAYLOAD) {
                cnt_len++;
                ps = (b == SOF) ? P_LEN : P_SOF;
                break;
            }
            fb[0] = b;
            fi    = 1;
            need  = (uint8_t)(b + 5);          /* len + type + seq + payload + 2 CRC */
            ps    = P_BODY;
            break;

        case P_BODY:
            fb[fi++] = b;
            if (fi == need) {
                uint8_t  len = fb[0];
                uint16_t rx  = (uint16_t)(fb[len + 3] | (fb[len + 4] << 8));
                if (crc16(fb, (uint16_t)(len + 3)) == rx)
                    on_frame(fb[1], fb[2], &fb[3], len);
                else
                    cnt_crc++;
                ps = P_SOF;
            }
            break;
        }
    }
}

/* ---------- Senden an den PC (nicht blockierend) ---------- */
#define STATE_PAYLOAD   28                      /* state, free, flags, reserve, 6x int32 pos */
#define DIAG_PAYLOAD    32                      /* 8x uint32 Zähler                          */
#define STATE_PERIOD_MS 10
#define DIAG_PERIOD_MS  100

static uint8_t tx_buf[DIAG_PAYLOAD + 6];        /* darf während der Übertragung nicht angefasst werden */
static uint8_t tx_seq;                          /* gemeinsam für alle gesendeten Frames */

static void put_u32(uint8_t *p, uint32_t u)     /* little-endian */
{
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}

/* Kopf + CRC um die bereits in tx_buf[4..] stehenden Nutzdaten bauen, Gesamtlänge zurück */
static uint16_t finish_frame(uint8_t type, uint8_t len)
{
    tx_buf[0] = SOF;
    tx_buf[1] = len;
    tx_buf[2] = type;
    tx_buf[3] = tx_seq;
    uint16_t c = crc16(&tx_buf[1], (uint16_t)(len + 3));
    tx_buf[len + 4] = (uint8_t)c;
    tx_buf[len + 5] = (uint8_t)(c >> 8);
    return (uint16_t)(len + 6);
}

static uint16_t build_state(void)
{
    uint8_t *pl = &tx_buf[4];
    int32_t pos[N_AX];
    MC_GetPos(pos);
    pl[0] = MC_State();
    pl[1] = MC_Free();
    pl[2] = MC_Flags();
    pl[3] = 0;                                  /* Reserve */
    for (uint8_t i = 0; i < N_AX; i++) put_u32(&pl[4 + 4 * i], (uint32_t)pos[i]);
    return finish_frame(T_STATE, STATE_PAYLOAD);
}

static uint16_t build_diag(void)
{
    uint8_t *pl = &tx_buf[4];
    const uint32_t c[8] = { cnt_ok, cnt_crc, cnt_len, cnt_timeout, cnt_full, cnt_uart, cnt_bad, cnt_rej };
    for (uint8_t i = 0; i < 8; i++) put_u32(&pl[4 * i], c[i]);
    return finish_frame(T_DIAG, DIAG_PAYLOAD);
}

/* Aus dem Hauptloop aufrufen. Zustand hat Vorrang; Diagnose geht in der nächsten freien Lücke raus. */
void Proto_Send(void)
{
    static uint32_t t_state, t_diag;
    uint32_t now = HAL_GetTick();
    uint8_t due_state = (now - t_state) >= STATE_PERIOD_MS;
    uint8_t due_diag  = (now - t_diag)  >= DIAG_PERIOD_MS;
    if (!due_state && !due_diag) return;

    /* UART sendet noch -> Zustandsframe überspringen, Diagnose bleibt fällig */
    if (huart2.gState != HAL_UART_STATE_READY) {
        if (due_state) t_state = now;
        return;
    }

    uint16_t n;
    if (due_state) { t_state = now; n = build_state(); }
    else           { t_diag  = now; n = build_diag();  }

    if (HAL_UART_Transmit_IT(&huart2, tx_buf, n) == HAL_OK) tx_seq++;
}