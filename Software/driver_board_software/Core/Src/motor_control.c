/*
 * motor_control.c
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

#include "motor_control.h"

//Propertys needed for generall moving & Ramps
volatile uint32_t move_start_time = 0;
volatile uint32_t axis_finish_time[6] = {0};
volatile uint8_t print_results_flag = 0;

//see struct in motor_control.c
AxisRamp axisRamp[6];

//arrays for STEP Timer Pins
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim8, htim10; //in main.c
TIM_HandleTypeDef* AXIS_TIMERS[6] = {&htim1, &htim2, &htim3, &htim4, &htim8, &htim10};
uint32_t AXIS_CHANNELS[6] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1};

// Arrays for direction pins
GPIO_TypeDef* DIR_PORTS[6] = {TMC1_DIR_GPIO_Port, TMC2_DIR_GPIO_Port,  TMC3_DIR_GPIO_Port, TMC4_DIR_GPIO_Port, TMC5_DIR_GPIO_Port, TMC6_DIR_GPIO_Port};
uint16_t DIR_PINS[6]  = {TMC1_DIR_Pin, TMC2_DIR_Pin,  TMC3_DIR_Pin, TMC4_DIR_Pin, TMC5_DIR_Pin, TMC6_DIR_Pin};

//Propertys needed for multiple waypoints
float wp_queue[MAX_WP][6];
volatile uint8_t wp_head = 0;
volatile uint8_t wp_tail = 0;
volatile uint8_t wp_count = 0;

//=======================================================================================
//------------------------------Private Functions----------------------------------------
//=======================================================================================

//Start
static void Step_Start(uint8_t ax)
{

    HAL_TIM_PWM_Start(AXIS_TIMERS[ax], AXIS_CHANNELS[ax]);

    // MOE force enable only Advanced Timern
    TIM_HandleTypeDef* htim = AXIS_TIMERS[ax];
    if (htim->Instance == TIM1 || htim->Instance == TIM8) {
    	__HAL_TIM_MOE_ENABLE(htim);
    }
}
//Stop
static void Step_Stop(uint8_t ax)
{
    TIM_HandleTypeDef* htim = AXIS_TIMERS[ax];
    HAL_TIM_PWM_Stop(htim, AXIS_CHANNELS[ax]);
    HAL_TIM_Base_Stop_IT(htim);

    // MOE force disable only Advanced Timern
    if (htim->Instance == TIM1 || htim->Instance == TIM8) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(htim);
    }
}

//set frequenzy
static void Step_SetFreqHz(uint8_t ax, uint32_t freq_hz)
{
    TIM_HandleTypeDef* htim = AXIS_TIMERS[ax];
    uint32_t channel = AXIS_CHANNELS[ax];

    if (freq_hz == 0) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0); // keine Pulse
        return;
    }

    const uint32_t timer_tick_hz = 1000000; // 1 MHz bei Prescaler=15 und 16MHz clock
    uint32_t arr = (timer_tick_hz / freq_hz) - 1;

    if (arr < 2) arr = 2;
    if (arr > 0xFFFF) arr = 0xFFFF;

    htim->Instance->CR1 |= TIM_CR1_ARPE;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, (arr + 1) / 2); // 50% duty
}

// Berechnet vmax für eine Achse, damit sie in genau t Sekunden fertig ist
// bei fester Beschleunigung a und Distanz d
static float ComputeSyncedVmax(float d, float a, float t)
{
    float disc = (a*t)*(a*t) - 4.0f*a*d;
    if (disc < 0.0f) disc = 0.0f;
    return (a*t - sqrtf(disc)) / 2.0f;
}


//=======================================================================================
//------------------------------Public Functions----------------------------------------
//=======================================================================================

//Ramps
void UpdateAllRamps(float dt, uint8_t min_freq)
{
    // ---LOOK-AHEAD LOGIK ---
    uint8_t entering_decel = 0;
    
    // 1. Prüfen: Nähert sich eine aktive Achse der Bremszone?
    for(uint8_t ax = 0; ax < axis; ax++) {
        if(axisRamp[ax].active) {
            float rem = fabsf(axisRamp[ax].target - axisRamp[ax].pos);
            float dec = (axisRamp[ax].current_freq * axisRamp[ax].current_freq) / (2.0f * axisRamp[ax].accel);
            
            // Wenn wir kurz vor der Bremszone sind (+5 Schritte Puffer)
            if (rem <= (dec + 5.0f)) {
                entering_decel = 1;
                break;
            }
        }
    }

    // 2. Verschleifen: Wenn wir bremsen müssten, aber noch Punkte in der Queue sind
    if (entering_decel && wp_count > 0) {
        uint8_t can_blend = 1;
        
        // Prüfen, ob eine Richtungsumkehr stattfindet
        for(uint8_t ax = 0; ax < axis; ax++) {
            float current_dir = (axisRamp[ax].target - axisRamp[ax].pos >= 0) ? 1.0f : -1.0f;
            float next_dir = (wp_queue[wp_tail][ax] >= 0) ? 1.0f : -1.0f;
            
            // Bei einem Richtungswechsel dürfen wir NICHT verschleifen!
            if (wp_queue[wp_tail][ax] != 0.0f && current_dir != next_dir) {
                can_blend = 0;
                break;
            }
        }

        // Wenn die Richtung gleich bleibt: Verschleifen UND Synchronisieren!
        if (can_blend) {
            float t_sync_new = 0.0f;
            float next_rem[6];

            // 1. Neues Gesamtziel setzen und verbleibende Distanz ermitteln
            for(uint8_t ax = 0; ax < axis; ax++) {
                axisRamp[ax].target += wp_queue[wp_tail][ax];
                next_rem[ax] = fabsf(axisRamp[ax].target - axisRamp[ax].pos);
                
                // 2. Das neue längste t_sync für diese verbleibende Strecke berechnen
                if (next_rem[ax] > 0.0f) {
                    float t_i = (2.0f * GLOBAL_VMAX <= sqrtf(4.0f * next_rem[ax] * GLOBAL_ACCEL)) ? 
                                (next_rem[ax] / GLOBAL_VMAX + GLOBAL_VMAX / GLOBAL_ACCEL) : 
                                (2.0f * sqrtf(next_rem[ax] / GLOBAL_ACCEL));
                    if (t_i > t_sync_new) {
                        t_sync_new = t_i;
                    }
                }
            }

            // 3. Neue synchronisierte VMAX für jede Achse setzen
            for(uint8_t ax = 0; ax < axis; ax++) {
                if (next_rem[ax] > 0.0f) {
                    axisRamp[ax].vmax = ComputeSyncedVmax(next_rem[ax], GLOBAL_ACCEL, t_sync_new);
                }
            }

            // Punkt aus der Queue entfernen
            wp_tail = (wp_tail + 1) % MAX_WP;
            wp_count--;
        }
    }
    // --- ENDE LOOK-AHEAD ---

    for (uint8_t ax = 0; ax < axis; ax++) {
        if (!axisRamp[ax].active) continue;

        float remaining = axisRamp[ax].target - axisRamp[ax].pos;
        float dir_sign = (remaining >= 0) ? 1.0f : -1.0f;
        float remaining_abs = fabsf(remaining);

        // calculate breaking distance
        float decel_dist = (axisRamp[ax].current_freq * axisRamp[ax].current_freq)
                            / (2.0f * axisRamp[ax].accel);

        //calculating if decceleration or acceleration is needed
        if (remaining_abs <= decel_dist) {
            axisRamp[ax].current_freq -= axisRamp[ax].accel * dt;
        } else if (axisRamp[ax].current_freq < axisRamp[ax].vmax) {
            axisRamp[ax].current_freq += axisRamp[ax].accel * dt;
        }

        if (axisRamp[ax].current_freq < min_freq){
        	axisRamp[ax].current_freq = min_freq;
        }
        if (axisRamp[ax].current_freq > axisRamp[ax].vmax){
        	axisRamp[ax].current_freq = axisRamp[ax].vmax;
        }

        // Position update
        axisRamp[ax].pos += dir_sign * axisRamp[ax].current_freq * dt;

        //target reached
        if (remaining_abs < 1.0f) {
            Step_Stop(ax);
            axisRamp[ax].active = 0;
            axis_finish_time[ax] = HAL_GetTick() - move_start_time;

            continue;
        }
        //set a new frequency everytime for every axis
        Step_SetFreqHz(ax, (uint32_t)axisRamp[ax].current_freq);
    }
}

//chek if a Motor is running
uint8_t MotorsActive(void) {
    for(uint8_t i = 0; i < axis; i++) {
        if(axisRamp[i].active) return 1;
    }
    return 0;
}

//start the movement
void StartMove(uint8_t ax, float target_steps, float vmax, float accel)
{

	printf("IN STARTMOVE\r\n");
    axisRamp[ax].pos = 0.0f;
    axisRamp[ax].target = target_steps;
    axisRamp[ax].vmax = vmax;
    axisRamp[ax].accel = accel;
    axisRamp[ax].current_freq = 50.0f;
    axisRamp[ax].active = 1;

    HAL_GPIO_WritePin(DIR_PORTS[ax], DIR_PINS[ax], target_steps>0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    Step_Start(ax);

}

//compute all propertys to start a synced movment with multiple Motors
void StartSyncedMove(float target_steps[6], float accel, float vmax_max, uint8_t axis_count)
{
    float t_sync = 0.0f;

    // Calculating the time every axis would need for the distance if it would go to vmax.
    for (uint8_t i = 0; i < axis_count; i++) {
        float d = fabsf(target_steps[i]);
        if (d == 0.0f) continue;
        float t_i = (2.0f*vmax_max <= sqrtf(4*d*accel)) ? (d/vmax_max + vmax_max/accel) : (2.0f*sqrtf(d/accel)); //Trapez oder Dreieck ? Trapezformel : Dreiecksformel
        if (t_i > t_sync){
        	t_sync = t_i;	//es wird nach der längsten dauer gesucht und diese wird in t_synch gespeichert
        }
    }

    //Messvariablen zurücksetzen
        for(uint8_t i = 0; i < 6; i++) {
            axis_finish_time[i] = 0;
        }
        print_results_flag = 0;

        // 2. Startzeit merken (genau bevor die ersten Steps generiert werden)
        move_start_time = HAL_GetTick();

    //Für jede Achse vmax neu berechnen, damit alle bei t_sync fertig sind
    for (uint8_t i = 0; i < axis_count; i++) {
        float d = fabsf(target_steps[i]);
        float v = ComputeSyncedVmax(d, accel, t_sync);
        StartMove(i, target_steps[i], v, accel);
    }
}


