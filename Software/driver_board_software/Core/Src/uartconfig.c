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

volatile uint8_t cmd_ready = 0;
char rx_buffer[128];

uint8_t process_uart_command(void) {
    if (!cmd_ready) return 0;
    uint8_t result = 0;
    cmd_ready = 0;

    // Wegpunkt zur Queue hinzufügen
    if (strncmp(rx_buffer, "MOVE", 4) == 0) {
        if (wp_count < MAX_WP) {
            int parsed = sscanf(rx_buffer, "MOVE J1:%f J2:%f J3:%f J4:%f J5:%f J6:%f",
                                &wp_queue[wp_head][0], &wp_queue[wp_head][1], 
                                &wp_queue[wp_head][2], &wp_queue[wp_head][3], 
                                &wp_queue[wp_head][4], &wp_queue[wp_head][5]);
            if (parsed == 6) {
                wp_head = (wp_head + 1) % MAX_WP;
                wp_count++;
                printf("Waypoint %d/%d geladen.\r\n", wp_count, MAX_WP);
            }
        } else {
            printf("ERROR: Queue voll!\r\n");
        }
    }

    memset(rx_buffer, 0, sizeof(rx_buffer));
    return result; 
}