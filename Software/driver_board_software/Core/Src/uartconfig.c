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
    // Wenn kein neuer Befehl vorliegt, sofort abbrechen
    if (!cmd_ready) {
        return 0;
    }
    uint8_t result=0;
    cmd_ready = 0; // Flag zurücksetzen

    // Fall 1: Homing-Befehl
    if (strncmp(rx_buffer, "HOME", 4) == 0) {
        //printf("OK\n");
        result = 1;
    }
    // Fall 2: Bewegungsbefehl ("MOVE J1:... J2:...")
    else if (strncmp(rx_buffer, "MOVE", 4) == 0) {
        int parsed = sscanf(rx_buffer, "MOVE J1:%f J2:%f J3:%f J4:%f J5:%f J6:%f",
                            &target_steps[0], &target_steps[1], &target_steps[2],
                            &target_steps[3], &target_steps[4], &target_steps[5]);

        if (parsed == 6.0) {
        	result = 2;//normale sachen abfahren
        	//for (uint8_t i = 0; i<6; i++){
			//	printf("target steps %d: %f", i, target_steps[i]);
			} else {
				result = -1;//error
			}

    }

    // Puffer leeren für die nächste Nachricht
    memset(rx_buffer, 0, sizeof(rx_buffer));

    return result;
}