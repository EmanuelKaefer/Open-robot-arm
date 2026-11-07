/*
 * SPIconfig.h
 *
 *  Created on: 27.04.2026
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

#ifndef INC_SPICONFIG_H_
#define INC_SPICONFIG_H_

//includes
#include "main.h"
#include <stdio.h>

//prototypes
void TMC_WriteReg(uint8_t ax, uint8_t addr, uint32_t value);
uint32_t TMC_ReadReg(uint8_t ax, uint8_t addr);
uint8_t TMC_Service(void);   // Hauptloop: Treiber-Init mit Retry, 1 = alle ok

#endif /* INC_SPICONFIG_H_ */

