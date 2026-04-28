/*
 * SPIconfig.h
 *
 *  Created on: 27.04.2026
 *      Author: matteo gassler
 *      LICENCED under the GNU GENERAL PUBLIC LICENSE
 *
 */

#ifndef INC_SPICONFIG_H_
#define INC_SPICONFIG_H_

//includes
#include "main.h"
#include <stdio.h>

//prototypes
void TMC_WriteReg(uint8_t addr, uint32_t value);
uint32_t TMC_ReadReg(uint8_t addr);

#endif /* INC_SPICONFIG_H_ */

