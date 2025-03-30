/*
 * nd100em - ND100 Virtual Machine
 *
 *  Copyright (c) 2025 Ronny Hansen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (in the main directory of the nd100em
 * distribution in the file COPYING); if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IO_NEW_H
#define IO_NEW_H

#include <stdint.h>
#include <stdbool.h>
#include "iox/devicemanager.h"

// I/O system functions
void IO_Init(void);
void IO_Destroy(void);
uint16_t IO_Read(uint32_t address);
void IO_Write(uint32_t address, uint16_t value);
int IO_Ident(uint16_t level);
void IO_Tick(void);

extern void device_interrupt(ushort interruptBits);

extern struct CpuRegs *gReg;

char *FDD_IMAGE_NAME;
bool FDD_IMAGE_RO;

char *HAWK_IMAGE_NAME;

char *BIGDISK_IMAGE_NAME;

#endif // IO_NEW_H 