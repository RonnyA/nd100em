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

#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include <stdint.h>
#include "device.h"

// External function declarations
void interrupt(uint16_t level, uint16_t bits);

// Log levels
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
} LogLevel;

// External declaration of level strings
extern const char* level_str[];

// Device types
typedef enum {
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_RTC,
    DEVICE_TYPE_TERMINAL,
    DEVICE_TYPE_PAPER_TAPE,
    DEVICE_TYPE_FLOPPY_PIO,
    DEVICE_TYPE_FLOPPY_DMA,    
    DEVICE_TYPE_DISC_SMD,
    DEVICE_TYPE_MAX
} DeviceType;

// Device info structure
typedef struct {
    Device *device;
} DeviceInfo;

// Device manager structure
typedef struct {
    DeviceInfo *devices;
    int deviceCount;
    int deviceCapacity;
    LogLevel minLogLevel;  // Minimum log level for filtering messages
} DeviceManager;

// Function declarations
void DeviceManager_Init(LogLevel level);
void DeviceManager_Destroy(void);
void DeviceManager_MasterClear(void);
bool DeviceManager_AddDevice(DeviceType type, uint8_t thumbwheel);
uint16_t DeviceManager_Read(uint32_t address);
void DeviceManager_Write(uint32_t address, uint16_t value);
int DeviceManager_Ident(uint16_t level);
uint16_t DeviceManager_Tick(void);
Device* DeviceManager_GetDeviceByAddress(uint32_t address);
void DeviceManager_AddAllDevices(void);

void DeviceManager_ClearRTC_INT();

// Logging function
void Log(LogLevel level, const char* format, ...);

#endif /* DEVICEMANAGER_H */ 