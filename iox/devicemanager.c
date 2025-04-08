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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "deviceTerminal.h"
#include "devicemanager.h"
#include "devicePapertape.h"
#include "deviceFloppyPIO.h"
#include "deviceFloppyDMA.h"
#include "deviceRTC.h"
#include "deviceSMD.h"

#define INITIAL_DEVICE_CAPACITY 16

//#define LOG_DEVICE_NOT_FOUND

// Define the level strings array
const char *level_str[] = {
    "DEBUG",
    "INFO",
    "WARNING",
    "ERROR"};

static DeviceManager deviceManager = {0}; // Initialize to zero

// Logging function implementation
void Log(LogLevel level, const char *format, ...)
{
    // Skip if the message level is below the minimum level
    if (level < deviceManager.minLogLevel)
    {
        return;
    }

    va_list args;
    va_start(args, format);
    printf("[%s] ", level_str[level]);
    vprintf(format, args);
    va_end(args);
}

void DeviceManager_Init(LogLevel level)
{
    // Set the minimum log level
    deviceManager.minLogLevel = level;
    Log(LOG_INFO, "Initializing device manager (min log level: %s)\n", level_str[level]);

    deviceManager.deviceCapacity = INITIAL_DEVICE_CAPACITY;
    deviceManager.deviceCount = 0;
    deviceManager.devices = malloc(sizeof(DeviceInfo) * INITIAL_DEVICE_CAPACITY);
    if (deviceManager.devices)
    {
        // Zero initialize the device array
        memset(deviceManager.devices, 0, sizeof(DeviceInfo) * INITIAL_DEVICE_CAPACITY);
        Log(LOG_INFO, "Successfully allocated device array with capacity %d\n", deviceManager.deviceCapacity);
    }
    else
    {
        Log(LOG_ERROR, "Failed to allocate device array\n");
        // Should handle allocation failure
        exit(1);
    }
}

void DeviceManager_Destroy(void)
{
    // Clean up all devices
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        if (deviceManager.devices[i].device)
        {
            Device_Destroy(deviceManager.devices[i].device);
            free(deviceManager.devices[i].device); // Free the device itself
            deviceManager.devices[i].device = NULL;
        }
    }

    if (deviceManager.devices)
    {
        free(deviceManager.devices);
        deviceManager.devices = NULL;
    }

    deviceManager.deviceCount = 0;
    deviceManager.deviceCapacity = 0;
}

void DeviceManager_AddAllDevices(void)
{
    // Add the RTC at octal 1570-1577
    DeviceManager_AddDevice(DEVICE_TYPE_RTC, 0);

    // Add the Console at octal 300-307
    DeviceManager_AddDevice(DEVICE_TYPE_TERMINAL, 0);

    // Add the PaperTape (TapeReader) at octal 400-403
    DeviceManager_AddDevice(DEVICE_TYPE_PAPER_TAPE, 0);

    // Add the FloppyPIO at octal 1560-1567
    // DeviceManager_AddDevice(DEVICE_TYPE_FLOPPY_PIO, 0);

    // Add the FloppyDMA at octal 1560-1567
    DeviceManager_AddDevice(DEVICE_TYPE_FLOPPY_DMA, 0);

    // Add the SMD at octal 1540-1547
    DeviceManager_AddDevice(DEVICE_TYPE_DISC_SMD, 0);
}

static Device *CreateDevice(DeviceType type, uint8_t thumbwheel)
{
    Device *dev = NULL;

    // Set up device-specific initialization based on type
    switch (type)
    {
    case DEVICE_TYPE_RTC:
        dev = CreateRTCDevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create RTC device\n");
            return NULL;
        }
        break;
    case DEVICE_TYPE_TERMINAL:
        dev = CreateTerminalDevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create terminal device\n");
            return NULL;
        }
        break;
    case DEVICE_TYPE_PAPER_TAPE:
        dev = CreatePaperTapeDevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create paper tape device\n");
            return NULL;
        }
        break;
    case DEVICE_TYPE_FLOPPY_PIO:
        dev = CreateFloppyPIODevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create floppy PIO device\n");
            return NULL;
        }
        break;

    case DEVICE_TYPE_DISC_SMD:
        dev = CreateSMDDevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create SMD device\n");
            return NULL;
        }
        break;
    case DEVICE_TYPE_FLOPPY_DMA:
        dev = CreateFloppyDMADevice(thumbwheel);
        if (!dev)
        {
            Log(LOG_ERROR, "Failed to create floppy DMA device\n");
            return NULL;
        }
        break;
    default:
        Log(LOG_ERROR, "Unknown device type: %d\n", type);
        return NULL;
    }

    // Reset the device
    if (dev)
    {
        Device_Reset(dev);
    }

    return dev;
}

void DeviceManager_MasterClear(void)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        if (deviceManager.devices[i].device)
        {
            Device_Reset(deviceManager.devices[i].device);
        }
    }
}

bool DeviceManager_AddDevice(DeviceType type, uint8_t thumbwheel)
{
    // Check if we have capacity
    if (deviceManager.deviceCount >= deviceManager.deviceCapacity)
    {
        Log(LOG_ERROR, "Failed to add device: device array is full (capacity: %d, count: %d)\n",
            deviceManager.deviceCapacity, deviceManager.deviceCount);
        return false;
    }

    // Create and add new device
    Device *dev = CreateDevice(type, thumbwheel);
    if (dev)
    {
        deviceManager.devices[deviceManager.deviceCount].device = dev;
        deviceManager.deviceCount++;
        return true;
    }
    else
    {
        Log(LOG_ERROR, "Failed to create device\n");
    }

    return false;
}

uint16_t DeviceManager_Read(uint32_t address)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {

        Device *dev = deviceManager.devices[i].device;

        if (dev && Device_IsInAddress(dev, address))
        {
            // Log(LOG_DEBUG, "Device found for READ address: %o\n", address);
            return Device_Read(dev, address);
        }
    }

    interrupt(14, 1 << 7); /* IOX error lvl14 */
#ifdef LOG_DEVICE_NOT_FOUND    
    Log(LOG_WARNING, "No device found for READ address: %o\n", address);
#endif
    return 0;
}

void DeviceManager_Write(uint32_t address, uint16_t value)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (!dev)
        {
            Log(LOG_ERROR, "Device at index %d is NULL\n", i);
            continue;
        }

        if (Device_IsInAddress(dev, address))
        {
            //Log(LOG_DEBUG, "Device found for WRITE address: %o\n", address);
            Device_Write(dev, address, value);
            return;
        }
    }

    interrupt(14, 1 << 7); /* IOX error lvl14 */
#ifdef LOG_DEVICE_NOT_FOUND    
    Log(LOG_WARNING, "No device found for WRITE address: %o\n", address);
#endif
}

int DeviceManager_Ident(uint16_t level)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev)
        {
            uint16_t id = Device_Ident(dev, level);
            if (id > 0)
            {
                return id;
            }
        }
    }

#ifdef LOG_DEVICE_NOT_FOUND
    // interrupt(14,1<<7); /* IOX error lvl14 */
     Log(LOG_WARNING, "No device found for IDENT level: %d\n", level);
#endif    

    return 0;
}

static Device *rtc_dev;
/// @brief Special function to clear interrupt on RTC clock
/// Returns the new active interrupt bits from alle devices
uint16_t DeviceManager_ClearRTC_INT()
{
    uint16_t interruptBits = 0;

    //printf("Clearing RTC INT\n");
    // Optimize for speed by using eralier found reference
    if (rtc_dev)
    {
        rtc_dev->interruptBits &= ~(1<<13);
    }

    // Find rtc device
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev)
        {
            rtc_dev = dev;
            if (dev->isRTC)
            {
                dev->interruptBits &= ~(1<<13);
            }
        }

        interruptBits |= dev->interruptBits;
    }

    return interruptBits;
}

uint16_t DeviceManager_Tick(void)
{
    uint16_t interruptBits = 0;
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev)
        {
            interruptBits |= Device_Tick(dev);
        }
    }

    return interruptBits;
}

uint16_t DeviceManager_Tick_RTC(void)
{
    uint16_t interruptBits = 0;
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev)
        {
            if (dev->isRTC)
            {
                interruptBits |= Device_Tick(dev);
            }
        }
    }

    return interruptBits;
}


Device *DeviceManager_GetDeviceByAddress(uint32_t address)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev && Device_IsInAddress(dev, address))
        {
            return dev;
        }
    }

    return NULL;
}

// Loads boot code from disk to memory. Returns the boot address, or -1 if error
int DeviceManager_Boot(uint16_t device_id)
{

    Device *dev = DeviceManager_GetDeviceByAddress(device_id & ~(1<<15 | 1<<13)); // mask off bit 15 and 13 when searching for device
    if (!dev) return -1;


    // Boot the device
    // Autodetect if the boot is a BPUN, MEMORY BOOT or BOOTSTRAP
    //
    // If BPUN, then we need to load the BPUN from the device 
    // If MEMORY BOOT, then we need to load the memory image from the device 
    // If BOOTSTRAP, then we need to load the bootstrap code from the device 

    // Load the BPUN image IF bit 15 in device_id is 1 - Typical paper-tape or floppy disk (400 or 1560)
    // Load using "Bootstrap"" IF bit 13 in device is 1 - Used for device 500 (Winchester disk) and 1540 (SMD disk)
    // Load the memory image IF bit 15 in device_id is 0 - Winchester disk or SMD disk (1540) (first 2KB of disk is loaded to memory at 000000-001777)

    
    // At the moment.. 
    // Only implemented for SMD, and only MEMORY boot
    
    return Device_Boot(dev,device_id);


    

#ifdef LOG_DEVICE_NOT_FOUND
    // interrupt(14,1<<7); /* IOX error lvl14 */
     Log(LOG_WARNING, "No device found for BOOT id: %d\n", level);
#endif    

    return -1;
}
