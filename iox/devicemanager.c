#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "deviceTerminal.h"
#include "devicemanager.h"
#include "devicePapertape.h"
#include "deviceFloppyPIO.h"
#include "deviceRTC.h"
#define INITIAL_DEVICE_CAPACITY 16

// Define the level strings array
const char* level_str[] = {
    "DEBUG",
    "INFO",
    "WARNING",
    "ERROR"
};

static DeviceManager deviceManager = {0}; // Initialize to zero

// Logging function implementation
void Log(LogLevel level, const char* format, ...) {
    // Skip if the message level is below the minimum level
    if (level < deviceManager.minLogLevel) {
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
    DeviceManager_AddDevice(DEVICE_TYPE_FLOPPY_PIO, 0);

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
    case DEVICE_TYPE_FLOPPY_DMA:
        // TODO: Implement FloppyDMA device
        Log(LOG_ERROR, "FloppyDMA device not implemented yet\n");
        return NULL;
    default:
        Log(LOG_ERROR, "Unknown device type: %d\n", type);
        return NULL;
    }

    // Reset the device
    if(dev)
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

    return false;
}

uint16_t DeviceManager_Read(uint32_t address)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (dev && Device_IsInAddress(dev, address))
        {
            Log(LOG_DEBUG, "Device found for READ address: %o\n", address);
            return Device_Read(dev, address);
        }
    }

    interrupt(14,1<<7); /* IOX error lvl14 */
    //Log(LOG_WARNING, "No device found for READ address: %o\n", address);
    return 0;
}

void DeviceManager_Write(uint32_t address, uint16_t value)
{
    for (int i = 0; i < deviceManager.deviceCount; i++)
    {
        Device *dev = deviceManager.devices[i].device;
        if (!dev)
        {
            Log(LOG_WARNING, "Device at index %d is NULL\n", i);
            continue;
        }


        if (Device_IsInAddress(dev, address))
        {
            Log(LOG_DEBUG, "Device found for WRITE address: %o\n", address);
            Device_Write(dev, address, value);
            return;
        }
    }

    interrupt(14,1<<7); /* IOX error lvl14 */
    //Log(LOG_WARNING, "No device found for WRITE address: %o\n", address);
}

uint16_t DeviceManager_Ident(uint16_t level)
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

    interrupt(14,1<<7); /* IOX error lvl14 */
    //Log(LOG_WARNING, "No device found for IDENT level: %d\n", level);
    return 0;
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
    if (interruptBits>0)
    {
       // printf("Interrupt bits: %o\n", interruptBits);
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
