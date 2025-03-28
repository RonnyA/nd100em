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
    DEVICE_TYPE_RTC,    
    DEVICE_TYPE_TERMINAL,
    DEVICE_TYPE_PAPER_TAPE,
    DEVICE_TYPE_FLOPPY_PIO,
    DEVICE_TYPE_FLOPPY_DMA,    
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
uint16_t DeviceManager_Ident(uint16_t level);
uint16_t DeviceManager_Tick(void);
Device* DeviceManager_GetDeviceByAddress(uint32_t address);
void DeviceManager_AddAllDevices(void);

// Logging function
void Log(LogLevel level, const char* format, ...);

#endif /* DEVICEMANAGER_H */ 