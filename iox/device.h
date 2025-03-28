#ifndef DEVICE_H
#define DEVICE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_DEVICES 16
#define MAX_DEVICE_NAME 64

// IO Delay definitions
#define IODELAY_TERMINAL 1000
#define IODELAY_FLOPPY 3000
#define IODELAY_HDD 100
#define IODELAY_HDD_SMD 100
#define IODELAY_SLOW 100
#define IODELAY_SCSI_SHORT 100
#define IODELAY_SCSI_TIMEOUT 0xFFFF

// Parity table size
#define PARITY_TABLE_SIZE 256

// IO Delay callback function type
typedef bool (*IODelayedCallback)(void *context, int param);

// IO Delay information structure
typedef struct {
    int delayTicks;
    IODelayedCallback callback;
    void *context;
    int parameter;
    uint8_t level;
} DelayedIoInfo;

// Device structure
typedef struct Device {
    // Device memory range
    uint32_t startAddress;
    uint32_t endAddress;
    
    // Interrupt handling
    uint16_t interruptBits;
    uint16_t interruptLevel;  // Default interrupt level
    uint16_t identCode;      // Identcode for this device
    
    // Device name
    char memoryName[MAX_DEVICE_NAME];
    
    // IO Delay handling
    DelayedIoInfo *ioDelays;
    int ioDelayCount;
    int ioDelayCapacity;
    
    // Device functions
    void (*Reset)(struct Device *self);
    uint16_t (*Tick)(struct Device *self);
    uint16_t (*Read)(struct Device *self, uint32_t address);
    void (*Write)(struct Device *self, uint32_t address, uint16_t value);
    uint16_t (*Ident)(struct Device *self, uint16_t level);
    
    // Device-specific data
    void *deviceData;
} Device;

typedef struct {
    Device *devices[MAX_DEVICES];
    int count;
} DeviceList;

// Function declarations
void AddDevice(DeviceList *list, Device *dev);
void TickAllDevices(DeviceList *list);
void FreeAllDevices(DeviceList *list);
void AddAllIODevices(void);
void Device_Init(Device *dev, uint8_t thumbwheel);
void Device_Destroy(Device *dev);
void Device_Reset(Device *dev);
uint16_t Device_Tick(Device *dev);
bool Device_IsInAddress(Device *dev, uint32_t address);
uint32_t Device_RegisterAddress(Device *dev, uint32_t address);
uint16_t Device_Read(Device *dev, uint32_t address);
void Device_Write(Device *dev, uint32_t address, uint16_t value);
uint16_t Device_Ident(Device *dev, uint16_t level);
void Device_QueueIODelay(Device *dev, uint16_t ticks, IODelayedCallback cb, int param, uint8_t irqlevel);
void Device_TickIODelay(Device *dev);
void Device_ClearInterrupt(Device *dev, uint16_t level);
void Device_GenerateInterrupt(Device *dev, uint16_t level);
void Device_SetInterruptStatus(Device *dev, bool active, uint16_t level);
int32_t Device_IO_ReadWord(Device *dev, FILE *f);
int32_t Device_IO_WriteWord(Device *dev, FILE *f, uint16_t data);
int32_t Device_IO_Seek(Device *dev, FILE *f, long offset);

// DMA function declarations
uint32_t Device_DMAWrite(uint32_t coreAddress, uint16_t data);
    
int32_t Device_DMARead(uint32_t coreAddress);

// Parity functions
extern const uint8_t Device_OddParityTable[PARITY_TABLE_SIZE];
uint8_t Device_GetOddParity(uint8_t value);

#endif /* DEVICE_H */ 