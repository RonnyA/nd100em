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
uint16_t IO_Ident(uint16_t level);
void IO_Tick(void);

extern void device_interrupt(ushort interruptBits);

extern struct CpuRegs *gReg;

char *FDD_IMAGE_NAME;
bool FDD_IMAGE_RO;

char *HAWK_IMAGE_NAME;

char *BIGDISK_IMAGE_NAME;

#endif // IO_NEW_H 