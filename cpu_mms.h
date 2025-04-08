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

#ifndef CPU_MMS_H
#define CPU_MMS_H

#include <stdbool.h>
#include <stdint.h>
#include "nd100.h"
// Type definitions
typedef uint16_t ushort;
typedef uint32_t uint;
typedef uint64_t ulong;

// Memory Management System configuration
#define ENABLE_BREAKPOINTS   // Enable breakpoint support for debugging
#define _DEGRADE_           // Enable ring-down trap

// Maximum memory size (8 MWords / 16 MBytes)
// Note: Max memory could be 16MW/32MB with 24-bit addressing, but this configuration is not commonly used
#define ND_Memsize	(sizeof(VolatileMemory)/sizeof(ushort))

// Page table flags
#define PGU_FLAG (1 << 27)  // Bit 27 - Page used
#define WIP_FLAG (1 << 28)  // Bit 28 - Written In Page

// Shadow RAM addresses
#define SHADOW_RAM_NORMAL_MODE_4PT  0xFF00  // 177400
#define SHADOW_RAM_EXTENDED_MODE_4PT 0xFE00  // 177000
#define SHADOW_RAM_EXTENDED_MODE_16PT 0xF800 // 177400

// Memory Management System types
typedef enum {
    MMS1,   // 4 page tables
    MMS2    // 16 page tables - Type 2 is necessary for VSX (Virtual Storage Extended)
} MMSType;

// Page table modes
typedef enum {
    Four,    // 4 page tables
    Sixteen  // 16 page tables
} PageTableMode;

// Memory access modes
typedef enum {
    READ = 1 << 0,
    WRITE = 1 << 1,
    FETCH = 1 << 2,
    READ_FETCH = READ | FETCH
} AccessMode;

// Write modes
typedef enum {
    WRITEMODE_MSB,    // Most significant byte
    WRITEMODE_LSB,    // Least significant byte
    WRITEMODE_WORD    // Full word
} WriteMode;

// Paging Tables structure
typedef struct {
    MMSType mmsType;           // What kind of MMS is this
    ushort* shadowRam;         // The Shadow RAM "chip"
    uint shadowRamAddress;     // Start address of Shadow RAM
    uint16_t shadowRamSize;      // Size of shadow RAM array
    bool isInitialized;        // Whether the paging tables have been initialized
} PagingTables;

// Paging Tables functions
bool CreatePagingTables(); 
void DestroyPagingTables();


// Read/Write functions for the page tables
void PT_Write(uint address, ushort value);
ushort PT_Read(uint address);

// Calculate offset into ShadowRam array
ushort GetPTShadowAddress(uint pageTable, uint VPN, PageTableMode ptm);

// Get the page table entry for a given virtual page number
uint GetPageTableEntry( uint pageTable, uint VPN, PageTableMode ptm);

// Update the page table entry for a given virtual page number
bool UpdatePageTableEntry( uint pageTable, uint VPN, PageTableMode ptm, uint PTe);

// Set the page used flag for a given virtual page number
uint SetPageUsed( uint pageTable, uint VPN, PageTableMode ptm, uint PTe);

// Set the page written flag for a given virtual page number
uint SetPageWritten( uint pageTable, uint VPN, PageTableMode ptm, uint PTe);

// Get debug info for page table entry
char* GetPageTableEntryDebugInfo(ulong PTe);

// Memory Management System functions
int mapVirtualToPhysical(uint virtualAddress, AccessMode am, bool UseAPT);

bool checkPageProtection(uint VPN, uint pageTable, ulong pageTableEntry, bool UseAPT, AccessMode am, uint virtualAddress);
bool IsAddressShadowMemory(uint addr, bool privileged);

// Virtual memory functions
int ReadVirtualMemory(uint virtualAddress, bool UseAPT);
int ReadIndirectVirtualMemory(uint virtualAddress, bool UseAPT);
int FetchVirtualMemory(uint virtualAddress,bool UseAPT);
void WriteVirtualMemory(uint virtualAddress, ushort value, bool UseAPT,WriteMode wm);


// Physical memory functions
int ReadPhysicalMemory(int physicalAddress, bool privileged);
void WritePhysicalMemory(int physicalAddress, uint16_t value, bool privileged);
void WritePhysicalMemoryWM(int physicalAddress, uint16_t value, bool privileged, WriteMode wm);


// Memory Management System failure handling functions
void UpdatePGS(uint pageTable, uint VPN, AccessMode am, bool permitViolation);
void HandleMemoryOutOfRange(uint physicalAddress);
void HandleMPV(uint virtualAddress);
void HandlePF(uint virtualAddress);

// Set PEA and PES
void setPEA(ushort pea);
void setPES(ushort pes);
void setPGS(ushort pgs);

// Global MMS type variable (extern, but could not include their .h files)
extern MMSType mmsType;
extern PagingTables pt;
extern struct CpuRegs *gReg; // cpu.c
extern void interrupt(ushort lvl,ushort sub); // cpu.c
extern _NDRAM_		VolatileMemory;

#endif /* CPU_MMS_H */  