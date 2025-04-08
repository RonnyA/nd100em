#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>
#include "nd100.h"
#include "cpu_mms.h"

// Global variables
extern _NDRAM_ VolatileMemory;
extern _NDPT_ PageTable;
extern _RUNMODE_ CurrentCPURunMode;
extern _CPUTYPE_ CurrentCPUType;
extern struct CpuRegs *gReg;
extern union NewPT *gPT;
extern struct MemTraceList *gMemTrace;
extern struct IdentChain *gIdentChain;
extern double instr_counter;
extern ushort PANEL_PROCESSOR;

#endif // GLOBALS_H 