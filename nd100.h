/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2006 Per-Olof Astrom
 * Copyright (c) 2006-2008 Roger Abrahamsson
 *
 * This file is originated from the nd100em project.
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

#ifndef ND100_H
#define ND100_H

#include <stdbool.h>

/* A complete listing of registers in a program level regbank including the 8 scratch regs. */
#define _STS 0
#define _D 1
#define _P 2
#define _B 3
#define _L 4
#define _A 5
#define _T 6
#define _X 7
#define _U0 8
#define _U1 9
#define _U2 10
#define _U3 11
#define _U4 12
#define _U5 13
#define _U6 14
#define _U7 15

/* A complete listing of privileged system registers */
/* Since some of them have the same "number" but are different */
/* Or even are part of some other register or take in bits from */
/* external sources, special care need to be taken when handling these */

#define PANS 0 /* Read */
#define PANC 0 /* Write */
#define STS 1 /* Read and write, but spread out over 16 levels too in the register file */
#define OPR 2 /* Read */
#define LMP 2 /* Write */
#define PGS 3 /* Read */
#define PCR 3 /* Write */
#define PVL 4 /* Read */
#define IIC 5 /* Read */
#define IIE 5 /* Write */
#define PID 6 /* Read and write */
#define PIE 7 /* Read and write */
#define CSR 8 /* Read */
#define CCL 8 /* Write */
#define ACTL 9 /* Read */
#define LCIL 9 /* Write */
#define ALD 10 /* Read */
#define UCIL 10 /* Write */
#define PES 11 /* Read */
#define PGC 12 /* Read */
#define PEA 13 /* Read */
#define ECCR 13 /* Write */

/*
 * PANS
 *
 * | 15 | 14 | 13 | 12 | 11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 |  3 |  2 |  1 |
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 * |DISP|INP |RPAN|PAN |  0 |    PFUNC     |             RPAN                 |
 * |PRES|PDY |VAL |INT |    |              |                                  |
 * +----+----+----+----+----+--------------+----+----+----+----+----+----+----+
 *
 */

/*
 * PANC
 *
 * | 15 | 14 | 13 | 12 | 11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 |  3 |  2 |  1 |
 * +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+
 * |  0 |  0 |READ|N.A.|  0 |    PFUNC     |             WPAN                 |
 * |    |    | RQ |    |    |              |                                  |
 * +----+----+----+----+----+--------------+----+----+----+----+----+----+----+
 *
 */


/* Status register flags */

#define _PTM 0
#define _TG 1
#define _K 2
#define _Z 3
#define _Q 4
#define _O 5
#define _C 6
#define _M 7
#define _PL 8
#define _N100 12
#define _SEXI 13
#define _PONI 14
#define _IONI 15


typedef unsigned short int ushort;
typedef signed short int sshort;
typedef unsigned long int ulong;
typedef signed long int slong;

#define PAGINGSYSTEM 0
#define OPERATORSPANEL 0

/*************************************************/
/* NEW ORGANIZATION OF MEMORY AND REGISTERS!!    */
/*************************************************/

/* Lets use the full 16MWord space now (32MB ram in host)*/
//#define MEMPTSIZE 16384

// Lets just use 4MW (8MB) for now.. seems like 'CONFIG' is having some strange issues with 16MW (32MB) - at least detection is saying  "Total memory size....: 65504.000 Mbytes"
#define MEMPTSIZE 1024*8

/* Volatile Memory
 * Fixed to MEMPTSIZE KWords for now.
 */
typedef union ndram {
	unsigned char	c_Array[MEMPTSIZE*1024*2];
	ushort		n_Array[MEMPTSIZE*1024];
	ushort		n_Pages[MEMPTSIZE][1024];
} _NDRAM_ ;

/* Paging tables(Shadow memory) */
typedef union ndpt {
	ushort	word_array[4*64*2];
	ushort	normal[4][64];
	ulong	extended[4][64];
} _NDPT_ ;

/* Paging tables(Shadow memory) */
union NewPT {
	ulong	pt_arr[4*64];
	ulong	pt[4][64];
};

struct CpuRegs {
	ushort	reg[16][16];	/* main CPU registers for all runlevels */

	ushort	reg_STS;	/* STS register HIGH bits - not unique pr runlevel - used to be in reg[0][_STS]*/

	ushort	reg_PANS;	/* */
	ushort	reg_PANC;	/* */
	ushort	reg_OPR;	/* */
	ushort	reg_LMP;	/* */
	ushort	reg_PGS;	/* */	
	ushort	reg_PCR[16];	/* Paging Control Registers */
	ushort	reg_PVL;	/* */
	ushort	reg_IIC;	/* IIC is actually just a priority encoded (IID | IIE) */
	ushort	reg_IID;	/* Actual interrupt reg */
	ushort	reg_IIE;	/* */
	ushort	reg_PID;	/* */
	ushort	reg_PIE;	/* */
	ushort	reg_CSR;	/* */
	ushort	reg_CCL;	/* */
	ushort	reg_ACTL;	/* active runlevel for this cpu */
				/* NOTE:: Not sure if this is stored as a bitfield or not.. CHECK!!!! */
				/* For now we just use it as a normal value */
	ushort	reg_LCIL;	/* */
	ushort	reg_ALD;	/* */
	ushort	reg_UCIL;	/* */
	ushort	reg_PES;	/* */
	ushort	reg_PGC;	/* */
	ushort	reg_PEA;	/* */
	ushort	reg_ECCR;	/* */

	/* Personally Added to do Prefetch and Instruction more alike ND */
	ushort	myreg_IR;	/* InstructionRegister */
	ushort	myreg_PFB;	/* PrefetchBuffer */

	// Calculated EA and pagetable info (updated before opcode is executed)
	ushort effectiveAddress;
	bool useAPT;

	/* "locks" for registers that according to manual works that way (PES, PGS, IIC) */
	/* 1 = "locked" */
	/* :TODO: Check if PEA and PES should have a common lock */
	bool	mylock_PEA;
	bool	mylock_PES;
	bool	mylock_PGS;


	/* taking a shortcut by creating a PK 4bit register */
	/* always modify this as well when touching PID or PIE */
	ushort	myreg_PK;
	
	// should cpu levels be checked ?
	bool    chkit;

	/* For MOPC/OPCOM tracing and breakpoint functionality */
	/* counter for semirun mode*/
	bool	has_instr_cntr;
	ushort	instructioncounter;
	/* flag for breakpoint and breakpoint address */
	bool	has_breakpoint;
	ushort	breakpoint;
};

/*
 * A structure to trace all memoryaccesses for an instruction to be able to debug better.
 * Works as a chained list, and should be built up during an instruction, and destroyed after.
 * That way we can print all memoryaccesses to the instruction they belong even with EXRs.
 * We can add more functionality later if we want to debug more aspects of the memory accesses.
 */
struct MemTraceList {
	unsigned int addr;
	char funct; /* W=Write, R=Read, F=Fetch */
	struct MemTraceList *next;
};

typedef enum {IGNORE, CANCEL, JOIN} _THREAD_KILL_MODE_;




typedef enum {SHUTDOWN, STOP, SEMIRUN, RUN} _RUNMODE_;

typedef enum {ND1, ND4, ND10, ND100, ND100CE, ND100CX, ND110, ND110CE, ND110CX, ND110PCX} _CPUTYPE_;

#define gPC	gReg->reg[gPIL][_P]
#define gA	gReg->reg[gPIL][_A]
#define gT	gReg->reg[gPIL][_T]
#define gB	gReg->reg[gPIL][_B]
#define gD	gReg->reg[gPIL][_D]
#define gX	gReg->reg[gPIL][_X]
#define gL	gReg->reg[gPIL][_L]

#define gPANC	gReg->reg_PANC
#define gPANS	gReg->reg_PANS
#define gOPR	gReg->reg_OPR
#define gLMP	gReg->reg_LMP
#define gPGS	gReg->reg_PGS
#define gPVL	gReg->reg_PVL
#define gIIC	gReg->reg_IIC
#define gIID	gReg->reg_IID
#define gIIE	gReg->reg_IIE
#define gPID	gReg->reg_PID
#define gPIE	gReg->reg_PIE
#define gCSR	gReg->reg_CSR
#define gCCL	gReg->reg_CCL
#define gACTL	gReg->reg_ACTL
#define gLCIL	gReg->reg_LCIL
#define gALD	gReg->reg_ALD
#define gUCIL	gReg->reg_UCIL
#define gPES	gReg->reg_PES
#define gPGC	gReg->reg_PGC
#define gPEA	gReg->reg_PEA
#define gECCR	gReg->reg_ECCR


#define gPEA_Lock 	gReg->mylock_PEA
#define gPES_Lock 	gReg->mylock_PES
#define gPGS_Lock 	gReg->mylock_PES
#define gIIC_Lock 	gReg->mylock_IIC


#define CurrLEVEL	((gReg->reg_STS & 0x0f00) >>8)
#define gPIL		((gReg->reg_STS & 0x0f00) >>8)

/* Highest runlevel with PIE AND PID bits both set */
#define gPK		gReg->myreg_PK

/* Should CPU levels be checked ? */
#define gCHKIT	gReg->chkit

/* The complete Status register both MSB and LSB for current runlevel. Read only MACRO */
#define gSTSr		((gReg->reg_STS & 0xFF00) | (gReg->reg[gPIL][_STS] & 0x00FF))

#define InstructionRegister	gReg->myreg_IR
#define PrefetchBuffer		gReg->myreg_PFB

#define gEA                 gReg->effectiveAddress
#define gUseAPT             gReg->useAPT

#define STS_PTM  ((gReg->reg[gPIL][_STS]>>0) & 0x01)	/* */
#define STS_TG   ((gReg->reg[gPIL][_STS]>>1) & 0x01)	/* */
#define STS_K    ((gReg->reg[gPIL][_STS]>>2) & 0x01)	/* */
#define STS_Z    ((gReg->reg[gPIL][_STS]>>3) & 0x01)	/* */
#define STS_Q    ((gReg->reg[gPIL][_STS]>>4) & 0x01)	/* */
#define STS_O    ((gReg->reg[gPIL][_STS]>>5) & 0x01)	/* */
#define STS_C    ((gReg->reg[gPIL][_STS]>>6) & 0x01)	/* */
#define STS_M    ((gReg->reg[gPIL][_STS]>>7) & 0x01)	/* */

#define STS_PL   ((gReg->reg_STS >>8  ) & 0x0F)	/* Program runlevel */
#define STS_N100 ((gReg->reg_STS >>12 ) & 0x01)	/* Nord 100 indicator */
#define STS_SEXI ((gReg->reg_STS >>13 ) & 0x01)	/* Extended MMS adressing on/off indicator (24 bit instead of 19 bit*/
#define STS_PONI ((gReg->reg_STS >>14 ) & 0x01)	/* Memory management on/off indicator */
#define STS_IONI ((gReg->reg_STS >>15 ) & 0x01)	/* Interrupt system on/off indicator */

#endif // ND100_H

/*

ALD SWITCH

+--------+------------------+-------------------+-----------------------------------------------------------------------
|SWITCH  | ALD VECTOR (hex) | ALD VALUE (octal) | DESCRIPTION
+--------+------------------+-------------------+-----------------------------------------------------------------------
|15      |     x0           | 0                 | (Note 2)
|14      |     x1           | 1560              | Switch setting 14 -  BPUN load from floppy (1560) and run (*3)
|13      |     x2           | 20500             | Bootstrap load from Winchester disk (500) and run (*3)
|12      |     x3           | 21540             | Bootstrap load from SMD disk (1540,) and run (*3)
|11      |     x4           | 400               | BPUN load from paper tape (400) and run (*3)
|10      |     x5           | 1600              | BPUN load from HDLC (1600) and run (*3)
|9       |     x6           | 21560             | Run (*3) (No load)
|8       |     x7           | 0                 | Run (*3) (No load)
|7       |     x8           | 100000            | (Note 2)
|6       |     x9           | 101560            | Binary load from 1560 (SCSI boot use this setting..?)
|5       |     xA           | 120500            | Mass storage from 500
|4       |     xB           | 121540            | Mass storage from 1540 (SMD disk)
|3       |     xC           | 100400            | Binary load from 400 (paper tape reader)
|2       |     xD           | 101600            | Switch setting 2 -  Binary load from 1600 (HDLC)
|1       |     xE           | 121560            |
|0       |     xF           | 100000            |
+--------+------------------+-------------------+-----------------------------------------------------------------------
*/