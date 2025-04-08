/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2006-2011 Roger Abrahamsson
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

#ifndef ND100LIB_H
#define ND100LIB_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <termios.h>
#include "nd100.h"
#include <libconfig.h>

extern struct config_t *pCFG;

extern int trace;
extern int debug;
extern char *debugname;
extern char *debugtype;
extern FILE *debugfile;
extern int emulatemon;

extern int debug;
extern FILE *debugfile;
extern char *debugname;
extern char *debugtype;
extern int debug_open(void);
extern int CONFIG_OK;	/* This should be set to 1 when config file has been loaded OK */
typedef enum {BP, BPUN, FLOPPY, SMD} _BOOT_TYPE_;
extern _BOOT_TYPE_	BootType; /* Variable holding the way we should boot up the emulator */
extern ushort	STARTADDR;
/* should we try and disassemble as we run? */
extern int DISASM;
/* Should we detatch and become a daemon or not? */
extern int DAEMON;
/* is console on a socket, or just the local one? */
extern int CONSOLE_IS_SOCKET;

extern struct config_t *pCFG;

extern char *FDD_IMAGE_NAME;
extern bool FDD_IMAGE_RO;

extern char *HAWK_IMAGE_NAME;
extern char *BIGDISK_IMAGE_NAME;

extern struct termios savetty;

bool CreatePagingTables();

int DeviceManager_Boot(uint16_t device_id);

extern void MemoryWrite(ushort value, ushort addr, bool UseAPT, unsigned char byte_select);
extern ushort MemoryRead(ushort addr, bool UseAPT);

/* Status register bit manipulation */
extern void setbit(ushort regnum, ushort stsbit, char val);
extern void setbit_STS_MSB(ushort stsbit, char val);

extern void disasm_addword(ushort addr, ushort myword);

/* Shutdown function */
extern void shutdown(void);

/* Thread handling */
pthread_t add_thread(void *funcpointer, bool is_jointype);

#endif // ND100LIB_H