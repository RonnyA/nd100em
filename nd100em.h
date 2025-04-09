/*
 * nd100em - ND100 Virtual Machine
 *
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

#ifndef ND100EM_H
#define ND100EM_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/resource.h>
#include <termios.h>
#include "nd100.h"

extern int trace;
extern int debug;
extern int DAEMON;
extern int DISASM;
extern ushort PANEL_PROCESSOR;

extern double instr_counter;
extern struct ThreadChain *gThreadChain;

extern float usertime, systemtime, totaltime;
extern struct rusage *used;

extern int octalstr_to_integer(char *str);
extern int mysleep(int sec, int usec);
extern int bpun_load(void);
extern int bp_load(void);
extern int debug_open(void);
extern void unsetcbreak (void);
extern void setcbreak (void);
extern struct ThreadChain *AddThreadChain(void);
extern void RemThreadChain(struct ThreadChain * elem);
extern int nd100emconf(void);
extern void shutdown(void);
extern void setsignals(void);
extern void daemonize(void);
extern void start_threads(void);
extern void stop_threads(void);
extern void setup_cpu(void);
extern void program_load(void);
extern void blocksignals();
extern int trace_open();
extern void disasm_addword(ushort addr, ushort myword);
extern void disasm_init();
extern void disasm_dump();
extern void setup_pap();
extern void cpu_start();
void cleanup_cpu(void);

int main(int argc, char *argv[]);

#endif // ND100EM_H
