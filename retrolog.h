/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2025 Ronny Hansen
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

#ifndef RETROLOG_H
#define RETROLOG_H

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

// Log levels
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR
} LogLevel;

// CPU register flags (Ptkzqocm)
typedef struct {    
    bool PTM;  // PTM flag
    bool TG;  // TG flag
    bool K;  // K flag
    bool Z;  // Z flag
    bool Q;  // Q flag
    bool O;  // O flag
    bool C;  // C flag
    bool M;  // M flag
} CpuFlags;

// CPU registers structure
typedef struct {
    uint16_t A;  // Register A
    uint16_t D;  // Register D
    uint16_t T;  // Register T
    uint16_t L;  // Register L
    uint16_t X;  // Register X
    uint16_t B;  // Register B
    uint16_t P;  // Register P (PC)
} CpuRegs;


typedef enum {
    LOG_TYPE_OPCODE,
    LOG_TYPE_DEVICE,
    LOG_TYPE_RTC
} LOG_TYPE;

// Log entry structure
typedef struct {
    LOG_TYPE log_type;         // Log type
    int line_number;           // Line number in the log file
    char time[20];             // Time stamp [HH:MM:SS.mmm]
    int level;                 // Program level (PIL)
    int ring;                  // Ring level
    uint16_t pid;              // Program Interrupt Detect
    int pil;                   // Program interrupt level
    CpuFlags flags;            // CPU flags
    bool poni;                 // PONI flag
    bool sexi;                 // SEXI flag
    bool ioni;                 // IONI flag
    uint16_t effective_addr;   // Effective address 
    bool breakpoint;           // Whether a breakpoint was hit (indicated by '*' in memory address)    
    uint16_t opcode;           // Opcode 
    char disassembly[100];     // Disassembled opcode
    uint16_t reg_a;            // Register A (in octal)
    uint16_t reg_d;            // Register D (in octal)
    uint16_t reg_t;            // Register T (in octal)
    uint16_t reg_l;            // Register L (in octal)
    uint16_t reg_x;            // Register X (in octal)    
    uint16_t reg_b;            // Register B (in octal)
    uint16_t reg_p;            // Register P (in octal)
} LogEntry;

// Match criteria structure for finding log entries
typedef struct {    
    uint16_t target_opcode;    // Target opcode
    CpuRegs target_regs;       // Target CPU registers
    bool target_poni;          // Target PONI flag
    bool target_sexi;          // Target SEXI flag
    bool target_ioni;          // Target IONI flag
    CpuFlags target_sts;       // Target status flags
    int target_level;          // Target program level
    int target_ring;           // Target ring level
    uint16_t target_pid;       // Target PID
} MatchCriteria;

// Initialize logging system
bool retrolog_init(const char *filename);

// Close logging system
void retrolog_close(void);

// Log a message with timestamp and level
void retrolog_message(LogLevel level, const char* format, ...);

// Parse a log file and extract entries
// Returns the number of entries parsed
int retrolog_parse_file(const char* filename, LogEntry* entries, int max_entries);

// Parse the next log entry from a file
// Returns true if an entry was successfully parsed, false otherwise
// The file handle must be opened before calling this function
int retrolog_parse_next(LogEntry *entry, int line_number);

// Parse a line that starts with "Opcodes;"
// Returns true if the line was successfully parsed, false otherwise
bool retrolog_parse_line_opcode(const char* line, int line_number, LogEntry* entry);

// Parse a line that starts with "Device;"
// Returns true if the line was successfully parsed, false otherwise
bool retrolog_parse_line_device(const char* line, int line_number, LogEntry* entry);

// Skip n lines in the log file
// Returns the number of lines actually skipped
int retrolog_skip_lines(int n, long new_pos);

// Find a matching log entry in a file
// Returns the line number of the match, or -1 if no match is found
int retrolog_find_match(FILE* file, const MatchCriteria* criteria);

// Find the next matching log entry in a file
// Returns the line number of the match, or -1 if no match is found
int retrolog_seek_next_match(const MatchCriteria* criteria, LogEntry* entry, int start_line);

// Print a log entry to stdout
void print_log_entry(const LogEntry* entry);

// Fill a MatchCriteria structure with the current CPU state
void retrolog_fill_match_criteria(MatchCriteria* criteria);

// Compare a log entry with the match criteria
// Returns true if the entry matches the criteria, false otherwise
bool retrolog_compare_log_and_entry(const MatchCriteria *criteria, LogEntry *entry);

// Log level macros
#define LOG_DEBUG(...) retrolog_message(LOG_DEBUG, __VA_ARGS__)
#define LOG_INFO(...) retrolog_message(LOG_INFO, __VA_ARGS__)
#define LOG_WARNING(...) retrolog_message(LOG_WARNING, __VA_ARGS__)
#define LOG_ERROR(...) retrolog_message(LOG_ERROR, __VA_ARGS__)

extern unsigned short getbit(unsigned short regnum, unsigned short stsbit);

#endif // RETROLOG_H 