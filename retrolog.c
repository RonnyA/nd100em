#include "retrolog.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>  // Add this for errno
#include "nd100.h" // Include nd100.h to access CPU state variables

// External declaration for gReg
extern struct CpuRegs *gReg;

static FILE *log_file = NULL;

// Helper function to check if address and opcode match
static bool check_addr_opcode_match(const LogEntry *entry, uint16_t target_P, uint16_t target_opcode)
{
    return (entry->reg_p == target_P && entry->opcode == target_opcode);
}

// Helper function to check if registers match
static bool check_registers_match(const LogEntry *entry, const CpuRegs *target_regs)
{
    return (entry->reg_a == target_regs->A &&
            entry->reg_d == target_regs->D &&
            entry->reg_t == target_regs->T &&
            entry->reg_l == target_regs->L &&
            entry->reg_x == target_regs->X &&
            entry->reg_b == target_regs->B);
}

// Check if interrupt flags match
static bool check_interrupt_flags_match(const LogEntry *entry, const MatchCriteria *criteria)
{
    if (entry == NULL || criteria == NULL)
    {
        return false;
    }

    // Check PONI flag
    if (entry->poni != criteria->target_poni)
    {
        return false;
    }

    // Check SEXI flag
    if (entry->sexi != criteria->target_sexi)
    {
        return false;
    }

    // Check IONI flag
    if (entry->ioni != criteria->target_ioni)
    {
        return false;
    }

    return true;
}

// Helper function to check if status flags match
static bool check_status_flags_match(const LogEntry *entry, const CpuFlags *target_sts)
{
    return (entry->flags.PTM == target_sts->PTM &&
            entry->flags.TG == target_sts->TG &&
            entry->flags.K == target_sts->K &&
            entry->flags.Z == target_sts->Z &&
            entry->flags.Q == target_sts->Q &&
            entry->flags.O == target_sts->O &&
            entry->flags.C == target_sts->C &&
            entry->flags.M == target_sts->M);
}

// Parse a single line from the log file
bool retrolog_parse_line_opcode(const char *start, int line_number, LogEntry *entry)
{
    if (start == NULL || entry == NULL)
    {
        return false;
    }

    // Initialize entry
    memset(entry, 0, sizeof(LogEntry));
    entry->line_number = line_number;
    
    // Parse timestamp [HH:MM:SS.mmm]
    // "Opcodes; " is 9 chars, then timestamp starts at position 9
    if (sscanf(start + 9, "[%[0-9:.] ]", entry->time) != 1)
    {
        return false;
    }

    // Parse level and ring
    // After timestamp, "[L:2 R:2]" starts at position 24
    if (sscanf(start + 24, "[L:%d R:%d]", &entry->level, &entry->ring) != 2)
    {
        return false;
    }

    // Parse PID and PIL
    // After level and ring, PID and PIL start at position 35
    if (sscanf(start + 35, "%ho %d", &entry->pid, &entry->pil) != 2)
    {
        return false;
    }

    // Parse CPU flags
    // "PtKzqocm" starts at position 49
    //  PtkzqocM    
    const char *flags_start = start + 49;
    if (flags_start != NULL)
    {
        // Check each flag directly from the string
        entry->flags.PTM = (flags_start[0] == 'P');
        entry->flags.TG = (flags_start[1] == 'T');
        entry->flags.K = (flags_start[2] == 'K');
        entry->flags.Z = (flags_start[3] == 'Z');
        entry->flags.Q = (flags_start[4] == 'Q');
        entry->flags.O = (flags_start[5] == 'O');
        entry->flags.C = (flags_start[6] == 'C');
        entry->flags.M = (flags_start[7] == 'M');
    }

    // Parse interrupt flags
    // First "|" is at position 58
    const char *int_start = start + 58;
    char int_status[10];
    if (int_start != NULL)
    {
        int_start++; // Skip "|"
        
        if (sscanf(int_start, "%[^|]", int_status) == 1)
        {
            // Set PONI and SEXI based on the status
            if (strcmp(int_status, "OFF") == 0)
            {
                entry->poni = false;
                entry->sexi = false;
            }
            else if (strcmp(int_status, "ON") == 0)
            {
                entry->poni = true;
                entry->sexi = true;
            }
            else if (strcmp(int_status, "ONx") == 0)
            {
                entry->poni = true;
                entry->sexi = false;
            }
            else
            {
                // Default to OFF
                entry->poni = false;
                entry->sexi = false;
            }
        }
    }

    // Parse IONI status
    // Second "|" is at position 62
    char ioni_status[10];
    const char *ioni_start = start + 62;
    if (ioni_start != NULL)
    {
        ioni_start++; // Skip "|"
        if (sscanf(ioni_start, "%[^ |]", ioni_status) == 1)
        {   
            // Set IONI flag - true only if status is "ON"
            entry->ioni = (strcmp(ioni_status, "ON") == 0);
        }
    }

    // Parse effective address and breakpoint
    // "EA:" is at position 151
    const char *ea_start = strstr(start, "EA:");
    if (ea_start != NULL)
    {
        ea_start += 3; // Skip "EA:"
        entry->breakpoint = (*ea_start == '*');
        if (entry->breakpoint)
        {
            ea_start++; // Skip '*'
        }
        if (sscanf(ea_start, "%ho", &entry->effective_addr) != 1)
        {
            return false;
        }
    }

    // Parse memory address (P)    
    const char *mem_start = start + 81;
    if (mem_start != NULL)
    {        
        if (sscanf(mem_start, "%ho", &entry->reg_p) != 1)
        {
            return false;
        }
    }

    // Parse opcode and disassembly    
    const char *op_start = start + 92;
    if (op_start != NULL)
    {        
        if (sscanf(op_start, "%ho", &entry->opcode) != 1)
        {
            return false;
        }
    }

    // Parse disassembly
    // Disassembly starts at position 120
    const char *dis_start = start + 120;
    if (dis_start != NULL)
    {
        if (sscanf(dis_start, "%[^[]", entry->disassembly) != 1)
        {
            return false;
        }
        // Trim trailing whitespace
        char *end = entry->disassembly + strlen(entry->disassembly) - 1;
        while (end > entry->disassembly && isspace(*end))
        {
            *end = '\0';
            end--;
        }
    }

    // Parse registers
    // Registers start at position 164
    const char *reg_start = strstr(start, "[A:");
    if (reg_start != NULL)
    {
        reg_start += 3; // Skip "[A:"
        if (sscanf(reg_start, "%ho  D:%ho  T:%ho  L:%ho  X:%ho  B:%ho",
                   &entry->reg_a, &entry->reg_d, &entry->reg_t,
                   &entry->reg_l, &entry->reg_x, &entry->reg_b) != 6)
        {
            return false;
        }
    }

    return true;
}

// Parse the next log entry from a file
int retrolog_parse_next(FILE *file, LogEntry *entry, int line_number)
{
    char line[1024];

    while (fgets(line, sizeof(line), file) != NULL)
    {
            
        line_number++;

        // Skip empty lines
        if (strlen(line) <= 1)
        {
            continue;
        }

        // Remove newline
        line[strcspn(line, "\r\n")] = 0;

        // Check the line prefix to determine which parser to use
        if (strncmp(line, "Opcodes;", 8) == 0)
        {
            // Use the opcode-specific parser
            if (retrolog_parse_line_opcode(line, line_number, entry))
            {
                return line_number;
            }
        }
        else
        {
            // TODO: add other parsers here
            continue;
        }
    }

    if (ferror(file))
    {
        fprintf(stderr, "Error reading file: %s\n", strerror(errno));
        return -1;
    }

    return -1;
}

// Find a matching log entry in a file
int retrolog_seek_next_match(FILE *file, const MatchCriteria *criteria, LogEntry *entry_found, int line_number)
{
    if (!file || !criteria)
    {
        return -1;
    }
    while (line_number != -1)
    {

        LogEntry entry; // new clean entry structure

        line_number = retrolog_parse_next(file, &entry, line_number);
        if (line_number == -1)
        {
            return -1;
        }

        // Check if address and opcode match
        if (!check_addr_opcode_match(&entry, criteria->target_regs.P, criteria->target_opcode))
        {
            continue;
        }

        // Check if level and ring match
        if (entry.level != criteria->target_level || entry.ring != criteria->target_ring)
        {
            continue;
        }



        // Check if registers match
        if (!check_registers_match(&entry, &criteria->target_regs))
        {
            continue;
        }

        // Check if interrupt flags match
        if (!check_interrupt_flags_match(&entry, criteria))
        {
            continue;
        }

        // Check if status flags match
        if (!check_status_flags_match(&entry, &criteria->target_sts))
        {
            continue;
        }

        // return the entry found
        if (entry_found != NULL)
        {
            memcpy(&entry, entry_found, sizeof(LogEntry));
        }
        return line_number;
    }

    return -1; // No match found
}

// Initialize logging system
bool retrolog_init(const char *filename)
{
    if (log_file != NULL)
    {
        fclose(log_file);
    }
    log_file = fopen(filename, "r");
    if (log_file == NULL)
    {
        fprintf(stderr, "Failed to open log file: %s\n", filename);
        return false;
    }
    return true;
}

// Close logging system
void retrolog_close(void)
{
    if (log_file != NULL)
    {
        fclose(log_file);
        log_file = NULL;
    }
}

// Print a log entry
void print_log_entry(const LogEntry *entry)
{
    if (entry == NULL)
    {
        return;
    }

    printf("Line %d: [%s] Level %d Ring %d PID %06o PIL %d\n",
           entry->line_number, entry->time, entry->level, entry->ring, entry->pid, entry->pil);

    printf("Flags: PTM=%d TG=%d K=%d Z=%d Q=%d O=%d C=%d M=%d\n",
           entry->flags.PTM, entry->flags.TG, entry->flags.K, entry->flags.Z,
           entry->flags.Q, entry->flags.O, entry->flags.C, entry->flags.M);

    printf("Interrupts: PONI=%d SEXI=%d IONI=%d\n",
           entry->poni, entry->sexi, entry->ioni);

    printf("EA: %06o%s Opcode: %06o %s\n",
           entry->effective_addr, entry->breakpoint ? "*" : "",
           entry->opcode, entry->disassembly);

    printf("Registers: A=%06o D=%06o T=%06o L=%06o X=%06o B=%06o\n",
           entry->reg_a, entry->reg_d, entry->reg_t, entry->reg_l, entry->reg_x, entry->reg_b);
}

void retrolog_fill_match_criteria_TEST(MatchCriteria *criteria)
{
    // Set target address and opcode
    criteria->target_regs.P = 010520;    // Memory address in octal
    criteria->target_opcode = 0146142; // Opcode in octal

    // Set target CPU registers
    criteria->target_regs.A = 0000000; // Register A in octal
    criteria->target_regs.D = 0002652; // Register D in octal
    criteria->target_regs.T = 0000000; // Register T in octal
    criteria->target_regs.L = 0056317; // Register L in octal
    criteria->target_regs.X = 0007635; // Register X in octal
    criteria->target_regs.B = 0146475; // Register B in octal

    // Set target interrupt flags
    criteria->target_poni = true;  // PONI flag
    criteria->target_sexi = false; // SEXI flag
    criteria->target_ioni = true; // IONI flag

    // Set target status flags
    criteria->target_sts.PTM = true; // P flag
    criteria->target_sts.TG = false; // T flag
    criteria->target_sts.K = false;  // K flag
    criteria->target_sts.Z = false;  // Z flag
    criteria->target_sts.Q = false;  // Q flag
    criteria->target_sts.O = false;  // O flag
    criteria->target_sts.C = false;  // C flag
    criteria->target_sts.M = true;   // M flag

    // Set target level and ring
    criteria->target_level = 1; // Program level
    criteria->target_ring = 2;  // Ring level
}

// Fill match criteria with current CPU state
void retrolog_fill_match_criteria(MatchCriteria *criteria)
{
    if (criteria == NULL)
    {
        return;
    }

    // Set target address and opcode    
    criteria->target_opcode = gReg->myreg_IR;

    // Set target registers
    criteria->target_regs.A = gA; // Assuming gA is defined in nd100.h
    criteria->target_regs.D = gD; // Assuming gD is defined in nd100.h
    criteria->target_regs.T = gT; // Assuming gT is defined in nd100.h
    criteria->target_regs.L = gL; // Assuming gL is defined in nd100.h
    criteria->target_regs.X = gX; // Assuming gX is defined in nd100.h
    criteria->target_regs.B = gB; // Assuming gB is defined in nd100.h
    criteria->target_regs.P = gPC;

    // Set target interrupt flags
    criteria->target_poni = STS_PONI; // Default value if gPONI is not defined
    criteria->target_sexi = STS_SEXI; // Default value if gSEXI is not defined
    criteria->target_ioni = STS_IONI; // Default value if gIONI is not defined

    // Set target status flags
    criteria->target_sts.PTM = getbit(_STS, _PTM); // Default value if gT is not defined
    criteria->target_sts.TG = getbit(_STS, _TG);   // Default value if gK is not defined
    criteria->target_sts.K = getbit(_STS, _K);     // Default value if gZ is not defined
    criteria->target_sts.Z = getbit(_STS, _Z);     // Default value if gQ is not defined
    criteria->target_sts.Q = getbit(_STS, _Q);     // Default value if gO is not defined
    criteria->target_sts.O = getbit(_STS, _O);     // Default value if gC is not defined
    criteria->target_sts.C = getbit(_STS, _C);     // Default value if gM is not defined
    criteria->target_sts.M = getbit(_STS, _M);     // Default value if gM is not defined

    // Set target level and ring
    criteria->target_level = gPIL;

    ushort pcr = gReg->reg_PCR[CurrLEVEL];
    criteria->target_ring = pcr & 0x03; 
}

int testparse()
{
    //const char *filename = "/mnt/d/ft.txt";
     const char* filename = "/mnt/d/file-trace.txt";

    MatchCriteria *criteria = malloc(sizeof(MatchCriteria));
    retrolog_fill_match_criteria_TEST(criteria);

    LogEntry *entry = malloc(sizeof(LogEntry));

    retrolog_init(filename);
    printf("Parsing log file: %s\n\n", filename);

    int line_number = retrolog_seek_next_match(log_file, criteria, entry, 0);

    if (line_number >0)
    {
        printf("Match found at line %d\n", line_number);
        print_log_entry(entry);
    }
    else
    {
        printf("No match found\n");
    }

    // Close the file
    retrolog_close();

    return 0;
}
