/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2006 Per-Olof Astrom
 * Copyright (c) 2006-2008 Roger Abrahamsson
 * Copyright (c) 2008 Zdravko
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

//#define DEBUG_TRAP
//#define DEBUG_PK_SWITCH
//  #define DEBUG_IONOFF

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>

#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "nd100.h"
#include "cpu.h"
#include "iox/panel.h"
#include "cpu_mms.h"
#include <setjmp.h>

#include "retrolog.h"

#define BUFSTRSIZE 24

extern struct termios savetty;

/* Performance stuff */

/* used by different trace stuff before sending off to tracing */
char trace_temp_str[256];

// Used for TRAP handling to exit an instruction that fails
jmp_buf cpu_jmp_buf;

/* OpToStr
 * IN: pointer to string ,raw operand
 * OUT: Sets the string with the dissassembled operand and values
 */
void OpToStr(char *opstr, ushort operand)
{
	ushort instr;
	char numstr[BUFSTRSIZE];
	char deltastr[BUFSTRSIZE];
	unsigned char nibble;
	char offset, delta;
	unsigned char relmode;
	bool isneg;

	offset = operand & 0x00ff;
	nibble = operand & 0x000f;
	relmode = (operand & 0x0700) >> 8;
	delta = (operand & 070);

	/* put offset into a string variable in octal with +/- sign for easy reading */
	((int)offset < 0) ? (void)snprintf(numstr, BUFSTRSIZE, "-%o", -(int)offset) : (void)snprintf(numstr, BUFSTRSIZE, "%o", offset);

	/* ND110 delta offset for some instructions */
	(void)snprintf(deltastr, sizeof(deltastr), "%o", delta);

	instr = extract_opcode(operand);
	switch (instr)
	{
	case 0000000: /* STZ */
		(void)snprintf(opstr, BUFSTRSIZE, "STZ %s%s", relmode_str[relmode], numstr);
		break;
	case 0004000: /* STA */
		(void)snprintf(opstr, BUFSTRSIZE, "STA %s%s", relmode_str[relmode], numstr);
		break;
	case 0010000: /* STT */
		(void)snprintf(opstr, BUFSTRSIZE, "STT %s%s", relmode_str[relmode], numstr);
		break;
	case 0014000: /* STX */
		(void)snprintf(opstr, BUFSTRSIZE, "STX %s%s", relmode_str[relmode], numstr);
		break;
	case 0020000: /* STD */
		(void)snprintf(opstr, BUFSTRSIZE, "STD %s%s", relmode_str[relmode], numstr);
		break;
	case 0024000: /* LDD */
		(void)snprintf(opstr, BUFSTRSIZE, "LDD %s%s", relmode_str[relmode], numstr);
		break;
	case 0030000: /* STF */
		(void)snprintf(opstr, BUFSTRSIZE, "STF %s%s", relmode_str[relmode], numstr);
		break;
	case 0034000: /* LDF */
		(void)snprintf(opstr, BUFSTRSIZE, "LDF %s%s", relmode_str[relmode], numstr);
		break;
	case 0040000: /* MIN */
		(void)snprintf(opstr, BUFSTRSIZE, "MIN %s%s", relmode_str[relmode], numstr);
		break;
	case 0044000: /* LDA */
		(void)snprintf(opstr, BUFSTRSIZE, "LDA %s%s", relmode_str[relmode], numstr);
		break;
	case 0050000: /* LDT */
		(void)snprintf(opstr, BUFSTRSIZE, "LDT %s%s", relmode_str[relmode], numstr);
		break;
	case 0054000: /* LDX */
		(void)snprintf(opstr, BUFSTRSIZE, "LDX %s%s", relmode_str[relmode], numstr);
		break;
	case 0060000: /* ADD */
		(void)snprintf(opstr, BUFSTRSIZE, "ADD %s%s", relmode_str[relmode], numstr);
		break;
	case 0064000: /* SUB */
		(void)snprintf(opstr, BUFSTRSIZE, "SUB %s%s", relmode_str[relmode], numstr);
		break;
	case 0070000: /* AND */
		(void)snprintf(opstr, BUFSTRSIZE, "AND %s%s", relmode_str[relmode], numstr);
		break;
	case 0074000: /* ORA */
		(void)snprintf(opstr, BUFSTRSIZE, "ORA %s%s", relmode_str[relmode], numstr);
		break;
	case 0100000: /* FAD */
		(void)snprintf(opstr, BUFSTRSIZE, "FAD %s%s", relmode_str[relmode], numstr);
		break;
	case 0104000: /* FSB */
		(void)snprintf(opstr, BUFSTRSIZE, "FSB %s%s", relmode_str[relmode], numstr);
		break;
	case 0110000: /* FMU */
		(void)snprintf(opstr, BUFSTRSIZE, "FMU %s%s", relmode_str[relmode], numstr);
		break;
	case 0114000: /* FDV */
		(void)snprintf(opstr, BUFSTRSIZE, "FDV %s%s", relmode_str[relmode], numstr);
		break;
	case 0120000: /* MPY */
		(void)snprintf(opstr, BUFSTRSIZE, "MPY %s%s", relmode_str[relmode], numstr);
		break;
	case 0124000: /* JMP */
		(void)snprintf(opstr, BUFSTRSIZE, "JMP %s%s", relmode_str[relmode], numstr);
		break;
	case 0130000: /* JAP */
		(void)snprintf(opstr, BUFSTRSIZE, "JAP %s", numstr);
		break;
	case 0130400: /* JAN */
		(void)snprintf(opstr, BUFSTRSIZE, "JAN %s", numstr);
		break;
	case 0131000: /* JAZ */
		(void)snprintf(opstr, BUFSTRSIZE, "JAZ %s", numstr);
		break;
	case 0131400: /* JAF */
		(void)snprintf(opstr, BUFSTRSIZE, "JAF %s", numstr);
		break;
	case 0132000: /* JPC */
		(void)snprintf(opstr, BUFSTRSIZE, "JPC %s", numstr);
		break;
	case 0132400: /* JNC */
		(void)snprintf(opstr, BUFSTRSIZE, "JNC %s", numstr);
		break;
	case 0133000: /* JXZ */
		(void)snprintf(opstr, BUFSTRSIZE, "JXZ %s", numstr);
		break;
	case 0133400: /* JXN */
		(void)snprintf(opstr, BUFSTRSIZE, "JXN %s", numstr);
		break;
	case 0134000: /* JPL */
		(void)snprintf(opstr, BUFSTRSIZE, "JPL %s%s", relmode_str[relmode], numstr);
		break;
	case 0140000: /* SKP */
		(void)snprintf(opstr, BUFSTRSIZE, "SKP IF %s %s %s", skipregn_dst[(operand & 0x0007)], skiptype_str[((operand & 0x0700) >> 8)], skipregn_src[((operand & 0x0038) >> 3)]);
		break;
	case 0140120: /* ADDD */
		(void)snprintf(opstr, BUFSTRSIZE, "ADDD");
		break;
	case 0140121: /* SUBD */
		(void)snprintf(opstr, BUFSTRSIZE, "SUBD");
		break;
	case 0140122: /* COMD */
		(void)snprintf(opstr, BUFSTRSIZE, "COMD");
		break;
	case 0140123: /* TSET */
		(void)snprintf(opstr, BUFSTRSIZE, "TSET");
		break;
	case 0140124: /* PACK */
		(void)snprintf(opstr, BUFSTRSIZE, "PACK");
		break;
	case 0140125: /* UPACK */
		(void)snprintf(opstr, BUFSTRSIZE, "UPACK");
		break;
	case 0140126: /* SHDE */
		(void)snprintf(opstr, BUFSTRSIZE, "SHDE");
		break;
	case 0140127: /* RDUS */
		(void)snprintf(opstr, BUFSTRSIZE, "RDUS");
		break;
	case 0140130: /* BFILL */
		(void)snprintf(opstr, BUFSTRSIZE, "BFILL");
		break;
	case 0140131: /* MOVB */
		(void)snprintf(opstr, BUFSTRSIZE, "MOVB");
		break;
	case 0140132: /* MOVBF */
		(void)snprintf(opstr, BUFSTRSIZE, "MOVBF");
		break;
	case 0140133:																				  /* VERSN - ND110 specific */
		if ((CurrentCPUType = ND100) || (CurrentCPUType = ND100CE) || (CurrentCPUType = ND100CX)) /* We are ND100 */
			break;
		else /* We are a ND110, print instruction */
			(void)snprintf(opstr, BUFSTRSIZE, "VERSN");
	case 0140134: /* INIT */
		(void)snprintf(opstr, BUFSTRSIZE, "INIT");
		break;
	case 0140135: /* ENTR */
		(void)snprintf(opstr, BUFSTRSIZE, "ENTR");
		break;
	case 0140136: /* LEAVE */
		(void)snprintf(opstr, BUFSTRSIZE, "LEAVE");
		break;
	case 0140137: /* ELEAV */
		(void)snprintf(opstr, BUFSTRSIZE, "ELEAV");
		break;
	case 0140300: /* SETPT */
		(void)snprintf(opstr, BUFSTRSIZE, "SETPT");
		break;
	case 0140301: /* CLEPT */
		(void)snprintf(opstr, BUFSTRSIZE, "CLEPT");
		break;
	case 0140302: /* CLNREENT */
		(void)snprintf(opstr, BUFSTRSIZE, "CLNREENT");
		break;
	case 0140303: /* CHREENT-PAGES */
		(void)snprintf(opstr, BUFSTRSIZE, "CHREENT-PAGES");
		break;
	case 0140304: /* CLEPU */
		(void)snprintf(opstr, BUFSTRSIZE, "CLEPU");
		break;
	case 0140200: /* USER0 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER0");
		break;
	case 0140500:																				  /* USER1 or ND110 instruction WGLOB */
		if ((CurrentCPUType = ND100) || (CurrentCPUType = ND100CE) || (CurrentCPUType = ND100CX)) /* We are ND100 */
			(void)snprintf(opstr, BUFSTRSIZE, "USER1");
		else
			(void)snprintf(opstr, BUFSTRSIZE, "WGLOB"); /* We are ND110 */
		break;
	case 0140501: /* RGLOB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "RGLOB");
		break;
	case 0140502: /* INSPL - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "INSPL");
		break;
	case 0140503: /* REMPL - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "REMPL");
		break;
	case 0140504: /* CNREK - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "CNREK");
		break;
	case 0140505: /* CLPT  - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "CLPT");
		break;
	case 0140506: /* ENPT  - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "ENPT");
		break;
	case 0140507: /* REPT  - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "REPT");
		break;
	case 0140510: /* LBIT  - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "LBIT");
		break;
	case 0140513: /* SBITP - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SBITP");
		break;
	case 0140514: /* LBYTP - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "LBYTP");
		break;
	case 0140515: /* SBYTP - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SBYTP");
		break;
	case 0140516: /* TSETP - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "TSETP");
		break;
	case 0140517: /* RDUSP - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "RDUSP");
		break;
	case 0140600: /* EXR */
		(void)snprintf(opstr, BUFSTRSIZE, "EXR %s", skipregn_src[((operand & 0x0038) >> 3)]);
		break;
	case 0140700:																				  /* USER2 */
		if ((CurrentCPUType = ND100) || (CurrentCPUType = ND100CE) || (CurrentCPUType = ND100CX)) /* We are ND100 */
			(void)snprintf(opstr, BUFSTRSIZE, "USER2");
		else
			(void)snprintf(opstr, BUFSTRSIZE, "LASB %s", deltastr); /* We are ND110 */
		break;
	case 0140701: /* SASB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SASB %s", deltastr);
		break;
	case 0140702: /* LACB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "LACB %s", deltastr);
		break;
	case 0140703: /* SASB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SASB %s", deltastr);
		break;
	case 0140704: /* LXSB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "LXSB %s", deltastr);
		break;
	case 0140705: /* LXCB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "LXCB %s", deltastr);
		break;
	case 0140706: /* SZSB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SZSB %s", deltastr);
		break;
	case 0140707: /* SZCB - ND110 Specific */
		(void)snprintf(opstr, BUFSTRSIZE, "SZCB %s", deltastr);
		break;
	case 0141100: /* USER3 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER3");
		break;
	case 0141200: /* RMPY */
		(void)snprintf(opstr, BUFSTRSIZE, "RMPY %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0141300: /* USER4 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER4");
		break;
	case 0141500: /* USER5 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER5");
		break;
	case 0141600: /* RDIV */
		(void)snprintf(opstr, BUFSTRSIZE, "RDIV %s", skipregn_src[((operand & 0x0038) >> 3)]);
		break;
	case 0141700: /* USER6 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER6");
		break;
	case 0142100: /* USER7 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER7");
		break;
	case 0142200: /* LBYT */
		/* NOTE : moved from old parsing, SKP part, might have introduced P++ probs here */
		(void)snprintf(opstr, BUFSTRSIZE, "LBYT");
		break;
	case 0142300: /* USER8 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER8");
		break;
	case 0142500: /* USER9 */
		(void)snprintf(opstr, BUFSTRSIZE, "USER9");
		break;
	case 0142600: /* SBYT */
		/* NOTE : moved from old parsing, SKP part, might have introduced P++ probs here */
		(void)snprintf(opstr, BUFSTRSIZE, "SBYT");
		break;
	case 0142700: /* GECO - Undocumented instruction */
		(void)snprintf(opstr, BUFSTRSIZE, "GECO");
		break;
	case 0143100: /* MOVEW */
		(void)snprintf(opstr, BUFSTRSIZE, "MOVEW");
		break;
	case 0143200: /* MIX3 */
		(void)snprintf(opstr, BUFSTRSIZE, "MIX3");
		break;
	case 0143300: /* LDATX */
		(void)snprintf(opstr, BUFSTRSIZE, "LDATX");
		break;
	case 0143301: /* LDXTX */
		(void)snprintf(opstr, BUFSTRSIZE, "LDXTX");
		break;
	case 0143302: /* LDDTX */
		(void)snprintf(opstr, BUFSTRSIZE, "LDDTX");
		break;
	case 0143303: /* LDBTX */
		(void)snprintf(opstr, BUFSTRSIZE, "LDBTX");
		break;
	case 0143304: /* STATX */
		(void)snprintf(opstr, BUFSTRSIZE, "STATX");
		break;
	case 0143305: /* STZTX */
		(void)snprintf(opstr, BUFSTRSIZE, "STZTX");
		break;
	case 0143306: /* STDTX */
		(void)snprintf(opstr, BUFSTRSIZE, "STDTX");
		break;
	case 0143500: /* LWCS */
		(void)snprintf(opstr, BUFSTRSIZE, "LWCS");
		break;
	case 0143604: /* IDENT PL10 */
		(void)snprintf(opstr, BUFSTRSIZE, "IDENT PL10");
		break;
	case 0143611: /* IDENT PL11 */
		(void)snprintf(opstr, BUFSTRSIZE, "IDENT PL11");
		break;
	case 0143622: /* IDENT PL12 */
		(void)snprintf(opstr, BUFSTRSIZE, "IDENT PL12");
		break;
	case 0143643: /* IDENT PL13 */
		(void)snprintf(opstr, BUFSTRSIZE, "IDENT PL13");
		break;
	case 0144000: /* SWAP */
		(void)snprintf(opstr, BUFSTRSIZE, "SWAP %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144100: /* SWAP CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "SWAP CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144200: /* SWAP CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "SWAP CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144300: /* SWAP CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "SWAP CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144400: /* RAND */
		(void)snprintf(opstr, BUFSTRSIZE, "RAND %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144500: /* RAND CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RAND CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144600: /* RAND CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RAND CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0144700: /* RAND CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RAND CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145000: /* REXO */
		(void)snprintf(opstr, BUFSTRSIZE, "REXO %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145100: /* REXO CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "REXO CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145200: /* REXO CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "REXO CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145300: /* REXO CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "REXO CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145400: /* RORA */
		(void)snprintf(opstr, BUFSTRSIZE, "RORA %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145500: /* RORA CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RORA CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145600: /* RORA CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RORA CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0145700: /* RORA CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RORA CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146000: /* RADD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146100: /* RADD CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146200: /* RADD CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146300: /* RADD CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146400: /* RADD AD1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD AD1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146500: /* RADD AD1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD AD1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146600: /* RADD AD1 CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD AD1 CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0146700: /* RADD AD1 CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD AD1 CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0147000: /* RADD ADC */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD ADC %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0147100: /* RADD ADC CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD ADC CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0147200: /* RADD ADC CM1 */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD ADC CM1 %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0147300: /* RADD ADC CM1 CLD */
		(void)snprintf(opstr, BUFSTRSIZE, "RADD ADC CM1 CLD %s %s", skipregn_src[((operand & 0x0038) >> 3)], skipregn_dst[(operand & 0x0007)]);
		break;
	case 0147400: /* NOOP */
	case 0147500: /* NOOP */
	case 0147600: /* NOOP */
	case 0147700: /* NOOP */
		(void)snprintf(opstr, BUFSTRSIZE, "ROP NOOP");
		break;
	case 0150000: /* TRA */
		(void)snprintf(opstr, BUFSTRSIZE, "TRA %s", intregn_r[nibble]);
		break;
	case 0150100: /* TRR */
		(void)snprintf(opstr, BUFSTRSIZE, "TRR %s", intregn_w[nibble]);
		break;
	case 0150200: /* MCL */
		(void)snprintf(opstr, BUFSTRSIZE, "MCL %s", intregn_w[nibble]);
		break;
	case 0150300: /* MST */
		(void)snprintf(opstr, BUFSTRSIZE, "MST %s", intregn_w[nibble]);
		break;
	case 0150400: /* OPCOM */
		(void)snprintf(opstr, BUFSTRSIZE, "OPCOM");
		break;
	case 0150401: /* IOF */
		(void)snprintf(opstr, BUFSTRSIZE, "IOF");
		break;
	case 0150402: /* ION */
		(void)snprintf(opstr, BUFSTRSIZE, "ION");
		break;
	case 0150404: /* POF */
		(void)snprintf(opstr, BUFSTRSIZE, "POF");
		break;
	case 0150405: /* PIOF */
		(void)snprintf(opstr, BUFSTRSIZE, "PIOF");
		break;
	case 0150406: /* SEX */
		(void)snprintf(opstr, BUFSTRSIZE, "SEX");
		break;
	case 0150407: /* REX */
		(void)snprintf(opstr, BUFSTRSIZE, "REX");
		break;
	case 0150410: /* PON */
		(void)snprintf(opstr, BUFSTRSIZE, "PON");
		break;
	case 0150412: /* PION */
		(void)snprintf(opstr, BUFSTRSIZE, "PION");
		break;
	case 0150415: /* IOXT */
		(void)snprintf(opstr, BUFSTRSIZE, "IOXT");
		break;
	case 0150416: /* EXAM */
		(void)snprintf(opstr, BUFSTRSIZE, "EXAM");
		break;
	case 0150417: /* DEPO */
		(void)snprintf(opstr, BUFSTRSIZE, "DEPO");
		break;
	case 0151000:								   /* WAIT */
		(void)snprintf(opstr, BUFSTRSIZE, "WAIT"); /* TODO:: number??*/
		break;
	case 0151400: /* NLZ*/
		(void)snprintf(opstr, BUFSTRSIZE, "NLZ %s", numstr);
		break;
	case 0152000: /* DNZ*/
		(void)snprintf(opstr, BUFSTRSIZE, "DNZ %s", numstr);
		break;
	case 0152400: /* SRB */ /* NOTE: These two seems to have bit req on 0-2 as well */
		(void)snprintf(opstr, BUFSTRSIZE, "SRB %o", (operand & 0x0078));
		break;
	case 0152600: /* LRB */ /* NOTE: These two seems to have bit req on 0-2 as well */
		(void)snprintf(opstr, BUFSTRSIZE, "LRB %o", (operand & 0x0078) >> 3);
		break;
	case 0153000: /* MON */
		(void)snprintf(opstr, BUFSTRSIZE, "MON %o", (operand & 0x00ff));
		break;
	case 0153400: /* IRW */
		(void)snprintf(opstr, BUFSTRSIZE, "IRW %o %s", (operand & 0x0078), regn_w[(operand & 0x0007)]);
		break;
	case 0153600: /* IRR */
		(void)snprintf(opstr, BUFSTRSIZE, "IRR %o %s", (operand & 0x0078), regn_w[(operand & 0x0007)]);
		break;
	case 0154000: /* SHT */
		/* negative value -> shift right  else shift left*/
		isneg = ((operand & 0x0020) >> 5) ? 1 : 0;
		//		offset = ((operand & 0x0020)>>5) ? (char)((operand & 0x003F) | 0x00C0) : (operand & 0x003F);
		offset = (isneg) ? (~((operand & 0x003F) | 0xFFC0) + 1) : (operand & 0x003F);
		(isneg) ? (void)snprintf(numstr, BUFSTRSIZE, "SHR %o", offset) : (void)snprintf(numstr, BUFSTRSIZE, "%o", offset);
		(void)snprintf(opstr, BUFSTRSIZE, "SHT %s%s", shtype_str[((operand & 0x0600) >> 9)], numstr);
		break;
	case 0154200: /* SHD */
		/* negative value -> shift right  else shift left*/
		isneg = ((operand & 0x0020) >> 5) ? 1 : 0;
		offset = (isneg) ? (~((operand & 0x003F) | 0xFFC0) + 1) : (operand & 0x003F);
		(isneg) ? (void)snprintf(numstr, BUFSTRSIZE, "SHR %o", offset) : (void)snprintf(numstr, BUFSTRSIZE, "%o", offset);
		//		offset = ((operand & 0x0020)>>5) ? (char)((operand & 0x003F) | 0x00C0) : (operand & 0x003F);
		//		((int)offset <0) ? (void)snprintf(numstr,BUFSTRSIZE,"SHR %o",-(int)offset) : (void)snprintf(numstr,BUFSTRSIZE,"%o",offset);
		(void)snprintf(opstr, BUFSTRSIZE, "SHD %s%s", shtype_str[((operand & 0x0600) >> 9)], numstr);
		break;
	case 0154400: /* SHA */
		/* negative value -> shift right  else shift left*/
		isneg = ((operand & 0x0020) >> 5) ? 1 : 0;
		offset = (isneg) ? (~((operand & 0x003F) | 0xFFC0) + 1) : (operand & 0x003F);
		(isneg) ? (void)snprintf(numstr, BUFSTRSIZE, "SHR %o", offset) : (void)snprintf(numstr, BUFSTRSIZE, "%o", offset);
		//		offset = ((operand & 0x0020)>>5) ? (char)((operand & 0x003F) | 0x00C0) : (operand & 0x003F);
		//		((int)offset <0) ? (void)snprintf(numstr,BUFSTRSIZE,"SHR %o",-(int)offset) : (void)snprintf(numstr,BUFSTRSIZE,"%o",offset);
		(void)snprintf(opstr, BUFSTRSIZE, "SHA %s%s", shtype_str[((operand & 0x0600) >> 9)], numstr);
		break;
	case 0154600: /* SAD */
		/* negative value -> shift right  else shift left*/
		isneg = ((operand & 0x0020) >> 5) ? 1 : 0;
		offset = (isneg) ? (~((operand & 0x003F) | 0xFFC0) + 1) : (operand & 0x003F);
		(isneg) ? (void)snprintf(numstr, BUFSTRSIZE, "SHR %o", offset) : (void)snprintf(numstr, BUFSTRSIZE, "%o", offset);
		//		offset = ((operand & 0x0020)>>5) ? (char)((operand & 0x003F) | 0x00C0) : (operand & 0x003F);
		//		((int)offset <0) ? (void)snprintf(numstr,BUFSTRSIZE,"SHR %o",-(int)offset) : (void)snprintf(numstr,BUFSTRSIZE,"%o",offset);
		(void)snprintf(opstr, BUFSTRSIZE, "SAD %s%s", shtype_str[((operand & 0x0600) >> 9)], numstr);
		break;
	case 0160000: /* IOT */
		(void)snprintf(opstr, BUFSTRSIZE, "IOT %o", (operand & 0x07ff));
		break;
	case 0164000: /* IOX */
		(void)snprintf(opstr, BUFSTRSIZE, "IOX %o", (operand & 0x07ff));
		break;
	case 0170000: /* SAB */
		(void)snprintf(opstr, BUFSTRSIZE, "SAB %s", numstr);
		break;
	case 0170400: /* SAA */
		(void)snprintf(opstr, BUFSTRSIZE, "SAA %s", numstr);
		break;
	case 0171000: /* SAT */
		(void)snprintf(opstr, BUFSTRSIZE, "SAT %s", numstr);
		break;
	case 0171400: /* SAX */
		(void)snprintf(opstr, BUFSTRSIZE, "SAX %s", numstr);
		break;
	case 0172000: /* AAB */
		(void)snprintf(opstr, BUFSTRSIZE, "AAB %s", numstr);
		break;
	case 0172400: /* AAA */
		(void)snprintf(opstr, BUFSTRSIZE, "AAA %s", numstr);
		break;
	case 0173000: /* AAT */
		(void)snprintf(opstr, BUFSTRSIZE, "AAT %s", numstr);
		break;
	case 0173400: /* AAX */
		(void)snprintf(opstr, BUFSTRSIZE, "AAX %s", numstr);
		break;
	case 0174000:				 /* BSET ZRO */
	case 0174200:				 /* BSET ONE */
	case 0174400:				 /* BSET BCM */
	case 0174600:				 /* BSET BAC */
	case 0175000:				 /* BSKP ZRO */
	case 0175200:				 /* BSKP ONE */
	case 0175400:				 /* BSKP BCM */
	case 0175600:				 /* BSKP BAC */
	case 0176000:				 /* BSTC */
	case 0176200:				 /* BSTA */
	case 0176400:				 /* BLDC */
	case 0176600:				 /* BLDA */
	case 0177000:				 /* BANC */
	case 0177200:				 /* BAND */
	case 0177400:				 /* BORC */
	case 0177600:				 /* BORA */
		if (!(operand & 0x0007)) /* STS reg bits handling FIXME:: what if it is bit >7 & STS??*/
			(void)snprintf(opstr, BUFSTRSIZE, "%s %s", bop_str[((operand & 0x0780) >> 7)], bopstsbit_str[((operand & 0x0078) >> 3)]);
		else
			(void)snprintf(opstr, BUFSTRSIZE, "%s %o D%s", bop_str[((operand & 0x0780) >> 7)], (int)(operand & 0x0078), regn[(operand & 0x0007)]);
		break;
	default: /* UNDEF */ /* Some ND instruction codes is undefined unfortunately. */
		(void)snprintf(opstr, BUFSTRSIZE, "UNDEF");
		break;
	}
}

// Set and lock PEA
void setPEA(ushort pea)
{
	if (gPEA_Lock)
		return;
	gPEA = pea;
	gPEA_Lock = true;
}

// Set and lock PES
void setPES(ushort pes)
{
	if (gPES_Lock)
		return;
	gPES = pes;
	gPES_Lock = true;
}

// Set and lock PGS
void setPGS(ushort pgs)
{
	if (gPGS_Lock)
		return;
	gPGS = pgs;
	if (pgs != 0)
	{
		gPGS_Lock = true;
	}
}

/* STZ
 */
void ndfunc_stz(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	MemoryWrite(0, gEA, gUseAPT, 2);
}

/* STA
 */
void ndfunc_sta(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	trace_step(1, "(%06o)<=A", (int)gEA);
	MemoryWrite(gA, gEA, gUseAPT, 2);
}

/* STT
 */
void ndfunc_stt(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	MemoryWrite(gT, gEA, gUseAPT, 2);
}

/* STX
 */
void ndfunc_stx(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	MemoryWrite(gX, gEA, gUseAPT, 2);
}

/* STD
 */
void ndfunc_std(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	MemoryWrite(gA, gEA + 0, gUseAPT, 2);
	MemoryWrite(gD, gEA + 1, gUseAPT, 2);
}

/* STF
 */
void ndfunc_stf(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	MemoryWrite(gT, gEA + 0, gUseAPT, 2);
	MemoryWrite(gA, gEA + 1, gUseAPT, 2);
	MemoryWrite(gD, gEA + 2, gUseAPT, 2);
}

/* LDA
 */
void ndfunc_lda(ushort operand)
{
	if (trace)
		trace_pre(1, "A", (int)gA);

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	gA = MemoryRead(gEA, gUseAPT);

	if (DISASM)
		disasm_set_isdata(gEA);
	trace_step(1, "A<=(%06o)", (int)gEA);
	if (trace)
		trace_post(1, "A", (int)gA);
}

/* LDT
 */
void ndfunc_ldt(ushort operand)
{
	if (trace)
		trace_pre(1, "T", (int)gT);

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	gT = MemoryRead(gEA, gUseAPT);

	if (DISASM)
		disasm_set_isdata(gEA);
	if (trace)
		trace_post(1, "T", (int)gT);
}

/* LDX
 */
void ndfunc_ldx(ushort operand)
{
	if (trace)
		trace_pre(1, "X", (int)gX);

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	gX = MemoryRead(gEA, gUseAPT);

	if (DISASM)
		disasm_set_isdata(gEA);
	if (trace)
		trace_post(1, "X", (int)gX);
}

/* LDD
 */
void ndfunc_ldd(ushort operand)
{
	if (trace)
		trace_pre(2, "A", (int)gA, "D", (int)gD);

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	gA = MemoryRead(gEA + 0, gUseAPT);
	gD = MemoryRead(gEA + 1, gUseAPT);

	if (DISASM)
	{
		disasm_set_isdata(gEA + 0);
		disasm_set_isdata(gEA + 1);
	}
	if (trace)
		trace_post(2, "A", (int)gA, "D", (int)gD);
}

/* LDF
 */
void ndfunc_ldf(ushort operand)
{
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	gT = MemoryRead(gEA + 0, gUseAPT);
	gA = MemoryRead(gEA + 1, gUseAPT);
	gD = MemoryRead(gEA + 2, gUseAPT);
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/// <summary>
/// Check if we are allowed to run a privileged instruction
///
/// Privileged intructions are only available to programs running in system mode (rings 2 and 3) or when memory protection is disabled;
/// </summary>
/// <returns>TRUE if allowed to execute</returns>
bool CheckPriv()
{
	if (!STS_PONI)
		return true; // memory protection disabled

	// Check ring
	ushort pcr = gReg->reg_PCR[CurrLEVEL];
	ushort ring = pcr & 0x03;

	if ((ring == 2) || (ring == 3))
		return true;

	// Failed, not allowed to execute
	// Generate a privileged instruction interrupt
	interrupt(14, 1 << 6); // Privileged instruction
	return false;
}

// Calculate effective address for LDnTX
unsigned int calcEL(uint8_t displacement)
{

	unsigned int EL = (gX + displacement) & 0xFFFF;
	EL = (gT & 0xFF) << 16 | EL;
	EL = EL & 0xFFFFFF; // Cap at 24 bits

	// printf("calcEL: EL=%6X, gX=%4X, gT=%4X, displacement=%d\n", EL,gX, gT, displacement	);
	return EL;
}

// read el value from memory
unsigned int ReadEL(unsigned el)
{
	return ReadPhysicalMemory(el, true);
}

// write el to memory
void WriteEL(uint el, ushort value)
{
	WritePhysicalMemory(el, value, true);
}

/* STZTX
 */
void ndfunc_stztx(ushort operand)
{
	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	uint EL = calcEL(displacement);
	WriteEL(EL, 0);

}

/* STATX
 */
void ndfunc_statx(ushort operand)
{
	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	uint EL = calcEL(displacement);
	WriteEL(EL, gA);

}

/* STDTX
 */
void ndfunc_stdtx(ushort operand)
{
	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	uint EL = calcEL(displacement);
	WriteEL(EL, gA);
	WriteEL(EL + 1, gD);

}

/// <summary>
/// Load A register
///
/// Code: 143 3n0
/// Format: LDATX
///
/// Load the contents of the physical memory location pointed to
/// by the effective address into the A register.
/// A := (EL)
///
/// Affected: (A)
/// </summary>
void ndfunc_ldatx(ushort operand)
{
	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;

	unsigned int EL = calcEL(displacement);
	gA = ReadEL(EL);
}

/// <summary>
/// Load X register
/// Code: 143 3n1
/// Format: LDXTX
///
/// Load the contents of the physical memory location pointed to
/// by the effective address into the X  register.
/// X := (EL)
///
/// Affected: (X)
/// </summary>
void ndfunc_ldxtx(ushort operand)
{
	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	unsigned int EL = calcEL(displacement);

	gX = ReadEL(EL);
}

/// <summary>
/// Load Double Word
/// Code: 143 3n2
/// Format: LDDTX
///
/// Load the contents of the physical memory location pointed to by the effective address
/// into the A register and the contents of the effective address plus one  into the D register
/// A := (EL), D := (EL + 1) .
///
/// Affected: (A,D)
/// </summary>
void ndfunc_lddtx(ushort operand)
{

	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	unsigned int EL = calcEL(displacement);

	gA = ReadEL(EL);
	EL++;
	gD = ReadEL(EL);
}

/// <summary>
/// Load B register
///
/// Code: 143 3n3
/// Format: LDBTX
///
/// Load the contents of the physical memory location pointed to by the twice the
/// effective address contents into the B register, then OR the value with 177 000
/// B := 177000 V ((EL) + (EL)) (V = inclusive OR)
///
/// Affected: (B)
/// </summary>
void ndfunc_ldbtx(ushort operand)
{
	ushort temp;
	unsigned int result;

	if (!CheckPriv())
		return;

	uint8_t displacement = (operand >> 3) & 0x07;
	unsigned int EL = calcEL(displacement);

	temp = ReadEL(EL);
	result = (temp + temp) & 0xFFFF;
	gB = result | 0xFE00; // 0177000
}

/// <summary>
/// MIN - Increment memory and skip if zero
/// Code: 040 000
///
/// Format: MIN <address mode> <disp.>
///
/// Effective word is read and incremented by one and then stored in the effective location.If the result becomes zero, the next instruction is skipped.
///
/// Affected: (EL), (P)
/// </summary>
void ndfunc_min(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	ushort temp = MemoryRead(gEA, gUseAPT);
	temp++;
	MemoryWrite(temp, gEA, gUseAPT, 2);

	if (temp == 0)
		gPC++; // Next instruction is skipped
}

/* ADD
 */
void ndfunc_add(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	ushort eff_word = MemoryRead(gEA, gUseAPT);
	gA = do_add(gA, eff_word, 0);
}

/* SUB
 */
void ndfunc_sub(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	ushort eff_word = MemoryRead(gEA, gUseAPT);
	gA = do_add(gA, ~eff_word, 1);
}

/* AND
 */
void ndfunc_and(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	gA = gA & MemoryRead(gEA, gUseAPT);
}

/* ORA
 */
void ndfunc_ora(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	gA = gA | MemoryRead(gEA, gUseAPT);
}

/* FAD
 */
void ndfunc_fad(ushort operand)
{
	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	ushort a[3], b[3], r[3];
	int res;

	a[0] = gT;
	a[1] = gA;
	a[2] = gD;
	b[0] = MemoryRead(gEA + 0, gUseAPT);
	b[1] = MemoryRead(gEA + 1, gUseAPT);
	b[2] = MemoryRead(gEA + 2, gUseAPT);
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	if (trace)
		trace_pre(3, "a+0", (int)b[0], "a+1", (int)b[1], "a+2", (int)b[2]);
	res = NDFloat_Add(a, b, r);
	gT = r[0];
	gA = r[1];
	gD = r[2];
	if (res == -1)
		setbit(_STS, _TG, 1);
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* FSB
 */
void ndfunc_fsb(ushort operand)
{
	ushort a[3], b[3], r[3];

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	b[0] = gT;
	a[0] = gT;
	a[1] = gA;
	a[2] = gD;
	b[0] = MemoryRead(gEA + 0, gUseAPT);
	b[1] = MemoryRead(gEA + 1, gUseAPT);
	b[2] = MemoryRead(gEA + 2, gUseAPT);
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	if (trace)
		trace_pre(3, "a+0", (int)b[0], "a+1", (int)b[1], "a+2", (int)b[2]);
	NDFloat_Sub(a, b, r);
	gT = r[0];
	gA = r[1];
	gD = r[2];
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* FMU
 */
void ndfunc_fmu(ushort operand)
{
	ushort a[3], b[3], r[3];

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	a[0] = gT;
	a[1] = gA;
	a[2] = gD;
	b[0] = MemoryRead(gEA + 0, gUseAPT);
	b[1] = MemoryRead(gEA + 1, gUseAPT);
	b[2] = MemoryRead(gEA + 2, gUseAPT);
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	if (trace)
		trace_pre(3, "a+0", (int)b[0], "a+1", (int)b[1], "a+2", (int)b[2]);
	NDFloat_Mul(a, b, r);
	gT = r[0];
	gA = r[1];
	gD = r[2];
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* FDV
 */
void ndfunc_fdv(ushort operand)
{
	ushort a[3], b[3], r[3];

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	a[0] = gT;
	a[1] = gA;
	a[2] = gD;
	b[0] = MemoryRead(gEA + 0, gUseAPT);
	b[1] = MemoryRead(gEA + 1, gUseAPT);
	b[2] = MemoryRead(gEA + 2, gUseAPT);
	// if (trace)
	// 	trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	// if (trace)
	// 	trace_pre(3, "a+0", (int)b[0], "a+1", (int)b[1], "a+2", (int)b[2]);
	NDFloat_Div(a, b, r);

	// printf("FDV: %06o %06o %06o %06o %06o %06o ==> %06o %06o %06o\n", a[0], a[1], a[2], b[0], b[1], b[2], r[0], r[1], r[2]);
	// This fails: FDV: 042000 100000 000000 040001 140000 000000 ==> 042000 100007 000000

	gT = r[0];
	gA = r[1];
	gD = r[2];
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* JMP
 */
void ndfunc_jmp(ushort operand)
{
	ushort old_gPC = gPC - 1;

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);


	gPC = gEA;
	if (DISASM)
		disasm_userel(old_gPC, gPC);
}

/* GECO
 */
void ndfunc_geco(ushort operand)
{
	/*
		* Microcode listing lists this instruction from micro address 004000. Page 99 in the PDF document "MICROPROGRAMLISTNING FOR ND-110_32 BIT VERSION K-Gandalf-OCR"
		* Page 134 listes the GECO offset address as 7427, assuming it is means opcode 1_427_nnn

		* https://www.ndwiki.org/wiki/GECO

		GECO is a customer-specifed instruction which appears to be included as part of the standard instruction set from ND-100/CE and later.
		The name comes from the customer, GECO (Geophysical Company of Norway).

		SINTRAN III version L, and probably version K and possibly earlier, tests for GECO as part of the startup.
		From this it looks like the registers B, D, A, and X are all used as input parameters. When all are set to 0 the instruction seems to do nothing.
	*/
}

/* VERSN - ND110+
 * IN: uses A reg bit 11-8 as a bitfield to addess the byte of the version number read, the total is 16 bytes
 * so instruction has to be called 16 times, with incremented A each time.
 * OUT: Sets A, T, D
 */
/* VERSN instruction constants */
static unsigned char installation_number[] = {0x01, 0x04, 0x00, 0x01, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; // 040171 = CPU?
static int microcode_version = 0x0708;																									 // During SINTRAN boot will load new microcode, but it must be minimum 013. Read from "control store" address 0100 for new microcode to load
static int print_version = 0x80C;

void ndfunc_versn(ushort operand)
{
	int offset = (gA >> 8) & 0x0F;

	if (trace)
		trace_step(3, "Reading VERSN. Offset=%d P=%06o A=%06o X=%06o", offset, gPC, gA, gX);

	// Set D register to the installation number byte at the specified offset
	gD = installation_number[offset];

	// Set A register with print version in upper 12 bits and preserve ALD in lower 4 bits
	gA = (print_version << 4) | (gALD & 0x0F);

	// Set T register with microcode version
	gT = microcode_version;
}

/// <summary>
/// Check if the IO address points to special "in memory" registers
///
/// Addresses from 100000 . - 100777, are used to specify system control registers which have to be accessed via the ND-100 bus.
/// An example is the Error Correction Control Register (ECCR), physically located on the memory modules.
/// </summary>
/// <returns>true if the IO address was handled, false otherwise</returns>
bool UpdateMemoryIO()
{
	if ((gT < 0x8000) || (gT > 0x81FF))
		return false;

	switch (gT)
	{
	case 0x804D: // 100115
		// By disabling this register ECCR test will say that there is no ECCR memory. Which is a benefit, then it can't fail :)
		// Test #5 in "MEMORY - Version: D00 - 1986-10-30" fails, because it expects and interrupt - but at the moment I dont know why..
		if (gECCR != gA)
		{
			gECCR = gA;

			if (debug)
			{
				char opts[256] = "";

				if ((gECCR & 1 << 0) != 0)
					strcat(opts, "[0TS | Simulate memory error in bit 0] ");
				if ((gECCR & 1 << 1) != 0)
					strcat(opts, "[15T | Simulate memory error in bit 15] ");
				if ((gECCR & 1 << 2) != 0)
					strcat(opts, "[ANY | Enable parity interrup on all errors] ");
				if ((gECCR & 1 << 3) != 0)
					strcat(opts, "[DIS | Disable ECC System and parity interrupt] ");
				if ((gECCR & 1 << 4) != 0)
					strcat(opts, "[6TS | Simualate memory error in bit 6] ");

				fprintf(debugfile, "ECCR = %06o %s\n", gECCR, opts);
				//printf("ECCR = %06o %s\n", gECCR, opts);
			}
		}
		return true;
	default:
		if (debug)
			fprintf(debugfile, "Unexpected system control register IOXT via NDBUS: (%06o)\n", gT);
		break;
	}
	return false;
}

/* IOT
 * This is really an ND1 instruction
 * NOTE:: Privileged instructions
 */
void ndfunc_iot(ushort operand)
{
	// ND110 Microcode:
	// IOT - INSTRUCTION IS PRIVILEGED WHEN RING = 0 OR 1
	//                  AND ILLEGAL    WHEN RING = 2 OR 3
	if (!CheckPriv())
		return;

	/* for now handle it as illegal instruction */
	illegal_instr(operand);
}

/* IOX (Privileged)
 */
void ndfunc_iox(ushort operand)
{
	if (!CheckPriv())
		return;
	if (trace)
		trace_pre(1, "A", (int)gA);

	if (!UpdateMemoryIO())
		io_op(operand & 0x07ff);

	if (trace)
	{
		if (gT & 0x01)
			trace_step(1, "(IO:%06o)<=A", (int)(operand & 0x07ff));
		else
		{
			trace_step(1, "A<=(IO:%06o)", (int)(operand & 0x07ff));
			trace_post(1, "A", (int)gA);
		}
	}
}

/* IOXT (Privileged)
 */
void ndfunc_ioxt(ushort operand)
{
	if (!CheckPriv())
		return;

	if (trace)
		trace_pre(2, "A", (int)gA, "T", (int)gT);

	if (!UpdateMemoryIO())
		io_op(gT);

	if (trace)
	{
		if (gT & 0x01)
			trace_step(1, "(IO:%06o)<=A", (int)gT);
		else
		{
			trace_step(1, "A<=(IO:%06o)", (int)gT);
			trace_post(1, "A", (int)gA);
		}
	}
}

/* SETPT - ND110+
 *
 * NOTE: Privileged instruction
 */
void ndfunc_setpt(ushort operand)
{
	if (!CheckPriv())
		return;

	/* ND110 Microcode:
	9217  004054  %        OPCODE 140300 : SETPT 4
	9218  004054  %
	9219  004054  % SETPT: JXZ * 10               % FINISHED
	9220  004054  %        LDDTX 20
	9221  004054  %        BSET ZRO 130 DA        % PGU-BIT
	9222  004054  %        LDBTX 10
	9223  004054  %        177777                 % OLD BUG IN LDBTX
	9224  004054  %        STD ,B                 % ALWAYS INSIDE PAGE TABLE
	9225  004054  %        LDXTX 00
	9226  004054  %        JMP *—7
	*/

	int cnt = 0;

	// JXZ * 10 % FINISHED
	while (gX != 0)
	{
		uint EL = 0;
		uint EffectiveAddress = 0;

		//  LDDTX 20 <=  A: = (EL), D: = (EL + 1)
		EL = calcEL(2); // Calculates using X, T and mriDisplacement // oct 020 >>3
		gA = (ushort)ReadEL(EL);
		gD = (ushort)ReadEL(EL + 1);

		// BSET ZRO 130 DA % PGU - BIT *

		gA = gA & ~(1 << 0x0b); // 0x0b = 13 octalt. Clear bit 013 in register A

		// LDBTX 10
		EL = calcEL(1); // oct 10 >> 3
		uint elval = ReadEL(EL);
		gB = (ushort)(((elval + elval) & 0xFFFF) | 0xFE00); // 177000

		// 177777					% OLD BUG IN LDBTX

		// STD ,B
		EffectiveAddress = (uint)(gB & 0xFFFF); // (+displacement, which is 0 here)
		WriteVirtualMemory(EffectiveAddress, gA, true, WRITEMODE_WORD);
		WriteVirtualMemory(EffectiveAddress + 1, gD, true, WRITEMODE_WORD);

		//  LDXTX 00 <=  X:= (EL)
		gX = (ushort)ReadEL(calcEL(0)); // Calculates using X, T and mriDisplacement

		// Increase counter
		cnt++;
	}

	gX = (ushort)cnt; // Report number of loops in X (undocumented, but testing using "INSTRUCTION - Version: C00 - 1986-10-30" sub-program "SEGMENTS" identified it.
}

// **************************************************************************************
// ****  ND100 and ND110CX only - segment instructions
// **************************************************************************************

// PDF Page 101 (page number 99) in "MICROPROGRAMLISTNING FOR ND-110_32 BIT VERSION K-Gandalf-OCR.pdf"
/// SINTRAN III CONTROL INSTRUCTIONS
/// ALL ARE PRIVILEGED

/// <summary>
/// Clear Page Tables
/// Code: 140 301
/// Format: CLEPT
///
/// Affected: (?)
/// </summary>
void ndfunc_clept(ushort operand)
{
	if (!CheckPriv())
		return;

	/* ND110 Microcode:
	9229  004054  %        OPCODE 140301 I CLEPT
	9230  004054  %9231  004054  % CLEPT: JXZ * 11               % FINISHED
	9232  004054  %        LDBTX 10
	9233  004054  %        177777                 % OLD BUG IN LDBTX
	9234  004054  %        LDA ,B
	9235  004054  %        JAZ * 3
	9236  004054  %        STATX 20
	9237  004054  %        STZ ,B                 % ALWAYS INSIDE PAGE TABLE
	9238  004054  %        LDXTX 00
	9239  004054  %        JMP *-10
	9240  004054  %*
	*/

	/*

	 *  Affected: Pagetables, A, T, X, B registers ????
	 *  T,X used as an adress reg  with 24 bits in the xxxTX instructions
	 *
	 * This instruction apparently is a replacement for this sequence:
	 * CLEPT:	JXZ * 10	(if X=0 goto END)
	 *		LDBTX 10	(B:=177000|(2*(EL)), EL=T,X+1)
	 *		LDA ,B		(A:=(B))
	 *		JAZ * 3		(if A=0 goto LOOP)
	 *		STATX 20	((EL):=A, EL=T,X+2)
	 *		STZ ,B		( (B):=0 )
	 *		LDXTX 00	(X:=(EL), EL=T,X)
	 * LOOP:	JMP *-7		(goto CLEPT)
	 * END:		...
	 */
	ushort cnt;

	while (gX)
	{
		uint EL = 0;

		/* LDBTX 10 */
		EL = calcEL(1);
		uint elval = ReadEL(EL);
		gB = (ushort)(((elval + elval) & 0xFFFF) | 0xFE00); // 177000

		// LDA, B
		gA = (ushort)ReadVirtualMemory(gB, true);

		// JAZ *3 (jump 3 instruction if A is zero)
		if (gA != 0)
		{
			// STATX 20  =>  (EL) = A
			EL = calcEL(2); // Calculates using X, T and mriDisplacement	//020 OCT  >>3
			WriteEL(EL, (ushort)gA);

			// STZ, B
			WriteVirtualMemory(gB, 0, true, WRITEMODE_WORD);
		}

		//  LDXTX 00 <=  X:= (EL)
		EL = calcEL(0); // Calculates using X, T and mriDisplacement
		gX = (ushort)ReadEL(EL);

		// Increase counter
		cnt++;
	}

	gX = cnt;
}

/// <summary>
/// Clear non re-entrant pages
/// Code: 140 302
/// Format: CLNREENT
///
/// Segment function
///
///
/// The contents of the memory address at A+2 are read to find the page table to be cleared along with the SINTRAN RT bitmap (addressed by the X and T registers).
/// The page table entries corresponding to those bits set in the RT bitmap are then cleared.
///
/// Affected: (?)
/// </summary>
void ndfunc_clnreent(ushort operand)
{
	if (!CheckPriv())
		return;
	// TODO: Implement

	/*
	OPCODE 140302 : CLNREENT

	READ ADDRESS A+2 TO FIND PAGE TABLE TO BE AFFECTED
	READ RT - DESCRIPTION BITMAP WORDS, FOUND FROM ADDRESS X + 25.
	CLEAR PAGE-TABLE ENTRIES CORRESPONDING TO 1 - BITS IN BITMAP.
	THE LAST BITMAP—ADDRESS IS IN ADDRESS X + T.
	*/
}

/// <summary>
/// Change Page Tables
/// Code 140 303
/// Format: CHREENTPAGES
///
/// Segment function
///
/// The X  register is used to address the current (R1) and previous(Rp)  scratch registers.
/// If the R1 is zero, the re-entrant page has nothing to change so the loop is left, otherwise the contents of the memory location pointed to by the R1+2 are loaded into T.
///
/// T then contains the protect table entry, if the page has not been written to (WIP bit 12 is zero )
/// T and R1 are loaded with Rp.
/// R1 (now containing Rp) is tested again for zero.
/// If the page has been written to, the T register is loaded with the contents of the second scratch register(R2) pointed to by R1,
/// and R2 becomes the address of Rp. X is loaded with R1 as the new pointer to the re­entrant pages and Rp is loaded into the D register pointed to by A.
///
/// Affected: (?)
/// </summary>
void ndfunc_chreent_pages(ushort operand)
{
	if (!CheckPriv())
		return;
	// TODO: Implement

	/*
		OPCODE 140303 : CHREENTPAGES

		1. READ ADDRESS D.X -> R1 ; D,X -> PREVIOUS (SCRATCH REG)
		2. IF R1 = 0; SKIP RETURN (FINISHED)
		3. READ ADDRESS T,R1+2
		4. IF NOT WIP; T.R1 —> PREVIOUS; READ ADDR T.R1 -> R1; GOTO 2
		5. READ ADDRESS T,R1  —> R2
		6. WRITE R2 -> ADDRESS PREVIOUS
		7. R1 -> X ; PREVIOUS -> D.A ; RETURN
	*/
}

/// <summary>
/// Clear page tables and collect PGU information.
/// Code: 140 304
/// Format: CLEPU
///
/// Segment function
///
/// Affected: (?)
/// </summary>
void ndfunc_clepu(ushort operand)
{
	if (!CheckPriv())
		return;

	// TODO: Implement

	/*
		OPCODE 140304 : CLEPU

		AS 'CLEPT" BUT INCLUDING WORKING SET INFORMATION
		FOR ALL PAGE-TABLE ENTRIES HANDLED
		IF PGU OF ENTRY IS 1
			D /ø 300
			B /ø 776 SHR 1 - D
			B-REG BITS 0—3 IS NOW BIT NUMBER
			B-REG BITS 4-6 IS NOW WORD NUMBER
			SET BIT IN 8—WORD TABLE IN PAGE-MAP BANK
			POINTED TO BY L-REGISTER

		LAYOUT 0F 8-WORD TABLE

							BIT 15									BIT O
							________________________________________________
		L-REG -> WORD	0	# PAGE 17								PAGE 0 #
		WORD			1	# PAGE 37									20 #
		WORD			2	# PAGE 57									40 #
		WORD			3	# PAGE 177								   160 #

	*/
}

/* IDENT
 *
 * NOTE: Privileged instruction
 */
void ndfunc_ident(ushort operand)
{
	if (!CheckPriv())
		return;

	switch ((operand & 0x003f))
	{
	case 004:
		DoIDENT(10);
		break;
	case 011:
		DoIDENT(11);
		break;
	case 022:
		DoIDENT(12);
		break;
	case 043:
		DoIDENT(13);
		break;
	default:
		illegal_instr(operand); /* Assume this is how we should hanle it.. TODO: Check!!! */
	}
}

/* OPCOM (Privileged)
 */
void ndfunc_opcom(ushort operand)
{
	if (!CheckPriv())
		return;

	MODE_OPCOM = 1;
}

/// <summary>
/// IRW - Inter-Register Write
///
/// Note: This instruction results in a no-operation if the A register of the current program level is used
/// </summary>
void ndfunc_irw(ushort operand)
{
	if (!CheckPriv())
		return;

	ushort level = (operand >> 3) & 0x0F;
	ushort dr = (operand & 0x07);

	if ((level == CurrLEVEL) && (dr == _A))
		return; // A on same level, do nothing (Write from A to A on same level== NOP)

	if ((level == CurrLEVEL) && (dr == _P))
		return; // P on same level, do nothing (Because this is what the microcode does)

	if (dr == _STS)
	{
		// Update STS lower bits (which is unique for each runlevel)
		gReg->reg[level][_STS] = (gA & 0x00FF);
	}
	else
	{
		gReg->reg[level][dr] = gA;
	}
}

/// <summary>
/// IRR - Inter-Register Read
/// Code 0153600
///
/// This instruction is used to read into the A register on current program level one of the general registers inside/outside the current program level.
/// If bits 0-2 are zero, the status registers on the specified program level will be read into the A register bits 0-7, with bits 8-15 cleared.
/// The IRR instruction is privileged.
/// </summary>
void ndfunc_irr(ushort operand)
{
	if (!CheckPriv())
		return;

	ushort level = (operand >> 3) & 0x0F;
	ushort sr = (operand & 0x07);

	if (sr == 0) // STS
	{
		gA = gReg->reg[level][_STS] & 0xFF; // read only lower 8 bits
	}
	else
	{
		gA = gReg->reg[level][sr];
	}
}

/* EXAM (Privileged)
 */
void ndfunc_exam(ushort operand)
{
	if (!CheckPriv())
		return;

	// int fulladdress = (((unsigned int)gA) << 16) | (ushort)gD;
	unsigned int fulladdress = ((gA & 0xFF) << 16) | gD;
	gT = ReadPhysicalMemory(fulladdress, true);
}

/* DEPO (Privileged)
 */

void ndfunc_depo(ushort operand)
{
	if (!CheckPriv())
		return;

	unsigned int fulladdress = ((gA & 0xFF) << 16) | gD;
	WritePhysicalMemory(fulladdress, gT, true);
}

/* POF (Privileged)
 */
void ndfunc_pof(ushort operand)
{
	
	if (!CheckPriv())
		return;
	setbit_STS_MSB(_PONI, 0);
}

/* PIOF (Privileged)
 */
void ndfunc_piof(ushort operand)
{
	
	if (!CheckPriv())
		return;

	setbit_STS_MSB(_IONI, 0);	
	setbit_STS_MSB(_PONI, 0);
}

/* PON
 */
void ndfunc_pon(ushort operand)
{
	setbit_STS_MSB(_PONI, 1);
}

/* PION
 */
void ndfunc_pion(ushort operand)
{
	setbit_STS_MSB(_IONI, 1);
	setbit_STS_MSB(_PONI, 1);
	gCHKIT = true; // recalc PK
}

/// <summary>
/// IOF - Turn off interrupt system
/// </summary>
void ndfunc_iof(ushort operand)
{
	if (!CheckPriv())
		return;

	setbit_STS_MSB(_IONI, 0);
}

/// <summary>
/// ION
///
/// Turn on interrupt system
/// </summary>
void ndfunc_ion(ushort operand)
{	
	setbit_STS_MSB(_IONI, 1);
	gCHKIT = true; // recalc PK
}


/* REX (Privileged)
 */
void ndfunc_rex(ushort operand)
{
	if (!CheckPriv())
		return;

	setbit_STS_MSB(_SEXI, 0);
}

/* SEX (Privileged)
 */
void ndfunc_sex(ushort operand)
{
	if (!CheckPriv())
		return;

	setbit_STS_MSB(_SEXI, 1);
}

/* AAA
 */
void ndfunc_aaa(ushort operand)
{
	short temp;

	temp = signExtend(operand & 0xFF);
	gA = do_add(gA, temp, 0);
}

/* AAB
 */
void ndfunc_aab(ushort operand)
{
	ushort temp;

	temp = signExtend(operand & 0xFF);
	gB = do_add(gB, temp, 0);
}

/* AAT
 */
void ndfunc_aat(ushort operand)
{
	ushort temp;

	temp = signExtend(operand & 0xFF);
	gT = do_add(gT, temp, 0);
}

/* AAX
 */
void ndfunc_aax(ushort operand)
{
	ushort temp;

	temp = signExtend(operand & 0xFF);

	gX = do_add(gX, temp, 0);
}

/* MON
 */
void ndfunc_mon(ushort operand)
{
	uint16_t monitor_number = (operand & 0x1ff);

	if (emulatemon)
		mon(monitor_number);
	else
	{
		if (CurrLEVEL < 14)
		{
			if ((monitor_number & (1 << 8)) != 0)
			{
				monitor_number |= 0xFE00; // Sign extend
			}

			gReg->reg[14][_T] = monitor_number;
			interrupt(14, 1 << 1); /* Monitor Call */
			gCHKIT = true;	
		}
	}
}

/* SAA
 */
void ndfunc_saa(ushort operand)
{
	setreg(_A, signExtend(operand & 0xFF));
}

/* SAB
 */
void ndfunc_sab(ushort operand)
{
	setreg(_B, signExtend(operand & 0xFF));
}

/* SAT
 */
void ndfunc_sat(ushort operand)
{
	setreg(_T, signExtend(operand & 0xFF));
}

/* SAX
 */
void ndfunc_sax(ushort operand)
{
	setreg(_X, signExtend(operand & 0xFF));
}

/* SHT, SHD, SHA, SAD
 */
void ndfunc_shifts(ushort operand)
{
	ulong double_reg;

	switch ((operand >> 7) & 0x03)
	{
	case 0: /* SHT */
		if (trace)
			trace_pre(1, "T", (int)gT);
		gT = ShiftReg(gT, operand);
		if (trace)
			trace_post(1, "T", (int)gT);
		break;
	case 1: /* SHD */
		if (trace)
			trace_pre(1, "D", (int)gD);
		gD = ShiftReg(gD, operand);
		if (trace)
			trace_post(1, "D", (int)gD);
		break;
	case 2: /* SHA */
		if (trace)
			trace_pre(1, "A", (int)gA);
		gA = ShiftReg(gA, operand);
		if (trace)
			trace_post(1, "A", (int)gA);
		break;
	case 3: /* SAD */
		if (trace)
			trace_pre(2, "A", (int)gA, "D", (int)gD);
		double_reg = ShiftDoubleReg(((ulong)gA << 16) | gD, operand);
		gA = double_reg >> 16;
		gD = double_reg & 0xFFFF;
		if (trace)
			trace_post(2, "A", (int)gA, "D", (int)gD);
		break;
	default: /* can never reach here but... */
		break;
	}
}

/* NLZ
 */
void ndfunc_nlz(ushort operand)
{
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	DoNLZ(operand & 0xFF);
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* DNZ
 */
void ndfunc_dnz(ushort operand)
{
	if (trace)
		trace_pre(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
	DoDNZ(operand & 0xFF);
	if (trace)
		trace_post(3, "T", (int)gT, "A", (int)gA, "D", (int)gD);
}

/* SRB (Privileged)
 */
void ndfunc_srb(ushort operand)
{
	if (!CheckPriv())
		return;

	/* SRB */ /* NOTE: These two seems to have bit req on 0-2 as well */
	DoSRB(operand);
}

/* LRB (Privileged)
 */
void ndfunc_lrb(ushort operand)
{
	if (!CheckPriv())
		return;

	/* SRB */ /* NOTE: These two seems to have bit req on 0-2 as well */
	DoLRB(operand);
}

/// <summary>
/// CJP - Conditional jump
/// Instruction bits 8-10 are used to specify one of 8 jump conditions.
///
/// If the specified condition becomes true, the displacement is added to the program counter and a jump relative to current location takes place.
/// The range is 128 locations backwards and 127 locations forwards. If the specified condition is false, no jump takes place.
///
/// Execution time depends on conditions, but is the same for all instructions.
///
/// A conditional jump instruction must be specified by means of the 8 mnemonics listed below.
/// It is illegal to specify CJP or any combinations of, B, | and , X.
/// </summary>
void CJP(bool jmp_flag, ushort operand)
{
	if (jmp_flag)
	{
		ushort old_gPC = gPC - 1;

		ushort temp = signExtend(operand & 0xff);
		gPC = do_add(gPC - 1, temp, 0);

		if (DISASM)
			disasm_userel(old_gPC, gPC);
	}
}

/// <summary>
/// JAP - Jump if A register is positive or zero, A bit 15 = 0.
/// Code: 130 000
/// Format: JAP <disp.>
///
/// Affected: (P)
/// </summary>
void ndfunc_jap(ushort operand)
{
	bool flag = ((1 << 15) & gA) == 0;
	CJP(flag, operand);
}

/// <summary>
/// JAN - Jump if A register is negative, A bit 15 = 1.
/// Code: 130 400
/// Format: JAN <disp.>
///
/// Affected: (P)
/// </summary>
void ndfunc_jan(ushort operand)
{
	bool flag = ((1 << 15) & gA) != 0;
	CJP(flag, operand);
}

/// <summary>
/// JAZ - Jump if A register is zero.
/// Code: 131 000
/// Format: JAZ<disp>
///
/// Affected: (P)
/// </summary>
void ndfunc_jaz(ushort operand)
{
	bool carry = (gA == 0);
	setbit(_STS, _C, carry);

	CJP(gA == 0, operand);
}

/// <summary>
/// JAF - Jump if A register is filled (not zero)
/// Code: 131 400
/// Format: JAF<disp. >
///
/// Affected: (P)
/// </summary>
void ndfunc_jaf(ushort operand)
{
	CJP(gA != 0, operand);
}

/// <summary>
/// JPC - Count and jump if X register is positive or zero.
/// Code: 132000
/// Format: JPC<disp. >
///
/// X is incremented by one, and if the X bit 15 equals zero after the incrementation, the jump takes place.
/// Affected: (P) and (X)
/// </summary>
void ndfunc_jpc(ushort operand)
{
	gX++;

	CJP(((1 << 15) & gX) == 0, operand);
}

/// <summary>
/// JNC - Count and jump if X register is negative.
/// Code: 132 400
/// Format: JNC<disp.>
/// X is incremented by one; if then the X bit 15 equals one, the jump takes place.
///
/// Affected: (P) and(X)
/// </summary>
void ndfunc_jnc(ushort operand)
{
	gX++;
	CJP((gX & (1 << 15)) != 0, operand);
}

/// <summary>
/// JXN - Jump if X register is negative. X bit 15 = 1.
/// Code: 133 400
/// Format: JXN <disp. >
///
/// Affected: (P)
/// </summary>
void ndfunc_jxn(ushort operand)
{
	CJP((gX & (1 << 15)) != 0, operand);
}

/// <summary>
/// JXZ - Jump if X register is zero.
/// Code: 133 000
/// Format: JXZ <disp. >_
///
/// Affected: (P)
/// </summary>
void ndfunc_jxz(ushort operand)
{
	CJP(gX == 0, operand);
}

/* JPL
 */
void ndfunc_jpl(ushort operand)
{
	ushort old_gPC = gPC - 1;

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);

	gL = gPC;
	gPC = gEA;
	if (DISASM)
		disasm_userel(old_gPC, gPC);
}

/* SKP
 * Skip instructions, this one interleaves with other instructions so might need some extra checkings.
 */
void ndfunc_skp(ushort operand)
{
	if (IsSkip(operand))
		gPC++;
}

/// <summary>
/// BFILL - Byte Fill
/// Code: 140 130
/// Format: BFILL
///
/// This instruction has only one operand. The destination operand is specified in the X, and T registers.
/// The right-most byte in the A-reg. (bits 0-7) is filled into the destination field.
///
/// After execution, the X-register and T-register bit 15 point to the end of the field(after the last byte).
/// The T-register bits(0-11) equal zero.
///
/// The instruction will always have a skip return (no error condition)
/// </summary>
void ndfunc_bfill_new(ushort operand)
{
	bool useAPT = false;
	WriteMode wm;

	// Check if we should use alternative page table, bit 14 in T register
	if ((gT & (1 << 14)) != 0)
		useAPT = true;

	while ((gT & 0xfff) != 0)
	{
		// Bit 15:  0=>MSB, 1=> LSB
		wm = (gT & (1 << 15)) ? WRITEMODE_LSB : WRITEMODE_MSB;
		WriteVirtualMemory(gX, gA & 0xFF, useAPT, wm);

		gT--;

		gT ^= (1 << 15); // Flip T bit 15
		if ((gT & (1 << 15)) == 0)
			gX++;
	}

	gPC++; // Skip return
}

void ndfunc_bfill(ushort operand)
{
	ushort d1, d2, len, addr, i;
	ushort right = (gT & ((ushort)1 << 15)) ? 1 : 0;	   /* Start with right byte? (LSB) */
	bool is_apt = (gT & ((ushort)1 << 14)) ? true : false; /* Use APT or not? */
	ushort thebyte = gA & 0xff;
	len = gT & 0x0fff; /* Number of bytes to do */
	addr = gX;		   /* just in case we do 0 bytes */
	d1 = gX;
	d2 = gT;
	if (trace)
		trace_pre(2, "X", (int)gX, "T", (int)gT);
	if (trace)
		trace_step(1, "S:(%06o)-", (int)gX);
	//	if (debug) fprintf(debugfile,"BFILL(ante): gX:%06o gT:%06o byte:%03o len:%d\n",gX,gT,thebyte,len);
	for (i = 0; i < len; i++)
	{
		addr = d1 + ((i + right) >> 1); /* Word adress of byte to write */
		MemoryWrite(thebyte, addr, is_apt, ((i + right) & 1));
	}
	gT &= 0x7000;				   /* Null number of bytes, as per manual, also null bit 15 */
	gT |= ((i + right) & 1) << 15; /* set bit 15 to point to next free byte */
	gX = d1 + ((i + right) >> 1);
	//	if (debug) fprintf(debugfile,"BFILL(post): gX:%06o gT:%06o addr:%d len:%d right:%d\n",gX,gT,addr,len,right);

	gPC++; /* This function has a SKIP return on no error, which is always? */
	if (trace)
		trace_step(1, "-E:(%06o)", (int)gX); /* -E:(%06o)<=%s */
	if (trace)
		trace_post(2, "X", (int)gX, "T", (int)gT);
}

/* INIT
 * INIT instruction:
 * IN: nothing. uses PC.
 * ADDR  : INIT
 * ADDR+1: Stack demand
 * ADDR+2: Address of stack start
 * ADDR+3: Maximum stack size
 * ADDR+4: Flag
 * ADDR+5: Not used
 * ADDR+6: Error return
 * ADDR+7: Normal return
 *
 */
void ndfunc_init(ushort operand)
{
	ushort demand, start, maxsize, flag;
	/*
		if (debug) {
			fprintf(debugfile,"INIT(ante): PC+1(stack demands):%06o\n",MemoryRead(gPC+1,0));
			fprintf(debugfile,"INIT(ante): PC+2(stack start):%06o\n",MemoryRead(gPC+2,0));
			fprintf(debugfile,"INIT(ante): PC+3(max stack size):%06o\n",MemoryRead(gPC+3,0));
			fprintf(debugfile,"INIT(ante): PC+4(flag):%06o\n",MemoryRead(gPC+4,0));
			fprintf(debugfile,"INIT(ante): PC+5(not used):%06o\n",MemoryRead(gPC+5,0));
			fprintf(debugfile,"INIT(ante): PC+6(error return):%06o\n",MemoryRead(gPC+6,0));
			fprintf(debugfile,"INIT(ante): PC+7(return):%06o\n",MemoryRead(gPC+7,0));
		}
	*/
	if (trace)
		trace_pre(2, "(gPC+0)", (int)MemoryRead(gPC + 0, 0), "(gPC+1)", (int)MemoryRead(gPC + 1, 0));
	if (trace)
		trace_pre(2, "(gPC+2)", (int)MemoryRead(gPC + 2, 0), "(gPC+3)", (int)MemoryRead(gPC + 4, 0));
	if (trace)
		trace_pre(2, "(gPC+4)", (int)MemoryRead(gPC + 6, 0), "(gPC+5)", (int)MemoryRead(gPC + 5, 0));

	demand = MemoryRead(gPC + 0, 0);
	start = MemoryRead(gPC + 1, 0);
	maxsize = MemoryRead(gPC + 2, 0);
	flag = MemoryRead(gPC + 3, 0);
	if ((start + 128 + demand - 122) > (start + maxsize))
	{ /* stack overflow */
		gPC += 5;
		return;
	}
	if ((flag & 0x01) != (gReg->reg[gPIL][_STS] & 0x01))
	{
		gPC += 5;
		return;
	}
	MemoryWrite(gL + 1, start, 0, 2); /* L+1 ==> LINK */
	trace_step(1, "LINK:(%06o)<=L+1", (int)start);
	trace_step(1, "L+1=%06o", (int)gL + 1);
	MemoryWrite(gB, start + 1, 0, 2); /* B   ==> PREVB */
	trace_step(1, "PREVB:(%06o)<=B", (int)start + 1);
	trace_step(1, "B=%06o", (int)gB);
	MemoryWrite(start + maxsize, start + 3, 0, 2); /* SMAX */
	trace_step(1, "SMAX:(%06o)<=MAX", (int)start + 3);
	trace_step(1, "MAX=%06o", (int)start + maxsize);
	gB = start + 128; /* + 200 oct. */
	trace_step(1, "B<=%06o", (int)start + 128);
	/*:TODO:  Flag */
	MemoryWrite(gB + demand - 122, start + 2, 0, 2); /* STP */
	trace_step(1, "STP:(%06o)<=B+demand-172", (int)start + 2);
	trace_step(1, "B+demand-172=%06o", (int)gB + demand - 122);
	gPC += 6;
	if (trace)
		trace_post(2, "gPC", gPC, "B", gB);
	/*
		if (debug) {
			fprintf(debugfile,"INIT(post): LINK(%06o):L+1(%06o)\n",start,MemoryRead(start,0));
			fprintf(debugfile,"INIT(post): PREVB(%06o):old B(%06o)\n",start+1,MemoryRead(start+1,0));
			fprintf(debugfile,"INIT(post): STP(%06o):gB+demand-172(%06o)\n",start+2,MemoryRead(start+2,0));
			fprintf(debugfile,"INIT(post): SMAX(%06o):gB+demand-172(%06o)\n",start+3,MemoryRead(start+3,0));
			fprintf(debugfile,"INIT(post): reserved(%06o):(%06o)\n",start+4,MemoryRead(start+4,0));
			fprintf(debugfile,"INIT(post): ERRCODE(%06o):(%06o)\n",start+5,MemoryRead(start+5,0));
		}
	*/
	return;
}

/* ENTR
 * IN: nothing. uses PC.
 * ADDR  : ENTR
 * ADDR+1: Stack demand
 * ADDR+2: Error return
 * ADDR+3: Normal return
 *
 */
void ndfunc_entr(ushort operand)
{
	ushort oldB, demand, smax, stp;
	if (trace)
		trace_pre(2, "(gPC+0)", (int)MemoryRead(gPC + 0, 0), "(gPC+1)", (int)MemoryRead(gPC + 1, 0));
	if (trace)
		trace_pre(1, "(gPC+2)", (int)MemoryRead(gPC + 2, 0));
	demand = MemoryRead(gPC + 0, 0);
	smax = MemoryRead(gB - 125, 0); /* SMAX */
	if ((gB + demand - 122) > (smax))
	{ /* stack overflow */
		gPC += 1;
		return;
	}
	stp = MemoryRead(gB - 126, 0); /* STP */
	oldB = gB;
	gB = stp + 128;									/* Advance stack frame */
	MemoryWrite(gL + 1, gB - 128, 0, 2);			/* L+1 ==> LINK */
	MemoryWrite(oldB, gB - 127, 0, 2);				/* B   ==> PREVB */
	MemoryWrite(smax, gB - 125, 0, 2);				/* SMAX */
	MemoryWrite(gB + demand - 122, gB - 126, 0, 2); /* STP */
	gPC += 2;
}

/* LEAVE
 */
void ndfunc_leave(ushort operand)
{
	gPC = MemoryRead(gB - 128, 0);
	gB = MemoryRead(gB - 127, 0);
}

/* ELEAV
 */
void ndfunc_eleav(ushort operand)
{
	ushort tmp;
	tmp = MemoryRead(gB - 128, 0) - 1;
	MemoryWrite(tmp, gB - 128, 0, 2); /* LINK */
	MemoryWrite(gA, gB - 123, 0, 2);  /* A ==> ERRCODE */
	gPC = MemoryRead(gB - 128, 0);
	gB = MemoryRead(gB - 127, 0);
}

/// <summary>
/// LBYT Load byte
/// Code: 142200
/// Format: LBYT
///
/// The 8 bit byte specified by the contents of the T and X registers is loaded into the A register bits 0-7, with the A register bits 8-15 cleared.
///
/// Affected: (A)
/// </summary>
void ndfunc_lbyt(ushort operand)
{

	ushort offset = gX >> 1;
	ushort memval = MemoryRead(gT + offset, true);

	if ((gX & 1) != 0)
	{ /* ODD BYTE = LOW */
		gA = memval & 0xFF;
	}
	else
	{
		/* EVEN BYTE = HIGH*/
		gA = (memval >> 8) & 0xFF;
	}
	if (DISASM)
		disasm_set_isdata(gT + offset);
}

/// <summary>
/// SBYT - Store byte
/// Code: 142 600
/// Format: SBYT
///
/// The byte contained in the A register bits 0-7 is stored in one half of the effective location pointed by the T and X registers,
/// the second half of this effective location being unchanged. The contents of the A register are unchanged.
///
/// Affected: (EL)
/// </summary>
void ndfunc_sbyt(ushort operand)
{

	ushort offset = gX >> 1; /* same as divide by 2 */

	if ((gX & 1) != 0)

	{
		// Odd byte, write LSB value
		WriteVirtualMemory((uint)(gT + offset), gA, true, WRITEMODE_LSB);
	}
	else
	{
		// Even byte, write MSB value
		WriteVirtualMemory((uint)(gT + offset), gA, true, WRITEMODE_MSB);
	}

	if (DISASM)
		disasm_set_isdata(gT + offset);
}

/// <summary>
/// MIX3 - Multiply index by 3
///
/// X <- ((A) — 1) *3
///
/// Format: MIX3
///
/// Code: 143 200
///
/// Multiply index by 3
/// The X register is set equal to the contents of the A register minus one multiplied by three, i.e., (X) <- [(A) - 1] *3
///
/// Affected: (X)
/// </summary>
void ndfunc_mix3(ushort operand)
{
	gX = (ushort)((gA - 1) * 3);
}

bool compare_started = false;

MatchCriteria criteria;
LogEntry entry; // new clean entry structure
int line_number = 0;

void do_op(ushort operand, bool isEXR)
{

	ushort old_gPC = gPC;

#ifdef COMPARE_INSTRUCTIONS_WITH_LOG // Compare every instruction with output from RetroCore log file
	//if ((compare_started)  && (gPIL != 13))
	if (compare_started)  
	{

		ushort savePIL = gPIL;
		// Clear entry with memset
		memset(&entry, 0, sizeof(LogEntry));

		
		// Parse next entry from log file (hopefully this matches the current instruction)
		line_number = retrolog_parse_next(&entry, line_number);
		if (line_number < 0)
		{
			printf("No more RC log entries\n");
			compare_started = false;
		}
		if (savePIL != gPIL)
		{
			// Switch hasd been forced
			operand = gReg->myreg_IR;			
		}


		// Clear criteria with memset
		memset(&criteria, 0, sizeof(MatchCriteria));

		// Fill criteria with current state
		retrolog_fill_match_criteria(&criteria);
	

		if (isEXR)
		{
			// hack to make it work for EXR ST
			criteria.target_regs.P--;
			ushort exreg = (criteria.target_opcode >> 3) & 0x0007;
			switch (exreg)
			{
			case _A:
				criteria.target_opcode = criteria.target_regs.A;
				break;
			case _B:
				criteria.target_opcode = criteria.target_regs.B;
				break;
			case _X:
				criteria.target_opcode = criteria.target_regs.X;
				break;
			case _T:
				criteria.target_opcode = criteria.target_regs.T;
				break;
			case _D:
				criteria.target_opcode = criteria.target_regs.D;
				break;
			case _L:
				criteria.target_opcode = criteria.target_regs.L;
				break;
			}
		}
		// printf("%10d PIL[%2d]R[%d] ADDR[%6o] %06o %20s A[%06o] X[%06o]\n",line_number, entry.pil, entry.ring, entry.reg_p, entry.opcode, entry.disassembly, entry.reg_a, entry.reg_x);

		if (!retrolog_compare_log_and_entry(&criteria, &entry))
		{
			printf("\n\n--------------------------------\n");
			printf("line_number: %d: %s\n", line_number, entry.disassembly);
			printf("LOG says : PIL[%d| P:[%06o], OPCODE[%06o]\n", entry.level, entry.reg_p, entry.opcode);
			printf("CPU is     PIL[%d| P:[%06o], OPCODE[%06o]\n", criteria.target_level, criteria.target_regs.P, criteria.target_opcode);
			printf("--------------------------------\n");
			// Print the log entry
			print_log_entry_diff(&criteria, &entry);
			printf("\n\n--------------------------------\n");
			if ((operand != 0140500) && (operand != 0140133) && (!isEXR)) // invalide opcode + not EXR
			{
				// Clear criteria with memset
				memset(&criteria, 0, sizeof(MatchCriteria));

				// Fill criteria with current state
				retrolog_fill_match_criteria(&criteria);	

				line_number = retrolog_seek_next_match(&criteria, &entry, line_number);
				if (line_number > 0)
				{			
					printf("Found NEW match on line: %d\n", line_number);
				}
				else
				{
					printf("No more matches found\n");
					compare_started = false;
					exit(1);
				}	
								
			}
		}
	}


	if ((operand ==-1)  && (!compare_started)) // 106452 -1  && opcode  // MON 70 - COMND
	{
		printf("START COMPARE: P=%06o, gPIL=%02o\r\n", gPC, gPIL);

		// Clear criteria with memset
		memset(&criteria, 0, sizeof(MatchCriteria));

		
		// Fill criteria with current state
		retrolog_fill_match_criteria(&criteria);		

		line_number = retrolog_seek_next_match(&criteria, &entry, 0);
		if (line_number > 0)
		{

			printf("Found match on line: %d\n", line_number);

			compare_started = true;
			print_log_entry(&entry);
		}
	}

#endif COMPARE_INSTRUCTIONS_WITH_LOG // Compare every instruction with output from RetroCore log file


	/***************** EXECUTE INSTRUCTION *****************/

	if (!isEXR)
		gPC++; // Move P before starting instruction. (but not if executed from register)

	instr_funcs[operand](operand); /* call using a function pointer from the array
				   this way we are as flexible as possible as we
				   implement io calls. */

	/***************** EXECUTE INSTRUCTION *****************/

}

void illegal_instr(ushort operand)
{
	// printf("Illegal instruction %6o  at %6o\r\n", operand, gPC);

	if (trace)
		trace_step(1, "CODE=%06o", (int)operand);

	interrupt(14, 1 << 4); /* Illegal Instruction <= WILL TRAP! */
}

void unimplemented_instr(ushort operand)
{
	printf("\r\n");
	printf("--------------------------------\r\n");
	printf("CPU: Unimplemented instruction: %06o at PC: %06o\r\n", operand, gPC);
	printf("--------------------------------\r\n");
	printf("\r\n");

	// CurrentCPURunMode = STOP; /* OK unimplemented function, lets stop CPU and end program that way */
}

void prefetch()
{
	ushort temp;
	temp = MemoryFetch(gPC, false);
	gReg->myreg_PFB = temp;
}

// Math register operations
void regop(ushort operand)
{ /* SWAP RAND REXO RORA RADD RCLR EXIT RDCR RING RSUB */
	int RAD, CLD, CM1, tmp;
	ushort sr, dr, source, destination;
	ushort old_gPC = gPC;

	RAD = ((operand & 0x0400) >> 10);
	CM1 = ((operand & 0x0080) >> 7);
	CLD = ((operand & 0x0040) >> 6);

	sr = ((operand & 0x0038) >> 3);
	dr = (operand & 0x0007);

	source = (sr == 0) ? 0 : gReg->reg[CurrLEVEL][sr] & 0xFFFF;	 /* handles special case when sr=STS reg */
	destination = (CLD) ? 0 : gReg->reg[CurrLEVEL][dr] & 0xFFFF; // Get destination value

	switch (RAD)
	{
	case 0: /* Logical operation - SWAP RAND REXO RORA */
		if (dr != 0)
		{
			switch ((operand & 0x0300) >> 8)
			{
			case 0:								/* SWAP */
				tmp = gReg->reg[CurrLEVEL][dr]; /* temp if we need to do the swap */
				gReg->reg[CurrLEVEL][dr] = (CM1) ? ~source : source;
				gReg->reg[CurrLEVEL][sr] = (CLD) ? 0 : (ushort)(tmp & 0xFFFF);
				break;
			case 1: /* RAND */
				gReg->reg[CurrLEVEL][dr] &= (CM1) ? ~source : source;
				gReg->reg[CurrLEVEL][dr] = (CLD) ? 0 : gReg->reg[CurrLEVEL][dr];
				break;
			case 2: /* REXO */
				gReg->reg[CurrLEVEL][dr] = (CLD) ? ((CM1) ? ~source : source) : ((CM1) ? gReg->reg[CurrLEVEL][dr] ^ ~source : gReg->reg[CurrLEVEL][dr] ^ source);
				break;
			case 3: /* RORA */
				gReg->reg[CurrLEVEL][dr] = (CLD) ? ((CM1) ? ~source : source) : ((CM1) ? gReg->reg[CurrLEVEL][dr] | ~source : gReg->reg[CurrLEVEL][dr] | source);
				break;
			}
		}
		break;
	case 1: /* Arithmetic operation - RADD RCLR EXIT RDCR RINC RSUB */
		if (dr != 0)
		{
			tmp = gReg->reg[CurrLEVEL][dr]; /* use this insted of (dr) as we need to check for carry and things */
			switch ((operand & 0x0380) >> 7)
			{
			case 0: /* RADD */
				tmp = do_add(destination, source, 0);
				break;
			case 1: /* RADD CM1 */
				tmp = do_add(destination, ~source, 0);
				break;
			case 2: /* RADD AD1 */
				tmp = do_add(destination, source, 1);
				break;
			case 3: /* RADD AD1 CM1 */
				tmp = do_add(destination, ~source, 1);
				break;
			case 4: /* RADD ADC */
				tmp = do_add(destination, source, getbit(_STS, _C));
				break;
			case 5: /* RADD ADC CM1 */
				tmp = do_add(destination, ~source, getbit(_STS, _C));
				break;
			case 6: /* NOOP */
				break;
			case 7: /* NOOP */
				break;
			}
			gReg->reg[CurrLEVEL][dr] = (ushort)(tmp & 0xFFFF);
		}
		else
		{
			setbit(_STS, _C, 0);
		}
		break;
	}
	if ((DISASM) && (dr == _P))
	{
		disasm_userel(old_gPC, gPC);
	}
}

/* Calculates the effective address to use.
 * Uses MemoryRead to do this so we get the Page Table handling
 * done correctly. Also sets the bool use_apt points to, to tell caller what PT
 * to use for the actual use of the address supplied
 * See Manual ND.06.014, Page 34
 */
ushort New_GetEffectiveAddr(ushort instr, bool *use_apt)
{
	int disp = signExtend(instr & 0xFF);
	ushort eff_addr;

	ushort P = (gPC - 1) & 0xFFFF;

	switch ((instr >> 8) & 0x07)
	{
	case 0: /* (P) + disp */
		eff_addr = P + disp;
		*use_apt = false;
		break;
	case 1: /* (B) + disp */
		eff_addr = gB + disp;
		*use_apt = true;
		break;
	case 2: /* ((P) + disp) */
		eff_addr = P + disp;
		eff_addr = ReadIndirectVirtualMemory(eff_addr, false);
		*use_apt = true;
		break;
	case 3: /* ((B) + disp) */
		eff_addr = gB + disp;
		eff_addr = ReadIndirectVirtualMemory(eff_addr, true);
		*use_apt = true;
		break;
	case 4: /* (X) + disp */
		eff_addr = gX + disp;
		*use_apt = true;
		break;
	case 5: /* (B) + disp + (X) */
		eff_addr = gB + gX + disp;
		*use_apt = true;
		break;
	case 6: /* ((P) + disp) + (X) */
		eff_addr = P + disp;
		eff_addr = gX + ReadIndirectVirtualMemory(eff_addr, false);
		*use_apt = true;
		break;
	case 7: /* ((B) + disp) + (X) */
		eff_addr = (gB + disp) & 0xFFFF;
		eff_addr = gX + ReadIndirectVirtualMemory(eff_addr, true);
		*use_apt = true;
		break;
	}
	return eff_addr;
}

/*
 * DoMCL - Masked Clear
 *  Affected: Internal register specified
 *  (Only STS, PID & PIE possible)
 *  <IR> = <IR> & (~A)
 *
 * NOTE:: STS need to be checked.
 * NOTE:: Privileged instructions
 */
void DoMCL(ushort instr)
{
	if (!CheckPriv())
		return;

	switch (instr & 0x0F)
	{
	case 01: // STS
		gReg->reg[CurrLEVEL][_STS] &= ~(gA & 0x00FF);
		break;
	case 06: // PID
		/* This affects interrupt, so do locking and checking. */
		if (trace)
			trace_pre(2, "PID", gPID, "A", gA);

		gPID &= ~gA;

		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "PID {AND}{NOT} A", 0);
		if (trace)
			trace_post(1, "PID", gPID);
		break;
	case 07: // PIE
		/* This affects interrupt, so do locking and checking. */
		if (trace)
			trace_pre(2, "PIE", gPIE, "A", gA);
		gPIE &= ~gA;
		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "PIE {AND}{NOT} A", 0);
		if (trace)
			trace_post(1, "PIE", gPIE);
		break;
	default:
		/* :TODO: Check if we need to do illegal instruction handling */
		break;
	}
}

/*
 * DoMST - Masked SET
 *  Affected: Internal register specified
 *  (Only STS, PID & PIE possible)
 *  <IR> = <IR> | (A)
 *
 * NOTE:: STS need to be checked.
 * NOTE:: Privileged instructions
 */
void DoMST(ushort instr)
{
	if (!CheckPriv())
		return;

	switch (instr & 0x0F)
	{
	case 01: // STS
		gReg->reg[CurrLEVEL][0] |= (gA & 0x00ff);
		break;
	case 06: // PID
		/* This affects interrupt, so do locking and checking. */
		if (trace)
			trace_pre(2, "PID", gPID, "A", gA);

		gPID |= gA;
		gCHKIT = true; // we need to check PK after this
		
		
		if (trace)
			trace_step(1, "PID {AND}{NOT} A", 0);
		if (trace)
			trace_post(1, "PID", gPID);
		break;
	case 07: // PIE
		/* This affects interrupt, so do locking and checking. */
		if (trace)
			trace_pre(2, "PIE", gPIE, "A", gA);
		gPIE |= gA;

		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "PIE {AND}{NOT} A", 0);
		if (trace)
			trace_post(1, "PIE", gPIE);
		break;
	default:
		/* :TODO: Check if we need to do illegal instruction handling */
		break;
	}
}

// Calculate internal Interrupt
/// <summary>
/// IIC - Internal Interrupt Code
///
/// This register will hold a code between 0 - 12 (oct), which will identify the internal source for the interrupt.
/// Priority encoded IID | IIE
/// </summary>
///
///                   (oct)
///      | IED bit  | IIC code |
///  ----+----------+----------+------------------------------------------------------------------------
///  n/a |   0      |    0     | Not assigned
///  MC  |   1      |    1     | Monitor Call
///  PV  |   2      |    2     | Protect Violation. Page number is found in the Paging Status Register.
///  PF  |   3      |    3     | Page fault. Page not in memory.
///  II  |   4      |    4     | lllegal instruction. Not implemented instruction.
///  Z   |   5      |    5     | Error indicator. The Z indicator is set.
///  PI  |   6      |    6     | Privileged instruction.
///  IOX |   7      |    7     | IOX error. No answer from external device.
///  PTY |   8      |    10    | Memory parity error
///  MOR |   9      |    11    | Memory out of range Addressing non-existent memory.
///  POW |   10     |    12    | Power fail interrupt
///  ----+----------+----------+------------------------------------------------------------------------
ushort calcIIC()
{
	ushort priorityCode = gIID & gIIE;
	if (priorityCode == 0)
		return 0;

	// printf("IID=0x%x, IIE=0x%x, priorityCode=0x%x\r\n", gIID, gIIE, priorityCode);

	for (int i = 10; i >= 0; i--)
	{
		if ((priorityCode & (1 << i)) != 0)
		{
			return (ushort)i;
		}
	}
	return 0;
}

/*
 * DoTRA - Transfer to register
 *  Affected: Accumulator
 *  A = <IR>;
 *
 * NOTE: Privileged instructions
 */
void DoTRA(ushort instr)
{
	if (!CheckPriv())
		return;

	ushort temp, level;
	ushort i;
	switch (instr & 0x0F)
	{
	case 00: /* TRA PANS */
		gA = gPANS;
		if (trace)
			trace_step(1, "A<=PANS", 0);
		if (debug)
			fprintf(debugfile, "TRA PANS: A <= %06o\n", gA);
		break;
	case 01:								 /* TRA STS */
		gA = gReg->reg[gPIL][_STS] & 0x00FF; /* Only lower 8 bits */
		gA |= gReg->reg_STS & 0xFF00;		 /* Upper 8 bits - SYSTEM bits*/

		if (trace)
			trace_step(1, "A<=STS", 0);
		break;
	case 02: /* TRA OPR */
		gA = gOPR;
		if (trace)
			trace_step(1, "A<=OPR", 0);
		if (debug)
			fprintf(debugfile, "TRA OPR:\n");
		break;
	case 03: /* TRA PGS */
		/* TODO:: Check that this also is supposed to clear the PGS as it "unlocks" it */
		gA = gPGS;
		gPGS_Lock = false;
		gPGS = 0;
		if (trace)
			trace_step(1, "A<=PGS", 0);
		break;
	case 04: /* TRA PVL */
		/* This one has a strange format. Described in ND-100 Functional Description section 2.9.2.5.4 */
		gA = 0;							  /* Clean it */
		gA = (gPVL & 0x0F) << 3 | 0xd782; /* = IRR (PVL) DP */
		if (trace)
			trace_step(1, "A<=PVL", 0);
		break;
	case 05: /* TRA IIC */
		/* Manuals says(2.2.4.3) that this should be a number equal to the highest bit set in (IID & IIE) - Roger */
		/* Only bit 1-10 is used, so we only return a value between 1 and 10  or else  zero */

		gIIC = calcIIC();

		gA = gIIC;

		gIIC = 0;
		gIID = 0;

		gCHKIT = true; // recalc PK

		if (trace)
			trace_step(1, "A<=IIC", 0);
		break;
	case 06: /* TRA PID */
		gA = gPID;
		if (trace)
			trace_step(1, "A<=PID", 0);
		break;
	case 07:
		gA = gPIE;
		if (trace)
			trace_step(1, "A<=PIE", 0);
		break;
	case 010:					  // CSR
		gA = (1 << 2) | (1 << 3); // Always report bit 2 and 3 as 1. Bit 2="MAN DIS" (Cache disabled manually as Emulator doesnt need caching. Bit 3=Cache Clear Finished
		// gA = gCSR;
		if (trace)
			trace_step(1, "A<=CSR", 0);
		break;
	case 011: /* TRA ACTL */
		gA = 1 << CurrLEVEL;
		break;
	case 012: /* TRA ALD */
		gA = gALD;
		if (trace)
			trace_step(1, "A<=ALD", 0);
		break;
	case 013: /* TRA PES */
		gA = gPES;
		break;
	case 014: /* PGC/PCR - Paging Control Register */
		temp = gA;
		level = (temp >> 3) & 0x0f;
		gA = gReg->reg_PCR[level];
		if (mmsType == MMS1)
		{
			gA &= ~(1 << 2); // Clear bit 2 for MMS1 mode
		}

		// Always clear bit 15, as thats the way of the ND110 microcode
		gA = gA & ~(1 << 15);

		break;
	case 015: /* TRA PEA */
		gA = gPEA;

		// Unlock PEA and PES
		gPEA_Lock = false;
		gPES_Lock = false;
		break;
	default: /* These registers dont exist, so just return 0 for now FIXME: Check correct behaviour.*/
			 // gA = 0;
		//  do nothing is the correct
		break;
	}
	if (trace)
		trace_post(1, "A", (int)gA);
}

/*
 * DoEXR - Run instruction in source register
 */
void DoEXR(ushort instr)
{
	ushort sr, exr_instr;
	char disasm_str[256];
	sr = (instr >> 3) & 0x07;
	if (sr)
		exr_instr = gReg->reg[CurrLEVEL][sr];
	else
		exr_instr = 0;
	if (trace & 0x01)
	{
		OpToStr(disasm_str, exr_instr);
		fprintf(tracefile,
				"#o (i,d) #v# (\"%d\",\"EXR instr: %s\");\n",
				(int)instr_counter, disasm_str);
		fprintf(tracefile,
				"#e (i,d) #v# (\"%d\",\"%s\");\n",
				(int)instr_counter, disasm_str);
	}
	if (trace & 0x20)
	{
	}
	if (0140600 == extract_opcode(exr_instr))
	{						 /* ILLEGAL:: EXR of EXR */
		setbit(_STS, _Z, 1); //: TODO: activate CPU trap on level 14!!!
		return;
	}
	if (DISASM)
		disasm_exr(gPC, exr_instr);

	// Execute opcode but do not touch Program Counter
	do_op(exr_instr, true);
}

/*
 * DoWAIT - Give up prio instruction
 * NOTE:: Only basic parts fixed yet, this is a fairly complex one
 *
 * NOTE:: Privileged instructions
 */
void DoWAIT(ushort instr)
{
	if (!CheckPriv())
		return;

	ushort temp;
	if (!STS_IONI)
	{
		// If the interrupt system is OFF
		// The ND-110 stops with the program counter (P register) pointing at the instruction after the WAIT and the front panel RUN indicator is turned off.
		// To restart the system, type ! on the console terminal
		printf("\r\nWAIT when IONI is off PIL[%d] PC[%6o] PID[0x%4X] PIE[0x%4X] IONI[%d] PONI[%d] STS_HI[%4X] STS_LO[%4X]\r\n", gPIL, gPC, gPID, gPIE, STS_IONI, STS_PONI, gReg->reg_STS, gReg->reg[gPIL][_STS]);
		CurrentCPURunMode = STOP;
		return;
	}

	if (CurrLEVEL == 0)
	{
		// Cant go lower
		return;
	}
	
	
	temp = ~(1 << CurrLEVEL); /* Now we have a 0 in the position we want */
	gPID &= temp;			  /* Give up this level */

	gCHKIT = true; // recalc PK (and do a level switch if needed)
}

/* LWCS (Privileged)
 */
void ndfunc_lwcs(ushort instr)
{
	// LWCS is a no-operation on the ND-110
	// The ND-110 is software compatible but nor microcode compatible and writing to the writable control store has no meaning in the ND-110.
	// A no-operation is executed so that programs written for the ND-100 and NORD-10 can continue

	if (!CheckPriv())
		return;

	// noop
}

/*
 * DoTRR - Transfer to register
 *  Affected: Internal register specified
 *  <IR> = A;
 *
 * NOTE: STS and PCR NOT fixed yet!!!
 *
 * NOTE: Privileged instructions
 */
void DoTRR(ushort instr)
{
	if (!CheckPriv())
		return;

	int s;
	ushort temp, level;
	if (trace)
		trace_pre(1, "A", (int)gA);
	switch (instr & 0x0F)
	{
	case 00: // TRR PANC
		gPANC = gA;
		if (trace)
			trace_step(1, "PANC<=A", 0);

		if (PANEL_PROCESSOR)
		{
			ProcessTerminalPanc();
		}
		break;
	case 01: // TRR STS
		/* ND-06.029.1 ND-110 Instruction Set, lists only lower 8 bits as changeable... */
		gReg->reg[CurrLEVEL][_STS] = (gReg->reg[CurrLEVEL][_STS] & 0xff00) | (gA & 0x00ff); /* Only change LSB  */
		if (trace)
			trace_step(1, "STS(LSB)<=A", 0);
		break;
	case 02: // TRR LMP
		gLMP = gA;

		if (PANEL_PROCESSOR)
		{
			ProcessTerminalLamp();
		}
		if (trace)
			trace_step(1, "LMP<=A", 0);
		if (debug)
			fprintf(debugfile, "TRR LMP: %06o => LMP\n", gA);
		break;
	case 03: /* PGC/PCR - Paging Control Register */
		temp = gA;
		level = (temp >> 3) & 0x0f;
		if (mmsType == MMS1)
		{
			temp &= ~(1 << 2); // Force Clear bit 2 for MMS1 mode
		}
		gReg->reg_PCR[level] = temp;

		if (trace)
			trace_step(1, "PCR(%d)<=A", level);
		break;
	case 05: // TRR IIE
		gIIE = gA;
		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "IIE<=A", 0);
		break;
	case 06: // TRR PID
		// TODO:? according to manual it can only set bit 15,13-12-11
		gPID = gA;
		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "PID<=A", 0);
		break;
	case 07: // TRR PIE
		gPIE = gA;
		gCHKIT = true; // we need to check PK after this
		if (trace)
			trace_step(1, "PIE<=A", 0);
		break;
	case 010: // TRR CCL (cache clear)
		gCCL = gA;
		if (trace)
			trace_step(1, "CCL<=A", 0);
		break;
	case 011: // TRR LCIL
		gLCIL = gA;
		if (trace)
			trace_step(1, "LCIL<=A", 0);
		break;
	case 012: // TRR UCIL
		gUCIL = gA;
		if (trace)
			trace_step(1, "UCIL<=A", 0);
		break;
	case 013: /* TRR CILP (ND110 only??) */
		break;
	case 015: /* TRR ECCR (ND110 only??) */
		gECCR = gA;
		if (debug)
		{
			char opts[256] = "";

			if ((gECCR & 1 << 0) != 0)
				strcat(opts, "[0TS | Simulate memory error in bit 0] ");
			if ((gECCR & 1 << 1) != 0)
				strcat(opts, "[15T | Simulate memory error in bit 15] ");
			if ((gECCR & 1 << 2) != 0)
				strcat(opts, "[ANY | Enable parity interrup on all errors] ");
			if ((gECCR & 1 << 3) != 0)
				strcat(opts, "[DIS | Disable ECC System and parity interrupt] ");
			if ((gECCR & 1 << 4) != 0)
				strcat(opts, "[6TS | Simualate memory error in bit 6] ");

			fprintf(debugfile, "ECCR = %06o %s\n", gECCR, opts);
		}

		break;
	case 017: /* TRR CS (ND110 only) */
		break;
	}
}

/*
 * DoSRB - Store register block.
 *  Affected:(EL),+ 1 +2 + 3 + 4 + 5 + 6 + 7
 *            P    X  T   A   D   L  STS  B
 *
 *  Uses the alternative pagetable!
 */
void DoSRB(ushort operand)
{

	if (!CheckPriv())
		return;

	ushort lvl, addr;
	ushort sts_temp;

	lvl = ((operand & 0x0078) >> 3);
	addr = gX;

	if (trace)
		trace_pre(1, "X", (int)gX);

	sts_temp = gReg->reg[lvl][_STS] & 0x00ff;

	// If the current program level is specified, the stored P register points to the instruction following SRB.
	MemoryWrite(gReg->reg[lvl][_P], addr, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=P[%01o]:(%06o)", addr, lvl, gReg->reg[lvl][_P]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_X], addr + 1, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=X[%01o]:(%06o)", addr + 1, lvl, gReg->reg[lvl][_X]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_T], addr + 2, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=T[%01o]:(%06o)", addr + 2, lvl, gReg->reg[lvl][_T]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_A], addr + 3, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=A[%01o]:(%06o)", addr + 3, lvl, gReg->reg[lvl][_A]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_D], addr + 4, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=D[%01o]:(%06o)", addr + 4, lvl, gReg->reg[lvl][_D]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_L], addr + 5, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=L[%01o]:(%06o)", addr + 5, lvl, gReg->reg[lvl][_L]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(sts_temp, addr + 6, true, 2); /* Only write LSB of STS */
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=STS[%01o]:(%06o)", addr + 6, lvl, gReg->reg[lvl][_STS]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
	MemoryWrite(gReg->reg[lvl][_B], addr + 7, true, 2);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "(%06o)<=B[%01o]:(%06o)", addr + 7, lvl, gReg->reg[lvl][_B]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
}

/*
 * DoLRB - Load register block.
 *           (EL),+ 1 +2 + 3 + 4 + 5 + 6 + 7
 *  Affected:  P    X  T   A   D   L  STS  B
 *
 *  Uses the alternative pagetable!
 */

/// <summary>
/// LRB - Load register Block
/// Code: 152 6n2
/// Format: SRB <level* 10>
///
/// The instruction <LRB level * 10B> loads the contents  of the register block on program level specified in the
/// level field of the instruction.
///
/// The specified register block is  loaded by the contents of succeeding memory locations starting at the location
/// specified by the contents of the X register.
///
/// If the current program level is specified, the P register is not affected.
///
/// The LBR instruction is privileged
/// </summary>
void DoLRB(ushort operand)
{

	if (!CheckPriv())
		return;

	ushort lvl, addr;

	lvl = ((operand & 0x0078) >> 3);
	addr = gX;

	if (trace)
		trace_pre(1, "X", (int)gX);

	if (lvl != CurrLEVEL)
	{ /* Dont change P on current level if this happens to be specified */
		gReg->reg[lvl][_P] = MemoryRead(addr, true);
		if (trace)
		{
			(void)snprintf(trace_temp_str, 255, "P[%01o]<=(%06o):%06o", lvl, addr, MemoryRead(addr, true));
			trace_step(1, (char *)trace_temp_str, 0);
		}
	}
	gReg->reg[lvl][_X] = MemoryRead(addr + 1, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "X[%01o]<=(%06o):%06o", lvl, addr + 1, MemoryRead(addr + 1, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_T] = MemoryRead(addr + 2, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "T[%01o]<=(%06o):%06o", lvl, addr + 2, MemoryRead(addr + 2, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_A] = MemoryRead(addr + 3, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "A[%01o]<=(%06o):%06o", lvl, addr + 3, MemoryRead(addr + 3, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_D] = MemoryRead(addr + 4, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "D[%01o]<=(%06o):%06o", lvl, addr + 4, MemoryRead(addr + 3, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_L] = MemoryRead(addr + 5, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "L[%01o]<=(%06o):%06o", lvl, addr + 5, MemoryRead(addr + 3, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_STS] =
		(gReg->reg[lvl][_STS] & 0xff00) | (MemoryRead(addr + 6, true) & 0x00ff); /* Only load LSB STS */
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "STS[%01o]<=(%06o):%06o", lvl, addr + 6, MemoryRead(addr + 3, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
	gReg->reg[lvl][_B] = MemoryRead(addr + 7, true);
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "B[%01o]<=(%06o):%06o", lvl, addr + 7, MemoryRead(addr + 3, true));
		trace_step(1, (char *)trace_temp_str, 0);
	}
}

bool IsSkip(ushort instr)
{
	ushort sr, dr, source, desti;
	signed short ss, sd, sgr, ovf;
	char z, o, c, s;
	sr = (instr >> 3) & 0x07;
	dr = (instr >> 0) & 0x07;
	source = (0 == sr) ? 0 : gReg->reg[CurrLEVEL][sr]; /* Never use STS reg but zero value instead */
	desti = (0 == dr) ? 0 : gReg->reg[CurrLEVEL][dr];  /* Never use STS reg but zero value instead */
	ss = (signed short)source;
	sd = (signed short)desti;

	if (trace)
	{
		if (sr)
			(void)snprintf(trace_temp_str, 255, "S%s:%06o", regn[sr], source);
		else
			(void)snprintf(trace_temp_str, 255, "0");
		trace_step(1, (char *)trace_temp_str, 0);
		if (dr)
			(void)snprintf(trace_temp_str, 255, "D%s:%06o", regn[dr], desti);
		else
			(void)snprintf(trace_temp_str, 255, "0");
		trace_step(1, (char *)trace_temp_str, 0);
	}

	/* Ok, lets set flags */
	z = (0 == (desti - source)) ? 1 : 0;
	sgr = sd - ss;
	ovf = (sd & ~ss & ~sgr) | (~sd & ss & sgr);
	o = (ovf < 0) ? 1 : 0;
	c = ((desti - source) < 0) ? 0 : 1;
	s = ((ushort)(sd - ss) >> 15) & 0x01;

	/* And use these to do the skipping, so we try and follow ND behaviour */
	switch ((instr >> 8) & 0x07)
	{
	case 0: /* EQL */
		if (z)
			return true;
		break;
	case 1: /* GEQ */
		if (!s)
			return true;
		break;
	case 2: /* GRE */
		if (!(s ^ o))
			return true;
		break;
	case 3: /* MGRE */
		if (c)
			return true;
		break;
	case 4: /* UEQ */
		if (!z)
			return true;
		break;
	case 5: /* LSS */
		if (s)
			return true;
		break;
	case 6: /* LST */
		if (s ^ o)
			return true;
		break;
	case 7: /* MLST */
		if (!c)
			return true;
		break;
	}
	return false;
}

void do_bops(ushort operand)
{
	ushort bn, dr, desti;
	bn = ((operand & 0x0078) >> 3);
	dr = (operand & 0x0007);
	if (trace)
		trace_pre(1, regn[dr], gReg->reg[CurrLEVEL][dr]);
	switch ((operand & 0x0780) >> 7)
	{
	case 0: /* BSET ZRO */
		setbit(dr, bn, 0);
		break;
	case 1: /* BSET ONE */
		setbit(dr, bn, 1);
		break;
	case 2: /* BSET BCM */
		desti = getbit(dr, bn);
		desti ^= 1; /* XOR with one to invert bit */
		setbit(dr, bn, desti);
		break;
	case 3: /* BSET BAC */
		setbit(dr, bn, getbit(_STS, _K));
		break;
	case 4: /* BSKP ZRO */
		if (!getbit(dr, bn))
			gPC++; /* Skip next instruction if zero */
		break;
	case 5: /* BSKP ONE */
		if (getbit(dr, bn))
			gPC++; /* Skip next instruction if one */
		break;
	case 6: /* BSKP BCM */
		if ((getbit(dr, bn) ^ 1) == getbit(_STS, _K))
			gPC++; /* Skip next instruction if bit complement */
		break;
	case 7: /* BSKP BAC */
		if (getbit(dr, bn) == getbit(_STS, _K))
			gPC++; /* Skip next instruction if equal */
		break;
	case 8: /* BSTC */
		setbit(dr, bn, (getbit(_STS, _K) ^ 1));
		setbit(_STS, _K, 1);
		break;
	case 9: /* BSTA */
		setbit(dr, bn, getbit(_STS, _K));
		setbit(_STS, _K, 0);
		break;
	case 10: /* BLDC */
		setbit(_STS, _K, getbit(dr, bn) ^ 1);
		break;
	case 11: /* BLDA */
		setbit(_STS, _K, getbit(dr, bn));
		break;
	case 12: /* BANC */
		setbit(_STS, _K, ((getbit(dr, bn) ^ 1) & getbit(_STS, _K)));
		break;
	case 13: /* BAND */
		setbit(_STS, _K, (getbit(dr, bn) & getbit(_STS, _K)));
		break;
	case 14: /* BORC */
		setbit(_STS, _K, ((getbit(dr, bn) ^ 1) | getbit(_STS, _K)));
		break;
	case 15: /* BORA */
		setbit(_STS, _K, (getbit(dr, bn) | getbit(_STS, _K)));
		break;
	}
	if (trace)
	{
		(void)snprintf(trace_temp_str, 255, "%s=%06o", regn[dr], gReg->reg[CurrLEVEL][dr]);
		trace_step(1, (char *)trace_temp_str, 0);
	}
}

ushort ShiftReg(ushort reg, ushort instr)
{
	bool isneg = ((instr & 0x0020) >> 5) ? 1 : 0;
	ushort offset = (isneg) ? (~((instr & 0x003F) | 0xFFC0) + 1) : (instr & 0x003F);
	ushort shifttype = ((instr >> 9) & 0x03);
	int i, tmp, msb;
	int m = getbit(_STS, _M);
	tmp = m; /* just in case.. */
	for (i = 1; i <= offset; i++)
	{
		tmp = (isneg) ? (reg & 0x01) : ((reg >> 15) & 0x01); /* tmp = bit shifted out */
		msb = reg >> 15 & 1;								 /* msb before shift */
		reg = (isneg) ? reg >> 1 : reg << 1;
		switch (shifttype)
		{
		case 0:																 /* Plain */
			reg = (isneg) ? ((reg & 0x7fff) | (msb << 15)) : (reg & 0xfffe); /* SHR : SHL */
			break;
		case 1: /* ROT */
			reg = (isneg) ? ((reg & 0x7fff) | (tmp << 15)) : ((reg & 0xfffe) | tmp);
			break;
		case 2: /* ZIN */
			reg = (isneg) ? (reg & 0x7fff) : (reg & 0xfffe);
			break;
		case 3: /* LIN */
			reg = (isneg) ? ((reg & 0x7fff) | (m << 15)) : ((reg & 0xfffe) | m);
			break;
		}
	}
	setbit(_STS, _M, tmp);
	return reg;
}

ulong ShiftDoubleReg(ulong reg, ushort instr)
{
	bool isneg = ((instr & 0x0020) >> 5) ? 1 : 0;
	ushort offset = (isneg) ? (~((instr & 0x003F) | 0xFFC0) + 1) : (instr & 0x003F);
	ushort shifttype = ((instr >> 9) & 0x03);
	int i, tmp, msb;
	int m = getbit(_STS, _M);
	tmp = m; /* just in case.. */
	for (i = 1; i <= offset; i++)
	{
		tmp = (isneg) ? (reg & 0x01) : ((reg >> 31) & 0x01); /* tmp = bit shifted out */
		msb = reg >> 31 & 1;								 /* msb before shift */
		reg = (isneg) ? reg >> 1 : reg << 1;
		switch (shifttype)
		{
		case 0:																		 /* Plain */
			reg = (isneg) ? ((reg & 0x7fffffff) | (msb << 31)) : (reg & 0xfffffffe); /* SHR : SHL */
			break;
		case 1: /* ROT */
			reg = (isneg) ? ((reg & 0x7fffffff) | (tmp << 31)) : ((reg & 0xfffffffe) | tmp);
			break;
		case 2: /* ZIN */
			reg = (isneg) ? (reg & 0x7fffffff) : (reg & 0xfffffffe);
			break;
		case 3: /* LIN */
			reg = (isneg) ? ((reg & 0x7fffffff) | (m << 31)) : ((reg & 0xfffffffe) | m);
			break;
		}
	}
	setbit(_STS, _M, tmp);
	return reg;
}

/*
 * DoIDENT
 * Handles IDENT PLxx instructions
 */
void DoIDENT(ushort priolevel)
{

	int id = IO_Ident(priolevel);
	if (id >= 0)
	{
		gA = id & 0xFFFF;
		if (priolevel != 13) // Dont trace RTC, its just too much
		{
			if (trace)
				trace_step(1, "A<=%06o", id);
		}
	}
	else
	{
		gA = 0;

		if (debug)
			fprintf(debugfile, "DoIDENT IOX Error lvl=%d\n", priolevel);
		if (priolevel != 13)	   // ignore RTC
			interrupt(14, 1 << 7); /* IOX Error if no IDENT code found */

		if (trace)
			trace_pre(2, "PID", gPID, "PIE", gPIE);
	}
	return;
}

/// <summary>
/// RDUS - Read don't use cache Code: 140127
/// Code: 140 127
/// Format: RDUS
///
///  This instruction reads the content of the memory location pointed to by the T—register into the A—register.
///  The address in the T-register is a logical memory address.Translation to a physical memory address is normally done by using the page tables.
///  However, the translation will use the alternative page table when PTM is on (Page Table Modus) (status register bit 0 is 1) and the paging system is on, PON.
/// </summary>

void DoRDUS(ushort instr)
{
	gA = MemoryRead(gT, true);
}

/// <summary>
/// TSET - Test and set
/// Code: 140 123
/// Format: TSET
///
/// This instruction writes -1 into the memory address pointed to by the T—register.
/// Simultaneously, the old content of the same address is read into the A-register.This read/write sequence is performed with the memory system ’locked',
/// so that the two memory accesses cannot be split by other accesses on other memory channels.
/// This may be used to implement processor synchronizing.
/// The address in the T—register is a logical memory address.
/// Translation to a physical memory address is normally done by using the page tables.
/// However, the translation will use the alternative page table when PTM is on (Page Table Modus) (status register bit 0 is 1) and the paging system is on, PON.
///
/// The old content of the memory address is always read from the memory, and never from the cache, Data is written both to memory and cache.
/// </summary>
void DoTSET(ushort instr)
{
	// regs.currentRegisters.A = (ushort)cpu.ReadVirtualMemory(regs.currentRegisters.T, PageTable.AlternativePageTable);
	// cpu.WriteVirtualMemory(regs.currentRegisters.T, 0xFFFF, PageTable.AlternativePageTable); // Write -1

	gA = MemoryRead(gT, true);
	MemoryWrite(0xFFFF, gT, true, 2);
}

/// <summary>
/// MOVEW - WORD BLOCK INSTRUCTION
///
/// Code 143 1nn
/// If the memory management system is off, bank 0 of physical memory is addressed. (Bit PTM of the STS register is zero) and the following transfer fields become equivalent:
///   nn = 00 = 01 = 03 = 04
///   nn = 02 = 05
///   nn = 06 = 07
///
/// MOVEW can be interrupted. L, A, D, X, T and P registers are then changed to restart execution.
///
/// A and D - Source address
/// X and T - Destination address
/// L		- The number of words to be moved (max 2048)
///
/// A and/or X are used for physical memory-block moves and are incremented when the D and/or T registers overflow.
///
/// If the L register contains a value grater then 2048 (L=o4000) no words are moved and A,D, T and X are unchanged.
///
/// After transfer the register contains: A,D, T,X - The addresses after the last moved word . L = zero
///
/// Format: MOVEW
/// </summary>
void DoMOVEW(ushort instr)
{
	unsigned int sourceAddress = gD;
	unsigned int destinationAddress = gT;
	ushort cnt = gL;

	ushort displacement = (instr & 0x00F);

	// Check if source and destination are in physical memory
	bool isSourcePhysical = false;
	bool isDestinationPhysical = false;

	switch (displacement)
	{
	case 2:
	case 5:
		destinationAddress |= (gX << 16);
		isDestinationPhysical = true;
		break;

	case 6:
	case 7:
		sourceAddress |= (gA << 16);
		isSourcePhysical = true;
		break;
	case 8:
		destinationAddress |= (gX << 16);
		isDestinationPhysical = true;

		sourceAddress |= (gA << 16);
		isSourcePhysical = true;
		break;
	}

	// Check for priveleged instruction
	if (isSourcePhysical || isDestinationPhysical)
	{
		if (!CheckPriv())
			return;
	}

	// Warning: In the loop of read/write below, PageFault can occur, and the instruction can be restarted.
	ushort temp = 0;

	while (cnt > 0)
	{
		switch (displacement)
		{
		case 0: // move from PT to PT
			temp = MemoryRead(sourceAddress, false);
			MemoryWrite(temp, destinationAddress, false, 2);
			break;
		case 1: // move from PT to APT
			temp = MemoryRead(sourceAddress, false);
			MemoryWrite(temp, destinationAddress, true, 2);
			break;
		case 2: // move from PT to physical memory
			temp = MemoryRead(sourceAddress, false);
			WritePhysicalMemory(destinationAddress, temp, true);
			break;
		case 3: // move from APT to PT
			temp = (ushort)MemoryRead(sourceAddress, true);
			MemoryWrite(temp, destinationAddress, false, 2);
			break;
		case 4: // move from APT to APT
			temp = (ushort)MemoryRead(sourceAddress, true);
			MemoryWrite(temp, destinationAddress, true, 2);
			break;
		case 5: // move from APT to physical memory
			temp = (ushort)MemoryRead(sourceAddress, true);
			WritePhysicalMemory(destinationAddress, temp, true);
			break;
		case 6: // move from physical memory to PT
			temp = ReadPhysicalMemory(sourceAddress, true);
			MemoryWrite(temp, destinationAddress, false, 2);
			break;
		case 7: // move from physical memory to APT
			temp = ReadPhysicalMemory(sourceAddress, true);
			MemoryWrite(temp, destinationAddress, true, 2);
			break;

		case 8: // move from physical memory to physical memory
			temp = ReadPhysicalMemory(sourceAddress, true);
			WritePhysicalMemory(destinationAddress, temp, true);
			break;

		default:
			break;
		}
		sourceAddress++;
		destinationAddress++;
		cnt--;
	}

	// After here, no PageFault can occur - update register values

	// update L
	gL = cnt;

	// Update Source with the new address
	gD = (sourceAddress & 0xFFFF);
	if (isSourcePhysical)
	{
		gA = (sourceAddress >> 16) & 0xFFFF;
	}

	// Update destination
	gT = (destinationAddress & 0xFFFF);
	if (isDestinationPhysical)
	{
		gX = (destinationAddress >> 16) & 0xFFFF;
	}
}

#define _removed_MOVB_AND_MOVBF_ 1
#if _removed_MOVB_AND_MOVBF_ // replaced with doMoveBytes
/*
 * MOVB instruction. TODO:: Fix edge case and document params here...
 */
void DoMOVB(ushort instr)
{
	ushort source, dest, lens, lend, len, s_lr, d_lr, s_apt, d_apt;
	int dir; /* direction, 0=low to high, 1 = high to low */
	int i;
	ushort thebyte;
	ushort addr_d, addr_s;

	addr_d = 0;
	addr_s = 0;
	dir = 0;
	source = gA;
	dest = gX;
	lens = gD & 0x0fff;
	lend = gT & 0x0fff;
	s_lr = ((gD >> 15) & 1);
	d_lr = ((gT >> 15) & 1);
	s_apt = ((gD >> 14) & 1);
	d_apt = ((gT >> 14) & 1);
	len = (((int)lens - lend) < 0) ? lens : lend; /* get smallest length as number to copy */
	/* Check overlap if any and direction to copy */
	if (((int)source - dest) < 0)
	{
		dir = 1;
	}
	else if (((int)source - dest) == 0)
	{ /* :TODO: check bytes to determine direction, or if no need to copy exist */
	}
	else
	{
		dir = 0;
	}
	//	if (debug) fprintf(debugfile,"MOVB(ante): gA:%06o gD:%06o gX:%06o gT:%06o len:%d\n",gA,gD,gX,gT,len);
	/* COPY */
	if (dir)
	{ /* high to low */
		for (i = len - 1; i >= 0; i--)
		{
			addr_s = source + ((i + s_lr) >> 1); /* Word adress of byte to read */
			thebyte = MemoryRead(addr_s, s_apt);
			thebyte = ((i + d_lr) & 1) ? thebyte : (thebyte >> 8) & 0xff; /* right, LSB : left, MSB */
			addr_d = dest + ((i + d_lr) >> 1);							  /* Word adress of byte to write */
			MemoryWrite(thebyte, addr_d, d_apt, ((i + d_lr) & 1));
		}
		i = 0;
	}
	else
	{ /* low to high */
		for (i = 0; i < len; i++)
		{
			addr_s = source + ((i + s_lr) >> 1); /* Word adress of byte to read */
			thebyte = MemoryRead(addr_s, s_apt);
			thebyte = ((i + d_lr) & 1) ? thebyte : (thebyte >> 8) & 0xff; /* right, LSB : left, MSB */
			addr_d = dest + ((i + d_lr) >> 1);							  /* Word adress of byte to write */
			MemoryWrite(thebyte, addr_d, d_apt, ((i + d_lr) & 1));
		}
	}

	gD &= 0x7000;				  /* Null number of bytes, as per manual, also null bit 15 */
	gT &= 0x7000;				  /* Null number of bytes, also null bit 15 */
	gD |= ((i + d_lr) & 1) << 15; /* set bit 15 to point to next free byte */
	gT |= ((i + d_lr) & 1) << 15; /* set bit 15 to point to next free byte */
	gT |= len & 0x0fff;			  /* number of bytes done to lowest 12 bits*/

	gA = addr_s + ((len + s_lr) >> 1);
	gX = addr_d + ((len + d_lr) >> 1);
	//	if (debug) fprintf(debugfile,"MOVB(post): gA:%06o gD:%06o gX:%06o gT:%06o len:%d\n",gA,gD,gX,gT,len);

	gPC++; /* This function has a SKIP return on no error, which is always? */
}

/*
 * MOVBF instruction. TODO:: ALL
 */
void DoMOVBF(ushort instr)
{
	ushort source, dest, lens, lend, len, s_lr, d_lr, s_apt, d_apt;
	int i;
	ushort thebyte;
	ushort addr_d, addr_s;
	source = gA;
	dest = gX;
	bool overlap;

	addr_d = 0;
	addr_s = 0;
	lens = gD & 0x0fff;
	lend = gT & 0x0fff;
	s_lr = ((gD >> 15) & 1);
	d_lr = ((gT >> 15) & 1);
	s_apt = ((gD >> 14) & 1);
	d_apt = ((gT >> 14) & 1);

	len = (((int)lens - lend) < 0) ? lens : lend; /* get smallest length as number to copy */

	if (source > dest)
		overlap = false;
	else if ((ushort)((ushort)(ceil(len / 2)) + source - 1) > dest)
		overlap = true;
	else
		overlap = false;

	if (debug)
		fprintf(debugfile, "MOVBF(pre): gA:%06o gD:%06o gX:%06o gT:%06o gPC:%06o len:%d\n", gA, gD, gX, gT, gPC, len);
	if (debug)
		fprintf(debugfile, "MOVBF(dec): gA:%06d gD:%06d gX:%06d gT:%06d gPC:%06d len:%06d\n", gA, gD, gX, gT, gPC, len);
	if (debug)
		fprintf(debugfile, "MOVBF(pre): overlap=%d\n", overlap);

	for (i = 0; i < len; i++)
	{
		addr_s = source + ((i + s_lr) >> 1); /* Word adress of byte to read */
		thebyte = MemoryRead(addr_s, s_apt);
		thebyte = ((i + d_lr) & 1) ? thebyte : (thebyte >> 8) & 0xff; /* right, LSB : left, MSB */
		addr_d = dest + ((i + d_lr) >> 1);							  /* Word adress of byte to write */
		MemoryWrite(thebyte, addr_d, d_apt, ((i + d_lr) & 1));
		lens--;
		lend--;
	}

	gA = source + ((len + s_lr) >> 1);
	gX = dest + ((len + d_lr) >> 1);

	gD &= 0xefff;				  /* Null bit 12 */
	gT &= 0xcfff;				  /* Null bit 12 & 13 */
	gD |= ((i + d_lr) & 1) << 15; /* set bit 15 to point to next free byte */
	gT |= ((i + d_lr) & 1) << 15; /* set bit 15 to point to next free byte */

	gD &= 0xf000;		 /* clean lowest bits before or */
	gT &= 0xf000;		 /* clean lowest bits before or */
	gD |= lens & 0x0fff; /* decremented byte counter to lowest 12 bits*/
	gT |= lend & 0x0fff; /* decremented byte counter to lowest 12 bits*/

	if (!overlap)
		gPC++; /* This function has a SKIP return on no error */

	if (debug)
		fprintf(debugfile, "MOVBF(post): gA:%06o gD:%06o gX:%06o gT:%06o gPC:%06o len:%d\n", gA, gD, gX, gT, gPC, len);
	if (debug)
		fprintf(debugfile, "MOVBF(post): addr_s=%d addr_d=%d s_lr=%d d_lr=%d len:%d\n", addr_s, addr_d, s_lr, d_lr, len);
	if (debug)
		fprintf(debugfile, "*********************************************************************\n");
	return;
}
#endif
void add_A_mem(ushort eff_addr, bool UseAPT)
{
	int temp, data, oldreg;
	oldreg = gA;
	data = MemoryRead(eff_addr, UseAPT);
	temp = gA + data;

	// FIXME - ADD FLAG HANDLING CORRECTLY FOR C,O,Q FLAGS (CHECK AGAIN THINK WE MIGHT HAVE SUBTLE BUGS)

	if ((temp > 0xFFFF) || (temp < 0))
	{
		setbit(_STS, _C, 1);
		if ((oldreg & 0x8000) && (data & 0x8000) && !(temp & 0x8000))
		{
			setbit(_STS, _Q, 1);
		}
		else
		{
			setbit(_STS, _Q, 0);
		}
	}
	else
	{
		setbit(_STS, _C, 0);
		if (!(oldreg & 0x8000) && !(data & 0x8000) && (temp & 0x8000))
		{
			setbit(_STS, _Q, 1);
		}
		else
		{
			setbit(_STS, _Q, 0);
		}
	}

	gA = (temp & 0xFFFF);
}

/*
 * Move bytes in memory
 * Note: This is part of the commercial instruction set
 * It seems SINTRAN doesnt use this function for booting and operating
 * checkOverlapping: MOVBF sets this to true, MOVB sets this to false
 */
void doMoveBytes(bool checkOverlapping)
{
	const int LEN_MASK = 0xFFF;
	int readValue;

	int numBytesSource = gD & LEN_MASK; // Source length
	int numBytesDest = gT & LEN_MASK;	// Destination length
	if (numBytesDest < numBytesSource)
		numBytesSource = numBytesDest; // Cap number of bytes to max length of Destination

	// If Bit 13 is set, then setup has been executed and we are returning from an interrupt
	if (!(gD & (1 << 13)))
	{
		gT = (gT & 0xC000) | numBytesSource;
		gD = (gT & 0xC000);

		// Mark D bit 13 with setup done
		gD |= (1 << 13);
	}

	if (checkOverlapping)
	{
		// Convert byte count to word count for addressing
		int numWordsD = numBytesSource >> 1; // Same as numBytesD / 2

		// Calculate start and end positions for source and destination in terms of words
		int sourceStart = gA;
		int destinationStart = gX;
		int sourceEnd = sourceStart + numWordsD;
		int destinationEnd = destinationStart + numWordsD;

		// Check for forbidden overlap
		// Overlap is forbidden if destination overlaps source before it is read
		if (destinationStart < sourceEnd && destinationEnd > sourceStart)
		{
			// OVERLAP EXISTS - ILLEGAL IF 'MOVBF'!!
			// Forbidden overlap exists, return with error (no skip)
			return;
		}
	}

	bool useAPT = true; // Use alternative page table
	WriteMode readMode;
	WriteMode writeMode;

	if (gX < gA)
	{
		// High to low
		for (int i = (gT & LEN_MASK); i > 0; i--)
		{
			// Bit 15: 0=>MSB, 1=> LSB
			readMode = (gD & (1 << 15)) ? WRITEMODE_LSB : WRITEMODE_MSB;
			readValue = MemoryRead(gA, useAPT);

			if (readMode == WRITEMODE_MSB)
			{
				readValue = (readValue >> 8) & 0xFF;
			}
			else
			{
				readValue = readValue & 0xFF;
			}

			WriteMode writeMode = (gT & (1 << 15)) ? WRITEMODE_LSB : WRITEMODE_MSB;
			MemoryWrite(readValue, gX, useAPT, writeMode);

			gD ^= (1 << 15); // Flip D bit 15
			if (!(gD & (1 << 15)))
				gA--;

			gT ^= (1 << 15); // Flip T bit 15
			if (!(gT & (1 << 15)))
				gX--;
		}
	}
	else
	{
		// Low to High
		for (int i = (gD & LEN_MASK); i < (gT & LEN_MASK); i++)
		{
			// Bit 15: 0=>MSB, 1=> LSB
			WriteMode readMode = (gD & (1 << 15)) ? WRITEMODE_LSB : WRITEMODE_MSB;
			readValue = MemoryRead(gA, useAPT);

			if (readMode == WRITEMODE_MSB)
			{
				readValue = (readValue >> 8) & 0xFF;
			}
			else
			{
				readValue = readValue & 0xFF;
			}

			WriteMode writeMode = (gT & (1 << 15)) ? WRITEMODE_LSB : WRITEMODE_MSB;
			MemoryWrite(readValue, gX, useAPT, writeMode);

			gD ^= (1 << 15); // Flip D bit 15
			if (!(gD & (1 << 15)))
				gA++;

			gT ^= (1 << 15); // Flip T bit 15
			if (!(gT & (1 << 15)))
				gX++;
		}
	}

	// After execution, bit 15 of the D and T registers point to the end of the field that has been moved.
	// Note: DON'T CLEAR bit 15 of D and T, but clear bits 13 and 12.

	// After execution the field length of the D (source) equals Zero
	// Note: Clear setup and count bits
	gD &= 0xC000;

	// Documentation for MOVB and MOVBF says the same but implementation differs
	if (checkOverlapping)
		gT &= 0xC000; // MOVBF
	else
		gT &= 0xCFFF; // MOVB

	gPC++; // SKIP return
}

/*
 * MOVB
 */
void ndfunc_movb(ushort instr)
{
	doMoveBytes(false);
}

/*
 * MOVBF instruction.
 */
void ndfunc_movbf(ushort instr)
{
	doMoveBytes(true);
}

void sub_A_mem(ushort eff_addr, bool UseAPT)
{
	int temp, data, oldreg;
	oldreg = gA;
	data = MemoryRead(eff_addr, UseAPT);
	temp = gA - data;
	/*
	 * FIXME - ADD FLAG HANDLING CORRECTLY FOR C,O,Q FLAGS (CHECK AGAIN THINK WE MIGHT HAVE SUBTLE BUGS)
	 */
	if ((temp > 0xFFFF) || (temp < 0))
	{
		setbit(_STS, _C, 0);
		if ((oldreg & 0x8000) && (data & 0x8000) && !(temp & 0x8000))
		{
			setbit(_STS, _Q, 1);
		}
		else
		{
			setbit(_STS, _Q, 0);
		}
	}
	else
	{
		setbit(_STS, _C, 1);
		if (!(oldreg & 0x8000) && !(data & 0x8000) && (temp & 0x8000))
		{
			setbit(_STS, _Q, 1);
		}
		else
		{
			setbit(_STS, _Q, 0);
		}
	}

	gA = (temp & 0xFFFF);
}

/*
 * RDIV
 */
void rdiv_org(ushort instr)
{
	sshort divider;
	int dividend;
	div_t result3; /* stdlib.h */
	/* :TODO: Apparently Carry can be set too. CHECK that... Might be RAD=1??? */
	/* Overflow and division with zero also need to be fixed!! */
	/* :NOTE: The way it is described in the manual, we assume this is a fraction (numerator/denominator and return a quotient and remainder as per manual */
	divider = ((instr & 0x0038) >> 3) ? (sshort)gReg->reg[gPIL][((instr & 0x0038) >> 3)] : 0;

	if (divider == 0)
	{
		// Division by zero
		setbit(_STS, _Z, 1);
		return;
	}

	dividend = ((int)gA << 16) | gD;
	result3 = div(dividend, divider);
	gA = result3.quot;
	gD = result3.rem;
}

/// <summary>
/// RDIV - Integer inter-register divide
/// AD/<sr> —> A<- (Quotient) and D<- (Remainder)
///
/// Format: RDIV<sr>
///
/// Code: 141 600
///
/// The 32 bit signed integer contained in the double accumulator AD is divided by the contents of the register in the<sr> fieid, with the quotient in the A register
/// and the remainder in the D register, i.e., AD/sr = A< (quotient) and D<(remainder).
/// The sign of the remainder is always equal to the sign of the dividend (AD). The destination field of the instruction is not used.
///
/// If the division causes overflow, the error indicator Z is set to one.
/// The numbers are considered as fixed point integers with the fixed point after the rightmost position.
///
/// Divide double accumulator with source register.Quotient in A, remainder in D (AD= A*(sr)+ D)
/// A:= AD/(sr)

/// Affected: (A), (D), Z,C, O, Q
/// </summary>
void rdiv(ushort instr)
{
	int dividend = ((int)gA << 16) | gD;
	short divisor = ((instr & 0x0038) >> 3) ? (short)gReg->reg[gPIL][((instr & 0x0038) >> 3)] : 0;

	if (divisor == 0)
	{
		// Division by zero - set error
		setbit(_STS, _Z, 1);
		// TODO: check for Z error
		return;
	}

	int quotient = dividend / divisor;

	int reminder = dividend - (quotient * divisor);

	// Check for carry (ie, value is bigger than 16 bits)
	setbit(_STS, _C, ((quotient & 0xFFFF0000) != 0));

	if (abs(quotient) >= 32768)
	{
		setbit(_STS, _Z, 1);
		return;
	}
	gA = quotient;
	gD = reminder;
	;
}

/*
 * RMPY
 */
void rmpy_org(ushort instr)
{
	/* :TODO: Apparently Carry can be set too. CHECK that... Might be RAD=1??? */
	int a, b, result;
	a = ((instr & 0x0038) >> 3) ? (int)gReg->reg[gPIL][((instr & 0x0038) >> 3)] : 0;
	b = (instr & 0x0007) ? (int)gReg->reg[gPIL][(instr & 0x0007)] : 0;
	result = a * b;
	if (abs(result) > INT_MAX)
	{ /* Set O and Q */
		setbit(_STS, _Q, 1);
		setbit(_STS, _O, 1);
	}
	else
	{
		; //: TODO: Carry???;
		setbit(_STS, _Q, 0);
		setbit(_STS, _O, 0);
	}
	gA = (sshort)((result & 0xffff0000) >> 16);
	gD = (sshort)(result & 0x0000ffff);
}

/// <summary>
/// RMPY - Integer inter-register multiply
/// AD <- dr * sr
///
/// Format: RMPY<sr><dr>
///
/// Code: 141 200
///
/// The <sr> and <dr> fields are used to specify the two operands to be mutiplied (represented as two's complement integers), the codes are the same as for ROP.
/// The result is a 32 bit signed integer which will be placed in the A and D registers with the 16 most significant bits in the A register and the 16 least significant bits in the D register.
///
/// Multiply source with destination.Result in double accumulator
/// AD: = (sr)*(dr)
///
/// Affected: (A),(D), C,O,Q
/// </summary>
void rmpy(ushort instr)
{
	int minusCnt = 0;
	short source_value = (short)((instr & 0x0038) >> 3) ? (short)gReg->reg[gPIL][((instr & 0x0038) >> 3)] : 0;
	short dest_value = (short)(instr & 0x0007) ? (short)gReg->reg[gPIL][(instr & 0x0007)] : 0;

	// Below logic multiply-logic is 100% correct and verified against ND-100 microcode
	if ((source_value & (1 << 15)) != 0)
	{
		source_value *= -1;
		minusCnt++;
	}

	if ((dest_value & (1 << 15)) != 0)
	{
		dest_value *= -1;
		minusCnt++;
	}

	int result = source_value * dest_value;


	if (abs(result) > INT_MAX)
	{
		// Set O and Q
		setbit(_STS, _Q, 1);
		setbit(_STS, _O, 1);
	}
	else
	{
		setbit(_STS, _Q, 0);
		//setbit(_STS, _O, 0); NO!
	}

	// Check for carry (ie, value is bigger than 16 bits)
	setbit(_STS, _C, ((result & 0xFFFF0000) != 0));

	if (minusCnt == 1)
	{
		gA = (ushort)(((short)((result >> 16) & 0xFFFF) * -1) & 0x3FF);
		gD = (ushort)((short)(result & 0xFFFF) * -1);
	}
	else
	{
		// set A and D registers
		gA = (ushort)((result >> 16) & 0xFFFF);
		gD = (ushort)(result & 0xFFFF);
	}
}

/*
 * MPY
 */
void mpy(ushort operand)
{
	int a, b, result;
	a = (sshort)gA;

	gEA = New_GetEffectiveAddr(operand, &gUseAPT);
	ushort mem = MemoryRead(gEA, gUseAPT);
	b = (sshort)mem;

	setbit(_STS, _Q, 0);

	result = a * b;
	if (debug)
		fprintf(debugfile, "MPY: %d = %d * %d\n", result, a, b);


	if (abs(result) > 32767)
	{ /* Set O and Q */
		setbit(_STS, _Q, 1);
		setbit(_STS, _O, 1);
	}
	gA = (sshort)result;
}

/************************ BCD instructions *************************/

/* BCD registers and helper functions */
ushort D1 = 0;
ushort D2 = 0;

void GetBCD(ushort address)
{
	D1 = MemoryRead(address, true);
	D2 = MemoryRead((address + 1) & 0xFFFF, true);
}

void StoreBCD(ushort address)
{
	MemoryWrite(address, D1, true, WRITEMODE_WORD);
	MemoryWrite((address + 1) & 0xFFFF, D2, true, WRITEMODE_WORD);
}

/* ADDD  */
void ndfunc_addd(ushort instr)
{
}

/* SUBD  */
void ndfunc_subd(ushort instr)
{
}

/* COMD  */
void ndfunc_comd(ushort instr)
{
}

/* PACK  */
void ndfunc_pack(ushort instr)
{
}

/* UPACK */
void ndfunc_unpack(ushort instr)
{
}

/* SHDE  */
void ndfunc_shde(ushort instr)
{
}

/************************ BCD instructions *************************/

void setreg(int r, int val)
{
	if (r == _STS)
	{
		gReg->reg[CurrLEVEL][r] = (ushort)(val & 0x00FF); // Only lower 8 bits
	}
	else
	{
		gReg->reg[CurrLEVEL][r] = (ushort)(val & 0xFFFF);
	}
}

ushort getbit(ushort regnum, ushort stsbit)
{
	ushort result, tmp;
	if (regnum == _STS)
	{
		// Undoocumented, but all 16 STS bits are read
		tmp = gSTSr;
	}
	else
	{
		tmp = gReg->reg[CurrLEVEL][regnum];
	}
	result = (tmp >> stsbit) & 1;
	return result;
}

void clrbit(ushort regnum, ushort stsbit)
{
	ushort thebit;
	thebit = (1 << stsbit) ^ 0xFFFF;
	gReg->reg[CurrLEVEL][regnum] = (thebit & gReg->reg[CurrLEVEL][regnum]);
}

/*
 * setbit_STS_MSB:
 * This function handles all setting of MSB STS bits
 * NOTE:: PIL handling is done by setPIL function!!
 */
void setbit_STS_MSB(ushort stsbit, char val)
{
	int i;
	ushort thebit = 0;

	if (val)
	{
		thebit = (1 << stsbit);
		gReg->reg_STS = gReg->reg_STS | thebit;
	}
	else
	{
		thebit = (1 << stsbit) ^ 0xFFFF;
		gReg->reg_STS = gReg->reg_STS & thebit;
	}
}

bool setPIL(char newLevel)
{
	if (newLevel >= 16)
		return false;
	if (newLevel == gPIL)
		return true; // already set

	gPVL = gPIL; /* Save current runlevel */
	int i;

	// Update SYSTEM bits - PIL
	gReg->reg_STS = (gReg->reg_STS & 0xF000) | ((newLevel & 0x0f) << 8);
	return true;
}

void setbit(ushort regnum, ushort stsbit, char val)
{

	if ((regnum == _STS) && (stsbit > 7))
	{
		setbit_STS_MSB(stsbit, val);
		return;
	}

	ushort thebit = 0;
	if (val)
	{
		thebit = (1 << stsbit);
		gReg->reg[CurrLEVEL][regnum] = (thebit | gReg->reg[CurrLEVEL][regnum]);

		if (stsbit == _Z) // error bit is set
		{
			gCHKIT = true; // we need to check PK after this
		}
	}
	else
	{
		thebit = (1 << stsbit) ^ 0xFFFF;
		gReg->reg[CurrLEVEL][regnum] = (thebit & gReg->reg[CurrLEVEL][regnum]);
	}
}

short signExtend(ushort x)
{
	short res = (ushort)x;

	// If negative (bit 7==1), extend high 8 bits with 1's
	if ((x & 1 << 7) != 0)
		res |= 0xFF00;

	return res;
}

ushort do_add(ushort a, ushort b, ushort k)
{
	int tmp;
	bool is_diff;
	tmp = ((int)a) + ((int)b) + ((int)k);
	/* C (carry) */
	if (tmp & 0xffff0000)
		setbit(_STS, _C, 1);
	else
		setbit(_STS, _C, 0);
	/* O(static overflow), Q (dynamic overflow) */
	is_diff = (((1 << 15) & a) ^ ((1 << 15) & b)); /* is bit 15 of the two operands different? */
	if (!(is_diff) && (((1 << 15) & a) ^ ((1 << 15) & tmp)))
	{						 /* if equal and result is different... */
		setbit(_STS, _O, 1); // Static overflow
		setbit(_STS, _Q, 1); // Dynamic overflow (Instruction test shows Q must be set)
	}
	else
	{
		setbit(_STS, _Q, 0);
		//setbit(_STS, _O, 0); NO!
	}
	return (ushort)tmp;
}

void AdjustSTS(ushort reg_a, ushort operand, int result)
{
	/* C (carry) */
	if (result > 0xFFFF)
		setbit(_STS, _C, 1);
	else
		setbit(_STS, _C, 0);

	/* O(static overflow), Q (dynamic overflow) */
	if (!(((1 << 15) & reg_a) ^ ((1 << 15) & operand)) && (((1 << 15) & reg_a) ^ ((1 << 15) & result)))
	{
		setbit(_STS, _O, 1);
		setbit(_STS, _Q, 1);
	}
	else
		setbit(_STS, _Q, 0);
}

/*
 * converts an ascii octal number to an integer.
 * only handles positive values, so if result is negative
 * we have an error. this also means we only handle numbers
 * up to max positive int on the platform.
 */
int aoct2int(char *str)
{
	double tmp = 0;
	int num, count;

	num = strlen(str);
	for (count = 0; num > 0; num--)
	{
		if ((str[num - 1] >= '0') && (str[num - 1] <= '7'))
		{
			tmp += (double)((str[num - 1] - '0') * pow((double)8, (double)count));
			count++;
		}
		else
		{
			return (-1);
		}
	}
	if (tmp > (double)INT_MAX)
		tmp = -1;
	return ((int)tmp);
}

/*
 *
 */
void mopc_cmd(char *cmdstr, char cmdc)
{
	int len;
	int val;
	bool has_val = false;

	len = strlen((const char *)cmdstr);
	if (len > 255)
		return; /* This is BAAD, so we just silently fail the command at the moment */

	if (len)
	{ /* We probably have an octal argument here */
		val = aoct2int(cmdstr);
		has_val = true;
	}
	switch (cmdc)
	{
	case '.':
		/* Set breakpoint */
		if (!(has_val))
			; /*TODO: Check whats needed here */
		if ((val >= 0) && (val < 65536))
		{ /* valid range for 16 bit addr */
			gReg->has_breakpoint = true;
			gReg->breakpoint = (ushort)(val & 0xffff);
			CurrentCPURunMode = SEMIRUN;
		}
		break;
	default:
		break;
	}

	if (debug)
		fprintf(debugfile, "(##)mopc_cmd: ");
	if (debug)
		fprintf(debugfile, "%s%c\n", cmdstr, cmdc);
	if (debug)
		fflush(debugfile);
}

/* We run mopc as a thread here, but ticks it either from panel or rtc to get more correct nd behaviour */
/* TODO:: NO ERROR CHECKING CURRENTLY DONE!!!! Need to see how real ND mopc behaves first */
void mopc_thread()
{
	int s;
	char ch;
	char str[256];
	unsigned char ptr = 0; /* points to next free char position in str */
	int i;

	if (debug)
		fprintf(debugfile, "(##)mopc_thread running...\n");
	if (debug)
		fflush(debugfile);

	memset(str, '\0', sizeof(str));

	while (CurrentCPURunMode != SHUTDOWN)
	{
		/* This should trigger once every rtc/panel interrupt hopefully */

		if (debug)
			fprintf(debugfile, "(##)mopc tick...\n");
		if (debug)
			fflush(debugfile);

		if (mopc_in(&ch))
		{ /* char available */
			if (debug)
				fprintf(debugfile, "(##)mopc char available... char='%c'\n", ch);
			if (debug)
				fflush(debugfile);

			if ((ch >= '0' && ch <= '7') || (ch >= 'A' && ch <= 'Y'))
			{
				str[ptr] = ch;
				ptr++;
				mopc_out(ch);
			}
			else if ((ch == '@') || (ch == ' '))
			{
				ptr = 0;
				mopc_out(ch);
			}
			else if (ch == 10)
			{
				//				mopc_cmd(str,ch);
				mopc_out(ch);
				ptr = 0;
			}
			else if ((ch == '@') || (ch == ' ') || (ch == '<') || (ch == '/') || (ch == '*'))
			{
				mopc_out(ch);
			}
			else if ((ch == '&') || (ch == '$'))
			{
				mopc_out(ch);
			}
			else if (ch == '.')
			{
				mopc_out(ch);
				mopc_cmd(str, ch);
				ptr = 0;
				memset(str, '\0', sizeof(str));
			}
			else if (ch == 'Z')
			{
				mopc_out(ch);
			}
			else if (ch == '!')
			{
				if (CurrentCPURunMode == STOP)
				{
					mopc_out(ch);
					if (ptr)
					{					   /* ok we have some chars available */
						i = aoct2int(str); /* FIXME :: THIS IS WRONG, we should use octal input, not decimal!!! (just added this quickly to test)*/
						gPC = i;
					}
					CurrentCPURunMode = RUN;
				}
				else
				{
					mopc_out('?');
				}
			}
			else if (ch == '#')
			{
				mopc_out(ch);
			}
			else if (ch == 27)
			{
				if (CurrentCPURunMode != STOP)
					MODE_OPCOM = 0;
			}
			else
				mopc_out('?');
		}
	}
}

/*
 * Recalculate internal interrupt bits
 * Updates gIID, gPID and gPK
 */
void recalcInternalInterruptBits()
{
	// Check for Z (error) flag	
	if (getbit(_STS, _Z))
	{
		gIID |= 1 << 5;
	}

	if ((gIID & gIIE) != 0)
	{
		// Set PID bit 14 to trigger LVL change to 14
		gPID |= (1 << 14);
		gIIC = calcIIC();
		gCHKIT = true; // removing this makes sintran crash during boot  // System malfunction. Sintran halt in ERRFATAL. L-reg: 042713
	}
}

// Calculate PK based on PID and PIE
void calcPK()
{
	// Recalculate PK based on PID and PIE
	int s;
	ushort lvl;
	ushort i;
	gPK = 0;
	i = gPIE & gPID;
	//	if (debug) fprintf(debugfile,"PT DEB CheckPK: gPIE=%06o gPID=%06o i=%06o gPK=%d gPIL=%d\n",gPIE,gPID,i,gPK,gPIL);
	if (i)
	{
		// Check for detected and enabled bits. Highest bits has highest priority
		for (lvl = 15; lvl >= 0; lvl--)
		{
			if (i & 1 << lvl)
			{
				gPK = lvl;
				return;
			}
		}
	}
}

/*
 * Internal interrupt setting routine.
 * IN: interrupt level and possible subbitfield
 * for those levels that has that. (LVL 14).
 */
void interrupt(ushort lvl, ushort sub)
{
	int s;
	if (lvl == 14)
	{
		gIID |= sub;
		if (gIID & gIIE)
			gPID |= (1 << 14);
	}
	else
	{
		gPID |= (1 << lvl);
	}

	// printf("Interrupt at %d, sub=0x%x. GID_BIT_= %d\r\n", lvl, sub, (gIID>>8)&1);
	recalcInternalInterruptBits();

	// Check for MPV (bit 2), PF (bit 3), or illegal instruction (bit 4)
	if (lvl == 14 && (sub & ((1 << 2) | (1 << 3) | (1 << 4))))
	{
#ifdef DEBUG_TRAP
		printf("TRAP at P:[%6o], sub=%d \r\n", gPC, sub);
#endif
		longjmp(cpu_jmp_buf, 1); // Jump back to cpurun() in cpu_thread
	}
}

void device_interrupt(ushort interruptBits)
{
	// Only process bits 10-13 and 15 for device interrupts
	ushort validBits = interruptBits & 0xBC00; // Mask for bits 10-13,15 (0b1111010000000000)

	ushort tmp = gPID;

	// clear gIID bits 10-13,15
	gPID &= ~validBits;

	// set gIID bits from device(s)
	gPID |= validBits;

	if (tmp != gPID)
	{
		gCHKIT = true; // Check if we need to update PK based on new interrupts		
	}
}

#if _removed_ // PageTables are now handled in cpu_mms.c
/*
 * Check if access is to the PageTables in shadow memory
 *
 */
bool IsShadowMemAccess(ulong addr)
{
	ushort pcr = gReg->reg_PCR[CurrLEVEL];
	unsigned char ring_num = pcr & 0x03;
	if ((3 == ring_num) || !(STS_PONI))
	{
		if (((STS_SEXI) && (addr >= 0177000) && (addr < 01000000)) || (!(STS_SEXI) && (addr >= 0177400) && (addr < 01000000)))
		{
			return true;
		}
	}
	return false;
}

/*
 * Write to shadow mem/pagetables.
 */
void PT_Write(ushort value, ushort addr, ushort byte_select)
{
	ushort ptadd;
	ulong temp;
	ptadd = (STS_SEXI) ? (addr & 0x01ff) >> 1 : (addr & 0x00ff);
	temp = gPT->pt_arr[ptadd];
	//	if (debug) fprintf(debugfile,"PT_Write: addr=%06o(%d) ptadd=%d temp=%08x byte_select=%d value=%04x SEXI=%d\n",
	//		addr,addr,ptadd,temp,byte_select,value,STS_SEXI);
	switch (byte_select)
	{
	case 0: /* MSB in ND */
		if (STS_SEXI)
			temp = (!(addr & 0x01)) ? (temp & 0x00ffffff) | (value << 24) : /* Even addr, MSB = bits 31-24 in PT */
					   (temp & 0xffff00ff) | (value << 8);					/* Odd addr, MSB = bits 15-8 in PT */
		else
			temp = (temp & 0x01fffeff) | (ulong)(value & 0xfe) << 25 | ((value & 0x01) << 8);
		break;
	case 1: /* LSB in ND */
		if (STS_SEXI)
			temp = (!(addr & 0x01)) ? (temp & 0xff00ffff) | (value & 0xff) << 16 : /* Even addr, LSB = bits 23-16 in PT */
					   (temp & 0xffffff00) | (value & 0xff);					   /* Odd addr, LSB = bits 7-0 in PT */
		else
			temp = (temp & 0xffffff00) | (value & 0xff);
		break;
	default: /* whole word write */
		if (STS_SEXI)
			temp = (!(addr & 0x01)) ? (temp & 0x0000ffff) | (value << 16) : (temp & 0xffff0000) | value;
		else
			temp = (temp & 0x01fffe00) | (ulong)(value & 0xfe00) << 16 | (value & 0x01ff);
		break;
	}
	//	if (debug) fprintf(debugfile,"PT_Write: ==> temp=%08x\n",temp);
	gPT->pt_arr[ptadd] = temp;
	if (trace & 0x08)
		fprintf(tracefile,
				"#m (i,t,a) #v# (\"%d\",\"Write PageTables\",\"%08o\");\n",
				(int)instr_counter, addr);
	return;
}

/*
 * Read from shadow mem/pagetables.
 */
ushort PT_Read(ushort addr)
{
	ushort ptadd;
	//	ulong temp;
	unsigned int temp; /* This should be 32 bit always */
	ushort res;
	ptadd = (STS_SEXI) ? (addr & 0x01ff) >> 1 : (addr & 0x00ff);
	temp = gPT->pt_arr[ptadd];
	if (debug)
		fprintf(debugfile, "PT_Read: addr=%06o(%d) ptadd=%d temp=%08x SEXI=%d\n",
				addr, addr, ptadd, temp, STS_SEXI);
	if (STS_SEXI)
		res = (!(addr & 0x01)) ? ((temp & 0xffff0000) >> 16) : /* Even addr */
				  (temp & 0x0000ffff);						   /* Odd addr */
	else
		res = ((temp & 0xfe000000) >> 16) | (temp & 0x000001ff);
	if (debug)
		fprintf(debugfile, "PT_Read: <== res=%04x\n", res);
	return (res); /* PT data */
}

#endif // _removed_
/*
 * Routine that handles phys mem writes and shadow memory.
 */
void PhysMemWrite(ushort value, ulong addr)
{
	WritePhysicalMemory(addr, value, false); // in cpu_mms.c
	return;

#if _removed_
	ushort *p_phy_addr;
	if (IsShadowMemAccess(addr))
	{									  /* Write to PageTables!!! */
		PT_Write(value, (ushort)addr, 2); /* 2 = word write */
		return;
	}
	// 11.09.22: Removed this mask and now read and wrtie physical seems to work
	// addr &= (ND_Memsize); /* Mask it to the memory size we have to prevent coredumps :) */

	// Check memory bounds
	if (addr < 0 || addr >= (sizeof(VolatileMemory.n_Array) / sizeof(VolatileMemory.n_Array[0])))
	{

		interrupt(14, 1 << 9); /* Memory out of range */
		return;
	}

	p_phy_addr = &VolatileMemory.n_Array[addr];
	*p_phy_addr = value;
#endif
}

/*
 * Routine that handles phys mem reads and shadow memory.
 */
ushort PhysMemRead(ulong addr)
{
	return ReadPhysicalMemory(addr, false); // in cpu_mms.c

#if _removed_
	ushort res;
	if (IsShadowMemAccess(addr))
	{ /* Read from PageTables!!! */
		res = PT_Read((ushort)addr);
		return (res); /* PT data */
	}

	// Check memory bounds
	if (addr < 0 || addr >= (sizeof(VolatileMemory.n_Array) / sizeof(VolatileMemory.n_Array[0])))
	{

		interrupt(14, 1 << 9); /* Memory out of range */
		return 0;
	}

	// 11.09.22: Removed this mask and now read and wrtie physical seems to work
	// addr &= (ND_Memsize); /* Mask it to the memory size we have to prevent coredumps :) */

	return VolatileMemory.n_Array[addr];
#endif
}

/*
 * Write a word to memory.
 * Here we implement all Memory Management System functions.
 */
void MemoryWrite(ushort value, ushort addr, bool UseAPT, unsigned char byte_select)
{
	WriteVirtualMemory(addr, value, UseAPT, byte_select); // in cpu_mms.c
	return;

#if _removed_
	ushort pcr = gReg->reg_PCR[CurrLEVEL];
	unsigned char ring_num = pcr & 0x03;
	unsigned char vpn = addr >> 10;
	ushort ppn;
	unsigned char pt_num;
	ulong PTe;
	ushort *p_phy_addr;
	//	bool error = false;

	/* just debug the virtual address for now. later on we got to get the real address I think */
	/* this is for now so we can get output of all memory accesses in a program and debug instructions at full speed */
	//	if (trace) AddMemTrace((unsigned int)addr,'W');

	/* First we check if Shadow Memory is accessible. */
	if (IsShadowMemAccess((ulong)addr))
	{ /* Write to PageTables!!! */
		PT_Write(value, addr, byte_select);
		return;
	}
	if (STS_PONI)
	{
		if ((STS_PTM) && UseAPT)
			pt_num = (pcr >> 7) & 0x03; /* APT */
		else
			pt_num = (pcr >> 9) & 0x03; /* PT */

		PTe = gPT->pt[pt_num][vpn];

		/* Check if not WPM(Write Permit bit is not set) */
		if (!(PTe & (1 << 31)))
		{
			//			debug=1; /* PT DEBUGGING: remove once finished */
			if (!(PTe & (0x07 << 29)))
			{
				interrupt(14, 1 << 3);						 /* Page Fault */
				gPGS = pt_num << 6 | (vpn & 0x3f) | 1 << 14; /* Page Fault */
															 //				if (debug) fprintf(debugfile,
															 //					"WriteMemory: Page Fault, instr#=%d PTe=%08x pt_num=%d vpn=%d\n",
															 //					(int)instr_counter,PTe,pt_num,vpn);
			}
			else
			{
				interrupt(14, 1 << 2);						 /* Memory Protection Violation */
				gPGS = pt_num << 6 | (vpn & 0x3f) | 1 << 14; /* Permit Violation */
															 //				if (debug) fprintf(debugfile,
															 //					"WriteMemory: Memory Protection Violation, instr#=%d PTe=%08x pt_num=%d vpn=%d\n",
															 //						(int)instr_counter,PTe,pt_num,vpn);
			}
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Write Fail(WPM)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return;
			//			error=true;
		}

		/* Check if ring number is too low */
		if (((PTe >> 24) & 0x03) > ring_num)
		{
			//			debug=1; /* PT DEBUGGING: remove once finished */
			//			if (debug) fprintf(debugfile,"WriteMemory: Ring Violation, PTe=%08x pt_num=%d vpn=%d\n",PTe,pt_num,vpn);
			gPGS = ((ushort)pt_num << 6) | vpn; /* Ring Violation */
			interrupt(14, 1 << 2);				/* Ring Protection Violation */
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Write Fail(Ring)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return;
			//			error=true;
		}
		//		if(error) return;

		/* Mark that the page was written and used */
		gPT->pt[pt_num][vpn] |= ((ulong)0x03 << 27); /* Set WIP and PGU */

		/* Get physical page number */
		ppn = (STS_SEXI) ? PTe & 0x3fff : PTe & 0x01ff;

		//		if (debug) fprintf(debugfile,"WriteMemory: OK, PTe=%08x pt_num=%d vpn=%d ppn=%04x\n",PTe,pt_num,vpn,ppn);
		//		if (debug) fprintf(debugfile,"WriteMemory: OK, gPT->pt[pt_num][vpn]=%08x\n",gPT->pt[pt_num][vpn]);

		p_phy_addr = &VolatileMemory.n_Pages[ppn][addr & (((ushort)1 << 10) - 1)];
		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Write (PT)\",\"%08o\");\n",
					(int)instr_counter, addr);
	}
	else
	{
		p_phy_addr = &VolatileMemory.n_Array[addr]; /* Only 16 address bits in POF mode */
		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Write ()\",\"%08o\");\n",
					(int)instr_counter, addr);
	}

	// :NOTE: ND memory is big endian but NDemulator is little endian!
	switch (byte_select)
	{
	case 0: /* Even, which means MSB byte, or bits 15-8 */
		*p_phy_addr = (*p_phy_addr & 0xFF) | (value << 8);
		break;
	case 1: /*Odd, which means LSB byte, or bits 7-0 */
		*p_phy_addr = (*p_phy_addr & 0xFF00) | value;
		break;
	default:
		*p_phy_addr = value;
		break;
	}
#endif
}

/*
 * Read a word from memory.
 * Here we implement all Memory Management System functions.
 */
ushort MemoryRead(ushort addr, bool UseAPT)
{
	return ReadVirtualMemory(addr, UseAPT); // in cpu_mms.c

#if _removed_
	ushort pcr = gReg->reg_PCR[CurrLEVEL];
	unsigned char ring_num = pcr & 0x03;
	ulong PTe;
	ushort res;
	unsigned char vpn = addr >> 10;
	ushort ppn;
	unsigned char pt_num;
	//	bool error = false;

	/* just debug the virtual address for now. later on we got to get the real address I think */
	/* this is for now so we can get output of all memory accesses in a program and debug instructions at full speed */
	//	if (trace) AddMemTrace((unsigned int)addr,'R');

	/* First we check if Shadow Memory is accessible. */
	if (IsShadowMemAccess((ulong)addr))
	{ /* Read from PageTables!!! */
		res = PT_Read(addr);
		return (res); /* PT data */
	}

	if (STS_PONI)
	{
		if ((STS_PTM) && UseAPT)
			pt_num = (pcr >> 7) & 0x03; // APT
		else
			pt_num = (pcr >> 9) & 0x03; // PT

		PTe = gPT->pt[pt_num][vpn];

		/* Check if not RPM(Read Permit bit is not set) */
		if (!(PTe & ((ulong)1 << 30)))
		{
			//			debug=1; /* PT DEBUGGING: remove once finished */
			gPGS = ((ushort)1 << 14) | ((ushort)pt_num << 6) | vpn;
			if (!(PTe & ((ulong)0x07 << 29)))
			{
				//				if (debug) fprintf(debugfile,"ReadMemory: Page Fault, PTe=%08x\n",PTe);
				interrupt(14, 1 << 3); /* Page Fault */
			}
			else
			{
				//				if (debug) fprintf(debugfile,"ReadMemory: Memory protection Violation, PTe=%08x\n",PTe);
				interrupt(14, 1 << 2); /* Memory Protection Violation */
			}
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Read Fail(RPM)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return (0); /* TODO:: We should rethink MemoryRead to handle errors more gracefully. */
						//			error=true;
		}

		/* Check if ring number is too low */
		if (((PTe >> 24) & 0x03) > ring_num)
		{
			// 			debug=1; /* PT DEBUGGING: remove once finished */
			gPGS = ((ushort)pt_num << 6) | vpn;
			interrupt(14, 1 << 2); /* Ring Protection Violation */
								   //			if (debug) fprintf(debugfile,"ReadMemory: Ring Violation, PTe=%08x\n",PTe);
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Read Fail(Ring)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return (0); /* TODO:: We should rethink MemoryRead to handle errors more gracefully. */
						//			error=true;
		}

		//		if (error) return;

		/* Mark that the page was used */
		gPT->pt[pt_num][vpn] |= ((ulong)0x01 << 27); /* Set PGU */

		/* Get physical page number */
		ppn = (STS_SEXI) ? PTe & 0x3fff : PTe & 0x01ff;

		//		if (debug) fprintf(debugfile,"ReadMemory: OK, PTe=%08x pt_num=%d vpn=%d ppn=%04x\n",PTe,pt_num,vpn,ppn);
		//		if (debug) fprintf(debugfile,"ReadMemory: OK, gPT->pt[pt_num][vpn]=%08x\n",gPT->pt[pt_num][vpn]);

		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Read (PT)\",\"%08o\");\n",
					(int)instr_counter, addr);
		return VolatileMemory.n_Pages[ppn][addr & (((ushort)1 << 10) - 1)];
	}
	else
	{
		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Read ()\",\"%08o\");\n",
					(int)instr_counter, addr);
		return VolatileMemory.n_Array[addr]; /* Only 16 address bits in POF mode */
	}
#endif
}

ushort MemoryFetch(ushort addr, bool UseAPT)
{
	return FetchVirtualMemory(addr, UseAPT); // in cpu_mms.c

#if _removed_
	ushort pcr = gReg->reg_PCR[CurrLEVEL];
	unsigned char ring_num = pcr & 0x03;
	ulong PTe;
	ushort res;
	unsigned char vpn = addr >> 10;
	ushort ppn;
	unsigned char pt_num;
	//	bool error = false;

	/* just debug the virtual address for now. later on we got to get the real address I think */
	/* this is for now so we can get output of all memory accesses in a program and debug instructions at full speed */
	//	if (trace) AddMemTrace((unsigned int)addr,'F');

	/* First we check if Shadow Memory is accessible. */
	if (IsShadowMemAccess((ulong)addr))
	{ /* Read from PageTables!!! */
		res = PT_Read(addr);
		return (res); /* PT data */
	}

	if (STS_PONI)
	{
		if ((STS_PTM) && UseAPT)
			pt_num = (pcr >> 7) & 0x03; // APT
		else
			pt_num = (pcr >> 9) & 0x03; // PT

		PTe = gPT->pt[pt_num][vpn];

		/* Check if not FPM(Fetch Permit bit is not set) */
		if (!(PTe & ((ulong)1 << 29)))
		{
			// 			debug=1; /* PT DEBUGGING: remove once finished */
			gPGS = ((ushort)3 << 14) | ((ushort)pt_num << 6) | vpn;
			if (!(PTe & ((ulong)0x07 << 29)))
				interrupt(14, 1 << 3); /* Page Fault */
			else
				interrupt(14, 1 << 2); /* Memory Protection Violation */
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Fetch Fail(FPM)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return (0); /* TODO:: We should rethink MemoryFetch to handle errors more gracefully. */
						//			error = true;
		}

		/* Check if ring number is too low */
		if (((PTe >> 24) & 0x03) > ring_num)
		{
			// 			debug=1; /* PT DEBUGGING: remove once finished */
			gPGS = ((ushort)1 << 15) | ((ushort)pt_num << 6) | vpn;
			interrupt(14, 1 << 2); /* Ring Protection Violation */
			if (trace & 0x08)
				fprintf(tracefile,
						"#m (i,t,a) #v# (\"%d\",\"Fetch Fail(Ring)\",\"%08o\");\n",
						(int)instr_counter, addr);
			return (0); /* TODO:: We should rethink MemoryFetch to handle errors more gracefully. */
						//			error = true;
		}

		//		if (error) return;

		/* Mark that the page was used */
		gPT->pt[pt_num][vpn] |= ((ulong)0x01 << 27); /* Set PGU */

		/* Get physical page number */
		ppn = (STS_SEXI) ? PTe & 0x3fff : PTe & 0x01ff;

		//		if (debug) fprintf(debugfile,"FetchMemory: OK, PTe=%08x pt_num=%d vpn=%d ppn=%04x\n",PTe,pt_num,vpn,ppn);
		//		if (debug) fprintf(debugfile,"FetchMemory: OK, gPT->pt[pt_num][vpn]=%08x\n",gPT->pt[pt_num][vpn]);

		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Fetch (PT)\",\"%08o\");\n",
					(int)instr_counter, addr);
		return VolatileMemory.n_Pages[ppn][addr & (((ushort)1 << 10) - 1)];
	}
	else
	{
		if (trace & 0x08)
			fprintf(tracefile,
					"#m (i,t,a) #v# (\"%d\",\"Fetch ()\",\"%08o\");\n",
					(int)instr_counter, addr);
		return VolatileMemory.n_Array[addr]; /* Only 16 address bits in POF mode */
	}
#endif
}

#if _removed_ // was used by DMA r/w
uint32_t MemoryReadPhysical(ulong addr)
{
	// return -1 if outside of memory
	if (addr < 0 || addr >= (sizeof(VolatileMemory.n_Array) / sizeof(VolatileMemory.n_Array[0])))
	{
		return -1;
	}
	return VolatileMemory.n_Array[addr];
}

uint32_t MemoryWritePhysical(ulong addr, uint32_t value)
{
	// return -1 if outside of memory
	if (addr < 0 || addr >= (sizeof(VolatileMemory.n_Array) / sizeof(VolatileMemory.n_Array[0])))
	{
		return -1;
		printf("MEM: OUT OF BOND\r\n");
	}
	// printf("MEM: Writing %08o to %08o\r\n", value, addr);
	VolatileMemory.n_Array[addr] = value;
	return 0;
}
#endif

void checkAndSwitch()
{
	if (gCHKIT)
	{
		gCHKIT = false; // reset flag

		// recalc internal interrupt bits
		recalcInternalInterruptBits();

		if (!STS_IONI)
			return false;

		calcPK();

		if (gPK != gPIL)
		{
			//printf("Switching from %d P[%6o] to %d P[%6o]\r\n", gPIL, gPC, gPK, gReg->reg[gPK][_P]);
			setPIL(gPK); /* Change to new runlevel */

#ifdef DEBUG_PK_SWITCH
			bool isRTC = ((gPVL == 13) || (gPIL == 13));
			if (!isRTC)
			{
				printf("Switched from %d P[%6o] to %d P[%6o]\r\n", gPVL, gReg->reg[gPVL][_P], gPIL, gReg->reg[gPIL][_P]);
				printf("New pc after switch %6o\r\n", gPC);
			}
#endif
		}
	}
	return false;
}

ushort lastA = 0;
bool dump_started = false;
uint savePC;

/// @brief Run the CPU instructions as long as the CPU is not stopped or shutdown
void cpurun()
{
	int s;
	ushort operand, p_now;
	char disasm_str[256];

	while ((CurrentCPURunMode != STOP) && (CurrentCPURunMode != SHUTDOWN))
	{
		if (CurrentCPURunMode == SEMIRUN)
		{ /* Here we should handle single step, breakpoints etc */
			if (gReg->has_breakpoint)
				if (gReg->breakpoint == gPC)
				{ /* TODO:: Check if we should execute the instruction at breakpoint address or not */
					CurrentCPURunMode = STOP;
					return;
				}
			if (gReg->has_instr_cntr)
				if (gReg->instructioncounter > 0)
					gReg->instructioncounter--;
				else
				{
					CurrentCPURunMode = STOP;
					return;
				}
		}

		// Check for level shift (typically after an interrupt or WAIT instruction)
		checkAndSwitch();

		prefetch(); /* works because gPC should already be setup when cpurun is called */
		gReg->myreg_IR = gReg->myreg_PFB;

		savePC = gPC;

		instr_counter++;
		if (trace)
			trace_pre(1, "S", gReg->reg[CurrLEVEL][0]);
		operand = gReg->myreg_IR;
		//		operand=MemoryFetch(gPC,true);
		p_now = gPC;
		if (trace)
			trace_instr(operand);
		if (DISASM)
			disasm_instr(gPC, operand);

		do_op(operand, false);

		if (trace & 0x16)
			trace_regs();
		if (trace)
			trace_post(1, "S", gReg->reg[CurrLEVEL][0]);
		if (trace)
			trace_flush();

		// Tick IO devices SYNC (not using thread)
		IO_Tick();
	}
}

/// @brief Start the CPU
/// @return Returns when the CPU is shut down
void cpu_start()
{

	int s;
	if (debug)
		fprintf(debugfile, "(##)cpu_thread running...\n");
	if (debug)
		fflush(debugfile);
	if (DISASM)
		disasm_setlbl(gPC);

	// Set up longjmp target once outside the loop
	if (setjmp(cpu_jmp_buf) != 0)
	{
		// We had an interrupt (MPV, PF, or illegal instruction)
		// PGS bit 15 indicates if fault was during fetch (1) or data cycle (0)

		// PGS:
		//
		// if bit 15 is a one, the page fault or protection violation occurred during the fetch of an instruction.
		// In this case, the P register has not been incremented and the instruction causing the violation(and the restart point)
		//
		// If bit 15 is zero, the page fault or protection violation occurred during the data cycles of an instruction.
		// In this case, the P register points to the instruction after the instruction causing the internal hardware status interrupt.
		// When the cause of the internal hardware status interrupt has been removed, the restart point will be found by subtracting one from the P register.

#ifdef DEBUG_TRAP
		printf("CPU: Interrupt handler returned, PC=%06o, PGS=%04x\n", gPC, gPGS);
#endif
	}

	while (CurrentCPURunMode != SHUTDOWN)
	{
		if (CurrentCPURunMode != STOP)
		{
			cpurun();
		}

		if (CurrentCPURunMode == STOP)
		{
			// OPCOM MODE ?
			printf("CPU: WAS STOPPED, SHUTTING DOWN\r\n");
			CurrentCPURunMode = SHUTDOWN;
		}
	}
}

void AddMemTrace(unsigned int addr, char whom)
{
	struct MemTraceList *curr, *new;
	new = malloc(sizeof(struct MemTraceList));
	new->next = 0;
	new->addr = addr;
	new->funct = whom;
	curr = gMemTrace;
	if (curr != 0)
	{
		while (curr->next != 0)
			curr = curr->next;
	}
	else
	{
		gMemTrace = new;
		return;
	}
	curr->next = new;
}

void DelMemTrace()
{
	struct MemTraceList *curr, *head;
	head = gMemTrace;
	while (head)
	{
		curr = head->next;
		free(head);
		head = curr;
	}
	gMemTrace = head;
}

void PrintMemTrace()
{
	struct MemTraceList *curr;
	char temp;
	curr = gMemTrace;
	while (curr)
	{
		temp = curr->funct;
		switch (temp)
		{
		case 'F':
			fprintf(tracefile, "INSERT INTO tracemem (instrnum,type,addr) values (\"%d\",\"Fetch\",\"%08o\");\n", (int)instr_counter, curr->addr);
			break;
		case 'R':
			fprintf(tracefile, "INSERT INTO tracemem (instrnum,type,addr) values (\"%d\",\"Read\",\"%08o\");\n", (int)instr_counter, curr->addr);
			break;
		case 'W':
			fprintf(tracefile, "INSERT INTO tracemem (instrnum,type,addr) values (\"%d\",\"Write\",\"%08o\");\n", (int)instr_counter, curr->addr);
			break;
		default:
			fprintf(tracefile, "INSERT INTO tracemem (instrnum,type,addr) values (\"%d\",\"Impossible\",\"%08o\");\n", (int)instr_counter, curr->addr);
			break;
		}
		curr = curr->next;
	}
}

void Instruction_Add(int opcode, void *funcpointer)
{
	instr_funcs[opcode] = funcpointer;
}

void Instruction_Add_Range(int start, int stop, void *funcpointer)
{
	int i;
	for (i = start; i <= stop; i++)
		instr_funcs[i] = funcpointer;
	return;
}

void Instruction_Add_Mask(int opcode, int mask, void *funcpointer)
{
	int i;
	int signature = opcode & mask;

	for (i = opcode; i <= 0xFFFF; i++)
	{
		if ((i & mask) == signature)
		{
			instr_funcs[i] = funcpointer;
		}
	}
	return;
}

/*
 * Add IO handler addresses in this function
 * This also thus actually acts as the new instruction parser also.
 */
void Setup_Instructions()
{
	Instruction_Add_Range(0000000, 0177777, &illegal_instr); /* First make all instructions by default point to illegal_instr  */

	// Instruction_Add_Range(0000000, 0003777, &ndfunc_stz); /* STZ  */
	Instruction_Add_Mask(0000000, 0xF800, &ndfunc_stz);

	// Instruction_Add_Range(0004000, 0007777, &ndfunc_sta); /* STA  */
	Instruction_Add_Mask(0004000, 0xF800, &ndfunc_sta);

	// Instruction_Add_Range(0010000, 0013777, &ndfunc_stt); /* STT  */
	Instruction_Add_Mask(0010000, 0xF800, &ndfunc_stt);

	// Instruction_Add_Range(0014000, 0017777, &ndfunc_stx); /* STX  */
	Instruction_Add_Mask(0014000, 0xF800, &ndfunc_stx);

	// Instruction_Add_Range(0020000, 0023777, &ndfunc_std); /* STD  */
	Instruction_Add_Mask(0020000, 0xF800, &ndfunc_std);

	// Instruction_Add_Range(0024000, 0027777, &ndfunc_ldd); /* LDD  */
	Instruction_Add_Mask(0024000, 0xF800, &ndfunc_ldd);

	// Instruction_Add_Range(0030000, 0033777, &ndfunc_stf); /* STF  */
	Instruction_Add_Mask(0030000, 0xF800, &ndfunc_stf);

	// Instruction_Add_Range(0034000, 0037777, &ndfunc_ldf); /* LDF  */
	Instruction_Add_Mask(0034000, 0xF800, &ndfunc_ldf);

	// Instruction_Add_Range(0040000, 0043777, &ndfunc_min); /* MIN  */
	Instruction_Add_Mask(0040000, 0xF800, &ndfunc_min);

	// Instruction_Add_Range(0044000, 0047777, &ndfunc_lda); /* LDA  */
	Instruction_Add_Mask(0044000, 0xF800, &ndfunc_lda);

	// Instruction_Add_Range(0050000, 0053777, &ndfunc_ldt); /* LDT  */
	Instruction_Add_Mask(0050000, 0xF800, &ndfunc_ldt);

	// Instruction_Add_Range(0054000, 0057777, &ndfunc_ldx); /* LDX  */
	Instruction_Add_Mask(0054000, 0xF800, &ndfunc_ldx);

	// Instruction_Add_Range(0060000, 0063777, &ndfunc_add); /* ADD  */
	Instruction_Add_Mask(0060000, 0xF800, &ndfunc_add);

	// Instruction_Add_Range(0064000, 0067777, &ndfunc_sub); /* SUB  */
	Instruction_Add_Mask(0064000, 0xF800, &ndfunc_sub);

	// Instruction_Add(0070000, 0073777, &ndfunc_and); /* AND  */
	Instruction_Add_Mask(0070000, 0xF800, &ndfunc_and);

	// Instruction_Add_Range(0074000, 0077777, &ndfunc_ora); /* ORA  */
	Instruction_Add_Mask(0074000, 0xF800, &ndfunc_ora);

	// Instruction_Add_Range(0100000, 0103777, &ndfunc_fad); /* FAD  */
	Instruction_Add_Mask(0100000, 0xF800, &ndfunc_fad);

	// Instruction_Add_Range(0104000, 0107777, &ndfunc_fsb); /* FSB  */
	Instruction_Add_Mask(0104000, 0xF800, &ndfunc_fsb);

	// Instruction_Add_Range(0110000, 0113777, &ndfunc_fmu); /* FMU  */
	Instruction_Add_Mask(0110000, 0xF800, &ndfunc_fmu);

	// Instruction_Add_Range(0114000, 0117777, &ndfunc_fdv); /* FDV  */
	Instruction_Add_Mask(0114000, 0xF800, &ndfunc_fdv);

	// Instruction_Add_Range(0120000, 0123777, &mpy);		/* MPY  */
	Instruction_Add_Mask(0120000, 0xF800, &mpy);

	// Instruction_Add_Range(0124000, 0127777, &ndfunc_jmp); /* JMP  */
	Instruction_Add_Mask(0124000, 0xF800, &ndfunc_jmp);

	// Instruction_Add_Range(0134000, 0137777, &ndfunc_jpl); /* JPL  */
	Instruction_Add_Mask(0134000, 0xF800, &ndfunc_jpl);

	// CJPs - Conditional jumps
	// Instruction_Add_Range(0130000, 0130377, &ndfunc_jap); /* JAP */
	Instruction_Add_Mask(0130000, 0xFF00, &ndfunc_jap);

	// Instruction_Add_Range(0130400, 0130777, &ndfunc_jan); /* JAN */
	Instruction_Add_Mask(0130400, 0xFF00, &ndfunc_jan);

	// Instruction_Add_Range(0131000, 0131377, &ndfunc_jaz); /* JAZ */
	Instruction_Add_Mask(0131000, 0xFF00, &ndfunc_jaz);

	// Instruction_Add_Range(0131400, 0131777, &ndfunc_jaf); /* JAF */
	Instruction_Add_Mask(0131400, 0xFF00, &ndfunc_jaf);

	// Instruction_Add_Range(0132000, 0132377, &ndfunc_jpc); /* JPC */
	Instruction_Add_Mask(0132000, 0xFF00, &ndfunc_jpc);

	// Instruction_Add_Range(0132400, 0132777, &ndfunc_jnc); /* JNC */
	Instruction_Add_Mask(0132400, 0xFF00, &ndfunc_jnc);

	// Instruction_Add_Range(0133000, 0133377, &ndfunc_jxz); /* JXZ */
	Instruction_Add_Mask(0133000, 0xFF00, &ndfunc_jxz);

	// Instruction_Add_Range(0133400, 0133777, &ndfunc_jxn); /* JXN */
	Instruction_Add_Mask(0133400, 0xFF00, &ndfunc_jxn);

	// Instruction_Add(0140000, 0143777, &ndfunc_skp);
	Instruction_Add_Mask(0140000, 0xF8C0, &ndfunc_skp);

	// BCD (CX)
	Instruction_Add(0140120, &ndfunc_addd);	  /* ADDD  */
	Instruction_Add(0140121, &ndfunc_subd);	  /* SUBD  */
	Instruction_Add(0140122, &ndfunc_comd);	  /* COMD  */
	Instruction_Add(0140124, &ndfunc_pack);	  /* PACK  */
	Instruction_Add(0140125, &ndfunc_unpack); /* UPACK */
	Instruction_Add(0140126, &ndfunc_shde);	  /* SHDE  */

	Instruction_Add(0140123, &DoTSET); /* TSET  */
	Instruction_Add(0140127, &DoRDUS); /* RDUS  */

	{ // CE; CX

		Instruction_Add(0140130, &ndfunc_bfill); /* BFILL */
		Instruction_Add(0140131, &DoMOVB);		 /* MOVB  */
		Instruction_Add(0140132, &DoMOVBF);		 /* MOVBF */

		// Instruction_Add(0140131, &ndfunc_movb);  /* MOVB  */
		// Instruction_Add(0140132, &ndfunc_movbf); /* MOVBF */
	}

	switch (CurrentCPUType)
	{
	case ND110:
	case ND110CE:
	case ND110CX:
	case ND110PCX:
		Instruction_Add(0140133, &ndfunc_versn); /* VERSN - ND110+ */
		break;
	default:
		break;
	}

	{ // CE; CX

		Instruction_Add(0140134, &ndfunc_init);	 /* INIT  */
		Instruction_Add(0140135, &ndfunc_entr);	 /* ENTR  */
		Instruction_Add(0140136, &ndfunc_leave); /* LEAVE */
		Instruction_Add(0140137, &ndfunc_eleav); /* ELEAV */
	}
	// Instruction_Add(0140200, 0140277, &illegal_instr); /* USER1 (microcode defined by user or illegal instruction otherwise) */

	switch (CurrentCPUType)
	{
	case ND110:
	case ND110CE:
	case ND110CX:
	case ND110PCX:
		// ALL are priveleged!
		Instruction_Add(0140500, &unimplemented_instr); /* WGLOB - ND110 Specific */
		Instruction_Add(0140501, &unimplemented_instr); /* RGLOB - ND110 Specific */
		Instruction_Add(0140502, &unimplemented_instr); /* INSPL - ND110 Specific */
		Instruction_Add(0140503, &unimplemented_instr); /* REMPL - ND110 Specific */
		Instruction_Add(0140504, &unimplemented_instr); /* CNREK - ND110 Specific */
		Instruction_Add(0140505, &unimplemented_instr); /* CLPT  - ND110 Specific */
		Instruction_Add(0140506, &unimplemented_instr); /* ENPT  - ND110 Specific */
		Instruction_Add(0140507, &unimplemented_instr); /* REPT  - ND110 Specific */
		Instruction_Add(0140510, &unimplemented_instr); /* LBIT  - ND110 Specific */

		Instruction_Add(0140513, &unimplemented_instr); /* SBITP - ND110 Specific */
		Instruction_Add(0140514, &unimplemented_instr); /* LBYTP - ND110 Specific */
		Instruction_Add(0140515, &unimplemented_instr); /* SBYTP - ND110 Specific */
		Instruction_Add(0140516, &unimplemented_instr); /* TSETP - ND110 Specific */
		Instruction_Add(0140517, &unimplemented_instr); /* RDUSP - ND110 Specific */

		break;
	default:
		// Instruction_Add_Range(0140500, 0140577, &illegal_instr); /* USER2 (microcode defined by user or illegal instruction otherwise) */
		break;
	}

	Instruction_Add_Mask(0140600, 0xFFC0, &DoEXR); /* EXR */
	switch (CurrentCPUType)
	{
	case ND110:
	case ND110CE:
	case ND110CX:
	case ND110PCX:
		// ALL are priveleged!
		Instruction_Add(0140700, &unimplemented_instr); /* LASB - ND110 Specific */
		Instruction_Add(0140701, &unimplemented_instr); /* SASB - ND110 Specific */
		Instruction_Add(0140702, &unimplemented_instr); /* LACB - ND110 Specific */
		Instruction_Add(0140703, &unimplemented_instr); /* SASB - ND110 Specific */
		Instruction_Add(0140704, &unimplemented_instr); /* LXSB - ND110 Specific */
		Instruction_Add(0140705, &unimplemented_instr); /* LXCB - ND110 Specific */
		Instruction_Add(0140706, &unimplemented_instr); /* SZSB - ND110 Specific */
		Instruction_Add(0140707, &unimplemented_instr); /* SZCB - ND110 Specific */
		break;
	default:
		break;
	}

	if (true)
	{
		// ND100-CX and ND110-CX only

		Instruction_Add(0140300, &ndfunc_setpt);		 /* SETPT */
		Instruction_Add(0140301, &ndfunc_clept);		 /* CLEPT */
		Instruction_Add(0140302, &ndfunc_clnreent);		 /* CLNREENT */
		Instruction_Add(0140303, &ndfunc_chreent_pages); /* CHREENT-PAGES */
		Instruction_Add(0140304, &ndfunc_clepu);		 /* CLEPU */
	}
	Instruction_Add_Mask(0141200, 0xFFC0, &rmpy);		 /* RMPY */
	Instruction_Add_Mask(0141600, 0xFFC0, &rdiv);		 /* RDIV */
	Instruction_Add_Mask(0142200, 0xFFC0, &ndfunc_lbyt); /* LBYT */
	Instruction_Add_Mask(0142600, 0xFFC0, &ndfunc_sbyt); /* SBYT */

	// CX instructions
	Instruction_Add(0142700, &ndfunc_geco);				 /* GECO - Undocumented instruction */
	Instruction_Add_Mask(0143100, 0xFFC0, &DoMOVEW);	 /* MOVEW */
	Instruction_Add_Mask(0143200, 0xFFC0, &ndfunc_mix3); /* MIX3 */

	Instruction_Add_Mask(0143300, 0xFFC7, &ndfunc_ldatx); /* LDATX */
	Instruction_Add_Mask(0143301, 0xFFC7, &ndfunc_ldxtx); /* LDXTX */
	Instruction_Add_Mask(0143302, 0xFFC7, &ndfunc_lddtx); /* LDDTX */
	Instruction_Add_Mask(0143303, 0xFFC7, &ndfunc_ldbtx); /* LDBTX */
	Instruction_Add_Mask(0143304, 0xFFC7, &ndfunc_statx); /* STATX */
	Instruction_Add_Mask(0143305, 0xFFC7, &ndfunc_stztx); /* STZTX */
	Instruction_Add_Mask(0143306, 0xFFC7, &ndfunc_stdtx); /* STDTX */

	Instruction_Add(0143500, &ndfunc_lwcs); /* LWCS */

	Instruction_Add(0143604, &ndfunc_ident); /* IDENT PL10 */
	Instruction_Add(0143611, &ndfunc_ident); /* IDENT PL11 */
	Instruction_Add(0143622, &ndfunc_ident); /* IDENT PL12 */
	Instruction_Add(0143643, &ndfunc_ident); /* IDENT PL13 */

	Instruction_Add_Range(0144000, 0147777, &regop); /* --ROPS-- */
	Instruction_Add_Mask(0150000, 0xFFF0, &DoTRA);	 /* TRA */
	Instruction_Add_Mask(0150100, 0xFFF0, &DoTRR);	 /* TRR */
	Instruction_Add_Mask(0150200, 0xFFF0, &DoMCL);	 /* MCL */
	Instruction_Add_Mask(0150300, 0xFFF0, &DoMST);	 /* MST */
	Instruction_Add(0150400, &ndfunc_opcom);		 /* OPCOM */
	Instruction_Add(0150401, &ndfunc_iof);			 /* IOF */
	Instruction_Add(0150402, &ndfunc_ion);			 /* ION */
	switch (CurrentCPUType)
	{
	case ND110PCX:
		/* ND110 Butterfly only instruction */
		Instruction_Add(0150403, &unimplemented_instr); /* RTNSIM (SECRE) */
		break;
	default:
		break;
	}
	Instruction_Add(0150404, &ndfunc_pof);	/* POF */
	Instruction_Add(0150405, &ndfunc_piof); /* PIOF */
	Instruction_Add(0150406, &ndfunc_sex);	/* SEX */
	Instruction_Add(0150407, &ndfunc_rex);	/* REX */
	Instruction_Add(0150410, &ndfunc_pon);	/* PON */
	Instruction_Add(0150412, &ndfunc_pion); /* PION */

	Instruction_Add(0150415, &ndfunc_ioxt); /* IOXT */
	Instruction_Add(0150416, &ndfunc_exam); /* EXAM */
	Instruction_Add(0150417, &ndfunc_depo); /* DEPO */

	Instruction_Add_Mask(0151000, 0xFF00, &DoWAIT);		/* WAIT */
	Instruction_Add_Mask(0151400, 0xFF00, &ndfunc_nlz); /* NLZ */
	Instruction_Add_Mask(0152000, 0xFF00, &ndfunc_dnz); /* DNZ */
	Instruction_Add_Mask(0152402, 0xFF07, &ndfunc_srb); /* SRB */
	Instruction_Add_Mask(0152600, 0xFF07, &ndfunc_lrb); /* LRB */
	Instruction_Add_Mask(0153000, 0xFF00, &ndfunc_mon); /* MON */
	Instruction_Add_Mask(0153400, 0xFF80, &ndfunc_irw); /* IRW */
	Instruction_Add_Mask(0153600, 0xFF80, &ndfunc_irr); /* IRR */

	// Instruction_Add_Range(0154000, 0157777, &ndfunc_shifts); /* SHT, SHD, SHA, SAD */  /* NOTE: this is actually a ND1 instruction, so need to check which NDs implement it later */
	Instruction_Add_Mask(0154000, 0x7980, &ndfunc_shifts); // SHT
	Instruction_Add_Mask(0154200, 0x7980, &ndfunc_shifts); // SHD
	Instruction_Add_Mask(0154400, 0x7980, &ndfunc_shifts); // SHA
	Instruction_Add_Mask(0154600, 0x7980, &ndfunc_shifts); // SAD

	// Instruction_Add_Mask(0160000, 0xFFF0, &ndfunc_iot); /* IOT  - ND1 specific, but exists on all CPU's*/
	Instruction_Add_Mask(0161000, 0xFFF0, &ndfunc_iot); /* IOT  - ND1 specific, but exists on all CPU's*/

	// Instruction_Add_Mask(0161000, 0xF800, &ndfunc_iox); /* IOX */
	Instruction_Add_Mask(0164000, 0xF800, &ndfunc_iox); /* IOX */

	Instruction_Add_Mask(0170000, 0xFF00, &ndfunc_sab); /* SAB */
	Instruction_Add_Mask(0170400, 0xFF00, &ndfunc_saa); /* SAA */
	Instruction_Add_Mask(0171000, 0xFF00, &ndfunc_sat); /* SAT */
	Instruction_Add_Mask(0171400, 0xFF00, &ndfunc_sax); /* SAX */
	Instruction_Add_Mask(0172000, 0xFF00, &ndfunc_aab); /* AAB */
	Instruction_Add_Mask(0172400, 0xFF00, &ndfunc_aaa); /* AAA */
	Instruction_Add_Mask(0173000, 0xFF00, &ndfunc_aat); /* AAT */
	Instruction_Add_Mask(0173400, 0xFF00, &ndfunc_aax); /* AAX */

	Instruction_Add_Range(0174000, 0177777, &do_bops); /* Bit Operation Instructions */
													   /* Bit operations, 16 of them, 4 BSET,4 BSKP and 8 others */
}
