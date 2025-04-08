#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>
#include "nd100em.h"
#include "nd100lib.h"
#include "io_new.h"
#include "iox/panel.h"
#include "globals.h"
#include "cpu.h"

// Define all global variables here
float usertime = 0, systemtime = 0, totaltime = 0;
struct rusage *used = NULL;

// Global variable definitions
_NDRAM_ VolatileMemory;
_NDPT_ PageTable;
_RUNMODE_ CurrentCPURunMode;
_CPUTYPE_ CurrentCPUType;
struct CpuRegs *gReg = NULL;
union NewPT *gPT = NULL;
struct MemTraceList *gMemTrace = NULL;
struct IdentChain *gIdentChain = NULL;
double instr_counter = 0;
ushort PANEL_PROCESSOR = 0;

struct display_panel *gPAP = NULL;

char *FDD_IMAGE_NAME = NULL;
bool FDD_IMAGE_RO = false;
char *HAWK_IMAGE_NAME = NULL;
char *BIGDISK_IMAGE_NAME = NULL;

int CONFIG_OK = 0;
_BOOT_TYPE_ BootType;
ushort STARTADDR = 0;
int DISASM = 0;
int DAEMON = 0;
int CONSOLE_IS_SOCKET = 0;

struct termios savetty; 
struct config_t *pCFG = NULL;




/*** CPU VARIABLES ***/
/* Global variable definitions */
char *regn[] = {"S","D","P","B","L","A","T","X","U0","U1"};
char *regn_w[] = {"DS","DD","DP","DB","DL","DA","DT","DX"};

char *intregn_r[] = {"PANS","STS","OPR","PGS","PVL","IIC","PID","PIE","CSR","ACTL", "ALD" ,"PES","PGC","PEA","16","17"};
char *intregn_w[] = {"PANC","STS","LMP","PCR", "4", "IIE","PID","PIE","CCL","LCIL","UCILR","13", "14", "15" ,"16","17"};

char *relmode_str[] ={"",",B ","I ","I ,B ",",X ",",X ,B ","I ,X ","I ,B ,X "};
char *shtype_str[] ={"","ROT ","ZIN ","LIN "};

char *skiptype_str[] = {"EQL","GEQ","GRE","MGRE","UEQ","LSS","LST","MLST"};
char *skipregn_dst[] = {"0","DD","DP","DB","DL","DA","DT","DX"};
char *skipregn_src[] = {"0","SD","SP","SB","SL","SA","ST","SX"};

char *bopstsbit_str[] = {"SSPTM","SSTG","SSK","SSZ","SSQ","SSO","SSC","SSM","","","","","","","",""};

char *bop_str[] = {"BSET ZRO","BSET ONE","BSET BCM","BSET BAC","BSKP ZRO","BSKP ONE",
           "BSKP BCM","BSKP BAC","BSTC","BSTA","BLDC","BLDA","BANC","BAND","BORC","BORA"};

/* Instruction handling array */
void (*instr_funcs[65536])(ushort);

/* Device register addresses */
unsigned short devices[65536];

/* Mode flags */
unsigned short MON_RUN = 1;
unsigned short MODE_RUN = 1;
unsigned short MODE_OPCOM = 0;

/* Emulator state */
int emulatemon = 0; 


/*** DEBUG VARIABLES ***/
int debug = 0;
FILE *debugfile = NULL;
char *debugname = NULL;
char *debugtype = NULL;
