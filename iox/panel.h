#ifndef PANEL_H
#define PANEL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>



/*
 */
struct control_panel {
	int lock_key;
//	bool stop_button;
//	bool load_button;
//	bool opcom_button;
//	bool mcl_button;

	bool power_lamp;
	bool run_lamp;
	bool opcom_lamp;
};



struct display_panel {
//	bool opcom_button;

	bool trr_panc;	/* TRR has been issued, process command */
	bool sec_tick;	/* Seconds tick from rtc, update counters */

	uint16_t pap_curr_command;

	char func_display[40];	/* Max 40 chars in display buffer */
	uint16_t fdisp_cntr;	/* pointer to where we are */

	uint16_t seconds;		/* 16 bit second counter for realtime clock, ticks day counter every 12h */
	uint16_t days;		/* 16 bit day counter for realtime clock */
				/* So seconds should wrap at 3600x12 = 43200, and day possibly lowest bit is am/pm */

	bool power_lamp;
	bool run_lamp;
	bool opcom_lamp;
	int function_util;
	int function_hit;
	int function_ring;
	int function_mode;
};

struct display_panel *gPAP;


#endif // PANEL_H
