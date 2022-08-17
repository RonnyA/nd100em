/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2008 Roger Abrahamsson
 * Copyright (c) 2008 Zdravko Dimitrov
 * Copyright (c) 2008 Goran Axelsson
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "nd100.h"
#include "io.h"

/* console synchronization*/
sem_t sem_cons;

/* io synchronization*/
sem_t sem_io;

/* floppy synchronization*/
sem_t sem_floppy;

/* hawk disk synchronization*/
sem_t sem_hawk;

/* panel processor synchronization*/
sem_t sem_pap;

struct display_panel *gPAP;

/*
 * IOX and IOXT operation, dummy for now.
 * The Norm is: even address is read, odd address is write
 * source: ND-100 Input/Output Manual
 */
void io_op (ushort ioadd) {
        ioarr[ioadd](ioadd);	/* call using a function pointer from the array
				this way we are as flexible as possible as we
				implement io calls. */
}

/*
 * Access to unpopulated IO area
 */
void Default_IO(ushort ioadd) {
	mysleep(0,10); /* Sleep 10us for IOX timeout time simulation */
	if(gReg->reg_IIE & 0x80) {
		if (trace & 0x01) fprintf(tracefile,
			"#o (i,d) #v# (\"%d\",\"No IO device, IOX error interrupt after 10 us.\");\n",
			(int)instr_counter);
		interrupt(14,1<<7); /* IOX error lvl14 */
	}
}

/*
 * Read and write from/to floppy
 */
void Floppy_IO(ushort ioadd) {
	int s;
	int a = ioadd & 0x07;
	int i;	
	bool trigger_floppy_thread = 0;	
	struct floppy_data *dev = iodata[ioadd];
	if (debug) fprintf(debugfile,"Floppy_IO: IOX %d - A=%d\n",ioadd,gA);
	fflush(debugfile);
	

	while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
		continue; /* Restart if interrupted by handler */

	trigger_floppy_thread = 0;

	//if (a & 0x01) printf("Floppy WRITE %d  (%X)\r\n",a, (gA & 0xFFFF));

	switch(a) 
	{
		case 0: /* IOX RDAD - Read data buffer */
			gA = dev->buff[dev->bufptr];
			dev->bufptr = (dev->bufptr +1) & FDD_BUFFER_MAX;	
			//printf("R");
			break;

		case 1: /* IOX WDAT - Write data buffer */
			dev->buff[dev->bufptr] = gA;
			dev->bufptr = (dev->bufptr +1) & FDD_BUFFER_MAX;
			break;

		case 2: /* IOX RSR1 - Read status register No. 1 */

			gA = 0; /* Put A in consistent state */			
			/* bit 0 not used */
			gA |= (dev->irq_en) ? (1<<1) : 0;	/* IRQ enabled (bit 1)*/
			gA |= (dev->busy) ? (1<<2) : 0;		/* Device is busy (bit 2) */
			gA |= (dev->ready_for_transfer) ? (1<<3) : 0;		/* Ready for transfer (bit 3)*/		

			if (dev->drive_not_rdy || dev->write_protect ||dev->missing) gA |= (1<<4);	/* inclusive or of error (bit 4) =>check STS reg 2 for details*/


			gA |= (dev->record_deleted) ? (1<<5) : 0;	/*  Deleted record  (bit 5) */
			gA |= (dev->rw_complete) ? (1<<6) : 0;	/* ReadWrite complete (bit 6) */
			gA |= (dev->seek_complete) ? (1<<7) : 0;	/* Seek Complete (bit 7) */

			if (debug) fprintf(debugfile,"Floppy_IO: IOX %o RSR1 - A=%04x\n",ioadd,gA);
			break;

		case 3: /* IOX WCWD - Write control word */		
			if (debug) fprintf(debugfile,"IOX 1563 - Write Control Word...\n");

			bool old_irq = dev->irq_en;			
			dev->irq_en = (gA  >> 1) & 0x01;  // BIT 1 - Interrupt Enabled
			if (dev->irq_en && !old_irq)
			{
				// If bit is "turned on" generate interrupt
				floppy_interrupt(dev);
			}

			if ((gA >> 3) & 0x01) {  // BIT 3 - Test mode
				dev->test_mode = 1;
			}
			if ((gA >> 4) & 0x01) {		/* Device clear */
				//printf("Device clear\r\n");
				//dev->unit_select = -1;
				dev->bufptr = 0;
				dev->ready_for_transfer=1;
				
				// Clear errors
				dev->drive_not_rdy=0;
				dev->write_protect=0;
				dev->missing=0;
			}
			if ((gA >> 5) & 0x01) {		/* Clear Interface buffer address */
				//printf("Clear interface buffer\r\n");
				dev->bufptr = 0;
				dev->ready_for_transfer=1;
			}
			if (gA & 0xff00) {
				dev->busy= 1;
				int tmp = ((gA & 0xff00 ) >>8);
				for (i = 0; i < 8; i++)
				{
					if ((tmp & (1 << i)) != 0)
					{
						dev->command = i;
						trigger_floppy_thread = 1;
						break;
					}
				}			
			}	
			
			if (debug) fprintf(debugfile,"Floppy_IO: IOX %o WCWD - A=%04x\n",ioadd,gA);
			break;

		case 4: /* IOX RSR2 - Read status register No. 2 */
			gA = 0; /* Put A in consistent state */
			gA |= (dev->drive_not_rdy) ? (1<<8) : 0;	/* drive not ready (bit 8)*/
			gA |= (dev->write_protect) ? (1<<9) : 0;	/* set if trying to write to write protected diskette (file) (bit 9) */
			gA |= (dev->missing) ? (1<<11) : 0;		/* sector missing  (bit 11) */

			if (debug) fprintf(debugfile,"Floppy_IO: IOX %o RSR2 - A=%04x\n",ioadd,gA);

			break;

		case 5: /* IOX WDAD - Write Drive Address/ Write Difference */
			if (gA & 0x1) 
			{ 
				// Bit 0=1 => Write Drive Address

				if (debug) fprintf(debugfile,"IOX 1565 - Write Drive Address...\n");

				// Bit 1-2
				//loadDriveAddress = (value>>1) & 0b11; // This instruction selects drive and format. But what does it really do ?

				// Bit 8-9, Selected drive
				dev->unit_select = ((gA >> 8) & 0x03);				
				
				// Bit 14-15, Selected format
				dev->selected_format = ((gA >> 14) & 0x03);			

				if (gA & 1<<11) // Clear selected drive
					dev->unit_select = -1;					
				
				switch (dev->selected_format)
				{
					case 0:
					case 1:
						// IBM 3740
						dev->bytes_pr_sector = 128;
						dev->sectors_pr_track = 26;
						break;
					case 2:
						// IBM 3600
						dev->bytes_pr_sector = 256;
						dev->sectors_pr_track = 15;
						break;
					case 3:
						// IBM System 32-II
						dev->bytes_pr_sector = 512;
						dev->sectors_pr_track = 8;
					break;
				}

			} 
			else
			{ 
				// Bit 0=0 => Write difference
				if (debug) fprintf(debugfile,"IOX 1565 - Write Difference...\n");				

				if(dev->unit_select != -1) {
					int diff_track = (gA >> 8) & 0x7f; // 7 bits difference setting
					int dir_track = (gA >> 15 ) & 0x01; // 1 bit direction flag
					
					//printf("Track WAS %d\r\n", dev->track);

					// Auto seek to new track
					if (dir_track)
					{
						//printf("Moving IN +%d tracks ", diff_track);
						dev->track += diff_track;
					}
					else
					{
						//printf("Moving OUT -%d tracks ", diff_track);
						dev->track -= diff_track;
					}

					if (dev->track <0) dev->track = 0;
					if (dev->track>76) dev->track = 76;

					//printf("Track is now %d\r\n", dev->track);
				}
			}

			break;
		case 6: /* Read Test */
			/*
					 This instruction is used for simulation of data transfer between the Floppy Disk Systern and N-10 intnterface.
					 It does not transfer data from the N-10 interface ro the A register, but puts one 8 bit byte into the interface buffer, each time the instructioni s executed.
					 The bytes aree packed to 16 bit words in the buffer and may later be read by IOX RDAT instruction.
					 The byte may be chosen by using the IOX WSCT instruction (see description of IOX WSCT).

					IOX RTST is used for test purposes oniy and does not generate interrupt and busy signals.
					The insrruction is oniy active when the interface is set in test mode by the foilowiny instruction:

						SAA 10
						IOX WCWD
			*/
			if (dev->test_mode)
			{
				if (dev->bufptr_msb) {
					dev->buff[dev->bufptr] = (dev->buff[dev->bufptr] & 0xff00) | ((dev->test_byte) & 0x00ff);				
					dev->bufptr = (dev->bufptr +1) & FDD_BUFFER_MAX;
					//Reset flip-flop
					dev->bufptr_msb = 0;				

				} else {
					dev->buff[dev->bufptr] = (dev->buff[dev->bufptr] & 0x00ff) | ((dev->test_byte << 8) & 0xff00);
					dev->bufptr_msb = 1;
				}
			}	
			break;

		case 7: /*IOX WSCT - Write Sector/ Write Test Byte */		
			if (dev->test_mode) 
			{
				dev->test_byte = (gA >>8) & 0xff;
			}
			else
			{
				// Bit 8-14 Sector to be used in subsequent Read/write command
													  // Sector ranges (octal)
													  // 1-32 for UBM 3740
													  // 1-17 for IBM 3600
													  // 1-10 for IBM System 32I1
													  // Sector 0 must not be used
				dev->sector = (gA >> 8) & 0x7f;

				// If this bit is true the sector regiser is automatically incremented after each read/write command.
				// NB! This autoicrment is not valid pas the last sector of a track
				dev->sector_autoinc = (gA>>15) & 0x01;
				
			}

		break;
	}

	//if (a!=0)
	//	if (!(a & 0x01)) printf("Floppy READ %d  (%X)\r\n",a, (gA & 0xFFFF));


	if (sem_post(&sem_io) == -1) { /* release io lock */
		if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Floppy_IO\n");
		CurrentCPURunMode = SHUTDOWN;
	}


	if (trigger_floppy_thread )
	{
		/* TRIGGER FLOPPY THREAD */
		if (sem_post(&sem_floppy) == -1) 
		{ /* release floppy lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_floppy failure Floppy_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}
	}
}



/*
 * TODO:: Not having figured out exactly what these are supposed to do.
 * MIGHT be described in Sintran III Reference Manual...
 * So just add functionality here for now to get further in testing programs
 * by guessing what they want :)
 */
void Parity_Mem_IO(ushort ioadd) {
	switch(ioadd) {
	case 04: /* Read */
		break;
	case 05: /* Write */
		break;
	case 06: /* Read */
		gA = 0x0008; /* test program seems to look for this */
		break;
	case 07: /* Write */
		break;
	}
}

/*
 * Read and write from/to floppy
 */
void HDD_10MB_IO(ushort ioadd) {
	int reladd = (int)(ioadd & 0x07); /* just get lowest three bits, to work with both disk system I and II */
	switch(reladd) {
	case 0: /* Read Memory Address */
		break;
	case 1: /* Load Memory Address */
		break;
	case 2: /* Read Sector Counter */
		break;
	case 3: /* Load Block Address */
		break;
	case 4: /* Read Status Register */
		break;
	case 5: /* Load Control Word */
		break;
	case 6: /* Read Block Address */
		break;
	case 7: /* Load Word Counter Register */
		break;
	}
	return;
}

/*
 * mopc function to scan for an available char
 * returns nonzero if char was available and the char
 * in the address pointed to by chptr.
 */
int mopc_in(char *chptr) {
	int s;
	unsigned char cp,pp,ptr; /* ringbuffer pointers */
	ushort status;

	if (debug) fprintf(debugfile,"(##) mopc_in...\n");
	if (debug) fflush(debugfile);

	if(!(tty_arr[0]))	/* array dont exists, so no chars available */
		return(0);

	cp = tty_arr[0]->rcv_cp;
	pp = tty_arr[0]->rcv_fp;
	status = tty_arr[0]->in_status;
	if (sem_post(&sem_io) == -1) { /* release io lock */
		if (debug) fprintf(debugfile,"ERROR!!! sem_post failure mopc_in\n");
		CurrentCPURunMode = SHUTDOWN;
	}

	if (!(status & 0x0008))	/* Bit 3=0 device not ready for transfer */
		return(0);

	if (debug) fprintf(debugfile,"(##) mopc_in looking for char...\n");
	if (debug) fflush(debugfile);

	if (pp != cp) {	/* ok we have some data here */
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		ptr = tty_arr[0]->rcv_cp;
		*chptr =  (tty_arr[0]->rcv_arr[ptr] & 0x7f);
		tty_arr[0]->rcv_cp +=1;
		if (tty_arr[0]->rcv_fp == tty_arr[0]->rcv_cp) {
			tty_arr[0]->in_status &= ~0x0008;	/* Bit 3=0 device not ready for transfer */
		}
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure mopc_in\n");
			CurrentCPURunMode = SHUTDOWN;
		}

		if (debug) fprintf(debugfile,"(##) mopc_in data found...\n");
		if (debug) fflush(debugfile);

		return(1);
	} else {
		if (debug) fprintf(debugfile,"(##) mopc_in data not found...\n");
		if (debug) fflush(debugfile);
		return(0);
	}
}

/*
 * mopc function to output a char ch.
 */
void mopc_out(char ch) {
	int s;
	unsigned char ptr; /* ringbuffer pointer */

	if (debug) fprintf(debugfile,"(##) mopc_out...\n");
	if (debug) fflush(debugfile);

	if(tty_arr[0]){ /* array exists so we can work with this now */
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		ptr=tty_arr[0]->snd_fp;
		tty_arr[0]->snd_arr[ptr]= ch;
		tty_arr[0]->snd_fp++;
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure mopc_out\n");
			CurrentCPURunMode = SHUTDOWN;
		}

		if (sem_post(&sem_cons) == -1) { /* release console lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure mopc_out\n");
			CurrentCPURunMode = SHUTDOWN;
		}
	}
}

/* Block defining what byte has odd parity or not */
bool oddParity[256] =
{
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
	0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
};

/*
 * Read and write to system console
 */
void Console_IO(ushort ioadd) {
	int s;
	unsigned char ptr; /* ringbuffer pointer */
	switch(ioadd) {
	case 0300: /* Read input data */
		if (!tty_arr[0]){ /* tty structure not created yet... return zero(safety function) */
			gA=0;
			return;
		}
		if (MODE_OPCOM) { /* mopc has authority */
			return;
		}
		if(tty_arr[0]) { /* array exists so we can work with this now */
			while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
				continue; /* Restart if interrupted by handler */
			if (tty_arr[0]->rcv_fp != tty_arr[0]->rcv_cp) { /* ok we have some data here */
				ptr = tty_arr[0]->rcv_cp;
				gA =  (tty_arr[0]->rcv_arr[ptr] & 0x00ff);

				if (oddParity[gA]==1)
					gA |= (1<<7); // MACM requires that the parity is EVEN

				tty_arr[0]->rcv_cp++;
				if (tty_arr[0]->rcv_fp == tty_arr[0]->rcv_cp) {
					tty_arr[0]->in_status &= ~0x0008; /* Bit 3=0 device not ready for transfer */
				}
			} else {
				gA=0;
			}
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}
		}
		break;
	case 0301: /* NOOP*/
		break;
	case 0302: /* Read input status */
		if (!tty_arr[0]){ /* tty structure not created yet... return zero (safety function) */
			gA=0;
			return;
		}
		if (MODE_OPCOM) { /* mopc has authority */
			while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
				continue; /* Restart if interrupted by handler */
			gA =tty_arr[0]->in_status & ~0x00008;	/* Bit 3=0 device not ready for transfer */
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}
			return;
		}
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		gA =tty_arr[0]->in_status;
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}
		break;
	case 0303: /* Set input control */
		if (!tty_arr[0]) return; /* tty structure not created yet... return zero (safety function) */
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		tty_arr[0]->in_control = gA; /* sets control reg all flags */
		if (gA & 0x0004) {	/* activate device */
			tty_arr[0]->in_status |= 0x0004;
		} else {		/* deactivate device */
			tty_arr[0]->in_status &= ~0x0004;
		}
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}
		break;
	case 0304: /* Returns 0 in A */
		gA = 0;
		break;
	case 0305: /* Write data */
		if(tty_arr[0]){ /* array exists so we can work with this now */
			while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
				continue; /* Restart if interrupted by handler */
			ptr=tty_arr[0]->snd_fp;
			tty_arr[0]->snd_arr[ptr]=gA & 0x007F;
			tty_arr[0]->snd_fp++;
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}

			if (sem_post(&sem_cons) == -1) { /* release console lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}
		}
		break;
	case 0306: /* Read output status */
		if (!tty_arr[0]){ /* tty structure not created yet... return zero (safety function) */
			gA=0;
			return;
		}
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		gA=tty_arr[0]->out_status;
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Console_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}
		break;
	case 0307: /* Set output control */
		if (!tty_arr[0]) return; /* tty structure not created yet... return zero (safety function) */
		break;
	}
}

void IO_Handler_Add(int startdev, int stopdev, void *funcpointer, void *datapointer) {
	int i;
	for(i=startdev;i<=stopdev;i++)
		ioarr[i] = funcpointer;
	return;
}

void IO_Data_Add(int startdev, int stopdev,  void * datastructptr) {
	int i;
	for(i=startdev;i<=stopdev;i++)
		iodata[i] = datastructptr;
	return;
}


/* Add IO handler addresses in this function */
void Setup_IO_Handlers () {
	int i;

	/* to be removed, and changed to new structure */
	for(i=0; i<=255;i++){
		tty_arr[i] = 0;
	}

	IO_Handler_Add(0,65535,&Default_IO,NULL);		/* Default setup to dummy routine */
	IO_Data_Add(0,65535,NULL);				/* Make sure all data pointers are NULL */
	IO_Handler_Add(4,7,&Parity_Mem_IO,NULL);		/* Parity Memory something, 4-7 octal */
	IO_Handler_Add(8,11,&RTC_IO,NULL);			/* CPU RTC 10-13 octal */
	IO_Handler_Add(192,199,&Console_IO,NULL);		/* Console terminal 300-307 octal */
	IO_Handler_Add(880,887,&Floppy_IO,NULL);		/* Floppy Disk 1 at 1560-1567 octal */	
	IO_Handler_Add(320,327,&HDD_10MB_IO,NULL);		/* Disk System I at 500-507 octal. , ident 01, interrupt 11*/	
	floppy_init(); // removed init here, asits initialized from nd100lib.c
	hawk_init();	
}

/*
 * floppy_init
 * NOTE: Not sure if this is the best way, but needed to start somewhere. Might be a totally different solution in the end.
 *
 */
void floppy_init() {
	struct floppy_data * ptr;
	struct fdd_unit * ptr2;

	ptr =calloc(1,sizeof(struct floppy_data));
	if (ptr) {
		ptr2 = calloc(1,sizeof(struct fdd_unit));
		if (ptr2) {
			ptr->unit[0] = ptr2;
			if(FDD_IMAGE_NAME) {
				ptr->unit[0]->filename = strdup(FDD_IMAGE_NAME);
				ptr->unit[0]->readonly = FDD_IMAGE_RO;
				if (FDD_IMAGE_RO && ptr->unit[0]->filename) {
					ptr->unit[0]->fp = fopen(FDD_IMAGE_NAME, "r");
				} else if(ptr->unit[0]->filename){
					ptr->unit[0]->fp = fopen(FDD_IMAGE_NAME, "r+");
				}
			}
		}
		ptr2 = calloc(1,sizeof(struct fdd_unit));
		if (ptr2) {
			ptr->unit[1] = ptr2;
		}
		ptr2 = calloc(1,sizeof(struct fdd_unit));
		if (ptr2) {
			ptr->unit[2] = ptr2;
		}
	}	

	ptr->our_rnd_id=rand(); /* our unique identifier for not generating multiple idents on the same device, TODO: Check if needed*/

	// Set default sector size
	ptr->bytes_pr_sector = 256;
	ptr->sectors_pr_track = 15;

	// clear registers
	ptr->bufptr =0;	
	ptr->ready_for_transfer = 1;
	ptr->test_mode = 0;
	ptr->rw_complete=0;
	ptr->seek_complete = 0;
	ptr->record_deleted=0;	

	IO_Data_Add(880,887,ptr);
}


void hawk_init()
{
	struct hawk_data *ptr;
	struct hawk_unit *ptr2;
	
	ptr =calloc(1,sizeof(struct hawk_data));
	if (ptr) {
		ptr->our_rnd_id=rand(); /* our unique identifier for not generating multiple idents on the same device, TODO: Check if needed*/

		ptr2 = calloc(1,sizeof(struct hawk_unit));
		if (ptr2) {
			ptr->unit[0] = ptr2;
			if(HAWK_IMAGE_NAME) {
				ptr->unit[0]->filename = strdup(HAWK_IMAGE_NAME);
				
				/*
				ptr->unit[0]->readonly = HAWK_IMAGE_RO;
				if (HAWK_IMAGE_RO && ptr->unit[0]->filename) {
					ptr->unit[0]->fp = fopen(HAWK_IMAGE_NAME, "r");
				} else if(ptr->unit[0]->filename){
					ptr->unit[0]->fp = fopen(_IMAGE_NAME, "r+");
				}
				*/
			}
		}
		ptr2 = calloc(1,sizeof(struct hawk_unit));
		if (ptr2) {
			ptr->unit[1] = ptr2;
		}
		ptr2 = calloc(1,sizeof(struct hawk_unit));
		if (ptr2) {
			ptr->unit[2] = ptr2;
		}
	}	
	IO_Data_Add(320,327,ptr);
}

/*
 * Here we do the stuff to setup a socket and listen to it.
 * The rest is supposed to be done in the function calling it,
 * like starting up threads for accepting connections etc.
 */
void do_listen(int port, int numconn, int * sock) {

	int istrue = 1;
	struct sockaddr_in server_addr;
	char errbuff[256];

	if ((*sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		if( strerror_r( errno, errbuff, 256 ) == 0 ) {
			if (debug) fprintf(debugfile,"(#)SOCKET error -- %s\n",errbuff);
		}
		CurrentCPURunMode = SHUTDOWN;
		return;
	}
	if (setsockopt(*sock,SOL_SOCKET,SO_REUSEADDR,&istrue,sizeof(int)) == -1) {
		if( strerror_r( errno, errbuff, 256 ) == 0 ) {
			if (debug) fprintf(debugfile,"(#)SOCKET error -- %s\n",errbuff);
		}
		CurrentCPURunMode = SHUTDOWN;
		return;
	}
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);
	server_addr.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr.sin_zero),8);
	if (bind(*sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
		if( strerror_r( errno, errbuff, 256 ) == 0 ) {
			if (debug) fprintf(debugfile,"(#)SOCKET bind error -- %s\n",errbuff);
		}
		CurrentCPURunMode = SHUTDOWN;
		return;
	}
	if (listen(*sock, numconn) == -1) {
		if( strerror_r( errno, errbuff, 256 ) == 0 ) {
			if (debug) fprintf(debugfile,"(#)SOCKET listen error -- %s\n",errbuff);
		}
		CurrentCPURunMode = SHUTDOWN;
		return;
	}
}

void console_stdio_in() {
	int s;
	char ch,parity;
	char recv_data[1024];
	int numbytes;
	unsigned char pp,cp,tmp,cnt,cnt2;
	int numread;
	ushort status,control;
	fd_set rfds;
	int retval;

	parity=0;

	if (debug) fprintf(debugfile,"(#)console_stdio_in running...\n");
	if (debug) fflush(debugfile);

	/* Watch stdin (fd 0) to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(0, &rfds);

	while(CurrentCPURunMode != SHUTDOWN) {

		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		status = tty_arr[0]->in_status;
		control = tty_arr[0]->in_control;
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
			CurrentCPURunMode = SHUTDOWN;
		}

		if (status & 0x0004) { /* Bit 2=1 device is active */
			while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
				continue; /* Restart if interrupted by handler */
			pp=tty_arr[0]->rcv_fp;
			cp=tty_arr[0]->rcv_cp;
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
				CurrentCPURunMode = SHUTDOWN;
			}

			/* this gets us max buffer size we can use */
			numbytes = (cp < pp) ? 256-pp+cp-2 : (pp < cp) ? cp-pp-2 : 254;
			numbytes = ( numbytes > 0) ? numbytes : 0;

			retval = select(1, &rfds, NULL, NULL, NULL);

			/* ok lets send what we got... */
			numread = read (0, recv_data, numbytes);
			tmp = numread;
			if (numread > 0) {
				tmp = numread;
				for (cnt=0;tmp>0; tmp--){
					ch=recv_data[cnt];

					if (debug) fprintf (debugfile,"(##) ch=%i (%c)\n",ch,ch);
					if (debug) fflush(debugfile);

					ch = (ch == 10) ? 13 : ch; /* change lf to cr */
					switch((control & 0x1800)>>11){
					case 0:/* 8 bits */
						break;
					case 1:/* 7 bits */
						ch &= 0x7f;
						break;
					case 2:/* 6 bits */
						ch &= 0x3f;
						break;
					case 3:/* 5 bits */
						ch &= 0x1f;
						break;
					}
					if(control & 0x4000) {	/* Bit 14=1 even parity is used */
						/* set parity to 0 for even parity or 1 for odd parity  */
						parity=0;
						for (cnt2 = 0; cnt2 < 8; cnt2++)
							parity ^= ((ch >> cnt2) & 1);
						ch = (parity) ? ch | 0x80 : ch;
					}

					if (debug) fprintf (debugfile,"(##) ch=%i (%c) parity=%i statusreg(bits)=%d\n",ch,ch,parity,(int)((control & 0x1800)>>11));
					if (debug) fflush(debugfile);

//					if (ch == 10) { /* lf?? */
//						cnt++;
//					} else {
						while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
							continue; /* Restart if interrupted by handler */
						pp=tty_arr[0]->rcv_fp;
						tty_arr[0]->rcv_arr[pp]=ch;
						tty_arr[0]->rcv_fp++;
						if (sem_post(&sem_io) == -1) { /* release io lock */
							if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
							CurrentCPURunMode = SHUTDOWN;
						}

						cnt++;
						pp++;
//					}
				}
				while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
					continue; /* Restart if interrupted by handler */
				tty_arr[0]->in_status |= 0x0008;	/* Bit 3=1 device ready for transfer */
				if (sem_post(&sem_io) == -1) { /* release io lock */
					if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
					CurrentCPURunMode = SHUTDOWN;
				}
			}
		}
	}
}

void console_stdio_thread() {
	int s;
//	char ch;
	int numbytes;
	unsigned char pp,cp,tmp;

	struct ThreadChain *tc_elem;

	if (debug) fprintf(debugfile,"(#)console_stdio_thread running...\n");
	if (debug) fflush(debugfile);
	tty_arr[0] = calloc(1,sizeof(struct tty_io_data));

	tc_elem=AddThreadChain();
	pthread_attr_init(&tc_elem->tattr);
	pthread_attr_setdetachstate(&tc_elem->tattr,PTHREAD_CREATE_DETACHED);
	tc_elem->tk = CANCEL;
	pthread_create(&tc_elem->thread, &tc_elem->tattr, (void *)&console_stdio_in, NULL);

	while(CurrentCPURunMode != SHUTDOWN) {
		tty_arr[0]->out_status |= 0x0008; /* Bit 3=1 ready for transfer */
		while(CurrentCPURunMode != SHUTDOWN) {
			while ((s = sem_wait(&sem_cons)) == -1 && errno == EINTR) /* wait for console lock to be free */
				continue; /* Restart if interrupted by handler */

			/* ok lets send what we got... */
			pp=tty_arr[0]->snd_fp;
			cp=tty_arr[0]->snd_cp;
			numbytes=0;
			if (cp != pp) {
				numbytes= (cp < pp) ? pp-cp : 256-cp+pp;
				for(tmp=0;tmp<numbytes;tmp++) {
					printf("%c",tty_arr[0]->snd_arr[cp]);
//					switch (ch) {
//					case 12:
//						break;
//					default:
//						printf("%c",tty_arr[0]->snd_arr[cp]);
//						break;
//					}
					cp++;
				}
				tty_arr[0]->snd_cp=cp;
			}
		}
	}
	return;
}

void console_socket_in(int *connected) {
	int s;
	char ch,parity;
	char recv_data[1024];
	int numbytes;
	ushort status,control;
	unsigned char pp,cp,tmp,cnt,cnt2;
	int numread;
	fd_set rfds;
	int fdmax;
	int retval;
	int throw =0;

	parity = 0;
	fdmax = *connected;

	if (debug) fprintf(debugfile,"(#)console_socket_in running...\n");
	if (debug) fflush(debugfile);

	/* Watch socket to see when it has input. */
	FD_ZERO(&rfds);
	FD_SET(*connected, &rfds);

	while(CurrentCPURunMode != SHUTDOWN) {
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */
		status = tty_arr[0]->in_status;
		control = tty_arr[0]->in_control;
		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
			CurrentCPURunMode = SHUTDOWN;
		}
		if (status & 0x0004) { /* Bit 2=1 device is active */
			while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
				continue; /* Restart if interrupted by handler */
			pp=tty_arr[0]->rcv_fp;
			cp=tty_arr[0]->rcv_cp;
			if (sem_post(&sem_io) == -1) { /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure console_stdio_in\n");
				CurrentCPURunMode = SHUTDOWN;
			}

			/* this gets us max buffer size we can use */
			numbytes = (cp < pp) ? 256-pp+cp-2 : (pp < cp) ? cp-pp-2 : 254;
			numbytes = ( numbytes > 0) ? numbytes : 0;

			retval = select(fdmax+1, &rfds, NULL, NULL, NULL);

			/* ok lets send what we got... */

			numread = recv(*connected,recv_data,numbytes,0);
			if (numread > 0) {
				tmp = numread;
				for (cnt=0;tmp>0; tmp--){
					ch=recv_data[cnt];
					if((unsigned int)ch == 255) { /* Telnet IAC command */
						throw=2;
						cnt++;
					} else if (throw) { /* previous IAC command, throw out 2 chars after */
						throw--;
						cnt++;
					} else {
						if (debug) fprintf (debugfile,"(##) ch=%i (%c)\n",ch,ch);
						if (debug) fflush(debugfile);
//						ch = (ch == 10) ? 13 : ch; /* change lf to cr */
						ch &= 0x7f;	/* trim to 7 bit ASCII  FIXME:: This should depend on bits 11-12 in control reg*/
						if(tty_arr[0]->in_control && 0x4000){	/* Bit 14=1 even parity is used */
							/* set parity to 0 for even parity or 1 for odd parity  */
							parity=0;
							for (cnt2 = 0; cnt2 < 8; cnt2++)
								parity ^= ((ch >> cnt2) & 1);
							ch = (parity) ? ch | 0x80 : ch;
						}
						if (debug) fprintf (debugfile,"(##) ch=%i (%c) parity=%i\n",ch,ch,parity);
						if (debug) fflush(debugfile);
//						if (ch == 10) { /* lf?? */
//							cnt++;
//						} else {
							tty_arr[0]->rcv_arr[pp]=ch;
							cnt++;
							pp++;
//						}
					}
				}
				tty_arr[0]->rcv_fp=pp;
				tty_arr[0]->in_status |= 0x0008;	/* Bit 3=1 device ready for transfer */
			}
		}
	}
}

void console_socket_thread() {
	int s;
	int sock, connected;
	char send_data [1024];
	struct sockaddr_in client_addr;
	socklen_t sin_size;
	int numbytes;
	unsigned char pp,cp,tmp;

	/* IAC WILL ECHO IAC WILL SUPPRESS-GO-AHEAD IAC DO SUPPRESS-GO-AHEAD */
	char telnet_setup[9] = {0xff,0xfb,0x01,0xff,0xfb,0x03,0xff,0xfd,0x0f3};

	struct ThreadChain *tc_elem;

	if (debug) fprintf(debugfile,"(#)console_socket_thread running...\n");
	if (debug) fflush(debugfile);
	tty_arr[0] = calloc(1,sizeof(struct tty_io_data));

	do_listen(5001, 1, &sock);
	if (debug) fprintf(debugfile,"\n(#)TCPServer Waiting for client on port 5001\n");
	if (debug) fflush(debugfile);

	while(CurrentCPURunMode != SHUTDOWN) {
		sin_size = (socklen_t) sizeof(struct sockaddr_in);
		connected = accept(sock, (struct sockaddr *)&client_addr,&sin_size);
		if (debug) fprintf(debugfile,"(#)I got a console connection from (%s , %d)\n",
			inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
		if (debug) fflush(debugfile);

		/* setup the other side to "uncooked" data */
		send(connected,telnet_setup,9,0);

		tc_elem=AddThreadChain();
		pthread_attr_init(&tc_elem->tattr);
		pthread_attr_setdetachstate(&tc_elem->tattr,PTHREAD_CREATE_DETACHED);
		tc_elem->tk = CANCEL;
		pthread_create(&tc_elem->thread, &tc_elem->tattr, (void *)&console_socket_in, &connected);

		tty_arr[0]->out_status |= 0x0008; /* Bit 3=1 ready for transfer */
		while(CurrentCPURunMode != SHUTDOWN) {
			while ((s = sem_wait(&sem_cons)) == -1 && errno == EINTR) /* wait for console lock to be free */
				continue; /* Restart if interrupted by handler */

			/* ok lets send what we got... */
			pp=tty_arr[0]->snd_fp;
			cp=tty_arr[0]->snd_cp;
			numbytes=0;
			if (cp != pp) {

				numbytes= (cp < pp) ? pp-cp : 256-cp+pp;
				for(tmp=0;tmp<numbytes;tmp++) {
					send_data[tmp]=tty_arr[0]->snd_arr[cp];
					cp++;
				}
				tty_arr[0]->snd_cp=cp;
			}
			if (numbytes) send(connected,send_data,numbytes,0);
		}
	}
	close(sock);
	return;
}

/*
Floppy_IO: IOX 880 - A=0
Floppy_IO: IOX 880 - A=0
Floppy_IO: IOX 882 - A=880
Floppy_IO: IOX 883 - A=2
Floppy_IO: IOX 882 - A=10831
Floppy_IO: IOX 882 - A=2
*/
void floppy_thread()
{
	int s;
	struct floppy_data *dev;
	FILE *floppy_file;
	int position;
	ushort read_word;
	ushort tmp2;
	int readcnt ;
	
	int transfer_word_count;	
	int error;

	char loadtype[]="r+";	

	while(CurrentCPURunMode != SHUTDOWN) 
	{
		while ((s = sem_wait(&sem_floppy)) == -1 && errno == EINTR) /* wait for floppu lock to be free and take it */
			continue; /* Restart if interrupted by handler */

		/* Do our stuff now and then return and wait for next freeing of lock. */
		dev = iodata[880];	/*TODO:: This is just a temporary solution!!! */
		// Clear everything

		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */

		// Only excute logic below after main thread flagged the device as busy
		if (!dev->busy)
		{
			if (sem_post(&sem_io) == -1) 
			{ /* release io lock */
				if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Floppy_IO\n");
				CurrentCPURunMode = SHUTDOWN;
			}
			continue;;
	
		}		

		// Execute command , only drive 0 is available in this emulator code. TODO: Add support for more drives
		if (dev->unit_select!= 0)  // TODO: should be ok for unit in range 0-2
		{
			dev->drive_not_rdy = 1;
			dev->busy = 0;	
			error =-1;		
		}
		else
		{			
			// Clear flags
			dev->ready_for_transfer=0;
			dev->rw_complete=0;
			dev->seek_complete = 0;
			dev->record_deleted=0;
			dev->write_protect=0;
			
			error = 0;

			transfer_word_count = dev->bytes_pr_sector >> 1;

			

			// Validate
			if (dev->sector<=0)
				dev->sector = 1; // Force a valid sector
	
			// Position inside file
			position = ((dev->sector-1)*dev->bytes_pr_sector) + (dev->track * dev->bytes_pr_sector * dev->sectors_pr_track);
	

			//printf("FloppyPIO: Executing command %d on drive %d. Position %d Sector %d Track %d [%d]\r\n",dev->command, dev->unit_select, position, dev->sector, dev->track, dev->bufptr );

			if (debug) fprintf(debugfile,"FloppyPIO: Executing command %d on drive %d. Position %d Sector %d Track %d [%d]\r\n",dev->command, dev->unit_select, position, dev->sector, dev->track, dev->bufptr );

			// Safely open image and seek			
			if (dev->unit[dev->unit_select]->filename != NULL)
			{
				//printf("Opening %s\r\n", dev->unit[dev->unit_select]->filename);
				floppy_file=fopen(dev->unit[dev->unit_select]->filename,loadtype);	

				if (floppy_file==NULL)
				{
					if (debug) fprintf(debugfile,"File open failed. File '%s' errno %d\r\n", dev->unit[dev->unit_select]->filename, errno);
					error++;
				}
				else
				{
					if (fseek(floppy_file,position,SEEK_SET) != 0)					
						error ++;
				}
			}
			else
				error++;
			

			if (error)
			{
				dev->missing=1;
				dev->busy=0;
			}
			else
			{				
				switch(dev->command)
				{
					case 0: // FormatTrack
							
							break;
					case 1: // WriteData
							break;
			
					case 2: // WriteDeletedData
							break;
			
					case 3: // ReadID
							/*
							if (SectorIsDeleted(dev->sector, dev->track))
							{
								dev->buff[0] = 0xFF00;
								dev->buff[1] = 0xFF01;

							}
							else
							*/
							{
								dev->buff[0] = (ushort)(dev->track<<8) |0x00;
								dev->buff[1] = (ushort)(dev->sector<<8)|0x01;
							}
							dev->buff[2] = 0x02;
							dev->buff[3] = 0x03;
							break;
			
					case 4: // ReadData						
							//printf("WC=%d",transfer_word_count)	;
							while (transfer_word_count > 0)
							{
								readcnt = fread(&read_word,1,2,floppy_file);
								if (readcnt <=0)
								{
									dev->command = -1;
									dev->missing=1;
									dev->busy=0;
									break; // Exit while loop									
								}
								else
								{								
									// Swap HI/LO to make endian correct (ND is BIG ENDIAN)
									tmp2=(read_word & 0xff00)>>8;
									tmp2= tmp2 | ((read_word & 0x00ff) << 8);								

									// put value in floppy buffer
									dev->buff[dev->bufptr] = (ushort)tmp2;									
									dev->bufptr = (dev->bufptr + 1) & FDD_BUFFER_MAX;

									transfer_word_count--;
								}
							}
							break;
			
					case 5: // Seek
							if (dev->sector <=0)
							{
								dev->missing = 1;
							}
							break;
			
					case 6: // Recalibrate,
							dev->sector = 1;
							dev->track=0;
							//printf("Track is now %d\r\n", dev->track);
							break;
			
					case 7: // ControlReset
							break;		

				}
				if (floppy_file != NULL)
				{
					fclose(floppy_file);
					floppy_file = NULL;
				}				
			}			
		}	

		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Floppy_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}

		// Must run AFTER io-lock has been released to avoid race conditions
		if (error == 0 ) floppy_command_end(dev);

	}
}

void floppy_command_end(struct floppy_data *dev)
{
	mysleep(0,10000); // 10.000 us =>10 ms // Simulate that the drive is doing some IO
	dev->busy = 0;
	dev->ready_for_transfer =1;
	switch(dev->command)
	{
		case 4: // Read
			// Auto increment ?
			if (dev->sector_autoinc)
			{
				if (dev->sector <= dev->sectors_pr_track)
					dev->sector++;			
			}
			dev->rw_complete=1;	
			break;
		case 5: // seek
			dev->seek_complete = 1;
			break;
		case 6: // Recalibrate
			dev->seek_complete = 1;
			break;
		default:
			dev->rw_complete=1;	
			break;
	}
	
	if (dev->sector_autoinc)
	{
		if (dev->sector < dev->sectors_pr_track)
		{
			dev->sector++;
		}
	}

	if (dev->irq_en)
	{		
		floppy_interrupt(dev);
	}
}

void floppy_interrupt(struct floppy_data *dev)
{
	int s;
	int ident = 0x11; // octal 21
	

	if(CurrentCPURunMode != STOP) 
	{
		while ((s = sem_wait(&sem_int)) == -1 && errno == EINTR) /* wait for interrupt lock to be free */
				continue;       /* Restart if interrupted by handler */
		
		gPID |= 0x800; /* Bit 11 */				
		// interrupt(11,0); <= hangs
		AddIdentChain(11,ident,dev->our_rnd_id); /* Add interrupt to ident (o21) chain, lvl13, ident code 1, and identify us */
		
		if (sem_post(&sem_int) == -1) 
		{ 
			/* release interrupt lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure DOMCL\n");
			CurrentCPURunMode = SHUTDOWN;
		}
		checkPK();
	}
}

void panel_thread() {
	int s;
	int sock, connected, bytes_recieved;
	char recv_data[1024];
	struct sockaddr_in client_addr;
	socklen_t sin_size;

	if (debug) fprintf(debugfile,"(#)panel_thread running...\n");
	if (debug) fflush(debugfile);

	do_listen(5000, 1, &sock);
	if (debug) fprintf(debugfile,"\n(#)TCPServer Waiting for client on port 5000\n");
	if (debug) fflush(debugfile);

	while(CurrentCPURunMode != SHUTDOWN) {
		sin_size = (socklen_t) sizeof(struct sockaddr_in);
		connected = accept(sock, (struct sockaddr *)&client_addr,&sin_size);
		if (debug) fprintf(debugfile,"(#)I got a panel connection from (%s , %d)\n",
			inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
		if (debug) fflush(debugfile);
		while(CurrentCPURunMode != SHUTDOWN) {
			bytes_recieved = recv(connected,recv_data,1024,0);
			recv_data[bytes_recieved] = '\0';
			if (debug) fprintf(debugfile,"(#)PANEL DATA received\n");
			if(strncmp("OPCOM_PRESSED\n",recv_data,strlen("OPCOM_PRESSED"))==0){
				MODE_OPCOM=1;
				if (debug) fprintf(debugfile,"(#)OPCOM_PRESSED\n");

			} else if(strncmp("MCL_PRESSED\n",recv_data,strlen("MCL_PRESSED"))==0){
				if (debug) fprintf(debugfile,"(#)MCL_PRESSED\n");
				/* TODO:: this should be in a separate routine DoMCL later */
				CurrentCPURunMode = STOP;
				/* NOTE:: buggy in that we cannot do STOP and MCL without a running cpu between.. FIXME */
				while ((s = sem_wait(&sem_stop)) == -1 && errno == EINTR) /* wait for stop lock to be free and take it */
					continue; /* Restart if interrupted by handler */
				bzero(gReg,sizeof(struct CpuRegs));	/* clear cpu */
				setbit(_STS,_O,1);
				setbit_STS_MSB(_N100,1);
				gCSR = 1<<2;    /* this bit sets the cache as not available */

			} else if(strncmp("LOAD_PRESSED\n",recv_data,strlen("LOAD_PRESSED"))==0){
				if (debug) fprintf(debugfile,"(#)LOAD_PRESSED\n");
				gPC=STARTADDR;
				CurrentCPURunMode = RUN;
				if (sem_post(&sem_run) == -1) { /* release run lock */
					if (debug) fprintf(debugfile,"ERROR!!! sem_post failure panel_thread\n");
					CurrentCPURunMode = SHUTDOWN;
				}
			} else if(strncmp("STOP_PRESSED\n",recv_data,strlen("STOP_PRESSED"))==0){
				if (debug) fprintf(debugfile,"(#)STOP_PRESSED\n");
				CurrentCPURunMode = STOP;
				/* NOTE:: buggy in that we cannot do STOP and MCL without a running cpu between.. FIXME */
				while ((s = sem_wait(&sem_stop)) == -1 && errno == EINTR) /* wait for stop lock to be free and take it */
					continue; /* Restart if interrupted by handler */
			} else {
				if (debug) fprintf(debugfile,"(#)Panel received:%s\n",recv_data);
			}
			if (debug) fflush(debugfile);
		}
	}
	close(sock);
	return;
}

void setup_pap(){
	gPANS=0x8000;	/* Tell system we are here */
	gPANS=gPANS | 0x4000;	/* Set FULL which is active low, so not full */
	gPAP = calloc(1,sizeof(struct display_panel));
}

void panel_event(){
	char tmpbyte;

	if (gPAP->trr_panc) {	/* TRR has been issued, process command */
		if (debug) fprintf(debugfile,"panel_event: TRR\n");
		if (debug) fflush(debugfile);
		gPAP->trr_panc = false;
		switch ((gPANC & 0x0700)>>8) {
		case 0:		/* Illegal */
			break;
		case 1:		/* Future extension */
			break;
		case 2:		/* Message Append */	// TODO: Not Implemented yet except basic return info
			gPANS = 0xd200;
			break;
		case 3:		/* Message Control */	// TODO: Not Implemented yet except basic return info
			gPANS = 0xd300;
			break;
		case 4:		/* Update Low Seconds */
			if (gPANC & 0x2000){	/* Read */
				tmpbyte = (gPAP->seconds) & 0x00ff;
				gPANS = 0xf400 | tmpbyte;
			} else {		/*Write */
				tmpbyte = gPANC & 0x00ff;
				gPAP->seconds = (gPAP->seconds & 0xff00) | tmpbyte;
				gPANS = 0xd400;
			}
			break;
		case 5:		/* Update High Seconds */
			if (gPANC & 0x2000){	/* Read */
				tmpbyte = (gPAP->seconds >> 8);
				gPANS = 0xf500 | tmpbyte;
			} else {		/*Write */
				tmpbyte = gPANC & 0x00ff;
				gPAP->seconds = (gPAP->seconds & 0x00ff) | ((ushort)tmpbyte)<<8;
				gPANS = 0xd500;
			}
			break;
		case 6:		/* Update Low Days */
			if (gPANC & 0x2000){	/* Read */
				tmpbyte = (gPAP->days) & 0x00ff;
				gPANS = 0xf600 | tmpbyte;
			} else {		/*Write */
				tmpbyte = gPANC & 0x00ff;
				gPAP->days = (gPAP->days & 0xff00) | tmpbyte;
				gPANS = 0xd600;
			}
			break;
		case 7:		/* Update High Days */
			if (gPANC & 0x2000){	/* Read */
				tmpbyte = (gPAP->days >> 8);
				gPANS = 0xf700 | tmpbyte;
			} else {		/*Write */
				tmpbyte = gPANC & 0x00ff;
				gPAP->days = (gPAP->days & 0x00ff) | ((ushort)tmpbyte)<<8;
				gPANS = 0xd700;
			}
			break;
		default :	/* This should never happen */
			break;
		}
		if (debug) fprintf(debugfile,"panel_event: TRR - result: gPANS = %0x04\n",gPANS);
		if (debug) fflush(debugfile);
	}
	if (gPAP->sec_tick) {	/* Seconds tick from rtc, update counters */
		if (debug) fprintf(debugfile,"panel_event: 1 second tick\n");
		if (debug) fflush(debugfile);
		gPAP->sec_tick = false;
		gPAP->seconds++;
		if (gPAP->seconds >= 43200){	/* 12h wraparound */
			gPAP->seconds = 0;
			gPAP->days++;
		}
	}

}

void panel_processor_thread() {
	int s;
	while(CurrentCPURunMode != SHUTDOWN) {
		while ((s = sem_wait(&sem_pap)) == -1 && errno == EINTR) /* wait for pap 'kick' */
			continue; /* Restart if interrupted by handler */

		panel_event();
	}
	return;
}

void hawk_IO(ushort ioadd) 
{
	bool trigger_hawk_thread = 0;	
	struct hawk_data *dev = iodata[ioadd];
	if (debug) fprintf(debugfile,"HAWK_IO: IOX %d - A=%d\n",ioadd,gA);
	fflush(debugfile);

	int reladd = (int)(ioadd & 0x07); /* just get lowest three bits, to work with both disk system I and II */
	switch(reladd) {
	case 0: /* Read Memory Address */
		gA = dev->coreAddress;
		break;
	case 1: /* Load Memory Address */
		break;
	case 2: /* Read Sector Counter */
		break;
	case 3: /* Load Block Address */
		break;
	case 4: /* Read Status Register */
		break;
	case 5: /* Load Control Word */
		break;
	case 6: /* Read Block Address */
		break;
	case 7: /* Load Word Counter Register */
		break;
	}
	return;
}


void hawk_thread()
{
	int s;
	struct floppy_data *dev;
	FILE *hawk_file;
	int position;

	// INIT vars


	while(CurrentCPURunMode != SHUTDOWN) 
	{
		while ((s = sem_wait(&sem_hawk)) == -1 && errno == EINTR) /* wait for floppu lock to be free and take it */
			continue; /* Restart if interrupted by handler */

		/* Do our stuff now and then return and wait for next freeing of lock. */
		dev = iodata[880];	/*TODO:: This is just a temporary solution!!! */
		// Clear everything

		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */


		//switch() ?
		// MAIN LOOP

		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Floppy_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}
	}
}

void hawk_command_end(struct hawk_data *dev)
{
	//
}

void hawk_interrupt(struct hawk_data *dev)
{
    int s;
	int ident = 0x11; // octal 21
	

	if(CurrentCPURunMode != STOP) 
	{
		while ((s = sem_wait(&sem_int)) == -1 && errno == EINTR) /* wait for interrupt lock to be free */
				continue;       /* Restart if interrupted by handler */
		
		gPID |= 0x800; /* Bit 11 */				
		// interrupt(11,0); <= hangs
		AddIdentChain(11,ident,dev->our_rnd_id); /* Add interrupt to ident (o21) chain, lvl13, ident code 1, and identify us */
		
		if (sem_post(&sem_int) == -1) 
		{ 
			/* release interrupt lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure DOMCL\n");
			CurrentCPURunMode = SHUTDOWN;
		}
		checkPK();
	}
}
