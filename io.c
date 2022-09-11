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

// Variables used for the IO Tick routine
int tick_hawk_end;
int tick_hawk_error;
int tick_bigdisk_end;
int tick_bigdisk_error;


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
	//if (debug) fprintf(debugfile,"Floppy_IO: IOX %d - A=%d\n",ioadd,gA);
	//fflush(debugfile);
	

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

			//if (debug) fprintf(debugfile,"Floppy_IO: IOX %o RSR1 - A=%04x\n",ioadd,gA);
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
			
			//if (debug) fprintf(debugfile,"Floppy_IO: IOX %o WCWD - A=%04x\n",ioadd,gA);
			break;

		case 4: /* IOX RSR2 - Read status register No. 2 */
			gA = 0; /* Put A in consistent state */
			gA |= (dev->drive_not_rdy) ? (1<<8) : 0;	/* drive not ready (bit 8)*/
			gA |= (dev->write_protect) ? (1<<9) : 0;	/* set if trying to write to write protected diskette (file) (bit 9) */
			gA |= (dev->missing) ? (1<<11) : 0;		/* sector missing  (bit 11) */

			//if (debug) fprintf(debugfile,"Floppy_IO: IOX %o RSR2 - A=%04x\n",ioadd,gA);

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
	IO_Handler_Add(320,327,&hawk_IO,NULL);		/* Disk System I at 500-507 octal. , ident 01, interrupt 11*/	
	IO_Handler_Add(864,871,&bigdisk_IO,NULL);		/* BigDisk Disk System I at 1540-1547 octal. , ident o17 (d15), interrupt 11*/	
	floppy_init(); // removed init here, asits initialized from nd100lib.c
	hawk_init();	
	bigdisk_init();
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


void bigdisk_init()
{
	struct bigdisk_data *ptr;
	struct bigdisk_unit *ptr2;
	
	ptr =calloc(1,sizeof(struct  bigdisk_data));
	if (ptr) {
		ptr->our_rnd_id=rand(); /* our unique identifier for not generating multiple idents on the same device, TODO: Check if needed*/

		ptr2 = calloc(1,sizeof(struct bigdisk_unit));
		if (ptr2) {
			ptr->unit[0] = ptr2;
			if(BIGDISK_IMAGE_NAME) {
				ptr->unit[0]->filename = strdup(BIGDISK_IMAGE_NAME);
				
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
		ptr2 = calloc(1,sizeof(struct bigdisk_unit));
		if (ptr2) {
			ptr->unit[1] = ptr2;
		}
		ptr2 = calloc(1,sizeof(struct bigdisk_unit));
		if (ptr2) {
			ptr->unit[2] = ptr2;
		}


		// Configure disk to 38 MB disk
		ptr->bytesPrSector = 1024; // 1024bytes / 512 Words		 
		ptr->headsPrCylinder = 5; // HPC
		ptr->sectorsPrTrack = 18; // SPT
		ptr->maxCylinders = 411;  // 411 = 38 MB, 823 = 75 MB
		ptr->deviceType = 0; // 0=old, 1=new
	}	

	tick_bigdisk_end=0;
	tick_bigdisk_error=0;

	IO_Data_Add(864,871,ptr); // 1540-1547 oct
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

	bool checkAutoIncrease=0;

	switch(dev->command)
	{
		case 4: // Read			
			checkAutoIncrease = 1;
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
	
	// Auto increment ?
	if (checkAutoIncrease)
	{
		if (dev->sector_autoinc)
		{
			if (dev->sector < dev->sectors_pr_track)
			{
				dev->sector++;
			}
		}
	}

	// Interrupt ?
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
		//if (debug) fprintf(debugfile,"panel_event: 1 second tick\n");
		//if (debug) fflush(debugfile);
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

static int lastA = 0;
void hawk_IO(ushort ioadd) 
{
	bool trigger_hawk_thread = 0;	
	bool trigger_hawk_irq = 0;	
	bool oldIRQ;
	struct hawk_data *dev = iodata[ioadd];
	int s;

	while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
		continue; /* Restart if interrupted by handler */



	//if (debug) fprintf(debugfile,"HAWK_IO: IOX %d - A=%d\n",ioadd,gA);	
	//fflush(debugfile);  

	int reladd = (int)(ioadd & 0x07); /* just get lowest three bits, to work with both disk system I and II */

	if (reladd & 0x01) 
		if (debug) fprintf(debugfile,"HAWK WRITE %d  (0x%X)\r\n",reladd, (gA & 0xFFFF));

	switch(reladd) {
	case 0: /* Read Memory Address */
		gA = dev->coreAddress;
		break;
	case 1: /* Load Memory Address */
	 	dev->coreAddress = gA;
		break;
	case 2: /* Read Sector Counter */
		gA = dev->sectorCounter;
		break;
	case 3: /* Load Block Address */
		dev->blockAddress = gA & 0x7FFF;
		dev->fixedDrive = (gA>>15) & 0x01;
		break;
	case 4: /* Read Status Register */
		/*
		Status Word
			O	Ready for transfer, interrupt enabled
			1	Error interrupt enabled
			2	Device active
			3	Device reacty for transfer
			4	Inclusive OR of errors (status bits 5 - 11)
			5	Write protect viotate
			6	Time out
			7	Hardware error
			8	Address mismatch
			9	Parity error
			10	Compare error
			11	Missing clock(s)
			12	Transfer complete - When all words have been transfered (WC clocks down to 0) this will be set
			13	Transfer on  - Is active when performin a Read or a Write transfer. Indicated by the Read or Write gate being active. Any data transfter to/from HAWK will set bit 13.
			14	On cylinder - It means that the read/write heads have reached the cylinger address last issued from the controllwe. It will be inactive when the read/write heads are moving.
			15	Bit 15 loaded by previous control word
		*/
		gA=0;
		if (dev->irq_rdy_en) gA |= 1 << 0;
		if (dev->irq_err_en) gA |= 1 << 1;
		if (dev->deviceActive) gA |= 1 << 2;
		if (dev->deviceReadyForTransfer) gA |= 1 << 3;
		if (dev->compareError||dev->hardwareError) gA = (1 << 4);  // or of errror conditions
		//if () value = (1 << 5); // write protect violation
		//if () value = (1 << 6); // timeout
		if (dev->hardwareError) gA = (1 << 7); // hardware error
		//if () value = (1 << 8);  //address mismatch
		//if () value = (1 << 9); // parity error
		if (dev->compareError) gA = (1 << 10); // compare error
		//if () value = (1 << 11); // missing clocks
		if (dev->transferComplete) gA |= 1 << 12;
		if (dev->transferOn) gA |= 1 << 13;
		if (dev->onCylinder) gA |= 1 << 14;
		if (dev->writeFormat) gA |= 1 << 15;  // Will be set if the control words specifices "Write Format"
		break;

	case 5: /* Load Control Word */
		oldIRQ = dev->irq_rdy_en;
		dev->irq_rdy_en =  (gA & 0x01);
		if (dev->irq_rdy_en && !oldIRQ)
		{
			trigger_hawk_irq=true;			
		}

		dev->irq_err_en = (gA>>1) & 0x01;
		dev->deviceActive = (gA>>2) & 0x01;
		dev->testMode = (gA>>3) & 0x01;

		if ((gA >> 4) & 0x01)
		{
			if (debug) fprintf(debugfile,"Hawk: DeviceClear\n");

			dev->coreAddress = 0;
			dev->blockAddress = 0;
			dev->addressHiBits = 0;
			dev->wordCounter = 0;
			dev->compareError = 0;
			dev->hardwareError = 0;
			dev->deviceReadyForTransfer = 1;
		}

		dev->addressHiBits = ((gA >> 5) & 0x3); // Bit 16 and 17 for the address is stored in bit 5-6 bit in the ControlWord

		dev->unit_select = (gA >> 5) & 0x3;
		dev->command = (gA >> 11) & 0b11;

		//marginalRecovery = (value & 1 << 14) != 0; ?? What is this

		dev->writeFormat = (gA >> 15)& 0x01;

		// We are "always" on-cylinder :)
		dev->onCylinder = 1;

		if (dev->deviceActive)
		{			
			hawk_transfer();					
		}
		break;
	case 6: /* Read Block Address */
		gA = dev->blockAddress;
		break;
	case 7: /* Load Word Counter Register */
		dev->wordCounter = gA;
		break;
	}

	if (sem_post(&sem_io) == -1) { /* release io lock */
		if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Hawk_IO\n");
		CurrentCPURunMode = SHUTDOWN;
	}

	if (debug)
	{
		// Log that we Read value, but dont log repeting values (ie typically loop on reading status reg)
		if ((reladd & 0x01) ==0)
		{
			if (gA != lastA)				
				fprintf(debugfile,"HAWK READ %d  (0x%X)\r\n",reladd, (gA & 0xFFFF));
			lastA = gA;
		}
	}
		 

	if (trigger_hawk_irq)
	{
		if (debug) fprintf(debugfile,"HAWK: Triggering  IRQ...");
		hawk_interrupt(dev);
		if (debug) fprintf(debugfile,"Returned\n");
	}
		

	
	return;
}



void hawk_transfer()
{
	int s;
	struct hawk_data *dev;
	FILE *hawk_file;
	char loadtype[]="r+";	
	ushort read_word;
	ushort endian_word;
	int readcnt;

	// Find pointer to Device	
	dev = iodata[320];	/*TODO:: This is just a temporary solution!!! */

	// trigger for hawk thread .. pt. unused
	bool trigger_hawk_thread=0;


	// Block Address is defined like this in the documentation
	// Comment: At the moment it doesnt look like that, values coming from "DUMP-PAGE n" when running FILESYS-INV doesnt work like this

	/*
	+----+--------------+---+-----------+
	| 15 | 14   -    6  | 5 |  4  -  0  |
	+----+--------------+---+-----------+
	| D  |  Cylinder #  | S |  Sector # |
	+----+--------------+---+-----------+
	
	Bits	0-4		:	Designates the sector within a track.
	Bit		5		:	Surface of the (by Bit 15) selected disk (S = 0 upper surface; S = 1 lower surface).
	Bit		6-14	:	Designates the cylinder number.
	Bit		15		:	Designates the fixed (D = 1) or the removable (D = 0) disk

	*/

	int sector = dev->blockAddress & 0b11111;
	int surface = (dev->blockAddress >> 5) & 0x01;
	int cylinder = (dev->blockAddress >> 8) & 0x7F;

	bool isFixed = ((dev->blockAddress >> 15) & 0x01);	

	int position = (dev->blockAddress * 2048); //>>3; // divide by 8... WHY WHY WHY ?
	int wordsToRead = dev->wordCounter;
		
	// NOTES - Block addressering looks very odd. It doesnt follow the expected +1 structure
	// DUMP-PAGE 
	// Choosing device name "DISC-10MB-1"
	// At startup: Total no. of disk pages is 004554 => 2412 (dec) => Which is not 10 MB.. ?? 
	// ReadTransfer for a block, reads 1024 Words (aka 2048 bytes). But this doesnt add up..
	//
	// 		0 => BlockAddress 0
	//		1 => BlockAddress 8 	+8
	//		2 => BlockAddress 16	+8
	//		3 => BlockAddress 32	+16
	//		3 => BlockAddress 40	+8
	//		5 => BlockAddress 48	+8
	//		6 => BlockAddress 64	+16
	//		7 => BlockAddress 72	+8
	//		8 => BlockAddress 80	+8
	//		9 => BlockAddress 96	+16
	//		10 => BlockAddress 104	+8
	//		11 => BlockAddress 112	+8
	//		12 => BlockAddress 128	+16


	if (debug) fprintf(debugfile,"HAWK: Executing command %d on drive %d blockAddress %d [sector %d cylinder %d surface %d] => position %d\n", dev->command, dev->unit_select, dev->blockAddress, sector, cylinder, surface, position);
	if (debug) fflush(debugfile);


	
	// Safely open image and seek			
	if (dev->unit[dev->unit_select]->filename != NULL)
	{				
		hawk_file=fopen(dev->unit[dev->unit_select]->filename,loadtype);	

		if (hawk_file==NULL)
		{
			if (debug) fprintf(debugfile,"HAWK: File open failed. File '%s' errno %d\n", dev->unit[dev->unit_select]->filename, errno);
			dev->hardwareError=1;
			dev->deviceActive=0;
			return;
		}

		if (fseek(hawk_file,position,SEEK_SET) != 0)					
		{
			if (debug) fprintf(debugfile,"HAWK: File seek %d failed.\n",position);
			dev->hardwareError=1;
			dev->deviceActive=0;
			return;
		}
	}
	else
	{
		if (debug) fprintf(debugfile,"HAWK: ERROR - No filename defined\n");
		dev->hardwareError=1;
		dev->deviceActive=0;
		return;
	}


	dev->transferComplete=0;
	dev->hardwareError=0;
	dev->deviceReadyForTransfer=0;
	dev->transferOn =1;

	int error = 0;

	switch(dev->command) 
	{

		case 0: // ReadTransfer						
			if (debug) fprintf(debugfile,"HAWK: Starting ReadTransfer, disk position %d wc %d\n", position, wordsToRead);	
			if (debug) fflush(debugfile);					
			error = 0;
			while (wordsToRead>0)
			{

				readcnt = fread(&read_word,1,2,hawk_file);							

				if (readcnt <=0)
				{
					
					dev->hardwareError=1;
					dev->deviceActive=0;
					
					wordsToRead = 0;
					error = 1;
					break; // Exit while loop	
					
				}
				else
				{								
					// Swap HI/LO to make endian correct (ND is BIG ENDIAN)
					endian_word=(read_word & 0xff00)>>8;
					endian_word= endian_word | ((read_word & 0x00ff) << 8);								

					// Write to memory				
					//if (debug) fprintf(debugfile,"X: 0x%X => [%d, %d]  \n", endian_word, dev->coreAddress,dev->addressHiBits	);
					
					ulong fulladdress = dev->coreAddress | dev->addressHiBits<<16;
					PhysMemWrite(endian_word, fulladdress);
					//PhysMemWrite(wordsToRead, fulladdress);

					dev->coreAddress++;

					// next please 
					wordsToRead--;							
				}						
			}						
			break;

		case 1: // WriteTransfer
			if (debug) fprintf(debugfile,"HAWK: Starting WriteTransfer, disk position %d wc %d", position, wordsToRead);
			if (debug) fflush(debugfile);					

			error = 0;
			while (wordsToRead>0)
			{
				ulong fulladdress = dev->coreAddress | dev->addressHiBits<<16;
				endian_word = PhysMemRead(fulladdress);

				// Swap HI/LO to make endian correct (ND is BIG ENDIAN)
				endian_word=(read_word & 0xff00)>>8;
				endian_word= endian_word | ((read_word & 0x00ff) << 8);								

					
				readcnt = fwrite(&endian_word,1,2,hawk_file);							
				if (readcnt <=0)
				{					
					dev->hardwareError=1;
					dev->deviceActive=0;
				
					wordsToRead = 0;
					error = 1;
					break; // Exit while loop						
				}
				
				// Move memory pointer
				dev->coreAddress++;

				// next please 
				wordsToRead--;							
			}						
			break;

			break;
		case 2: //ReadParity
			if (debug) fprintf(debugfile,"HAWK: Starting ReadParity, disk position %d wc %d", position, wordsToRead);
			break;
		case 3: // Compare
			if (debug) fprintf(debugfile,"HAWK: Starting Compare, disk position %d wc %d", position, wordsToRead);
			break;

		default:
			error=0x10;
	}	
		
	//printf("closing HAWK file\n");
	if (hawk_file != NULL)
	{
		fclose(hawk_file);
		hawk_file = NULL;
	}				

	tick_hawk_end = 10;  // In 10 CPU ticks trigger hawk_command_end
	tick_hawk_error = error; // and this is the saved error code
	//hawk_command_end(dev, error);		
	

	/*
	if (trigger_hawk_thread )
	{
		if (debug) fprintf(debugfile,"HAWK: Triggering Thread..\n");	
		if (sem_post(&sem_hawk) == -1) 
		{ 
			// release hawk 
			if (debug) fprintf(debugfile,"ERROR!!! sem_hawk failure HAWK_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}	

		trigger_hawk_thread = 0;	
	}
	*/
}




void hawk_command_end(int error)
{	

	struct hawk_data *dev;

	// Find pointer to Device	
	dev = iodata[320];	

	if (debug) fprintf(debugfile,"HAWK:Command End. Error = %d\n", error);
	if (debug) fflush(debugfile);					

	//mysleep(0,1000); // 1000 us =>1 ms // Simulate that the drive is doing some IO
	

	dev->deviceReadyForTransfer =1;
	dev->deviceActive = 0;	
	dev->transferOn =0;
	dev->transferComplete=1;

	if (dev->irq_rdy_en)
	{		
		hawk_interrupt(dev);
	}

	// Error interrupt enabled ?
	if ((error >0)&&(dev->irq_err_en))
		hawk_interrupt(dev);

}

void hawk_interrupt(struct hawk_data *dev)
{
    int s;
	int ident = 0x1; //HDD DISK1 uses 0x01, HDD DISK 2 uses 0x5
	
	if (debug) fprintf(debugfile,"HAWK:Interrupt 11\n");

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


// at the moment this does nothing.. maybe use it for delayed register settings
void hawk_thread()
{
	int s;
	struct hawk_data *dev;
	
	// INIT vars


	while(CurrentCPURunMode != SHUTDOWN) 
	{
		while ((s = sem_wait(&sem_hawk)) == -1 && errno == EINTR) /* wait for floppu lock to be free and take it */
			continue; /* Restart if interrupted by handler */

		/* Do our stuff now and then return and wait for next freeing of lock. */
		dev = iodata[320];	/*TODO:: This is just a temporary solution!!! */
		// Clear everything
		
		while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
			continue; /* Restart if interrupted by handler */


		if (sem_post(&sem_io) == -1) { /* release io lock */
			if (debug) fprintf(debugfile,"ERROR!!! sem_post failure Floppy_IO\n");
			CurrentCPURunMode = SHUTDOWN;
		}

	}

}


void bigdisk_IO(ushort ioadd) 
{
	bool oldIRQ;
	struct bigdisk_data *dev = iodata[ioadd];
	int s;
	bool oldInterruptEnabled;
	bool trigger_bigdisk_irq=false;

	while ((s = sem_wait(&sem_io)) == -1 && errno == EINTR) /* wait for io lock to be free and take it */
		continue; /* Restart if interrupted by handler */



	//if (debug) fprintf(debugfile,"BIGDISK_IO: IOX %d - A=%d\n",ioadd,gA);	
	//fflush(debugfile);  

	int reladd = (int)(ioadd & 0x07); /* just get lowest three bits, to work with both disk system I and II */

	if (false)
	//if (debug)
	{
		if (reladd & 0x01) 
			 fprintf(debugfile,"BIGDISK WRITE %d  (0x%X)\r\n",reladd, (gA & 0xFFFF));
	}
	
	switch(reladd) {
		case 0:
			if (dev->CWRBit)
			{
				// TODO: Add flip flopp
				// The Word counter register is read the same way as the memory address register.
				// After a transfer, the upper/ lower memory address(or word count) control bit(flip—flop) is reset.A Read Status instruction(DEV.NO. + 4) or a Device Clear will also reset this bit

				gA = dev->wordCounter;
			}
			else
			{
				// TODO: Add flip flopp
				// The Memory Address regster is read by two successive IOX instructions.
				// The first one gets the lower 16 bits(Address bits 0—15 into the A - reg. 0—15), and the second one gets the upper bits
				// (Address bits 16 - 23 into A—reg. 0 - 7). When reading the most significant bits, the upper byte of the A—reg. is undefined and has tobe masked.


				gA = dev->coreAddress;

			}
			break;

		case 1:
			if (dev->CWRBit) // Count Memory Address & Word Count
			{
				// TODO: FIX

				// Count Memory Address & Word Count: This instruction is implemented for maintenance purposes only.By first loading the control word with
				// 102010, a special test mode, each of these instructions will increment the memory address and decrement the word count by one. (Refer to section 3.1, the DMA transfer.)

			}
			else // Load Memory Address
			{
				// TODO: Add flip flopp
				// Ihe Load Memory Address Register is loaded by two successive instructions. The first loads the 8 upper bits(A-reg. 0 - 7 into Address bits 16 - 23),
				// and the second one loads the lower 16 bits(A—reg. 0 - 15 into Address bits 0 - 15).
				// After a transfer, the upper/ lower memory address control bit(flip—flop) is reset.A Read Status instruction(DEV.NO. +4) or a Device Clear will also reset this bit.

				dev->coreAddress = gA;
			}
			break;

		case 2:
			if (dev->CWRBit)
			{
				gA = (ushort)dev->eccCount;
			}
			else
			{
				gA = 0;
				gA = (ushort)(dev->seekCompleteBits & 0xF);  // 0-3
															//4-7 - Not used
				gA |= (ushort)(dev->selectedUnit & 0x03 << 8);  //8-9
																// 10 - Not used
				if (dev->seekError) gA |= (1 << 11);
				if (dev->deviceType ==1)
				{
					gA |= (1 << 12);  // 12 = Always 1 for 15Mhz SMD. This bit was always 0 on the NORD—10 controller.
				}
			}
			break;
		case 3:	
			if (dev->CWRBit)
			{
				dev->blockAddressII = gA;
			}
			else
			{
				dev->blockAddressI = gA;
			}
			break;
		case 4:
			if (dev->CWRBit)
			{

				dev->eccPatternRegister = 0;

				/*

				Read ECC Pattern Register:

				+----+----+----+----+----+--------------+
				| 15 | 14 | 13 | 12 | 11 | 1O    -    O |
				+----+----+----+----+----+--------------+
				| 1  | 0  |  1 |  1 | 1  | Error pattern|
				+----+----+----+----+----+--------------+
				Bits
					0 - 10 Error pattern.
					11—13 Always 1.
					14 Always 0.To distinguish from the old HD—100 SMD controller.
					15 Always 1.Read~back of Control Word bit 15.

				*/

				// Bits 0-10
				//eccPatternRegister |= eccErrorPattern & 0x3FF; NOT USED.. yet


				// Bits 11-13, Always one
				dev->eccPatternRegister |= (0b111 << 11);

				// Bits 14 - To distinguish from the old HD-100 SMD controller
				if (dev->deviceType == 0)
					dev->eccPatternRegister |= (1 << 14);


				// Bit 15 - Always 1. Read back of control word bit 15
				if (dev->CWRBit) dev->eccPatternRegister |= (1 << 15);

				gA = dev->eccPatternRegister;
			}
			else
			{
				gA = 0;
				//Read status register
				if (dev->irq_rdy_en) gA |= (1 << 0);
				if (dev->irq_err_en) gA |= (1 << 1);
				if (dev->deviceActive) gA |= (1 << 2); // Controller active
				if (dev->transferComplete) gA |= (1 << 3);
				if (dev->hardwareError) gA |= (1 << 4);    // inclusive or of errror conditions (bits 5,6,7,8 and 13)
															//if () value = (1 << 5);	// Illegal load (ie load while status bit 2 is true or load of block address while the unit is not on cylinder)
															//if () value = (1 << 6);	// Timeout
				if (dev->hardwareError) gA |= (1 << 7);    // Hardware error (disk fault, missing clocks, missing servo clokc, ECC parity error)
															//if () value = (1 << 8);	// Address mismatch
															//if () value = (1 << 9);	// Data error
															// if () value = (1 << 10); // Compare error
															//if () value = (1 << 11);	// DMA Channel error
															//if () value = (1 << 12);	// Abnormal completions						
				if (dev->diskUnitNotReady) gA |= (1 << 13); // Disk unit not ready
				if (dev->onCylinder) gA |= (1 << 14); // OnCylinder
				if (dev->CWRBit) gA |= (1 << 15);  // Register Multiplex bit (from CWR bit 15)
			}

			dev->wcFlipFlop = false; // Clear flip flop as stated in documentation
			break;
		case 5:
			/*
				Bit:
					0		Enable interrupt on device not active
					1		Enable interrupt on errors
					2		Active
								When control word bit 2 is activated, the content of the block address register II (cylinder number) is transfered to the servo system in the selected unit.
								Logic in the unit will calculate the difference between the current cylinder and the new one. The difference and direction will command the servo to seek the new cylinder.
					3		Test mode
					4		Device clear (clear the active flip-flop) and controller error bas,
					5		Address bit 16 - Extension of core address regeter
					6		Address bit 17 - Extension of core address regeter

					7-9		Unit select (maximum 4 units)
					10		Marginal recovery cycle
					11-14	Device operation code
					15		Register multiplex bit					 
			*/

			oldInterruptEnabled = dev->irq_rdy_en;
			dev->irq_rdy_en = (gA & 1 << 0) != 0;
			if (dev->irq_rdy_en && !oldInterruptEnabled)
			{
				trigger_bigdisk_irq = true;
			}
			dev->irq_err_en = (gA & 1 << 1) != 0;
			dev->deviceActive = (gA & 1 << 2) != 0;
			dev->testMode = (gA & 1 << 3) != 0;

			dev->coreAddressHiBits = ((ushort)((gA >> 5) & 0b11));
			dev->selectedUnit = ((gA >> 7) & 0b111);
			dev->marginalRecovery = (gA & 1 << 10) != 0;
			dev->deviceOperation = ((gA >> 11) & 0b1111);
			dev->transferComplete = false;

			if ((gA & 1 << 4) != 0)
			{
				// DEVICE CLEAR
				// Clear active flip-flop and error conditions
				dev->deviceActive  = false;

				// Clear errors
				dev->diskUnitNotReady = false;

				dev->coreAddress = 0;
				dev->blockAddressI = 0;
				dev->blockAddressII = 0;
				dev->wordCounter = 0;
				dev->coreAddressHiBits = 0;

				// Ready for transfer (?)
				dev->transferComplete = false;

				// Clear flip-flop
				dev->wcFlipFlop = false;

				dev->seekCompleteBits = 1 << dev->selectedUnit;
			}


			dev->CWRBit = (gA & (1 << 15)) != 0; // Some rgisters switch depending on this bit					

			// We are "always" on-cylinder :)
			dev->onCylinder = true;

			if (dev->deviceActive)
			{
				ExecuteBigDiskGO(dev);
			}
			break;
		case 6:
			if (dev->CWRBit)
			{
				gA = (ushort)dev->blockAddressII;
			}
			else
			{
				gA = (ushort)dev->blockAddressI;
			}
			break;
		case 7:
			if (dev->CWRBit)
			{
				// Load ECC Control

				/*
				ECC CONTROL

				Bit 0: Reset ECC
					This bit wil cause the ECC polynomisis to reset to the zero initial state. Ths function is only used when a dats error has occurred,
					otherwise the polynomials automatically go to the zero state upon completion of a Read or Write. Device Cleer function will also reset ECC.

				Bit 1: TST — Force Parity Error
					Used for maintenance purposes only, This bit will force ECC parity error to be set.

				Bit 2: Long
					Used for maintenance purposes only. When 8 sector is read or writwen, the date field of the sector is extended by 64 bits (
					the length of the ECC appendage plus “end of record” byte). The date and the extra bits sre read into of written from the memory of the CPU.
					This function i¢ used to diagnose the operation of the ECC circuits and can be used with the following Device Operations: MO, M1, M2 and M3.
					Thas bit is “echoed” in ECR bit 14.

				// NEW BITS FOR 15MHZ SMD DISK CONTROLLERS

				Bit 3: Format A
				Bit 4: Format B
				Bit 5: Format C
				Bit 6: Format D

				*/



				if ((dev->wcFlipFlop) || (dev->deviceType == 0))
				{
					// Load Word Count

					// Load Word Counter; The Word Count register is increased from 16 to 24 bits, and is loaded by two successive instructions.
					// The first loads the 8 upper bits(A—reg. 0 -? into Word Count bits 16—32), and the second one loads the lower 16 bits(A - reg. 0—15 into Word Count bits 0—15).

					// After a transfer, the upper/lower Word Count control bit (flip—flop) is reset.A Read Status instruction(DEV.NO. +4) or a Device Clear will also reset this bit.
					// The controller is able to transfer a whole cylinder, or up to 16M words(24 bits), with a hardware increment of the head and sector addresses.

					// For the 75 Mb disk, the maximum word count is 132000(45k); starting with the head and cylinder address equal to 0.
					// The Word Count is set to an integer multiple of the number of words in a sector when device operation is M0—M3.

					

					dev->eccControl = gA;
					dev->wcFlipFlop = true;

					// Bit 0 - Reset ECC
					if ((dev->eccControl & 1 << 0) != 0) dev->eccCount = 0;


					// Bit 1 - Force Parity Error
					if ((dev->eccControl & 1 << 1) != 0)
					{
						// Used for maintenance purposes only. (FILE-SYS-INV uses it!!)
						// This bit will force ECC parity error to be set.

						// TODO: WHat does this mean	
						dev->hardwareError = true; // Set Bit 7 for the Status Register => Disk fault, missing read clocks, missing servoclocks, ECC parity error.
					}

					// Bit 2 - Long
					if ((dev->eccControl & 1 << 2) != 0)
					{
						// Used for maintenance purposes only.

						// When a sector is read or written, the data field of the sector is extended by 64 bits (The length of the ECC pattern plus “End of Record" byte).
						// The data and the extra bits are read into or written from the memory of the CPU. This function is used to diagnose the
						// operation of the ECC circuits, and can be used with the following Device operations: M0, M1, M2, M3. This bit is "echoed" in ECR bit 14.

						// TODO: WHat does this mean	

					}

					// Bit 3-5 - Format A-D. Used when formatting the drive.
				}
				else
				{
					// The first loads the upper 8 bits
					dev->eccControlHI = (ushort)(gA & 0xFF);
				}
			}
			else
			{

				if ((dev->wcFlipFlop) || (dev->deviceType == 0)) // old
				{
					dev->wordCounter = gA;
				}
				else
				{
					dev->wordCounterHI = (ushort)(gA & 0xFF);
					dev->wcFlipFlop = true;
				}
			}
			break;
		break;
	}

	if (sem_post(&sem_io) == -1) { /* release io lock */
		if (debug) fprintf(debugfile,"ERROR!!! sem_post failure bigdisk_IO\n");
		CurrentCPURunMode = SHUTDOWN;
	}

	if (false)
	//if (debug)
	{
		// Log that we Read value, but dont log repeting values (ie typically loop on reading status reg)
		if ((reladd & 0x01) ==0)
		{
			if (gA != lastA)				
				fprintf(debugfile,"BIGDISK READ %d  (0x%X)\r\n",reladd, (gA & 0xFFFF));
			lastA = gA;
		}
	}
		 

	if (trigger_bigdisk_irq)
	{		
		bigdisk_interrupt(dev);		
	}
		

	
	return;
}


long ConvertCHStoLBA(struct bigdisk_data *dev,int cylinder, int head, int sector)
{
	// LBA = (C × HPC + H) × SPT + (S − 1)
	if ((cylinder == 0) && (head == 0) && (sector == 0)) return 0; // invalid, but used by SeekToZero
	return (cylinder * dev->headsPrCylinder + head) * dev->sectorsPrTrack + (sector - 1);
}

void ExecuteBigDiskGO(struct bigdisk_data *dev)
{
	FILE *hdd_file;
	char loadtype[]="r+";	

	ushort read_word;
	ushort endian_word;
	int readcnt;
	

	// Calculate file offset for read/write operations
	int sector = dev->blockAddressI & 0xFF;
	int head = (dev->blockAddressI >> 8) & 0xFF; // (0 - maxSurfaces);
	int cylinder = dev->blockAddressII; // = Cylinder(or Track as on a floppy ?)

	long lba = ConvertCHStoLBA(dev,cylinder, head, sector);

	long position = lba * dev->bytesPrSector;



	// Safely open image and seek			
	if (dev->unit[dev->selectedUnit]->filename != NULL)
	{				
		hdd_file=fopen(dev->unit[dev->selectedUnit]->filename,loadtype);	

		if (hdd_file==NULL)
		{
			if (debug) fprintf(debugfile,"BigDisk: File open failed. File '%s' errno %d\n", dev->unit[dev->selectedUnit]->filename, errno);
			dev->hardwareError=1;
			dev->deviceActive=0;
			return;
		}

		if (fseek(hdd_file,position,SEEK_SET) != 0)					
		{
			if (debug) fprintf(debugfile,"BigDisk: File seek %ld failed.\n",position);
			dev->hardwareError=1;
			dev->deviceActive=0;
			return;
		}
	}
	else
	{
		if (debug) fprintf(debugfile,"BigDisk: ERROR - No filename defined\n");
		dev->seekError=true;
		dev->deviceActive=0;
		return;
	}


	int error = 0;
	bool queue_int = false;


	// Read out information from floppy - is it write protceted ?
	//dev->diskIsWriteProtected = device.read_only;

	if (debug) fprintf(debugfile,"[CoreAddress %04X | %04X [%d] on drive %d. WC %d CHS [%d %d %d] => LBA [%08lX] => Position %ld\n", dev->coreAddress, dev->coreAddressHiBits,  dev->deviceOperation, dev->selectedUnit, dev->wordCounter, cylinder, head, sector, lba,position);
	if (debug) fflush(debugfile);


	switch (dev->deviceOperation)
	{
		case 0: // DeviceOperation.ReadTransfer
			if (debug) fprintf(debugfile,"Starting READ TRANSFER data on drive position %ld, WordCount %d\n", position, dev->wordCounter);

			//Log($"
			while (dev->wordCounter > 0)
			{
				readcnt = fread(&read_word,1,2,hdd_file);		

				if (readcnt <=0)
				{
					if (debug) fprintf(debugfile,"FAILED READ IN ReadTransfer, readcnt = %d\n", readcnt);
					dev->hardwareError=1;
					dev->deviceActive=0;
					
					dev->wordCounter = 0;
					error = 1;
					break; // Exit while loop	
					
				}
				else
				{	


					// Swap HI/LO to make endian correct (ND is BIG ENDIAN)
					endian_word=(read_word & 0xff00)>>8;
					endian_word= endian_word | ((read_word & 0x00ff) << 8);		

					

					// Write to memory				
					//if (debug) fprintf(debugfile,"X: 0x%X => [%d, %d]  \n", endian_word, dev->coreAddress,dev->addressHiBits	);
					
					ulong eff_addr = dev->coreAddress | dev->coreAddressHiBits<<16;
					//if (debug) fprintf(debugfile,"WC %d - READ %04X => ENDIAN %04X ===> [%08lX]\n", (short)dev->wordCounter, (short)read_word, endian_word, eff_addr);
					//if (debug) fflush(debugfile);

					
					PhysMemWrite(endian_word, eff_addr);
					//MemoryWrite(endian_word,eff_addr,false,2);
					dev->coreAddress++;
					dev->wordCounter--;
				}						
			}
			queue_int = true;
			break;
		case 1: // DeviceOperation.WriteTransfer:

			//Log($"Starting WRITE data on drive position {position}");

			if (dev->diskIsWriteProtected)
			{
				if (debug) fprintf(debugfile,"HDD Disk is WRITE PROTECTED\n");
				// Gives status bit 13 if trying to write on a write protected disk.
				dev->diskUnitNotReady = true;
				return;
			}

			// The block register I must be set to 125252 and block register II must be set to 1252 prior to a transfer test mode.

			while (dev->wordCounter > 0)
			{

				ulong eff_addr = dev->coreAddress | dev->coreAddressHiBits<<16;
				endian_word = PhysMemRead(eff_addr);
				//endian_word = MemoryRead(eff_addr,false);
				
				readcnt = fwrite(&endian_word,1,2,hdd_file);							
				if (readcnt <=0)
				{					
					dev->hardwareError=1;
					dev->deviceActive=0;
					
					dev->wordCounter = 0;
					error = 1;
					break; // Exit while loop	
				}

				// Move memory pointer
				dev->coreAddress++;

				// next please 
				dev->wordCounter--;
			}
			queue_int = true;
			break;

		case 2: //DeviceOperation.ReadParityTransfer:
			if (debug) fprintf(debugfile,"Starting ReadParity data on drive position %ld\n",position);
			queue_int = true;
			break;
		case 3: //DeviceOperation.CompareTransfer:
			if (debug) fprintf(debugfile,"Starting Comparer data on drive position %ld\n",position);
			queue_int = true;
			break;
		case 4: //DeviceOperation.InitiateSeek:
			if (debug) fprintf(debugfile,"Starting INITIATE SEEK on drive position %ld\n",position);
			dev->onCylinder = true;
			dev->seekCompleteBits |= 1 << dev->selectedUnit;
			dev->seekError = false;
			queue_int = true;
			break;
		case 5: //DeviceOperation.WriteFormat:
			if (debug) fprintf(debugfile,"Starting WRITE FORMAT on drive position %ld\n",position);
			queue_int = true;
			break;
		case 6: //DeviceOperation.SeekCompleteSearch:
			if (debug) fprintf(debugfile,"Starting SeekCompleteSearch on drive position %ld\n",position);
			dev->onCylinder = true;
			dev->seekError = false;
			dev->seekCompleteBits = 1 << dev->selectedUnit;
			queue_int = true;
			break;
		case 7: //DeviceOperation.ReturnToZeroSeek:
			if (debug) fprintf(debugfile,"Starting ReturnToZeroSeek on drive position %ld\n",position);
			dev->seekError = false;
			dev->onCylinder = true;
			dev->seekCompleteBits |= 1 << dev->selectedUnit;			
			queue_int = true;
			break;
		case 8: //DeviceOperation.RunECCOperation:
			if (debug) fprintf(debugfile,"Starting RunECCOperation on drive position %ld\n",position);	
			break;
		default:
			//error =-1;
			break;
	}
	//printf("closing DISK file\n");
	if (hdd_file != NULL)
	{
		fclose(hdd_file);
		hdd_file = NULL;
	}	

	if (queue_int)
	{		
 		tick_bigdisk_end = 10;  // In 10 CPU ticks trigger bigdisk_command_end
		tick_bigdisk_error= error; // and this is the saved error code

	}
}

void bigdisk_command_end(int error)
{
	struct bigdisk_data *dev = iodata[864]; //TODO: Find a better way to get pointer

	dev->transferComplete = true; //ControllerFinishedWithDeviceOperation = true;
	dev->deviceActive = false;
	dev->wcFlipFlop = false;
	if ((dev->irq_rdy_en) || (dev->irq_err_en && error))
		 bigdisk_interrupt(dev);

}

void bigdisk_interrupt(struct bigdisk_data *dev)
{
	int s;
	int ident = 0xF; //017; //Bigdisk uses 017 for address 1540-1547
	
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
			if (debug) fflush(debugfile);
			CurrentCPURunMode = SHUTDOWN;
		}
		checkPK();
	}
}


// need to use this to get the HAWK registers to change after N number of ticks
void TickIO()
{
	
	if (tick_hawk_end>0)
	{
		tick_hawk_end--;
		if (tick_hawk_end<=0)
		{
			hawk_command_end(tick_hawk_error);
			tick_hawk_error = 0;
		}
	}
	
	if (tick_bigdisk_end>0)
	{
		
		tick_bigdisk_end--;

		if (tick_bigdisk_end<=0)
		{			
			bigdisk_command_end(tick_bigdisk_error);
			tick_bigdisk_error = 0;
		}
	}

}