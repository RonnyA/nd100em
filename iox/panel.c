#include "panel.h"
#include <stdlib.h>

void setup_pap(){
    
//	gPANS=0x8000;	/* Tell system we are here */
	//gPANS=gPANS | 0x4000;	/* Set FULL which is active low, so not full */
    
	gPAP = calloc(1,sizeof(struct display_panel));
}


#if _later_
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
#endif