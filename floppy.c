/*
 * nd100em - ND100 Virtual Machine
 *
 * Copyright (c) 2008-2016 Roger Abrahamsson
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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "floppy.h"


/*
 * int sectorread (cyl, side, sector, *addr)
 * cyl can be 0-76, side 0-1, sector 1-8...
 * Reads an ND100 format floppy. 8 sectors per side, 2 sides per track, 77 tracks
 * each sector has an 8 byte sectorinfo in the format of:
 * +------+------+------+------+------+------+----------+
 * | ACYL | ASID | LCYL | LSID | LSEC | LLEN |  COUNT   |
 * +------+------+------+------+------+------+----------+
 *  ACYL      Actual cylinder, 1 byte
 *  ASID      Actual side, 1 byte
 *  LCYL      Logical cylinder; cylinder as read, 1 byte
 *  LSID      Logical side; or side as read, 1 byte
 *  LSEC      Sector number as read, 1 byte
 *  LLEN      Length code as read, 1 byte
 *  COUNT     Byte count of data to follow,  2 bytes.   If zero, no data is contained in this sector.
 *
 */
int oldsectorread (char cyl, char side, char sector, unsigned short *addr) {
	FILE *floppy_file;
	char floppyimage[]="floppy.nd100.img";
	char loadtype[]="r+";
	int offset, flat_sector;

	flat_sector = (((int)cyl*2)+((int)side))*8+((int)sector-1);
	offset=flat_sector*1032;
	offset+=8;

	floppy_file=fopen(floppyimage,loadtype);
	fseek(floppy_file,offset,SEEK_SET);
	fread(addr,2,512,floppy_file);
	fclose(floppy_file);
	return 0;
}

int sectorread (char cyl, char side, char sector, unsigned short *addr) {
	FILE *floppy_file;
	char floppyimage[]="floppy.nd100.img";
	char loadtype[]="r+";
	int offset, flat_sector;

        int i=0;
	unsigned short tmp,tmp2;

	flat_sector = (((int)cyl*2)+((int)side))*8+((int)sector-1);
	offset=flat_sector*1032;
	offset+=8;

	floppy_file=fopen(floppyimage,loadtype);
	if (floppy_file == NULL) {
		printf("Error: Could not open floppy image file %s\n", floppyimage);
		return -1;
	}
	
	if (fseek(floppy_file, offset, SEEK_SET) != 0) {
		printf("Error: Could not seek to offset %d in floppy image\n", offset);
		fclose(floppy_file);
		return -1;
	}

	while(i<512) {
		if (fread(&tmp, 2, 1, floppy_file) != 1) {
			printf("Error: Could not read from floppy image at offset %d\n", offset + i*2);
			fclose(floppy_file);
			return -1;
		}
		tmp2=(tmp & 0xff00)>>8;
		tmp2= tmp2 | ((tmp & 0x00ff) << 8);
		*addr=tmp2;
		addr++;
		i++;
	}

	fclose(floppy_file);
	return 0;
}

/*
 * imd_check
 * check that a file starts with the three magic chars 'IMD'
 * returns: -1 - open failed
 *           0 - not an IMD file
 *           1 - IMD magic marker found
 */
int imd_check(char *imgname) {
	FILE *fp;
	char buf[16];
	int res;

	fp = fopen(imgname,"r");
	if (fp == NULL)
		return -1;

	if (fread(buf, 1, 3, fp) == 3){
		res = ((buf[0]=='I') && (buf[1]=='M') && (buf[2]=='D')) ? 1 : 0;
	} else
		res = -1;

	fclose(fp);
	return res;
}

/*
 * imd_sectorread
 * Reads a sector from an Imagedisk IMD floppy image.
 * returns -1 if failed, or not an IMD image
 */

int imd_sectorread (char cyl, char side, char sector, unsigned short *addr, char *imgname) {
	FILE *fp;
	char buf[256];
	int res,i,j;
	int imod,icyl,ihead,isecs,isecsize,secsize;

	if (!(imd_check(imgname)>0))
		return -1;

	fp = fopen(imgname,"r");
	if (fp == NULL)
		return -1;

	/* Attempt to read until end of header comment block*/
	/* no error handling yet really */
	while (fread(buf, 1, 1, fp) == 1) {
		if (buf[0] == 0x1a)
			break;

	}

	if (fread(buf, 1, 5, fp) == 5){
		imod = buf[0];
		icyl = buf[1];
		ihead = buf[2];
		isecs = buf[3];
		isecsize = buf[4];

		secsize= (0x80 << isecsize);
	}

	if (ihead & 0x80){ /* Has a sector cylinder map */
	}
	if (ihead & 0x80){ /* Has a sector head map */
	}

	printf("MODE:        %02x \n",imod);
	printf("CYLINDER:    %02x \n",icyl);
	printf("HEAD:        %02x \n",ihead);
	printf("SECTORS:     %02x \n",isecs);
	printf("SECTOR SIZE: %02x \n",isecsize);

	printf("SECTOR Bytes: %d\n",secsize);

	/* read sector numbering map */
	printf("SECTOR NUMBERING MAP:\n");
	for(i=0; i < isecs; i++){
		if (fread(buf, 1, 1, fp) == 1)
			printf("%02x ",buf[0]);
	}
	printf("\n");

	/* read sector data records */
	printf("SECTOR DATA RECORDS:\n");
	for(i=0; i < isecs; i++){
		if (fread(buf, 1, 1, fp) == 1) {
			switch(buf[0]){
			case 0x00: /* Sector data unavailable - could not be read: 0 bytes follow*/
				printf("%02x: \n",buf[0]);
				break;
			case 0x01: /* Normal data: sector size bytes follow */
				printf("%02x: \n",buf[0]);
				for(j=0; j < secsize; j++){
					if (fread(buf, 1, 1, fp) == 1)
						printf("%02x ",((int)buf[0] & 0x00ff));
				}
				printf("\n");
				break;
			case 0x02: /* Compressed - all bytes in sector have same value: 1 bytes follow */
				if (fread(buf, 1, 1, fp) == 1)
					printf("%02x ",((int)buf[0] & 0x00ff));
				break;
			case 0x03: /* Data with deleted-data address mark: sector size bytes follow */
				printf("%02x: \n",buf[0]);
				for(j=0; j < secsize; j++){
					if (fread(buf, 1, 1, fp) == 1)
						printf("%02x ",((int)buf[0] & 0x00ff));
				}
				printf("\n");
				break;
			case 0x04: /* Compressed data with deleted-data address mark: 1 byte follow */
				if (fread(buf, 1, 1, fp) == 1)
					printf("%02x ",((int)buf[0] & 0x00ff));
				break;
			case 0x05: /* Data with read error on original: sector size bytes follow */
				printf("%02x: \n",buf[0]);
				for(j=0; j < secsize; j++){
					if (fread(buf, 1, 1, fp) == 1)
						printf("%02x ",((int)buf[0] & 0x00ff));
				}
				printf("\n");
				break;
			case 0x06: /* Compressed data with read error on original : 1 byte follow */
				if (fread(buf, 1, 1, fp) == 1)
					printf("%02x ",((int)buf[0] & 0x00ff));
				break;
			case 0x07: /* Deleted data with read error on original: sector size bytes follow */
				printf("%02x: \n",buf[0]);
				for(j=0; j < secsize; j++){
					if (fread(buf, 1, 1, fp) == 1)
						printf("%02x ",((int)buf[0] & 0x00ff));
				}
				printf("\n");
				break;
			case 0x08: /* Deleted data with read error on original, compressed: 1 byte follow */
				if (fread(buf, 1, 1, fp) == 1)
					printf("%02x ",((int)buf[0] & 0x00ff));
				break;
			default:
				printf("***ERROR***\n");
				break;
			}
		}
	}
	printf("\n");


	fclose(fp);
}

/*
         - For each track on the disk:
            1 byte  Mode value                  (0-5)
            1 byte  Cylinder                    (0-n)
            1 byte  Head                        (0-1)   (see Note)
            1 byte  number of sectors in track  (1-n)
            1 byte  sector size                 (0-6)
            sector numbering map                * number of sectors
            sector cylinder map (optional)      * number of sectors
            sector head map     (optional)      * number of sectors
            sector data records                 * number of sectors

*/




bool LoadBPUNStream(FILE* bpunStream, BPUN_Header* header) {
    // Initialize header    
    header->calculatedChecksum = 0;
    header->address = 0;
    header->count = 0;
    header->checksum = 0;
    header->action = 0;
    header->isFloMon = false;

    LoadState loadState = LoadState_Preamble;
    char tmpString[51] = {0};  // Max 50 chars + null terminator
    int tmpStringPos = 0;
    uint16_t currentLocationCounter = 0;
    uint16_t loadAddress = 0;
    uint16_t lastValue = 0;
    uint16_t dataCounter = 0;
	uint16_t dataLoadAddress = 0;
    rewind(bpunStream);  // Seek to start of file
    int b;
    while ((b = fgetc(bpunStream)) != EOF) {
        switch (loadState) {
            case LoadState_Preamble: {
                char c = (char)(b & 0x7F);  // Convert to 7-bit ASCII

                if (c == '!') {
                    if (tmpStringPos > 0) {
                        tmpString[tmpStringPos] = '\0';
                        int tmp = atoi(tmpString);
                        if (tmp >= 0) {
                            loadAddress = (uint16_t)tmp;
                        }
                    }
                    if (loadAddress == header->start) {
                        header->boot = lastValue;
                    } else {
                        header->boot = loadAddress;
                    }
                    loadState = LoadState_Address;
                    tmpStringPos = 0;
                    continue;
                }
                else if (c == '/') {
                    if (tmpStringPos > 0) {
                        tmpString[tmpStringPos] = '\0';
                        int tmp = atoi(tmpString);
                        if (tmp >= 0) {
                            currentLocationCounter = (uint16_t)tmp;
                            lastValue = currentLocationCounter;
                            if (currentLocationCounter >= 0) {
                                header->start = currentLocationCounter;
                            }
                            if (loadAddress == 0) {
                                loadAddress = currentLocationCounter;
                            }
                        }
                    }
                    tmpStringPos = 0;
                }
                else if (c >= '0' && c <= '9') {
                    if (tmpStringPos < 50) {
                        tmpString[tmpStringPos++] = c;
                    }
                }
                else if (c == 0x0D) {  // Carriage return
                    if (tmpStringPos > 0) {
                        tmpString[tmpStringPos] = '\0';
                        int tmp = atoi(tmpString);
                        if (tmp >= 0) {
                            lastValue = (uint16_t)tmp;
                        }
                        tmpStringPos = 0;
                    }
                }
                break;
            }

            case LoadState_Address:
                header->address = (uint16_t)(b << 8);
                b = fgetc(bpunStream);
                if (b == EOF) return false;
                header->address |= (uint8_t)b;
                loadState = LoadState_Count;

				dataLoadAddress = header->address;
                break;

            case LoadState_Count:
                header->count = (uint16_t)(b << 8);
                b = fgetc(bpunStream);
                if (b == EOF) return false;
                header->count |= (uint8_t)b;
                dataCounter = header->count * 2;  // Count is in words, we read bytes
                loadState = LoadState_Data;
                break;

            case LoadState_Data: {
                uint16_t data_word = 0;
                if (dataCounter > 0) {                    
                    dataCounter--;
                    data_word = (b << 8) & 0xFF00;
                }
				 
                if (dataCounter > 0) {
                    b = fgetc(bpunStream);
                    if (b == EOF) {
                        return false;
                    }                    
                    dataCounter--;
                    data_word |= (b & 0xFF);
                }

				printf("Writing %04X to %06o\n", data_word, dataLoadAddress);
				WritePhysicalMemory(dataLoadAddress++, data_word, false);

                if (dataCounter == 0) {
                    loadState = LoadState_Checksum;
                }

                header->calculatedChecksum = (uint16_t)(header->calculatedChecksum + data_word);
                break;
            }

            case LoadState_Checksum:
                header->checksum = (uint16_t)(b << 8);
                b = fgetc(bpunStream);
                if (b == EOF) return false;
                header->checksum |= (uint8_t)b;
                loadState = LoadState_Action;

                if (header->address == 0 && header->count == 0 && header->checksum == 0) {
                    loadState = LoadState_FloMonCount;
                }
                break;

            case LoadState_Action:
                header->action = (uint16_t)(b << 8);
                b = fgetc(bpunStream);
                if (b == EOF) return false;
                header->action |= (uint8_t)b;
                return true;

            case LoadState_FloMonCount:
                header->isFloMon = true;
                header->count = (uint16_t)b;
                loadState = LoadState_FloMonLoad;
                break;

            case LoadState_FloMonLoad: {
                uint16_t floWords = 0;
                while (floWords < header->count) {

					uint16_t data_word = 0;

					// Entering here the first 0x00 byte has already been read by the outside loop. Check it and contiue
                    if (b != 0) return false;

					// Read HI bits
                    b = fgetc(bpunStream);
                    if (b == EOF) return false;

					data_word = (b << 8);
                    
                    b = fgetc(bpunStream);
                    if (b == EOF || b != 0) return false;

                    b = fgetc(bpunStream);
                    if (b == EOF) return false;

					data_word |= b & 0xFF;

                    b = fgetc(bpunStream);
                    if (b == EOF || b != 0) return false;

					printf("FLOMON: Writing %06o to %06o\n", data_word, header->address+floWords);
                    WritePhysicalMemory(header->address+floWords, data_word, false);

                    floWords++;
                }
                return true;
            }
        }
    }

    return false;  // Unexpected end of file
} 

int LoadBPUN(const char* filename) {
    BPUN_Header bpun = {0};
    uint8_t hi = 0;
    uint8_t lo = 0;
    uint16_t word = 0;
    uint8_t err = 0;

	FILE* bpunStream = fopen(filename, "rb");
	if (!bpunStream) {
		printf("Failed to open BPUN file\n");
		return false;
	}

    bool loadOK = LoadBPUNStream(bpunStream,&bpun );        
	fclose(bpunStream);

	if (!loadOK) {
		printf("BPUN load failed\n");
        return -1;
    }
    printf("BPUN load OK\n");

    printf("--- Bootstrapper ---\n");
    printf("Start: %06o\n", bpun.start);
    printf("Boot: %06o\n", bpun.boot);

    printf("--- Data ---\n");
    printf("Address: %06o\n", bpun.address);
    printf("Count: %06o\n", bpun.count);

    const char* crc = "[OK]";
    if (bpun.checksum != bpun.calculatedChecksum) {
        printf("CRC ERROR != %02X\n", bpun.calculatedChecksum);
        crc = "[CRC ERROR]";
        err++;
    }

    printf("Checksum: %06o %s\n", bpun.checksum, crc);
    printf("Action: %06o\n", bpun.action);
 
  	printf("FloMon: %d\n", bpun.isFloMon);

	return bpun.boot;
}


