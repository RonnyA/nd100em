#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diskSMD.h"

//    data->diskFileName = "test.bpun"; // "INSTRUCTION-B.BPUN";

void DiskSMD_Init(DiskInfo *disk, uint8_t unit, char* diskFileName)
{
    disk->unit = unit;

    // Generat disk filename
    if (diskFileName == NULL) {
        disk->diskFileName = malloc(32);
        sprintf(disk->diskFileName, "SMD%d.IMG", unit);
    } else {
        disk->diskFileName = diskFileName;
    }

    disk->file = fopen(disk->diskFileName, "rb");

    if (!disk->file) {
        printf("Error: Could not open disk file %s\n", disk->diskFileName);
        return;
    }

    // TODO: Detect disk type (maybe automatically detected from file size?)    
    DiskSMD_SetDiskType(disk, DISK_75_MB);
}

void DiskSMD_SetDiskType(DiskInfo *disk,DiskType dt)
{
    disk->diskType = dt;
    disk->bytesPrSector = 1024; // 1024bytes / 512 Words		
    disk->maxWordCount = 4095; // TODO: Find the correct MAX - is it different pr disk or controller?

    switch (dt) {
        case DISK_38_MB:
            disk->headsPrCylinder = 5;
            disk->sectorsPrTrack = 18;
            disk->maxCylinders = 411;
            break;
        case DISK_75_MB:
            disk->headsPrCylinder = 5;
            disk->sectorsPrTrack = 18;
            disk->maxCylinders = 823;
            break;
        case DISK_150_MB:
            disk->headsPrCylinder = 10;
            disk->sectorsPrTrack = 18;
            disk->maxCylinders = 823;
            break;
        case DISK_288_MB:
            disk->headsPrCylinder = 19;
            disk->sectorsPrTrack = 18;
            disk->maxCylinders = 823;
            break;
        case DISK_474_MB:
            disk->headsPrCylinder = 20;
            disk->sectorsPrTrack = 24;
            disk->maxCylinders = 842;
            break;
        case DISK_515_MB:
            disk->headsPrCylinder = 24;
            disk->sectorsPrTrack = 26;
            disk->maxCylinders = 711;
            break;
        case DISK_825_MB:
            disk->headsPrCylinder = 16;
            disk->sectorsPrTrack = 44;
            disk->maxCylinders = 1024;
            break;
        default:
            break;
    }
}

