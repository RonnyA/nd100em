#ifndef DISK_SMD_H
#define DISK_SMD_H
#include "device.h"

// Disk types
typedef enum {
    // ***************
    // 10MHZ drives
    // See also page 520 in "SINTRAN III J VSX programlistning Vol2-Gandalf-OCR.PDF"
    // ***************

    // Total number of pages  044066 (oct) 18486 (dec) = 38MB (Block size = 2048 Bytes)
    // Total number of pages  110121 (oct) 36945 (dec) = 75MB
    //	max 5 surfaces
    //  max 18 sectors
    //  max 411 cylinders (0-410)
    //  110176 number of sectors
    //  Table Name: DT037
    DISK_38_MB,

    //	max 5 surfaces
    //  max 18 sectors			
    //  max 823 cylinders (0-822)
    //  220526 number of sectors
    //  Table Name: DT075
    DISK_75_MB,

    //	max 10 surfaces
    //  max 18 sectors
    //  max 823 cylinders
    //  441254 number of sectors
    //  Table Name:  DT140 or DT160 ??
    DISK_150_MB,

    //	max 19 surfaces
    //  max 18 sectors	
    //  max 823 cylinders
    //  1045572 number of sectors
    //  Table Name: DT288
    DISK_288_MB,

    // ***************
    // 15 MHZ drives
    // ***************

    // Fujitsu 474Mb M2351A (EAGLE)
    //	max 20 surfaces
    //  max 24 sectors (+1 spare)
    //  max 842 cylinders
    DISK_474_MB,

    //  CDC 515MB FSD
    //	max 24 surfaces
    //  max 26 sectors (+1 spare)
    //  max 711 cylinders
    DISK_515_MB,

    //  CDC 825MB XMD
    //	max 16 surfaces
    //  max 44 sectors (+1 spare)
    //  max 1024 cylinders
    DISK_825_MB
} DiskType;

// Disk structure
typedef struct {
    bool diskUnitNotReady;
    bool onCylinder;
    bool diskIsWriteProtected;
    int bytesPrSector;
    int headsPrCylinder;
    int sectorsPrTrack;
    int maxCylinders;
    int maxWordCount; // TODO: Find the correct MAX - is it different pr disk or controller?
    uint8_t unit; // disk unit number (0-3)
    DiskType diskType;

    char *diskFileName;
    FILE* file;
} DiskInfo;


void DiskSMD_SetDiskType(DiskInfo *disk,DiskType dt);
void DiskSMD_Init(DiskInfo *disk, uint8_t unit, char* diskFileName);





#endif /* DISK_SMD_H */
