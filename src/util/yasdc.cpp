/******************************************************************************
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL AUTHOR OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
*******************************************************************************/
#ifdef __AVR__
    #include <avr/pgmspace.h>
#endif // __AVR__
#include "../YATFT.h"
#include "yasdc.h"

uint8_t  SDC::SDSectorRead(uint32_t sector_addr, uint8_t *buffer) {
    if (!hcMode) sector_addr *= sectorSize;
    while(rdReg8(0x1124) & 0x02); SDReset(RESET_DATA); wrReg8(0x110C, 0x10);
    wrReg8(0x1104, (uint8_t)(sectorSize&0xFF));  wrReg8(0x1105, (uint8_t)((sectorSize>>8)&0xFF));
    wrReg16(0x1136, 0xffff); wrReg16(0x1132, 0xffff); wrReg8(0x1134, 0x23); wrReg8(0x1130, 0x23);
    if (!SDSendCmd(CMD_RD_SINGLE,RESPONSE_48|DATA_PRESENT|CMD_CRC_CHK|CMD_IDX_CHK,sector_addr)) return (false);
    while(!(rdReg8(0x1130)&0x20)); wrReg8(0x1130, 0x20);
    uint16_t  counter = 0;
    while (rdReg8(0x1125)&0x08) { if (counter<sectorSize) *buffer++ = rdReg8(0x1120); counter++; }
    if (counter>sectorSize) return (false);
    return (true);
}

uint8_t  SDC::SDSendCmd(uint8_t cmd_idx, uint8_t flags, uint32_t arg) {
    uint32_t  timeout; wrReg8(0x112E, 0x0E); while(rdReg8(0x1124) & 0x01);
    wrReg8(0x1130, 0xff); wrReg8(0x1134, rdReg8(0x1134) | 0x01);
    wrReg16(0x1132, 0xffff); wrReg16(0x1136, 0xffff);
    wrReg32(0x1108, arg); wrReg8(0x110E, flags); wrReg8(0x110F, cmd_idx);
    timeout = SD_TIMEOUT;
    while (1) { if (rdReg8(0x1130)&0x01) break; if (!timeout--) return (false);}
    return (true);
}

int  SDC::FSInit(void) {
    int fIndex;
    for( fIndex = 0; fIndex < FS_MAX_FILES_OPEN; fIndex++ )  gFileSlotOpen[fIndex] = TRUE;
    gBufferZeroed = FALSE; gNeedFATWrite = FALSE;
    gLastFATSectorRead = 0xFFFFFFFF; gLastDataSectorRead = 0xFFFFFFFF;
    if (DISKmount(&gDiskData) == CE_GOOD) {
        cwdptr->dsk = &gDiskData; cwdptr->sec = 0; cwdptr->pos = 0;
        cwdptr->seek = 0; cwdptr->size = 0; cwdptr->name[0] = '\\';
        for (fIndex = 1; fIndex < 11; fIndex++)  cwdptr->name[fIndex] = 0x20;
        cwdptr->entry = 0; cwdptr->attributes = ATTR_DIRECTORY;
        cwdptr->dirclus = FatRootDirClusterValue; cwdptr->dirccls = FatRootDirClusterValue;
#if defined(SUPPORT_LFN)
        cwdptr->AsciiEncodingType = TRUE; cwdptr->utf16LFNlength = 0x0000;
#endif
        FSerrno = 0;
        return TRUE;
    }
    return FALSE;
}

uint8_t  SDC::DISKmount(DISK *dsk) {
    uint8_t  error = CE_GOOD; MEDIA_INFORMATION *mediaInformation;
    dsk->mount = FALSE; dsk->buffer = gDataBuffer;
    mediaInformation = SDInitialize();
    if (mediaInformation->errorCode != MEDIA_NO_ERROR) { error = CE_INIT_ERROR; FSerrno = CE_INIT_ERROR;}
    else {
        if (mediaInformation->validityFlags.bits.sectorSize) {
            dsk->sectorSize = mediaInformation->sectorSize;
            if (mediaInformation->sectorSize > MEDIA_SECTOR_SIZE) {
                error   = CE_UNSUPPORTED_SECTOR_SIZE; FSerrno = CE_UNSUPPORTED_SECTOR_SIZE; return error;
            }
        }
        if ((error = LoadMBR(dsk)) == CE_GOOD) { if ((error = LoadBootSector(dsk)) == CE_GOOD) dsk->mount = TRUE;}
    }
    return(error);
}

MEDIA_INFORMATION * SDC::SDInitialize(void) {
    uint32_t  timeout; SDReset(RESET_DATA | RESET_CMD | RESET_ALL); SDPower(TRUE);
    mediaInformation.errorCode=MEDIA_NO_ERROR; mediaInformation.validityFlags.value=0;
    if (!SDSetClock(SD_CLK_INIT)) { mediaInformation.errorCode=MEDIA_DEVICE_NOT_PRESENT; return (&mediaInformation);}
    timeout = 10;
    while (!SDInit()) {
        if (!timeout--) { mediaInformation.errorCode=MEDIA_DEVICE_NOT_PRESENT; return (&mediaInformation);}
    }
    mediaInformation.validityFlags.bits.sectorSize=1; mediaInformation.sectorSize=sectorSize;
    return (&mediaInformation);
}

uint8_t  SDC::LoadMBR(DISK *dsk) {
    uint8_t  error = CE_GOOD; PT_MBR  Partition; uint8_t  type;  BootSec  BSec;
    if ( SDSectorRead( FO_MBR, dsk->buffer) != TRUE) {
        error = CE_BAD_SECTOR_READ; FSerrno = CE_BAD_SECTOR_READ;
    } else {
        BSec = (BootSec) dsk->buffer;
        if ((BSec->Signature0 == FAT_GOOD_SIGN_0) && (BSec->Signature1 == FAT_GOOD_SIGN_1)) {
            if ((ReadByte( dsk->buffer, BSI_FSTYPE + 0 ) == 'F') && \
                (ReadByte( dsk->buffer, BSI_FSTYPE + 1 ) == 'A') && \
                (ReadByte( dsk->buffer, BSI_FSTYPE + 2 ) == 'T') && \
                (ReadByte( dsk->buffer, BSI_FSTYPE + 3 ) == '1') && \
                (ReadByte( dsk->buffer, BSI_BOOTSIG) == 0x29)) {
                dsk->firsts = 0; dsk->type = FAT16; return CE_GOOD;
             } else {
                if ((ReadByte( dsk->buffer, BSI_FAT32_FSTYPE + 0 ) == 'F') && \
                    (ReadByte( dsk->buffer, BSI_FAT32_FSTYPE + 1 ) == 'A') && \
                    (ReadByte( dsk->buffer, BSI_FAT32_FSTYPE + 2 ) == 'T') && \
                    (ReadByte( dsk->buffer, BSI_FAT32_FSTYPE + 3 ) == '3') && \
                    (ReadByte( dsk->buffer, BSI_FAT32_BOOTSIG) == 0x29)) {
                    dsk->firsts = 0; dsk->type = FAT32; return CE_GOOD;
        }   }   }
        Partition = (PT_MBR)dsk->buffer;
        if ((Partition->Signature0 != FAT_GOOD_SIGN_0) || (Partition->Signature1 != FAT_GOOD_SIGN_1)) {
            FSerrno = CE_BAD_PARTITION; error = CE_BAD_PARTITION;
        } else {
            dsk->firsts = Partition->Partition0.PTE_FrstSect;
            type = Partition->Partition0.PTE_FSDesc;
            switch (type) {
                case 0x01:
                    dsk->type = FAT12;
                    break;
                case 0x04:
                case 0x06:
                case 0x0E:
                    dsk->type = FAT16;
                    break;
                case 0x0B:
                case 0x0C:
#ifdef SUPPORT_FAT32 // If FAT32 supported.
                    dsk->type = FAT32;    // FAT32 is supported too
#else
                    FSerrno = CE_CARDFAT32; error   = CE_CARDFAT32;
#endif
                    break;
                default:
                    FSerrno = CE_UNSUPPORTED_FS; error   = CE_UNSUPPORTED_FS;
    }   }   }
    return(error);
}

uint8_t  SDC::LoadBootSector(DISK *dsk) {
    uint8_t     error = CE_GOOD; uint32_t    RootDirSectors; uint32_t    TotSec,DataSec;
    BootSec     BSec; uint16_t    BytesPerSec; uint16_t    ReservedSectorCount;
#if defined(SUPPORT_FAT32)
    bool  TriedSpecifiedBackupBootSec  = false;
    bool  TriedBackupBootSecAtAddress6 = false;
#endif
    if ( SDSectorRead( dsk->firsts, dsk->buffer) != TRUE)  error = CE_BAD_SECTOR_READ;
    else {
        BSec = (BootSec)dsk->buffer;
        do {
            if ((BSec->Signature0 != FAT_GOOD_SIGN_0) || (BSec->Signature1 != FAT_GOOD_SIGN_1)) {
                error = CE_NOT_FORMATTED;
            } else {
                do {
                    ReservedSectorCount = ReadWord( dsk->buffer, BSI_RESRVSEC );
                    dsk->SecPerClus = ReadByte( dsk->buffer, BSI_SPC);  dsk->fat = dsk->firsts + ReservedSectorCount;
                    dsk->fatcopy = ReadByte( dsk->buffer, BSI_FATCOUNT); dsk->fatsize = ReadWord( dsk->buffer, BSI_SPF);
                    if(dsk->fatsize == 0) dsk->fatsize  = ReadDWord( dsk->buffer, BSI_FATSZ32 );
                    dsk->root = dsk->fat + (uint32_t)(dsk->fatcopy * (uint32_t)dsk->fatsize);
                    dsk->maxroot = ReadWord( dsk->buffer, BSI_ROOTDIRENTS );
                    TotSec = ReadWord( dsk->buffer, BSI_TOTSEC16 );
                    if( TotSec == 0 )  TotSec = ReadDWord( dsk->buffer, BSI_TOTSEC32 );
                    BytesPerSec = ReadWord( dsk->buffer, BSI_BPS );
                    if ((BytesPerSec == 0) || ((BytesPerSec & 1) == 1)) { error = CE_UNSUPPORTED_SECTOR_SIZE; break;}
                    RootDirSectors = ((dsk->maxroot * NUMBER_OF_BYTES_IN_DIR_ENTRY) + (BytesPerSec - 1)) / BytesPerSec;
                    DataSec = TotSec - (ReservedSectorCount + (dsk->fatcopy * dsk->fatsize )  + RootDirSectors);
                    dsk->maxcls = DataSec / dsk->SecPerClus;
                    if (dsk->maxcls < 4085)  dsk->type = FAT12;
                    else {
                        if (dsk->maxcls < 65525)  dsk->type = FAT16;
                        else
 #ifdef SUPPORT_FAT32
                            dsk->type = FAT32;
 #else
                            error = CE_CARDFAT32;
 #endif
                    }
 #ifdef SUPPORT_FAT32
                    if (dsk->type == FAT32) {
                        FatRootDirClusterValue = ReadDWord( dsk->buffer, BSI_ROOTCLUS );
                        dsk->data = dsk->root + RootDirSectors;
                    } else
 #endif
                    {
                        FatRootDirClusterValue=0; dsk->data=dsk->root+(dsk->maxroot>>4);
                    }
                    if(BytesPerSec > MEDIA_SECTOR_SIZE)  error = CE_UNSUPPORTED_SECTOR_SIZE;
                }while(0);
            }
 #if defined(SUPPORT_FAT32)
            if ((dsk->type == FAT32) || ((error != CE_GOOD) && ((BSec->FAT.FAT_32.BootSec_BootSig == 0x29) || (BSec->FAT.FAT_32.BootSec_BootSig == 0x28))))
            {
                if (  (BSec->FAT.FAT_32.BootSec_TotSec16 != 0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[0] !=0)||(BSec->FAT.FAT_32.BootSec_Reserved[1] !=0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[2] !=0)||(BSec->FAT.FAT_32.BootSec_Reserved[3] !=0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[4] !=0)||(BSec->FAT.FAT_32.BootSec_Reserved[5] !=0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[6] !=0)||(BSec->FAT.FAT_32.BootSec_Reserved[7] !=0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[8] !=0)||(BSec->FAT.FAT_32.BootSec_Reserved[9] !=0)
                    ||(BSec->FAT.FAT_32.BootSec_Reserved[10]!=0)||(BSec->FAT.FAT_32.BootSec_Reserved[11]!=0)
                    ||((BSec->FAT.FAT_32.BootSec_BootSig != 0x29) && (BSec->FAT.FAT_32.BootSec_BootSig != 0x28))
                  ) { error = CE_NOT_FORMATTED; }
                if ((error != CE_GOOD) && (TriedSpecifiedBackupBootSec == FALSE)) {
                    TriedSpecifiedBackupBootSec = TRUE;
                    if (SDSectorRead( dsk->firsts + BSec->FAT.FAT_32.BootSec_BkBootSec, dsk->buffer) != TRUE) {
                        FSerrno = CE_BAD_SECTOR_READ; return CE_BAD_SECTOR_READ;
                    } else { error = CE_GOOD; continue;}
                }
                if ((error != CE_GOOD) && (TriedBackupBootSecAtAddress6 == FALSE)) {
                    TriedBackupBootSecAtAddress6 = TRUE;
                    if ( SDSectorRead( dsk->firsts + 6, dsk->buffer) != TRUE) {
                        FSerrno = CE_BAD_SECTOR_READ; return CE_BAD_SECTOR_READ;
                    } else { error = CE_GOOD; continue; }
                }
            }
 #endif  //SUPPORT_FAT32
            break;
        }
        while(1);
    }
    if (error != CE_GOOD)  FSerrno = error;
    return(error);
}

uint8_t  SDC::SDSetClock(uint32_t clockMax) {
    uint32_t  mclk; uint8_t   reg, div; wrReg8(0x0127, 200); mclk = SYS_CLOCK * rdReg8(0x0127);
    mclk /= (rdReg8(0x0126) & 0x1F); mclk /= ((rdReg8(0x0004) & 0x1F) + 1); if(rdReg8(0x0004) & 0x01)  mclk /= 2;
    reg = rdReg8(0x112C); reg = reg &~((uint8_t) SD_CLK_ENABLE); wrReg8(0x112C, reg); reg |= SD_INT_CLK_ENABLE;
    wrReg8(0x112C, reg); delay(1); if (!(rdReg8(0x112C) & SD_INT_CLK_STABLE))  return (FALSE); div = 0;
    while (mclk > clockMax) { mclk >>= 1; div++; } if (div > 0)  div = 1 << (div - 1);
    wrReg8(0x112D, div); reg |= SD_CLK_ENABLE; wrReg8(0x112C, reg); return (TRUE);
}

uint8_t  SDC::SDInit(void) {
    uint32_t  timeout, response, voltage, CSD_C_SIZE, CSD_C_SIZE_MULT, CSD_RD_BL_LEN, RCA, CSD[4];
    if(!SDSendCmd(CMD_RESET, NO_RESPONSE, 0xFFFFFFFF))  return (FALSE);
    if(!SDSendCmd(CMD_SEND_IF_COND, RESPONSE_48 | CMD_CRC_CHK | CMD_IDX_CHK, 0x000001AA))  return (FALSE);
    response = GetCmdResp(0);
    if (response == 0x000001AA) { voltage = 0x40200000; hcMode = 1;}
    else { voltage = 0x00200000; hcMode = 0;} timeout = 10;
    do {if (!timeout--)  return (FALSE);
        SDReset(RESET_DATA|RESET_CMD);
        if (!SDSendAppCmd(ACMD_SEND_OCR, RESPONSE_48, 0, voltage))  return (FALSE);
        delay(150); response = GetCmdResp(0);
    } while((response&0x80000000) == 0);
    if ((response&0x40000000L) == 0)  hcMode = 0;
    if (!SDSendCmd(CMD_SEND_ALL_CID, RESPONSE_136 | CMD_CRC_CHK, 0xFFFFFFFF))  return (FALSE);
    delay(150);
    if (!SDSendCmd(CMD_SEND_RCA, RESPONSE_48 | CMD_CRC_CHK | CMD_IDX_CHK, 0xFFFFFFFF))  return (FALSE);
    RCA = GetCmdResp(0); RCA |= 0x0000ffff;
    if (!SDSendCmd(CMD_SEND_CSD, CMD_IDX_CHK | CMD_CRC_CHK | RESPONSE_136, RCA))  return (FALSE);
    ReadCmdResp(CSD, 4);
    CSD_C_SIZE = ((CSD[2] & 0x00000003) << 10) | ((CSD[1] & 0xFFC00000) >> 22);
    CSD_C_SIZE_MULT = ((CSD[1] & 0x00000380) >> 7); CSD_RD_BL_LEN = (CSD[2] & 0x00000F00) >> 8; 
    finalLBA = (CSD_C_SIZE + 1) * (1 << (CSD_C_SIZE_MULT + 2)); sectorSize = (1 << CSD_RD_BL_LEN);
    if ((CSD[3] & 0x000000FF) == 0x32)  maxBusClock = 25000000;
    else                                maxBusClock = 25000000;
    SDReset(RESET_CMD);
    if (!SDSendCmd(CMD_SELECT_CARD, RESPONSE_48_BUSY | CMD_CRC_CHK | CMD_IDX_CHK, RCA))  return (FALSE);
    sectorSize = 512L;
    if (!SDSendCmd(CMD_SET_BLKLEN, RESPONSE_48 | CMD_CRC_CHK | CMD_IDX_CHK, sectorSize))  return (FALSE);
    if (!SDSendAppCmd(ACMD_SET_BUS_WIDTH, RESPONSE_48 | CMD_CRC_CHK | CMD_IDX_CHK, RCA, 0xFFFFFFFE))  return (FALSE);
    wrReg8(0x1128, 0x02);
    return (TRUE);
}

uint32_t  SDC::GetCmdResp(uint8_t idx) {
    uint32_t rsp;  idx <<= 2;
    rsp = (uint32_t)rdReg8(0x1110+idx)|(uint32_t)rdReg8(0x1111+idx)<<8|(uint32_t)rdReg8(0x1112+idx)<<16|(uint32_t)rdReg8(0x1113+idx)<<24;
    return (rsp);
}

uint8_t  SDC::SDSendAppCmd(uint8_t cmd_idx, uint8_t flags, uint32_t arg1, uint32_t arg2) {
    if (!SDSendCmd(CMD_APP_CMD, RESPONSE_48, arg1))  return (FALSE);
    if (!SDSendCmd(cmd_idx, flags, arg2))  return (FALSE);
    return (TRUE);
}

int  SDC::FindFirst (const char * fileName, unsigned int attr, SearchRec * rec) {
    FSFILE  f; FILEOBJ fo = &f; uint16_t fHandle; uint8_t  j, Index; FSerrno = CE_GOOD;
#ifdef SUPPORT_LFN
    short int indexLFN; fo->utf16LFNptr = &recordSearchName[0]; rec->utf16LFNfound = &recordFoundName[0];
#endif
    if (!FormatFileName(fileName, fo, 1)) { FSerrno = CE_INVALID_FILENAME; return -1;}
    rec->initialized = FALSE;
#if defined(SUPPORT_LFN)
    rec->AsciiEncodingType = fo->AsciiEncodingType; recordSearchLength = fo->utf16LFNlength;
    if (!recordSearchLength)
#endif
    {
        for (Index = 0; (Index < 12) && (fileName[Index] != 0); Index++)  rec->searchname[Index] = fileName[Index];
        for (;Index < FILE_NAME_SIZE_8P3 + 2; Index++)  rec->searchname[Index] = 0;
    }
    rec->searchattr = attr; rec->cwdclus = cwdptr->dirclus; fo->dsk = &gDiskData;
    fo->cluster = 0; fo->ccls = 0; fo->entry = 0; fo->attributes = attr;
    fo->dirclus = cwdptr->dirclus; fo->dirccls = cwdptr->dirccls; FileObjectCopy(&gFileTemp, fo);
    if (FILEfind (fo, &gFileTemp,LOOK_FOR_MATCHING_ENTRY, 1) != CE_GOOD) {
        FSerrno = CE_FILE_NOT_FOUND; return -1;
    }
    fHandle = fo->entry;
    if (FILEopen (fo, &fHandle, 'r') == CE_GOOD) {
#if defined(SUPPORT_LFN)
        rec->utf16LFNfoundLength = fo->utf16LFNlength;
        if (fo->utf16LFNlength) {
            indexLFN = fo->utf16LFNlength; recordFoundName[indexLFN] = 0x0000;
            while (indexLFN--) recordFoundName[indexLFN] = fileFoundString[indexLFN];
        }
#endif
        for (j = 0; j < FILE_NAME_SIZE_8P3 + 2 ; j++)  rec->filename[j] = 0;
        if (fo->attributes != ATTR_VOLUME) {
            for (Index = 0, j = 0; (j < 8) && (fo->name[j] != 0x20); Index++, j++)  rec->filename[Index] = fo->name[j];
            if (fo->name[8] != 0x20) {
                rec->filename[Index++] = '.';
                for (j = 8; (j < 11) && (fo->name[j] != 0x20); Index++, j++)  rec->filename[Index] = fo->name[j];
            }
        } else { for (Index=0;Index<DIR_NAMECOMP;Index++) rec->filename[Index]=fo->name[Index]; }
        rec->attributes = fo->attributes; rec->filesize = fo->size;
        rec->timestamp = (uint32_t)((uint32_t)fo->date << 16) + fo->time;
        rec->entry = fo->entry; rec->initialized = TRUE; return 0;
    } else { FSerrno = CE_BADCACHEREAD; return -1;}
}

DEMO_FILE_TYPE  SDC::GetFileType(char *fileName) {
    while(*fileName != '.')  fileName++; fileName++;
    if((fileName[0]=='J')&&(fileName[1]=='P')&&(fileName[2]=='G')) return DEMO_FILE_TYPE_JPEG;
    if((fileName[0]=='j')&&(fileName[1]=='p')&&(fileName[2]=='g')) return DEMO_FILE_TYPE_JPEG;
    if((fileName[0]=='R')&&(fileName[1]=='G')&&(fileName[2]=='B')) return DEMO_FILE_TYPE_RGB;
    if((fileName[0]=='r')&&(fileName[1]=='g')&&(fileName[2]=='b')) return DEMO_FILE_TYPE_RGB;
    return DEMO_FILE_TYPE_OTHER;
}

int  SDC::FindNext (SearchRec * rec) {
    FSFILE  f;  FILEOBJ  fo = &f; uint8_t  i, j;
#ifdef SUPPORT_LFN
    short int indexLFN;
#endif
    FSerrno = CE_GOOD;
    if (rec->initialized == FALSE) { FSerrno = CE_NOT_INIT; return -1;}
    if (rec->cwdclus != cwdptr->dirclus) { FSerrno = CE_INVALID_ARGUMENT; return -1;}
#if defined(SUPPORT_LFN)
    fo->AsciiEncodingType = rec->AsciiEncodingType; fo->utf16LFNlength = recordSearchLength;
    if (fo->utf16LFNlength)  fo->utf16LFNptr = &recordSearchName[0];
    else
#endif
    { if (!FormatFileName(rec->searchname, fo, 1)) { FSerrno = CE_INVALID_FILENAME; return -1;}}
    fo->dsk = &gDiskData; fo->cluster = 0; fo->ccls = 0; fo->entry = rec->entry + 1; fo->attributes = rec->searchattr;
    fo->dirclus = cwdptr->dirclus; fo->dirccls = cwdptr->dirccls; FileObjectCopy(&gFileTemp, fo);
    if (CE_GOOD != FILEfind (fo, &gFileTemp,LOOK_FOR_MATCHING_ENTRY, 1)) {
        FSerrno = CE_FILE_NOT_FOUND; return -1;
    } else {
#if defined(SUPPORT_LFN)
        rec->utf16LFNfoundLength = fo->utf16LFNlength;
        if (fo->utf16LFNlength) {
            indexLFN = fo->utf16LFNlength; recordFoundName[indexLFN] = 0x0000;
            while (indexLFN--) recordFoundName[indexLFN] = fileFoundString[indexLFN];
        }
#endif
        for (j = 0; j < FILE_NAME_SIZE_8P3 + 2 ; j++)  rec->filename[j] = 0;
        if (fo->attributes != ATTR_VOLUME) {
            for (i = 0, j = 0; (j < 8) && (fo->name[j] != 0x20); i++, j++)  rec->filename[i] = fo->name[j];
            if (fo->name[8] != 0x20) {
                rec->filename[i++] = '.';
                for (j = 8; (j < 11) && (fo->name[j] != 0x20); i++, j++)  rec->filename[i] = fo->name[j];
            }
        } else { for (i = 0; i < DIR_NAMECOMP; i++)  rec->filename[i] = fo->name[i];}
        rec->attributes = fo->attributes; rec->filesize = fo->size;
        rec->timestamp = (uint32_t)((uint32_t)fo->date << 16) + fo->time;
        rec->entry = fo->entry; return 0;
    }
}

uint8_t  SDC::FormatFileName( const char* fileName, FILEOBJ fptr, uint8_t mode) {
    char *fN2; FILE_DIR_NAME_TYPE fileNameType; int  temp,count1,count2,count3,count4;
    bool supportLFN = false; char *localFileName = NULL;
#if defined(SUPPORT_LFN)
    unsigned short int tempString[256]; bool AscciIndication = TRUE; count1 = 256;
#else
    unsigned short int	tempString[13]; count1 = 12;
#endif
#ifdef SUPPORT_LFN
    if (utfModeFileName) {
        utf16Filename = (unsigned short int *)fileName; fileNameLength = 0;
        while (utf16Filename[fileNameLength]) { fileNameLength++;}
        if ((fileNameLength > count1) || (*utf16Filename == '.') || (*utf16Filename == 0)) { return FALSE;}
        for (count1 = 0;count1 < fileNameLength; count1++) { tempString[count1] = utf16Filename[count1];}
        utf16Filename = tempString;
    } else
#endif
    {
        fileNameLength = strlen(fileName);
        if ((fileNameLength > count1) || (*fileName == '.') || (*fileName == 0)) { return FALSE;}
        asciiFilename = (char *)tempString;
        for (count1 = 0;count1 < fileNameLength; count1++) { asciiFilename[count1] = fileName[count1];}
    }
    fileNameType = ValidateChars(mode);
    if (NAME_ERROR == fileNameType) { return FALSE;}
    temp = fileNameLength;
#if defined(SUPPORT_LFN)
    fptr->AsciiEncodingType = TRUE; fptr->utf16LFNlength = 0;
#endif
    if (NAME_8P3_ASCII_MIXED_TYPE == fileNameType) {
#if defined(SUPPORT_LFN)
        supportLFN = TRUE;
#endif
    }
#if defined(SUPPORT_LFN)
    if (NAME_8P3_UTF16_TYPE == fileNameType) {
        for (count3 = 0; count3 < temp; count3++) {
            if (utf16Filename[count3] > 0xFF) {
                fileNameType = NAME_8P3_UTF16_NONASCII_TYPE; supportLFN = TRUE;
                break;
        }   }
        if (count3 == temp) {
            fileNameType = NAME_8P3_UTF16_ASCII_CAPS_TYPE;
            for (count3 = 0; count3 < temp; count3++) {
                if ((utf16Filename[count3] >= 0x61) && (utf16Filename[count3] <= 0x7A)) {
                    fileNameType = NAME_8P3_UTF16_ASCII_MIXED_TYPE;
                    supportLFN = TRUE;
                    break;
    }   }   }   }
#endif
    if ((NAME_LFN_TYPE != fileNameType) && (FALSE == supportLFN)) {
        for (count3 = 0; count3 < temp; count3++) {
#ifdef SUPPORT_LFN
            if (utfModeFileName) {
                if (((utf16Filename[count3] == '.') && ((temp - count3) > 4)) || (count3 > 8)) {
                    supportLFN = TRUE;
                    break;
                } else if(utf16Filename[count3] == '.') { break;}
            } else
#endif
            {
                if (((asciiFilename[count3] == '.') && ((temp - count3) > 4)) || (count3 > 8)) {
#if !defined(SUPPORT_LFN)
                    return FALSE;
#endif
                    supportLFN = TRUE;
                    break;
                } else if(asciiFilename[count3] == '.') { break;}
            }
        }
        if (FALSE == supportLFN) {
            fN2 = fptr -> name;
            for (count1 = 0; count1 < FILE_NAME_SIZE_8P3; count1++) *(fN2 + count1) = ' ';
#ifdef SUPPORT_LFN
            if (utfModeFileName) {
                count4 = count3 * 2; temp = temp * 2;
                localFileName = (char *)utf16Filename;
            } else
#endif
            { count4 = count3; localFileName = asciiFilename; }
            for (count1 = 0,count2 = 0; (count2 < 8) && (count1 < count4);count1++ ) {
                if (localFileName[count1]) {
                    fN2[count2] = localFileName[count1];
                    if ((fN2[count2] >= 0x61) && (fN2[count2] <= 0x7A)) fN2[count2] -= 0x20;
                    count2++;
            }   }
            if (count4 < temp) {
                count4++;
                for (count3 = 8; (count3 < 11) && (count4 < temp);count4++ ) {
                    if (localFileName[count4]) {
                        fN2[count3] = localFileName[count4];
                        if ((fN2[count3] >= 0x61) && (fN2[count3] <= 0x7A)) fN2[count3] -= 0x20;
                        count3++;
    }   }   }   }   }
    if ((NAME_LFN_TYPE == fileNameType) || (TRUE == supportLFN)) {
#if defined(SUPPORT_LFN)
        fN2 = (char *)(fptr -> utf16LFNptr);
        if (!utfModeFileName) localFileName = asciiFilename;
        count2 = 0;
        for (count1 = 0;count1 < temp;count1++) {
            if (utfModeFileName) {
                fptr -> utf16LFNptr[count1] = utf16Filename[count1];
                if (AscciIndication) {
                    if (utf16Filename[count1] > 0xFF) {
                        fptr->AsciiEncodingType = FALSE; AscciIndication = FALSE;
                    }
                }
            } else { fN2[count2++]=localFileName[count1]; fN2[count2++]=(uint8_t)0x00;}
        }
        fptr -> utf16LFNptr[count1] = 0x0000; fptr->utf16LFNlength = fileNameLength;
#else
        return FALSE;
#endif
    }
    return TRUE;
}

void  SDC::FileObjectCopy(FILEOBJ foDest,FILEOBJ foSource) {
    int size; uint8_t *dest; uint8_t *source; int Index; dest = (uint8_t *)foDest;
    source = (uint8_t *)foSource; size = sizeof(FSFILE);
    for (Index=0;Index< size; Index++) { dest[Index] = source[Index];}
}

CETYPE  SDC::FILEfind( FILEOBJ foDest, FILEOBJ foCompareTo, uint8_t cmd, uint8_t mode) {
    uint16_t   attrib, compareAttrib;  uint16_t   fHandle = foDest->entry;
    CETYPE statusB = CE_FILE_NOT_FOUND;  uint8_t   character,test,state,index;
#if defined(SUPPORT_LFN)
    LFN_ENTRY lfnObject; unsigned char *dst = (unsigned char *)&fileFoundString[0];
    unsigned short int *templfnPtr = (unsigned short int *)foCompareTo -> utf16LFNptr;
    uint16_t  tempShift;
    short int fileCompareLfnIndex,fileFoundLfnIndex=0,fileFoundMaxLfnIndex=0,lfnCountIndex,fileFoundLength=0;
    bool lfnFirstCheck=FALSE,foundSFN,foundLFN,fileFoundDotPosition=FALSE,fileCompareDotPosition;
    uint8_t  lfnCompareMaxSequenceNum=0,lfnFoundMaxSequenceNum,reminder=0;
    char  tempDst[13]; fileNameLength=foCompareTo->utf16LFNlength;
    if (fileNameLength) {
        reminder = fileNameLength % MAX_UTF16_CHARS_IN_LFN_ENTRY;
        index = fileNameLength/MAX_UTF16_CHARS_IN_LFN_ENTRY;
        if (reminder || (fileNameLength < MAX_UTF16_CHARS_IN_LFN_ENTRY)) index++;
        lfnCompareMaxSequenceNum = index;
    }
#endif
    foDest->dirccls = foDest->dirclus;
    compareAttrib = 0xFFFF ^ foCompareTo->attributes;
    if (fHandle == 0) { if (CacheFileEntry(foDest,&fHandle,TRUE)==NULL) statusB=CE_BADCACHEREAD;}
    else {
        if ((fHandle & MASK_MAX_FILE_ENTRY_LIMIT_BITS) != 0) {
            if (CacheFileEntry (foDest, &fHandle, TRUE) == NULL) statusB = CE_BADCACHEREAD;
        }
    }
    if (statusB != CE_BADCACHEREAD) {
        while (1) {
            if (statusB != CE_GOOD) {
#if defined(SUPPORT_LFN)
                foundSFN = FALSE; foundLFN = FALSE;
                state = FillLFNObject(foDest,&lfnObject,&fHandle);
#else
                state = FillFileObject(foDest, &fHandle);
#endif
                if (state == NO_MORE) break;
            } else { break;}
            if (state == FOUND) {
#if defined(SUPPORT_LFN)
                if (lfnObject.LFN_Attribute != ATTR_LONG_NAME) {
                    lfnFirstCheck = FALSE; *dst = lfnObject.LFN_SequenceNo;
                    for (index=0; index<10; index++) dst[index+1]=lfnObject.LFN_Part1[index];
                    foundSFN = TRUE;
                } else {
                    if (lfnObject.LFN_SequenceNo & 0x40) {
                        lfnFoundMaxSequenceNum = lfnObject.LFN_SequenceNo & 0x1F;
                        if ((mode == 0x00) && ((fileNameLength && (lfnFoundMaxSequenceNum != lfnCompareMaxSequenceNum)) ||
                           (!fileNameLength && (lfnFoundMaxSequenceNum != 0x01))))
                        {
                            fHandle++; continue;
                        }
                        fileFoundLfnIndex = (lfnObject.LFN_SequenceNo & 0xBF) * MAX_UTF16_CHARS_IN_LFN_ENTRY - 1;
                        fileCompareLfnIndex = fileFoundLfnIndex;
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part3[1];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part3[0];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[5];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[4];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[3];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[2];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[1];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[0];
                        tempShift=(uint16_t)lfnObject.LFN_Part1[8]; tempShift|=(uint16_t)lfnObject.LFN_Part1[9]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[6]; tempShift|=(uint16_t)lfnObject.LFN_Part1[7]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[4]; tempShift|=(uint16_t)lfnObject.LFN_Part1[5]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[2]; tempShift|=(uint16_t)lfnObject.LFN_Part1[3]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[0]; tempShift|=(uint16_t)lfnObject.LFN_Part1[1]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        fileFoundLength = fileCompareLfnIndex + 1;
                        for (index = 1;index <= MAX_UTF16_CHARS_IN_LFN_ENTRY;index++) {
                            if (fileFoundString[fileFoundLfnIndex + index] == 0x0000)
                                fileFoundLength = fileFoundLfnIndex + index;
                        }
                        if (mode == 0x00) {
                            if ((fileNameLength != fileFoundLength) && fileNameLength) { fHandle++; continue;}
                        }
                        fileFoundMaxLfnIndex = fileFoundLength - 1; lfnFirstCheck = TRUE;
                    }
                    else if(lfnFirstCheck == TRUE)
                    {
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part3[1];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part3[0];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[5];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[4];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[3];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[2];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[1];
                        fileFoundString[fileFoundLfnIndex--] = lfnObject.LFN_Part2[0];
                        tempShift=(uint16_t)lfnObject.LFN_Part1[8]; tempShift|=(uint16_t)lfnObject.LFN_Part1[9]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[6]; tempShift|=(uint16_t)lfnObject.LFN_Part1[7]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[4]; tempShift|=(uint16_t)lfnObject.LFN_Part1[5]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[2]; tempShift|=(uint16_t)lfnObject.LFN_Part1[3]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                        tempShift=(uint16_t)lfnObject.LFN_Part1[0]; tempShift|=(uint16_t)lfnObject.LFN_Part1[1]<<8;
                        fileFoundString[fileFoundLfnIndex--] = tempShift;
                    } else { fHandle++; continue;}
                    if (fileFoundLfnIndex > 0) { fHandle++; continue;}
                    foundLFN = TRUE;
                }
                lfnFirstCheck = FALSE; statusB = CE_GOOD;
                switch (mode) {
                    case 0:
                        for (index = 0;index < FILE_NAME_SIZE_8P3;index++) tempDst[index] = dst[index];
                        if (dst[8] != ' ') {
                            for (index = 0;index < 8;index++) { if(dst[index] == ' ') break;}
                            tempDst[index++] = '.'; tempDst[index++] = dst[8];
                            if (dst[9]!=' ') tempDst[index++]=dst[9]; else tempDst[index++]=0x00;
                            if (dst[10]!=' ') tempDst[index++]=dst[10]; else tempDst[index++]=0x00;
                        } else { for (index=0; index<8; index++) { if (tempDst[index]==' ') break;}}
                        tempDst[index] = 0x00;
                        if (fileNameLength) {
                            if (foundLFN) {
                                for (fileCompareLfnIndex = 0;fileCompareLfnIndex < fileNameLength;fileCompareLfnIndex++) {
                                    if (foCompareTo -> AsciiEncodingType) {
                                        character = (uint8_t)templfnPtr[fileCompareLfnIndex];
                                        test = (uint8_t)fileFoundString[fileCompareLfnIndex];
                                        if ((fileFoundString[fileCompareLfnIndex]>0xFF)||(tolower(character)!=tolower(test))) {
                                            statusB = CE_FILE_NOT_FOUND; break;
                                        }
                                    } else {
                                        if (templfnPtr[fileCompareLfnIndex]!=fileFoundString[fileCompareLfnIndex]) {
                                            statusB = CE_FILE_NOT_FOUND; break;
                            }   }   }   }
                            else if (foundSFN && foCompareTo -> AsciiEncodingType)
                            {
                                if (strlen(tempDst) != fileNameLength) statusB = CE_FILE_NOT_FOUND;
                                else {
                                    for (fileCompareLfnIndex = 0;fileCompareLfnIndex < fileNameLength;fileCompareLfnIndex++) {
                                        character = (uint8_t)templfnPtr[fileCompareLfnIndex];
                                        test = tempDst[fileCompareLfnIndex];
                                        if (tolower(character)!=tolower(test)) { statusB=CE_FILE_NOT_FOUND; break; }
                                    }
                                }
                            } else { statusB = CE_FILE_NOT_FOUND;}
                        } else {
                            if (foundLFN) {
                                if (strlen(tempDst) != fileFoundLength) statusB = CE_FILE_NOT_FOUND;
                                else {
                                    for (fileCompareLfnIndex = 0;fileCompareLfnIndex < fileFoundLength;fileCompareLfnIndex++) {
                                        character = (uint8_t)fileFoundString[fileCompareLfnIndex];
                                        test = tempDst[fileCompareLfnIndex];
                                        if ((fileFoundString[fileCompareLfnIndex] > 0xFF) || (tolower(character) != tolower(test))) {
                                            statusB = CE_FILE_NOT_FOUND; // Nope its not a match
                                            break;
                            }   }   }   }
                            else {
                                for (index = 0; index < DIR_NAMECOMP; index++) {
                                    character = dst[index]; test = foCompareTo->name[index];
                                    if (tolower(character) != tolower(test)) { statusB = CE_FILE_NOT_FOUND; break;}
          	     	}   }   }
        		break;
                   case 1:
			if (fileNameLength) {
			    fileFoundDotPosition = FALSE;
			    if (foundLFN) {
				lfnCountIndex = fileFoundMaxLfnIndex;
				while (lfnCountIndex > 0) {
				    if (fileFoundString[lfnCountIndex] == '.') {
				        fileFoundDotPosition = TRUE; lfnCountIndex--; break;
				    }
				    lfnCountIndex--;
			        }
				if (fileFoundDotPosition == FALSE) lfnCountIndex = fileFoundMaxLfnIndex;
			    } else {
				if (dst[DIR_NAMESIZE] != ' ') fileFoundDotPosition = TRUE;
				lfnCountIndex = DIR_NAMESIZE - 1;
			    }
			    fileFoundLfnIndex = fileNameLength - 1; fileCompareDotPosition = FALSE;
			    while (fileFoundLfnIndex > 0) {
				if (templfnPtr[fileFoundLfnIndex] == '.') {
				    fileCompareDotPosition = TRUE; fileFoundLfnIndex--;
				    break;
				}
				fileFoundLfnIndex--;
			    }
			    if (fileCompareDotPosition == FALSE) fileFoundLfnIndex = fileNameLength - 1;
          	            for (fileCompareLfnIndex = 0;;) {
          	     	        if (templfnPtr[fileCompareLfnIndex] == '*') break;
				if (fileCompareLfnIndex > lfnCountIndex) { statusB = CE_FILE_NOT_FOUND; break;}
          	         	if (templfnPtr[fileCompareLfnIndex] != '?') {
				    if (foCompareTo -> AsciiEncodingType) {
          	         		character = (uint8_t)templfnPtr[fileCompareLfnIndex];
          	         		if(foundLFN) test = (uint8_t)fileFoundString[fileCompareLfnIndex];
					else  test = dst[fileCompareLfnIndex];
					if ((foundLFN && (fileFoundString[fileCompareLfnIndex] > 0xFF)) ||
			    		   (tolower(character) != tolower(test))) {
          	         		    statusB = CE_FILE_NOT_FOUND; break;
          	         		}
				    } else {
				  	if ((templfnPtr[fileCompareLfnIndex] != fileFoundString[fileCompareLfnIndex]) || foundSFN) {
          	    	   		    statusB = CE_FILE_NOT_FOUND;
				  	    break;
				}   }   }
               		    	fileCompareLfnIndex++;
               		  	if (fileCompareLfnIndex > fileFoundLfnIndex) {
               		            if (fileCompareLfnIndex <= lfnCountIndex) { statusB = CE_FILE_NOT_FOUND;}
               		    	    break;
				}
          	            }
			    if (fileCompareDotPosition == FALSE) {
				if (fileFoundDotPosition == TRUE) { statusB = CE_FILE_NOT_FOUND;}
				break;
			    } else {
				if (fileFoundDotPosition == FALSE) { statusB = CE_FILE_NOT_FOUND; break; }
			    if (foundLFN) lfnCountIndex = lfnCountIndex + 2;
			    else          lfnCountIndex = DIR_NAMESIZE;
			}
          	        for (fileCompareLfnIndex = fileFoundLfnIndex + 2;;) {
          	            if (templfnPtr[fileCompareLfnIndex] == '*') break;
		 	    if ((foundLFN && (lfnCountIndex > fileFoundMaxLfnIndex)) || (foundSFN && (lfnCountIndex == 11))) {
          	             	statusB = CE_FILE_NOT_FOUND;
          	             	break;
			    }
          	            if (templfnPtr[fileCompareLfnIndex] != '?') {
			        if (foCompareTo -> AsciiEncodingType) {
          	            	    character = (uint8_t)templfnPtr[fileCompareLfnIndex];
          	          	    if (foundLFN) test = (uint8_t)fileFoundString[lfnCountIndex];
				    else          test = dst[lfnCountIndex];
				    if ((foundLFN && (fileFoundString[lfnCountIndex] > 0xFF)) ||
          	         	       (tolower(character) != tolower(test))) {
          	         		statusB = CE_FILE_NOT_FOUND;
          	         		break;
          	         	    }
			        } else {
				    if ((templfnPtr[fileCompareLfnIndex] != fileFoundString[lfnCountIndex]) || foundSFN) {
          	    	   	        statusB = CE_FILE_NOT_FOUND;
				  	break;
		            }   }   }
               		    lfnCountIndex++; fileCompareLfnIndex++;
               		    if (fileCompareLfnIndex == fileNameLength) {
               		        if ((foundLFN && (lfnCountIndex <= fileFoundMaxLfnIndex)) || (foundSFN && (lfnCountIndex < 11) && (dst[lfnCountIndex] != ' '))) {
          	    	  	    statusB = CE_FILE_NOT_FOUND;
				}
               		    	break;
			    }
          	      	}
		    } else {
               		if (foundLFN) {
			    fileCompareLfnIndex = fileFoundMaxLfnIndex; fileFoundDotPosition = FALSE;
			    while (fileCompareLfnIndex > 0) {
				if (fileFoundString[fileCompareLfnIndex] == '.') {
			 	    fileFoundDotPosition = TRUE;
				    fileCompareLfnIndex--;
				    break;
				}
				fileCompareLfnIndex--;
			    }
			    if (fileFoundDotPosition == FALSE) fileCompareLfnIndex = fileFoundMaxLfnIndex;
			} else fileCompareLfnIndex = DIR_NAMESIZE - 1;
          		if (foCompareTo->name[0] != '*') {
               		    for (index = 0;;) {
               		        if (foundLFN) {
              		            if ((fileFoundString[index] > 0xFF) || (index > fileCompareLfnIndex)) {
               		    	       	statusB = CE_FILE_NOT_FOUND; break;
				    }
				}
          	         	if (foundLFN) character = (uint8_t)fileFoundString[index];
				else character = dst[index];
               		        test = foCompareTo->name[index];
               		        if (test == '*') break;
               		        if (test != '?') {
               		            if (tolower(character) != tolower(test)) { 
                                        statusB = CE_FILE_NOT_FOUND;
               		    	        break;
               		    	    }
               		    	}
               		    	index++;
               		    	if (index == DIR_NAMESIZE) {
               		    	    if (foundLFN && (index <= fileCompareLfnIndex)) {
               		    	      	statusB = CE_FILE_NOT_FOUND;
				    }
               		    	    break;
			}   }	}
               		if ((foCompareTo->name[8] != '*') && (statusB == CE_GOOD)) {
               		    if (foundLFN) {
			        if (foCompareTo->name[8] == ' ') {
				    if (fileFoundDotPosition == TRUE) { statusB = CE_FILE_NOT_FOUND;}
				    break;
				} else {
				    if (fileFoundDotPosition == FALSE) { statusB = CE_FILE_NOT_FOUND; break; }
				}
				fileCompareLfnIndex = fileCompareLfnIndex + 2;
			    } else fileCompareLfnIndex = DIR_NAMESIZE;
               		    for (index = 8;;) {
               		        if (foundLFN) {
               		    	    if ((fileFoundString[fileCompareLfnIndex] > 0xFF) || (fileCompareLfnIndex > fileFoundMaxLfnIndex)) {
               		    	        statusB = CE_FILE_NOT_FOUND;
               		    	        break;
				    }
				}
               		    	test = foCompareTo->name[index];
          	         	if (foundLFN) character = (uint8_t)fileFoundString[fileCompareLfnIndex++];
				else character = dst[fileCompareLfnIndex++];
               		        if (test == '*') break;
               		        if (test != '?') {
               		    	    if (tolower(character) != tolower(test)) {
               		    	        statusB = CE_FILE_NOT_FOUND;
               		    	        break;
               		    	    }
               		    	}
			        index++;
				if (index == DIR_NAMECOMP) {
				    if (foundLFN && (fileCompareLfnIndex <= fileFoundMaxLfnIndex)) statusB = CE_FILE_NOT_FOUND;
				    break;
		    }	}   }	}
          	    break;
		}
		if (statusB == CE_GOOD) {
		    if (foundLFN) fHandle++;
                    state = FillFileObject(foDest, &fHandle);
		    if (foundLFN) fHandle--;
               	    attrib = foDest->attributes; attrib &= ATTR_MASK;
               	    switch (mode) {
               	        case 0:
               	             if (attrib == ATTR_VOLUME) statusB = CE_FILE_NOT_FOUND;
               	             break;
               	        case 1:
               	             if ((attrib & compareAttrib) != 0) statusB = CE_FILE_NOT_FOUND;
               	             if (foundLFN) foDest->utf16LFNlength = fileFoundLength;
               	             else foDest->utf16LFNlength = 0;
               		     break;
               	    }
		}
#else /* (SUPPORT_LFN) */
                {
                    attrib = foDest->attributes; attrib &= ATTR_MASK;
                    switch (mode) {
               	        case 0:
               	             if (attrib != ATTR_VOLUME) {
               	                 statusB = CE_GOOD; character = (uint8_t)'m'; // random value
               	                 for (index = 0; index < DIR_NAMECOMP; index++) {
               	                     character = foDest->name[index]; test = foCompareTo->name[index];
               	                     if (tolower(character) != tolower(test)) {
               	                         statusB = CE_FILE_NOT_FOUND; break;
               	             }   }    }
               	             break;
               	        case 1:
               	             if (((attrib & compareAttrib) == 0) && (attrib != ATTR_LONG_NAME)) {
               	                 statusB = CE_GOOD; character = (uint8_t)'m';             // random value
               	                 if (foCompareTo->name[0] != '*')   //If "*" is passed for comparion as 1st char then don't proceed. Go back, file alreay found.
               	                 {
               	                     for (index = 0; index < DIR_NAMESIZE; index++) {
               	                         character = foDest->name[index]; test = foCompareTo->name[index];
               	                         if (test == '*') break;
               	                         if (test != '?') {
               	                             if (tolower(character) != tolower(test)) { statusB = CE_FILE_NOT_FOUND; break;}
               	                 }   }   }
               	                 if ((foCompareTo->name[8] != '*') && (statusB == CE_GOOD)) {
               	                     for (index = 8; index < DIR_NAMECOMP; index++) {
               	                         character = foDest->name[index]; test = foCompareTo->name[index];
               	                         if (test == '*') break;
               	                         if (test != '?') {
               	                             if (tolower(character) != tolower(test)) { statusB = CE_FILE_NOT_FOUND; break;}
               	             }   }   }   }
               	             break;
               	    }
                }
#endif /* (SUPPORT_LFN) */
            } else {
#if defined(SUPPORT_LFN)
                lfnFirstCheck = FALSE;
#endif
                if ( cmd == LOOK_FOR_EMPTY_ENTRY) statusB = CE_GOOD;
            }
            fHandle++;
        }
    }
    return(statusB);
} // FILEFind

CETYPE  SDC::FILEopen (FILEOBJ fo, uint16_t *fHandle, char type) {
    DISK * dsk;  uint8_t   r; uint32_t  l;
    CETYPE  error = CE_GOOD; dsk = (DISK *)(fo->dsk);
    if (dsk->mount == FALSE) { error = CE_NOT_INIT;}
    else  {
        fo->dirccls = fo->dirclus;
        if (*fHandle == 0) { if (CacheFileEntry(fo, fHandle, TRUE) == NULL) error = CE_BADCACHEREAD;}
        else {
            if ((*fHandle & 0xf) != 0) {
                if (CacheFileEntry (fo, fHandle, TRUE) == NULL) error = CE_BADCACHEREAD;
            }
        }
        r = FillFileObject(fo, fHandle);
        if (r != FOUND) error = CE_FILE_NOT_FOUND;
        else {
            fo->seek = 0; fo->ccls = fo->cluster; fo->sec = 0; fo->pos = 0;
            if (r == NOT_FOUND) { error = CE_FILE_NOT_FOUND;}
            else {
                l = Cluster2Sector(dsk,fo->ccls);
#ifdef ALLOW_WRITES
                if (gNeedDataWrite) if (FlushData())  return CE_WRITE_ERROR;
#endif
                gBufferOwner = fo;
                if (gLastDataSectorRead != l) {
                    gBufferZeroed = FALSE;
                    if ( !SDSectorRead( l, dsk->buffer)) error = CE_BAD_SECTOR_READ;
                    gLastDataSectorRead = l;
            }   }
            fo->flags.FileWriteEOF = FALSE;
#ifdef ALLOW_WRITES
            if ((type == 'w') || (type == 'a')) { fo->flags.write = 1; fo->flags.read = 0;}
            else {
#endif
                fo->flags.write = 0; fo->flags.read = 1;
#ifdef ALLOW_WRITES
            }
#endif
    }   }
    return (error);
} // -- FILEopen

FILE_DIR_NAME_TYPE  SDC::ValidateChars(uint8_t mode) {
    FILE_DIR_NAME_TYPE fileNameType;
    unsigned short int count1; unsigned char radix = FALSE,asciiValue;
#if defined(SUPPORT_LFN)
    unsigned short int utf16Value; unsigned short int count2; int count3;
    for (count1 = 0; count1 < fileNameLength; count1++) {
        if (utfModeFileName) {
            if ((utf16Filename[count1] != ' ') && (utf16Filename[count1] != '.')) {
                utf16Filename = utf16Filename + count1; break;
            }
        }
        else if((asciiFilename[count1] != ' ') && (asciiFilename[count1] != '.'))
        { asciiFilename = asciiFilename + count1; break; }
    }
    count2 = 0;
    for (count3 = fileNameLength - count1 - 1; count3 > 0; count3--) {
        if (utfModeFileName) {
            if ((utf16Filename[count3]!=' ')&&(utf16Filename[count3]!='.')) { break;}
        } else if ((asciiFilename[count3]!=' ')&&(asciiFilename[count3]!='.')) { break;}
        count2++;
    }
    fileNameLength = fileNameLength - count1 - count2;
    if ((fileNameLength > MAX_FILE_NAME_LENGTH_LFN ) || (fileNameLength == 0))  return NAME_ERROR;
#endif
#ifdef SUPPORT_LFN
    if (utfModeFileName) {
        if ((fileNameLength * 2) > (TOTAL_FILE_SIZE_8P3 * 2))  fileNameType = NAME_LFN_TYPE;
        else  fileNameType = NAME_8P3_UTF16_TYPE;
    } else
#endif
    {
        if (fileNameLength > TOTAL_FILE_SIZE_8P3)  fileNameType = NAME_LFN_TYPE;
        else fileNameType = NAME_8P3_ASCII_CAPS_TYPE;
    }
    for (count1 = 0; count1 < fileNameLength; count1++) {
#ifdef SUPPORT_LFN
        if (utfModeFileName) {
            utf16Value = utf16Filename[count1];
            if (((utf16Value<0x20)&&(utf16Value!=0x05))||(utf16Value==0x22)||(utf16Value==0x2F)||(utf16Value==0x3A)||(utf16Value==0x3C)||(utf16Value==0x3E)||(utf16Value==0x5C)||(utf16Value==0x7C))  return NAME_ERROR;
            if (mode == FALSE)  if ((utf16Value == '*') || (utf16Value == '?'))  return NAME_ERROR;
            if (fileNameType != NAME_LFN_TYPE) {
                if ((utf16Value==0x20)||(utf16Value==0x2B)||(utf16Value==0x2C)||(utf16Value==0x3B)||(utf16Value==0x3D)||(utf16Value==0x5B)||(utf16Value==0x5D)||((utf16Value==0x2E)&&(radix==TRUE))) {
                    fileNameType = NAME_LFN_TYPE; continue;
                }
                if (utf16Filename[count1] == 0x2E) radix=TRUE;
            }
        } else
#endif
        {
            asciiValue = asciiFilename[count1];
            if (((asciiValue<0x20)&&(asciiValue!=0x05))||(asciiValue==0x22)||(asciiValue==0x2F)||(asciiValue==0x3A)||(asciiValue==0x3C)||(asciiValue==0x3E)||(asciiValue==0x5C)||(asciiValue==0x7C)) return NAME_ERROR;
            if (mode == FALSE)  if ((asciiValue == '*') || (asciiValue == '?'))  return NAME_ERROR;
            if (fileNameType != NAME_LFN_TYPE) {
                if ((asciiValue==0x20)||(asciiValue==0x2B)||(asciiValue==0x2C)||(asciiValue==0x3B)||(asciiValue==0x3D)||(asciiValue==0x5B)||(asciiValue==0x5D)||((asciiValue==0x2E)&&(radix==TRUE))) {
                    fileNameType = NAME_LFN_TYPE; continue;
                }
                if (asciiValue == 0x2E)  radix = TRUE;
                if (fileNameType!=NAME_8P3_ASCII_MIXED_TYPE) if((asciiValue>=0x61)&&(asciiValue<=0x7A)) fileNameType=NAME_8P3_ASCII_MIXED_TYPE;
            }
        }
    }
    return fileNameType;
}

DIRENTRY  SDC::CacheFileEntry( FILEOBJ fo, uint16_t * curEntry, uint8_t ForceRead) {
    DIRENTRY dir; DISK *dsk; uint32_t sector, cluster, LastClusterLimit, ccls;
    uint8_t  offset2, numofclus, dirEntriesPerSector; dsk = fo->dsk; cluster = fo->dirclus;
    ccls = fo->dirccls; dirEntriesPerSector = dsk->sectorSize/32; offset2 = (*curEntry / dirEntriesPerSector);
    switch (dsk->type) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            offset2  = offset2 % (dsk->SecPerClus);
            LastClusterLimit = LAST_CLUSTER_FAT32;
            break;
#endif
        case FAT12:
        case FAT16:
        default:
            if(cluster != 0)  offset2  = offset2 % (dsk->SecPerClus);
            LastClusterLimit = LAST_CLUSTER_FAT16;
            break;
    }
    if (ForceRead || ((*curEntry & MASK_MAX_FILE_ENTRY_LIMIT_BITS) == 0)) {
        if (((offset2 == 0) && (*curEntry >= dirEntriesPerSector)) || ForceRead) {
            if (cluster == 0)  ccls = 0;
            else {
                if (ForceRead)  numofclus = ((uint16_t)(*curEntry) / (uint16_t)(((uint16_t)dirEntriesPerSector) * (uint16_t)dsk->SecPerClus));
                else  numofclus = 1;
                while (numofclus) {
                    ccls = ReadFAT(dsk, ccls);
                    if (ccls >= LastClusterLimit)  break;
                    else  numofclus--;
        }   }   }
        if (ccls < LastClusterLimit) {
            fo->dirccls = ccls; sector = Cluster2Sector(dsk,ccls);
            if ((ccls == FatRootDirClusterValue) && ((sector + offset2) >= dsk->data) && (FAT32 != dsk->type))  dir = ((DIRENTRY)NULL);
            else {
#ifdef ALLOW_WRITES
                if (gNeedDataWrite)  if (FlushData())  return NULL;
#endif
                gBufferOwner = NULL; gBufferZeroed = FALSE;
                if (SDSectorRead( sector + offset2, dsk->buffer) != TRUE)  dir = ((DIRENTRY)NULL);
                else {
                    if (ForceRead)  dir = (DIRENTRY)((DIRENTRY)dsk->buffer) + ((*curEntry)%dirEntriesPerSector);
                    else  dir = (DIRENTRY)dsk->buffer;
                }
                gLastDataSectorRead = 0xFFFFFFFF;
            }
        } else { nextClusterIsLast = TRUE; dir = ((DIRENTRY)NULL);}
    } else { dir = (DIRENTRY)((DIRENTRY)dsk->buffer) + ((*curEntry)%dirEntriesPerSector);}
    return(dir);
}

uint8_t  SDC::FillFileObject(FILEOBJ fo, uint16_t *fHandle) {
    DIRENTRY dir; uint8_t index, a, character, status, test = 0;
    if (((*fHandle & MASK_MAX_FILE_ENTRY_LIMIT_BITS) == 0) && (*fHandle != 0)) {
        fo->dirccls = fo->dirclus; dir = CacheFileEntry(fo, fHandle, TRUE);
    } else { dir = CacheFileEntry (fo, fHandle, FALSE); }
    if (dir == (DIRENTRY)NULL)  status = NO_MORE;
    else {
        a = dir->DIR_Name[0];
        if ( a == DIR_DEL)  status = NOT_FOUND;
        else if ( a == DIR_EMPTY)  status = NO_MORE;
        else
        {
            a = dir->DIR_Attr;
            for (index=0; index < DIR_NAMESIZE; index++) {
                character = dir->DIR_Name[index]; character = (uint8_t)toupper(character);
                fo->name[test++] = character;
            }
            a = dir->DIR_Attr; character = dir->DIR_Extension[0];
            for (index=0; index < DIR_EXTENSION; index++) {
                character = dir->DIR_Extension[index]; character = (uint8_t)toupper(character);
                fo->name[test++] = character;
            }
            fo->entry = *fHandle; a = dir->DIR_Name[0];
            if (a == DIR_DEL)  status = NOT_FOUND; else  status = FOUND;
            fo->size = (dir->DIR_FileSize); fo->cluster = GetFullClusterNumber(dir);
            a = dir->DIR_Attr; fo->attributes = a;
            if ((a & ATTR_DIRECTORY) != 0) { fo->time = dir->DIR_CrtTime; fo->date = dir->DIR_CrtDate;}
            else { fo->time = dir->DIR_WrtTime; fo->date = dir->DIR_WrtDate;}
        }
    }
    return(status);
}

uint32_t  SDC::Cluster2Sector(DISK * dsk, uint32_t cluster) {
    uint32_t sector;
    switch (dsk->type)
    {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            sector = (((uint32_t)cluster-2) * dsk->SecPerClus) + dsk->data;
            break;
#endif
        case FAT12:
        case FAT16:
        default:
            if((cluster == 0) || (cluster == 1))  sector = dsk->root + cluster;
            else  sector = (((uint32_t)cluster-2) * dsk->SecPerClus) + dsk->data;
            break;
    }
    return(sector);
}

uint8_t  SDC::SDFile2JPEGFIFO(uint32_t fifoAddress, uint32_t byteSize, FSFILE *stream) {
    DISK *dsk; uint32_t  len, seek, sec_sel, pos; len  = byteSize / MEDIA_SECTOR_SIZE;
    dsk  = stream->dsk; pos  = stream->pos; seek = stream->seek;
    sec_sel = (uint32_t) Cluster2Sector(dsk, stream->ccls); sec_sel += (uint32_t) stream->sec;
    while (len) {
        if (seek >= stream->size) { seek = stream->size; pos = 0; break;}
        if (stream->sec >= dsk->SecPerClus) {
            stream->sec = 0;
            if (FileGetNextCluster(stream, (uint32_t) 1) != CE_GOOD) { return (FALSE);}
            sec_sel = (uint32_t) Cluster2Sector(dsk, (uint32_t) stream->ccls);
            sec_sel += (uint32_t) stream->sec;
        }
        if (((uint32_t) (stream->sec) + (uint32_t) len) > (uint32_t) (dsk->SecPerClus))  pos = dsk->SecPerClus - stream->sec;
        else  pos = len;
        if (!SDSectorDMARead(sec_sel, fifoAddress, pos))  return (FALSE);
        len -= pos; stream->sec += pos; sec_sel += pos;
        seek += pos * MEDIA_SECTOR_SIZE; fifoAddress += pos * MEDIA_SECTOR_SIZE;
    }
    stream->pos = 0; stream->seek = seek;
    return (TRUE);
}

uint32_t  SDC::GetFullClusterNumber(DIRENTRY entry) {
    uint32_t TempFullClusterCalc = 0;
#ifndef SUPPORT_FAT32 // If FAT32 Not supported.
    entry->DIR_FstClusHI = 0;
#endif
    TempFullClusterCalc  = (entry->DIR_FstClusHI); TempFullClusterCalc  = TempFullClusterCalc << 16;
    TempFullClusterCalc |= entry->DIR_FstClusLO; return TempFullClusterCalc;
}

uint8_t  SDC::SDSectorDMARead(uint32_t sector_addr, uint32_t dma_addr, uint16_t num_blk) {
    uint32_t  dma_size; uint8_t  boundary; if (!hcMode)  sector_addr *= sectorSize;
    while (rdReg8(0x1124) & 0x02); SDReset(RESET_DATA); wrReg32(0x1100, dma_addr);
    wrReg8(0x110C, 0x33); dma_size = (uint32_t) num_blk * sectorSize; dma_size >>= 12; boundary = 0;
    while (dma_size) { dma_size >>= 1; boundary++;}
    boundary <<= 4; boundary |= ((uint8_t)(sectorSize>>8) & 0x0F);
    wrReg8(0x1104, (uint8_t) sectorSize); wrReg8(0x1105, boundary);
    wrReg16(0x1106, num_blk); wrReg8(0x1134, 0x0B);
    wrReg8(0x1130, rdReg8(0x1130) | 0x0B); wrReg8(0x1136, 0xFF);
    wrReg8(0x1132, 0xFF); wrReg8(0x1133, 0x01); wrReg8(0x1130, 0x08);
    if (!SDSendCmd(CMD_RD_MULTIPLE, RESPONSE_48 | DATA_PRESENT | CMD_CRC_CHK | CMD_IDX_CHK, sector_addr)) return (FALSE);
    while (!(rdReg8(0x1130) & 0x08) && (rdReg8(0x1125) & 0x02));
    wrReg8(0x1130, 0x08);
    if (!SDSendCmd(CMD_STOP_TRANSMISSION, CMD_TYPE_ABORT | RESPONSE_48_BUSY | CMD_CRC_CHK | CMD_IDX_CHK, 0xFFFFFFFF)) return (FALSE);
    wrReg8(0x1130, 0x01); wrReg16(0x1106, 0); wrReg8(0x110C, 0);
    return (TRUE);
}

FSFILE * SDC::FSfopen( const char * fileName, const char *mode ) {
    FILEOBJ  filePtr; int  fIndex; uint8_t  ModeC;
    uint16_t  fHandle; CETYPE  final; ModeC = mode[0];
    if (SDWriteProtectState() && (ModeC != 'r') && (ModeC != 'R')) { FSerrno = CE_WRITE_PROTECTED; return NULL;}
    filePtr = NULL;
    for (fIndex = 0; fIndex < FS_MAX_FILES_OPEN; fIndex++) {
        if (gFileSlotOpen[fIndex]) { gFileSlotOpen[fIndex] = FALSE; filePtr = &gFileArray[fIndex]; break;}
    }
    if (filePtr == NULL) { FSerrno = CE_TOO_MANY_FILES_OPEN; return NULL;}
#if defined(SUPPORT_LFN)
    filePtr->utf16LFNptr = &lfnData[fIndex][0];
#endif
    if (!FormatFileName(fileName, filePtr, 0)) {
        gFileSlotOpen[fIndex] = TRUE; FSerrno = CE_INVALID_FILENAME;
        return NULL;
    }
    filePtr->dsk = &gDiskData; filePtr->cluster = 0; filePtr->ccls = 0;
    filePtr->entry = 0; filePtr->attributes = ATTR_ARCHIVE; filePtr->dirclus = cwdptr->dirclus;
    filePtr->dirccls = cwdptr->dirccls; FileObjectCopy(&gFileTemp, filePtr);
    if (FILEfind (filePtr, &gFileTemp, LOOK_FOR_MATCHING_ENTRY, 0) == CE_GOOD) {
        switch (ModeC) {
#ifdef ALLOW_WRITES
            case 'w':
            case 'W':
            {
                fHandle = filePtr->entry; final = FileErase(filePtr, &fHandle, TRUE);
                if (final == CE_GOOD) {
                    final = CreateFileEntry (filePtr, &fHandle, 0, TRUE);
                    if (final == CE_GOOD) {
                        final = FILEopen (filePtr, &fHandle, 'w');
                        if (filePtr->attributes & ATTR_DIRECTORY) {
                            FSerrno = CE_INVALID_ARGUMENT; final = (CETYPE)0xFF;
                        }
                        if (final == CE_GOOD) {
                            final = (CETYPE)FSfseek (filePtr, 0, SEEK_END);
                            if (mode[1] == '+') filePtr->flags.read = 1;
                }   }   }
                break;
            }
            case 'A':
            case 'a':
            {
                if (filePtr->size != 0) {
                    fHandle = filePtr->entry; final = FILEopen (filePtr, &fHandle, 'w');
                    if (filePtr->attributes & ATTR_DIRECTORY) {
                        FSerrno = CE_INVALID_ARGUMENT; final = (CETYPE)0xFF;
                    }
                    if (final == CE_GOOD) {
                        final = (CETYPE)FSfseek (filePtr, 0, SEEK_END);
                        if (final != CE_GOOD)  FSerrno = CE_SEEK_ERROR;
                        else                   ReadFAT (&gDiskData, filePtr->ccls);
                        if (mode[1] == '+') filePtr->flags.read = 1;
                    }
                } else {
                    fHandle = filePtr->entry; final = FileErase(filePtr, &fHandle, TRUE);
                    if (final == CE_GOOD) {
                        final = CreateFileEntry (filePtr, &fHandle, 0, TRUE);
                        if (final == CE_GOOD) {
                            final = FILEopen (filePtr, &fHandle, 'w');
                            if (filePtr->attributes & ATTR_DIRECTORY) {
                                FSerrno = CE_INVALID_ARGUMENT; final = (CETYPE)0xFF;
                            }
                            if (final == CE_GOOD) {
                                final = (CETYPE)FSfseek (filePtr, 0, SEEK_END);
                                if (final != CE_GOOD) FSerrno = CE_SEEK_ERROR;
                                if (mode[1] == '+') filePtr->flags.read = 1;
                }   }   }   }
                break;
            }
#endif
            case 'R':
            case 'r':
            {
                fHandle = filePtr->entry; final = FILEopen (filePtr, &fHandle, 'r');
#ifdef ALLOW_WRITES
                if ((mode[1] == '+') && !(filePtr->attributes & ATTR_DIRECTORY))
                    filePtr->flags.write = 1;
#endif
                break;
            }
            default:
                FSerrno = CE_INVALID_ARGUMENT; final = (CETYPE)0xFF;;
                break;
        }
    } else {
#ifdef ALLOW_WRITES
        FileObjectCopy(filePtr, &gFileTemp);
        if ((ModeC == 'w') || (ModeC == 'W') || (ModeC == 'a') || (ModeC == 'A')) {
            fHandle = 0; final = CreateFileEntry (filePtr, &fHandle, 0, TRUE);
            if (final == CE_GOOD) {
                final = FILEopen (filePtr, &fHandle, 'w');
                if (filePtr->attributes & ATTR_DIRECTORY) {
                    FSerrno = CE_INVALID_ARGUMENT; final = (CETYPE)0xFF;
                }
                if (final == CE_GOOD) {
                    final = (CETYPE)FSfseek (filePtr, 0, SEEK_END);
                    if (final != CE_GOOD) FSerrno = CE_SEEK_ERROR;
                    if (mode[1] == '+') filePtr->flags.read = 1;
                }
            }
        } else
#endif
        { final = CE_FILE_NOT_FOUND; FSerrno = CE_FILE_NOT_FOUND; }
    }
    if (SDWriteProtectState()) { filePtr->flags.write = 0;; }
    if (final != CE_GOOD) {
        gFileSlotOpen[fIndex] = TRUE;   //put this slot back to the pool
        filePtr = NULL;
    } else { FSerrno = CE_GOOD; }
    return filePtr;
}

int  SDC::FSfseek(FSFILE *stream, long offset, int whence) {
    uint32_t  numsector, temp; DISK * dsk; uint8_t  test;
    long  offset2 = offset; dsk = stream->dsk;
    switch (whence) {
        case SEEK_CUR:
            offset2 += stream->seek;
            break;
        case SEEK_END:
            offset2 = stream->size - offset2;
            break;
        case SEEK_SET:
        default:
            break;
   }
#ifdef ALLOW_WRITES
    if (gNeedDataWrite) if (FlushData()) { FSerrno = CE_WRITE_ERROR; return EOF;}
#endif
    temp = stream->cluster; stream->ccls = temp; temp = stream->size;
    if (offset2 > temp) { FSerrno = CE_INVALID_ARGUMENT; return (-1);}
    else {
        stream->flags.FileWriteEOF = FALSE; stream->seek = offset2;
        numsector = offset2 / dsk->sectorSize;
        offset2   = offset2 - (numsector * dsk->sectorSize);
        stream->pos = offset2; temp = numsector / dsk->SecPerClus;
        numsector = numsector - (dsk->SecPerClus * temp);
        stream->sec = numsector;
        if (temp > 0) {
            test = FileGetNextCluster(stream, temp);
            if (test != CE_GOOD) {
                if (test == CE_FAT_EOF) {
#ifdef ALLOW_WRITES
                    if (stream->flags.write) {
                        stream->ccls = stream->cluster;
                        if (temp != 1) test = FileGetNextCluster(stream, temp - 1);
                        if (FileAllocateNewCluster(stream, 0) != CE_GOOD) {
                            FSerrno = CE_COULD_NOT_GET_CLUSTER; return -1;
                        }
                    } else {
#endif
                        stream->ccls = stream->cluster;
                        test = FileGetNextCluster(stream, temp - 1);
                        if (test != CE_GOOD) { FSerrno = CE_COULD_NOT_GET_CLUSTER; return (-1);}
                        stream->pos = dsk->sectorSize; stream->sec = dsk->SecPerClus - 1;
#ifdef ALLOW_WRITES
                    }
#endif
                } else { FSerrno = CE_COULD_NOT_GET_CLUSTER; return (-1); }
            }
        }
        temp = Cluster2Sector(dsk,stream->ccls); numsector = stream->sec;
        temp += numsector; gBufferOwner = NULL; gBufferZeroed = FALSE;
        if (!SDSectorRead(temp, dsk->buffer)) { FSerrno = CE_BADCACHEREAD; return (-1);}
        gLastDataSectorRead = temp;
    }
    FSerrno = CE_GOOD;
    return (0);
}

int  SDC::FSfclose(FSFILE * fo) {
    uint16_t  fHandle; uint16_t  fIndex; int  error = 72;
#ifdef ALLOW_WRITES
    DIRENTRY    dir;
#endif
    FSerrno = CE_GOOD; fHandle = fo->entry;
#ifdef ALLOW_WRITES
    if (fo->flags.write) {
        if (gNeedDataWrite) { if (FlushData()) { FSerrno = CE_WRITE_ERROR; return EOF;}}
        WriteFAT (fo->dsk, 0, 0, TRUE); gLastFATSectorRead = 0;
        ReadFAT (fo->dsk, fo->ccls); dir = LoadDirAttrib(fo, &fHandle);
        if (dir == NULL) { FSerrno = CE_BADCACHEREAD; error = EOF; return error;}
#ifdef INCREMENTTIMESTAMP
        IncrementTimeStamp(dir);
#elif defined USERDEFINEDCLOCK
        dir->DIR_WrtTime = gTimeWrtTime; dir->DIR_WrtDate = gTimeWrtDate;
#elif defined USEREALTIMECLOCK
        CacheTime(); dir->DIR_WrtTime = gTimeWrtTime; dir->DIR_WrtDate = gTimeWrtDate;
#endif
        dir->DIR_FileSize = fo->size; dir->DIR_Attr = fo->attributes;
        if (WriteFileEntry(fo,&fHandle)) { dir = LoadDirAttrib(fo, &fHandle); error = 0;}
        else { FSerrno = CE_WRITE_ERROR; error = EOF;}
        fo->flags.write = FALSE;
    }
#endif
    for (fIndex = 0; fIndex < FS_MAX_FILES_OPEN; fIndex++) {
        if (fo == &gFileArray[fIndex]) { gFileSlotOpen[fIndex] = TRUE; break;}
    }
    if (error == 72) error = 0;
    return(error);
}

uint8_t  SDC::FileGetNextCluster(FSFILE *fo, uint32_t n)
{
    uint32_t  c, c2, ClusterFailValue, LastClustervalue;
    uint8_t  error = CE_GOOD; DISK * disk; disk = fo->dsk;
    switch (disk->type) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            LastClustervalue  = LAST_CLUSTER_FAT32; ClusterFailValue  = CLUSTER_FAIL_FAT32;
            break;
#endif
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        case FAT12:
            LastClustervalue  = LAST_CLUSTER_FAT12; ClusterFailValue  = CLUSTER_FAIL_FAT16;
            break;
#endif
        case FAT16:
        default:
            LastClustervalue  = LAST_CLUSTER_FAT16; ClusterFailValue  = CLUSTER_FAIL_FAT16;
            break;
    }
    do {
        c2 = fo->ccls;
        if ((c = ReadFAT( disk, c2)) == ClusterFailValue) error = CE_BAD_SECTOR_READ;
        else {
            if (c >= disk->maxcls) error = CE_INVALID_CLUSTER;
            if (c >= LastClustervalue) error = CE_FAT_EOF;
        }
        fo->ccls = c;
    } while ((--n > 0) && (error == CE_GOOD));
    return(error);
}


size_t  SDC::FSfwrite(const void *data_to_write, size_t size, size_t n, FSFILE *stream) {
    uint32_t  count = size * n, l, seek, filesize; uint8_t  *src = (uint8_t *) data_to_write;
    DISK * dsk; CETYPE  error = CE_GOOD; uint16_t  pos, writeCount = 0;
    if (!(stream->flags.write)) { FSerrno = CE_READONLY; error = CE_WRITE_ERROR; return 0;}
    if (count == 0) return 0;
    if (SDWriteProtectState()) { FSerrno = CE_WRITE_PROTECTED; error = CE_WRITE_PROTECTED; return 0;}
    gBufferZeroed = FALSE; dsk = stream->dsk; pos = stream->pos; seek = stream->seek;
    l = Cluster2Sector(dsk,stream->ccls); l += (uint16_t)stream->sec;
    if (gBufferOwner != stream) {
        if (gNeedDataWrite) { if (FlushData()) { FSerrno = CE_WRITE_ERROR; return 0;}}
        gBufferOwner = stream;
    }
    if (gLastDataSectorRead != l) {
        if (gNeedDataWrite) { if (FlushData()) { FSerrno = CE_WRITE_ERROR; return 0;}}
        gBufferZeroed = FALSE;
        if (!SDSectorRead( l, dsk->buffer)) { FSerrno = CE_BADCACHEREAD; error = CE_BAD_SECTOR_READ;}
        gLastDataSectorRead = l;
    }
    filesize = stream->size;
    while ((error == CE_GOOD) && (count > 0)) {
        if (seek == filesize) stream->flags.FileWriteEOF = TRUE;
        if (pos == dsk->sectorSize) {
            uint8_t needRead = TRUE;
            if (gNeedDataWrite) { if (FlushData()) { FSerrno = CE_WRITE_ERROR; return 0;}}
            pos = 0; stream->sec++;
            if (stream->sec == dsk->SecPerClus) {
                stream->sec = 0;
                if (stream->flags.FileWriteEOF) { error = FileAllocateNewCluster(stream, 0); needRead = FALSE;}
                else  error = FileGetNextCluster( stream, 1);
            }
            if (error == CE_DISK_FULL) { FSerrno = CE_DISK_FULL; return 0;}
            if (error == CE_GOOD) {
                l = Cluster2Sector(dsk,stream->ccls); l += (uint16_t)stream->sec;
                gBufferOwner = stream;
                if (needRead) {
                    if (!SDSectorRead( l, dsk->buffer)) {
                        FSerrno = CE_BADCACHEREAD; error = CE_BAD_SECTOR_READ;
                        gLastDataSectorRead = 0xFFFFFFFF; return 0;
                    } else { gLastDataSectorRead = l;}
                } else gLastDataSectorRead = l;
            }
        }
        if (error == CE_GOOD) {
            RAMwrite(dsk->buffer, pos++, *(char *)src); src = src + 1; seek++; count--; writeCount++;
            if (stream->flags.FileWriteEOF) filesize++; gNeedDataWrite = TRUE;
        }
    }
    stream->pos = pos; stream->seek = seek; stream->size = filesize;
    return(writeCount / size);
}

size_t  SDC::FSfread (void *ptr, size_t size, size_t n, FSFILE *stream) {
    uint32_t len = size * n; uint8_t  *pointer = (uint8_t *) ptr;
    DISK * dsk; uint32_t  seek, sec_sel; uint16_t  pos;
    CETYPE error = CE_GOOD; uint16_t  readCount = 0; FSerrno = CE_GOOD;
    dsk = (DISK *)stream->dsk; pos = stream->pos; seek = stream->seek;
    if ( !stream->flags.read ) { FSerrno = CE_WRITEONLY; return 0; }
#ifdef ALLOW_WRITES
    if (gNeedDataWrite) if (FlushData()) { FSerrno = CE_WRITE_ERROR; return 0;}
#endif
    // if it not my buffer, then get it from the disk.
    if ((gBufferOwner != stream) && (pos != MEDIA_SECTOR_SIZE)) {
        gBufferOwner = stream; sec_sel = Cluster2Sector(dsk,stream->ccls);
        sec_sel += (uint16_t)stream->sec; gBufferZeroed = FALSE;
        if (!SDSectorRead(sec_sel, dsk->buffer)) {
            FSerrno = CE_BAD_SECTOR_READ; error = CE_BAD_SECTOR_READ; return 0;
        }
        gLastDataSectorRead = sec_sel;
    }
    //loop reading (count) bytes
    while( len )
    {
        if (seek == stream->size) { FSerrno = CE_EOF; error = CE_EOF; break;}
        // In fopen, pos is init to 0 and the sect is loaded
        if (pos == MEDIA_SECTOR_SIZE) {
            pos = 0; stream->sec++;
            if (stream->sec == dsk->SecPerClus) {
                stream->sec = 0;
                if ((error = FileGetNextCluster( stream, 1)) != CE_GOOD) {
                    FSerrno = CE_COULD_NOT_GET_CLUSTER; break;
                }
            }
            sec_sel = Cluster2Sector(dsk,stream->ccls); sec_sel += (uint16_t)stream->sec;
            gBufferOwner = stream; gBufferZeroed = FALSE;
            if (!SDSectorRead( sec_sel, dsk->buffer)) {
                FSerrno = CE_BAD_SECTOR_READ; error = CE_BAD_SECTOR_READ; break;
            }
            gLastDataSectorRead = sec_sel;
        }
        *pointer = RAMread( dsk->buffer, pos++); pointer++; seek++; readCount++; len--;
    }
    stream->pos = pos; stream->seek = seek;
    return(readCount / size);
}

uint8_t  SDC::FlushData (void) {
    uint32_t l; DISK * dsk; FILEOBJ stream = gBufferOwner; dsk = stream->dsk;
    l = Cluster2Sector(dsk,stream->ccls); l += (uint16_t)stream->sec;
    if (!SDSectorWrite( l, dsk->buffer, FALSE)) { return CE_WRITE_ERROR;}
    gNeedDataWrite = FALSE; return CE_GOOD;
}

uint8_t  SDC::FileAllocateNewCluster( FILEOBJ fo, uint8_t mode) {
    DISK * dsk; uint32_t  c,curcls; dsk = fo->dsk; c = fo->ccls; c = FATFindEmptyCluster(fo);
    if (c == 0) return CE_DISK_FULL;
#ifdef SUPPORT_FAT12 // If FAT12 supported.
    if (dsk->type == FAT12) WriteFAT( dsk, c, LAST_CLUSTER_FAT12, FALSE);
    else
#endif
    if (dsk->type == FAT16) WriteFAT( dsk, c, LAST_CLUSTER_FAT16, FALSE);
#ifdef SUPPORT_FAT32 // If FAT32 supported.
    else WriteFAT( dsk, c, LAST_CLUSTER_FAT32, FALSE);
#endif
    curcls = fo->ccls; WriteFAT(dsk, curcls, c, FALSE); fo->ccls = c;
    if (mode == 1)  return (EraseCluster(dsk, c));
    else            return CE_GOOD;
}

uint32_t  SDC::FATFindEmptyCluster(FILEOBJ fo) {
    DISK * disk; uint32_t  value = 0x0; uint32_t  c,curcls, EndClusterLimit, ClusterFailValue;
    disk = fo->dsk; c = fo->ccls;
    switch (disk->type) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            EndClusterLimit = END_CLUSTER_FAT32; ClusterFailValue = CLUSTER_FAIL_FAT32;
            break;
#endif

#ifdef SUPPORT_FAT12 // If FAT12 supported.
        case FAT12:
            EndClusterLimit = END_CLUSTER_FAT12; ClusterFailValue = CLUSTER_FAIL_FAT16;
            break;
#endif
        case FAT16:
        default:
            EndClusterLimit = END_CLUSTER_FAT16; ClusterFailValue = CLUSTER_FAIL_FAT16;
            break;
    }
    if (c < 2) c = 2;
    curcls = c; ReadFAT(disk, c);
    while (c) {
        if ((value = ReadFAT(disk, c)) == ClusterFailValue) { c = 0; break;}
        if (value == CLUSTER_EMPTY) break;
        c++;
        if ((value == EndClusterLimit) || (c >= (disk->maxcls+2))) c = 2;
        if ( c == curcls) { c = 0; break;}
    }
    return(c);
}

uint32_t  SDC::WriteFAT (DISK *dsk, uint32_t ccls, uint32_t value, uint8_t forceWrite) {
    uint8_t i, q, c;  uint32_t p, li, l, ClusterFailValue;
#ifdef SUPPORT_FAT32 // If FAT32 supported.
    if ((dsk->type != FAT32) && (dsk->type != FAT16) && (dsk->type != FAT12)) return CLUSTER_FAIL_FAT32;
#else // If FAT32 support not enabled
    if ((dsk->type != FAT16) && (dsk->type != FAT12)) return CLUSTER_FAIL_FAT16;
#endif
    switch (dsk->type) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            ClusterFailValue = CLUSTER_FAIL_FAT32;
            break;
#endif
        case FAT12:
        case FAT16:
        default:
            ClusterFailValue = CLUSTER_FAIL_FAT16;
            break;
    }
    gBufferZeroed = FALSE;
    if (forceWrite) {
        for (i = 0, li = gLastFATSectorRead; i < dsk->fatcopy; i++, li += dsk->fatsize) {
            if (!SDSectorWrite (li, gFATBuffer, FALSE)) { return ClusterFailValue;}
        }
        gNeedFATWrite = FALSE;
        return 0;
    }
    switch (dsk->type)
    {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            p = (uint32_t)ccls *4; q = 0;
            break;
#endif
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        case FAT12:
            p = (uint32_t) ccls * 3; q = p & 1; p >>= 1;
            break;
#endif
        case FAT16:
        default:
            p = (uint32_t) ccls *2; q = 0;
            break;
    }
    l = dsk->fat + (p / dsk->sectorSize); p &= dsk->sectorSize - 1;
    if (gLastFATSectorRead != l) {
        if (gNeedFATWrite) {
            for (i = 0, li = gLastFATSectorRead; i < dsk->fatcopy; i++, li += dsk->fatsize) {
                if (!SDSectorWrite (li, gFATBuffer, FALSE)) return ClusterFailValue;
            }
            gNeedFATWrite = FALSE;
        }
        if (!SDSectorRead (l, gFATBuffer)) {
            gLastFATSectorRead = 0xFFFF; return ClusterFailValue;
        } else { gLastFATSectorRead = l;}
    }
#ifdef SUPPORT_FAT32 // If FAT32 supported.
    if (dsk->type == FAT32) {
        RAMwrite (gFATBuffer, p,   ((value & 0x000000ff))); RAMwrite (gFATBuffer, p+1, ((value & 0x0000ff00) >> 8));
        RAMwrite (gFATBuffer, p+2, ((value & 0x00ff0000) >> 16)); RAMwrite (gFATBuffer, p+3, ((value & 0x0f000000) >> 24));
    } else
#endif
    {
        if (dsk->type == FAT16) {
            RAMwrite (gFATBuffer, p, value); RAMwrite (gFATBuffer, p+1, ((value&0x0000ff00) >> 8));
        } else
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        if (dsk->type == FAT12) {
            c = RAMread (gFATBuffer, p);
            if (q) { c = ((value & 0x0F) << 4) | ( c & 0x0F);} else { c = (value & 0xFF);}
            RAMwrite (gFATBuffer, p, c); p = (p +1) & (dsk->sectorSize-1);
            if (p == 0) {
                if (WriteFAT (dsk, 0,0,TRUE)) return ClusterFailValue;
                if (!SDSectorRead (l +1, gFATBuffer)) { gLastFATSectorRead = 0xFFFF; return ClusterFailValue;}
                else { gLastFATSectorRead = l + 1;}
            }
            c = RAMread (gFATBuffer, p);
            if (q) c = (value >> 4); else c = ((value >> 8) & 0x0F) | (c & 0xF0);
            RAMwrite (gFATBuffer, p, c);
        } else
#endif
        {}
    }
    gNeedFATWrite = TRUE;
    return 0;
}

uint32_t  SDC::ReadFAT (DISK *dsk, uint32_t ccls) {
    uint32_t  c = 0, d, ClusterFailValue,LastClusterLimit;
    uint8_t   q;  uint32_t  p, l; gBufferZeroed = FALSE;
    switch (dsk->type) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            p = (uint32_t)ccls * 4; q = 0; ClusterFailValue = CLUSTER_FAIL_FAT32; LastClusterLimit = LAST_CLUSTER_FAT32;
            break;
#endif
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        case FAT12:
            p = (uint32_t) ccls *3; q = p&1; p >>= 1; ClusterFailValue = CLUSTER_FAIL_FAT16; LastClusterLimit = LAST_CLUSTER_FAT12;
            break;
#endif
        case FAT16:
        default:
            p = (uint32_t)ccls *2; q = 0; ClusterFailValue = CLUSTER_FAIL_FAT16; LastClusterLimit = LAST_CLUSTER_FAT16;
            break;
    }
    l = dsk->fat + (p / dsk->sectorSize); p &= dsk->sectorSize - 1;
    if (gLastFATSectorRead == l) {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        if (dsk->type == FAT32) { c = RAMreadD (gFATBuffer, p);} else
#endif
        if (dsk->type == FAT16) { c = RAMreadW (gFATBuffer, p);} else
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        if (dsk->type == FAT12) {
            c = RAMread (gFATBuffer, p); if (q) { c >>= 4;} p = (p +1) & (dsk->sectorSize-1);
            if (p == 0) {
#ifdef ALLOW_WRITES
                if (gNeedFATWrite) { if (WriteFAT (dsk, 0, 0, TRUE)) return ClusterFailValue;}
#endif
                if (!SDSectorRead (l+1, gFATBuffer)) { gLastFATSectorRead = 0xFFFF; return ClusterFailValue;}
                else { gLastFATSectorRead = l + 1;}
            }
            d = RAMread (gFATBuffer, p); if (q) c += (d <<4); else c += ((d & 0x0F)<<8);
        } else
#endif
        {}
    } else {
#ifdef ALLOW_WRITES
        if (gNeedFATWrite) { if (WriteFAT (dsk, 0, 0, TRUE)) { return ClusterFailValue;}}
#endif
        if (!SDSectorRead (l, gFATBuffer)) { gLastFATSectorRead = 0xFFFF; return ClusterFailValue;}
        else { gLastFATSectorRead = l;
#ifdef SUPPORT_FAT32 // If FAT32 supported.
            if (dsk->type == FAT32) { c = RAMreadD (gFATBuffer, p);}
            else
#endif
            {
                if (dsk->type == FAT16) { c = RAMreadW (gFATBuffer, p);} else
#ifdef SUPPORT_FAT12 // If FAT12 supported.
                if (dsk->type == FAT12) {
                    c = RAMread (gFATBuffer, p); if (q) { c >>= 4;}
                    p = (p +1) & (dsk->sectorSize-1); d = RAMread (gFATBuffer, p);
                    if (q) c += (d <<4); else c += ((d & 0x0F)<<8);
                } else
#endif
                {}
    }   }   }
    if (c >= LastClusterLimit) c = LastClusterLimit;
    return c;
}

CETYPE  SDC::FileErase( FILEOBJ fo, uint16_t *fHandle, uint8_t EraseClusters) {
    DIRENTRY  dir; uint8_t  a; CETYPE status = CE_GOOD; uint32_t  clus = 0;
    DISK * disk; uint8_t numberOfFileEntries; bool forFirstTime = TRUE;
#if defined(SUPPORT_LFN)
    uint8_t   tempCalc1;
#endif
    disk = fo->dsk;
#if defined(SUPPORT_LFN)
    fileNameLength = fo->utf16LFNlength;
    if (fileNameLength) {
        tempCalc1 = fileNameLength % 13; numberOfFileEntries = fileNameLength/13;
        if(tempCalc1 || (fileNameLength < 13)) numberOfFileEntries = numberOfFileEntries + 2;
        else  numberOfFileEntries++;
    } else
#endif
    { numberOfFileEntries = 1; }
    FSerrno = CE_ERASE_FAIL;
    while (numberOfFileEntries--) {
        fo->dirccls = fo->dirclus; dir = CacheFileEntry(fo, fHandle, TRUE);
        if (dir == NULL) return CE_BADCACHEREAD;
        a = dir->DIR_Name[0];
        if ((dir == (DIRENTRY)NULL) || (a == DIR_EMPTY) || (a == DIR_DEL)) {
            status = CE_FILE_NOT_FOUND; break;
        } else {
            dir->DIR_Name[0] = DIR_DEL;
            if (!(WriteFileEntry( fo, fHandle))) { status = CE_ERASE_FAIL; break;}
        }
        if (forFirstTime) { clus = GetFullClusterNumber(dir); forFirstTime = FALSE;}
        *fHandle = *fHandle - 1;
    }
    if (status == CE_GOOD) {
        if (clus != FatRootDirClusterValue) {
            if (EraseClusters) status = ((FATEraseClusterChain(clus, disk)) ? CE_GOOD : CE_ERASE_FAIL);
        }
        FSerrno = status;
    }
    return (status);
}

uint8_t  SDC::SDSectorWrite(uint32_t sector_addr, uint8_t *buffer, uint8_t allowWriteToZero) {
    uint16_t  i; if (!hcMode) sector_addr *= sectorSize; while(rdReg8(0x1124) & 0x02);
    SDReset(RESET_DATA); wrReg8(0x110C, 0); wrReg16(0x1104, sectorSize); wrReg16(0x1136, 0xffff);
    wrReg16(0x1132, 0xffff); wrReg8(0x1134, 0x13); wrReg8(0x1130, 0x13);
    if (!SDSendCmd(CMD_WR_SINGLE, RESPONSE_48 | DATA_PRESENT | CMD_CRC_CHK | CMD_IDX_CHK, sector_addr)) return (FALSE);
    wrReg8(0x1130, 0x01); while(!(rdReg8(0x1130)&0x10) && !(rdReg8(0x1125) & 0x04)); wrReg8(0x1130, 0x10);
    for (i = 0; i < sectorSize; i++) { while(!(rdReg8(0x1125) & 0x04)); wrReg8(0x1120, *buffer++); }
    wrReg8(0x1130, 0x10); while(!(rdReg8(0x1130)&0x02)); wrReg8(0x1130, 0x02); return (TRUE);
}

uint8_t  SDC::EraseCluster(DISK *disk, uint32_t cluster) {
    uint8_t  index; uint32_t  SectorAddress; uint8_t  error = CE_GOOD;
    SectorAddress = Cluster2Sector(disk,cluster);
    if (gNeedDataWrite) if (FlushData()) return CE_WRITE_ERROR; gBufferOwner = NULL;
    if (gBufferZeroed == FALSE) { memset(disk->buffer, 0x00, disk->sectorSize); gBufferZeroed = TRUE;}
    for (index = 0; (index < disk->SecPerClus) && (error == CE_GOOD); index++) {
        if (SDSectorWrite( SectorAddress++, disk->buffer, FALSE) != TRUE) error = CE_WRITE_ERROR;
    }
    return(error);
}

uint8_t  SDC::WriteFileEntry( FILEOBJ fo, uint16_t * curEntry) {
    DISK * dsk; uint8_t  status, offset2; uint32_t  sector, ccls;
    dsk = fo->dsk; ccls = fo->dirccls; offset2 = (*curEntry / (dsk->sectorSize/32));
    switch (dsk->type)
    {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            offset2 = offset2 % (dsk->SecPerClus);
            break;
#endif
        case FAT12:
        case FAT16:
            if (ccls != FatRootDirClusterValue) offset2 = offset2 % (dsk->SecPerClus);
            break;
    }
    sector = Cluster2Sector(dsk,ccls);
    if (!SDSectorWrite( sector + offset2, dsk->buffer, FALSE)) status = FALSE;
    else status = TRUE;
    return(status);
}

uint8_t  SDC::FATEraseClusterChain (uint32_t cluster, DISK * dsk) {
    uint32_t  c,c2,ClusterFailValue; enum _status {Good, Fail, Exit}status; status = Good;
    switch (dsk->type)
    {
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        case FAT32:
            ClusterFailValue = CLUSTER_FAIL_FAT32; c2 = LAST_CLUSTER_FAT32;
            break;
#endif
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        case FAT12:
            ClusterFailValue = CLUSTER_FAIL_FAT16; c2 = LAST_CLUSTER_FAT12;
            break;
#endif
        case FAT16:
        default:
            ClusterFailValue = CLUSTER_FAIL_FAT16; c2 = LAST_CLUSTER_FAT16;
            break;
    }
    if ((cluster == 0) || (cluster == 1)) { status = Exit;}
    else {
        while (status == Good) {
            if ((c = ReadFAT( dsk, cluster)) == ClusterFailValue) status = Fail;
            else {
                if ((c == 0) || (c == 1)) { status = Exit;}
                else {
                    if ( c >= c2) status = Exit;
                    if (WriteFAT(dsk, cluster, CLUSTER_EMPTY, FALSE) == ClusterFailValue) status = Fail;
                    cluster = c;
    }   }   }   }
    WriteFAT (dsk, 0, 0, TRUE);
    if(status == Exit)  return(TRUE);
    else                return(FALSE);
}

#if defined(SUPPORT_LFN)
uint8_t  SDC::FillLFNObject(FILEOBJ fo, LFN_ENTRY *lfno, uint16_t *fHandle) {
    DIRENTRY  dir; uint8_t  tempVariable, *src,*dst, status;
    if (((*fHandle & MASK_MAX_FILE_ENTRY_LIMIT_BITS) == 0) && (*fHandle != 0)) {
        fo->dirccls = fo->dirclus; dir = CacheFileEntry(fo, fHandle, TRUE);
    } else { dir = CacheFileEntry (fo, fHandle, FALSE);}
    if (dir == (DIRENTRY)NULL) { status = NO_MORE;}
    else {
        tempVariable = dir->DIR_Name[0];
        if (tempVariable == DIR_DEL) { status = NOT_FOUND;}
        else if ( tempVariable == DIR_EMPTY)
        { status = NO_MORE; }
        else {
            status = FOUND; dst = (uint8_t *)lfno; src = (uint8_t *)dir;
            for (tempVariable = 0;tempVariable < 32;tempVariable++) { *dst++ = *src++;}
    }   }
    return(status);
}
#endif // SUPPORT_LFN

DIRENTRY  SDC::LoadDirAttrib(FILEOBJ fo, uint16_t *fHandle) {
    DIRENTRY  dir; uint8_t  a; fo->dirccls = fo->dirclus; dir = CacheFileEntry( fo, fHandle, TRUE);
    if (dir == NULL) return NULL; a = dir->DIR_Name[0]; if (a == DIR_EMPTY) dir = (DIRENTRY)NULL;
    if (dir != (DIRENTRY)NULL) {
        if (a == DIR_DEL) dir = (DIRENTRY)NULL;
        else {
            a = dir->DIR_Attr;
            while(a == ATTR_LONG_NAME) {
                (*fHandle)++; dir = CacheFileEntry( fo, fHandle, FALSE);
                if (dir == NULL) return NULL;
                a = dir->DIR_Attr;
    }   }   }
    return(dir);
}

#ifdef ALLOW_WRITES
CETYPE  SDC::CreateFileEntry(FILEOBJ fo, uint16_t *fHandle, uint8_t mode, bool createFirstCluster)
{
    CETYPE  error = CE_GOOD;
#if defined(SUPPORT_LFN)
    LFN_ENTRY *lfno;
    unsigned short int *templfnPtr = (unsigned short int *)fo -> utf16LFNptr,*dest;
    unsigned short int	tempString[MAX_UTF16_CHARS_IN_LFN_ENTRY];
    WORD_VAL tempShift; bool firstTime = TRUE;
    uint8_t  checksum,sequenceNumber,reminder,tempCalc1,numberOfFileEntries;
    char index;
    char *src;
#endif // SUPPORT_LFN
    FSerrno = CE_GOOD; *fHandle = 0;
    if (FindEmptyEntries(fo, fHandle) == FOUND) {
#if defined(SUPPORT_LFN)
        if (fo->utf16LFNlength) {
            if (!Alias_LFN_Object(fo)) { error = FSerrno = CE_FILENAME_EXISTS; return(error);}
            src = fo -> name; checksum = 0;
            for (index = 11; index != 0; index--) {
                checksum = ((checksum & 1) ? 0x80 : 0) + (checksum >> 1) + *src++;
            }
            fileNameLength = fo->utf16LFNlength;
            reminder = tempCalc1 = fileNameLength % MAX_UTF16_CHARS_IN_LFN_ENTRY;
            numberOfFileEntries = fileNameLength/MAX_UTF16_CHARS_IN_LFN_ENTRY;
            if (tempCalc1 || (fileNameLength < MAX_UTF16_CHARS_IN_LFN_ENTRY)) { numberOfFileEntries++;}
            sequenceNumber = numberOfFileEntries | 0x40;
            if (tempCalc1) {
                index = 0;
                while (tempCalc1) {
                    tempString[(uint8_t)index++] = templfnPtr[fileNameLength - tempCalc1]; tempCalc1--;
                }				 
                tempString[(uint8_t)index++] = 0x0000;
                for (;index < MAX_UTF16_CHARS_IN_LFN_ENTRY;index++) { tempString[(uint8_t)index] = 0xFFFF;}				 
            } else {
                for (index = MAX_UTF16_CHARS_IN_LFN_ENTRY;index > 0;index--) {
                    tempString[MAX_UTF16_CHARS_IN_LFN_ENTRY - (uint8_t)index] = templfnPtr[fileNameLength - (uint8_t)index];
                }
            }
            dest = &tempString[12];
            while (numberOfFileEntries) {
                fo->dirccls = fo->dirclus; lfno = (LFN_ENTRY *)CacheFileEntry( fo, fHandle, TRUE);
                if (lfno == NULL) { return CE_BADCACHEREAD;}
                lfno->LFN_SequenceNo = sequenceNumber--; lfno->LFN_Part3[1] = *dest--;
                lfno->LFN_Part3[0] = *dest--; lfno->LFN_Part2[5] = *dest--;
                lfno->LFN_Part2[4] = *dest--; lfno->LFN_Part2[3] = *dest--;
                lfno->LFN_Part2[2] = *dest--; lfno->LFN_Part2[1] = *dest--;
                lfno->LFN_Part2[0] = *dest--; tempShift.Val = *dest--;
                lfno->LFN_Part1[9] = tempShift.byte.HB; lfno->LFN_Part1[8] = tempShift.byte.LB;
                tempShift.Val = *dest--; lfno->LFN_Part1[7] = tempShift.byte.HB;
                lfno->LFN_Part1[6] = tempShift.byte.LB; tempShift.Val = *dest--;
                lfno->LFN_Part1[5] = tempShift.byte.HB; lfno->LFN_Part1[4] = tempShift.byte.LB;
                tempShift.Val = *dest--; lfno->LFN_Part1[3] = tempShift.byte.HB;
                lfno->LFN_Part1[2] = tempShift.byte.LB; tempShift.Val = *dest--;
                lfno->LFN_Part1[1] = tempShift.byte.HB; lfno->LFN_Part1[0] = tempShift.byte.LB;
                lfno->LFN_Attribute = ATTR_LONG_NAME; lfno->LFN_Type = 0;
                lfno->LFN_Checksum = checksum; lfno->LFN_Reserved2 = 0;
                if (WriteFileEntry(fo,fHandle) != TRUE) error = CE_WRITE_ERROR;
                sequenceNumber &= (~0x40); *fHandle = *fHandle + 1;
                numberOfFileEntries--;
                if (firstTime) {
                    dest = (unsigned short int *)(fo -> utf16LFNptr + fileNameLength - reminder - 1);
                    firstTime = FALSE;
        }   }   }
#endif // SUPPORT_LFN
        if ((error = PopulateEntries(fo, fHandle, mode)) == CE_GOOD) {
            if (createFirstCluster) error = CreateFirstCluster(fo);
        }
    } else { error = CE_DIR_FULL;}
    FSerrno = error;
    return(error);
}
#endif // ALLOW_WRITES

#ifdef ALLOW_WRITES
uint8_t  SDC::FindEmptyEntries(FILEOBJ fo, uint16_t *fHandle) {
    uint8_t   status = NOT_FOUND, amountfound,numberOfFileEntries, a = 0;
    uint16_t  bHandle = *fHandle; uint32_t  b; DIRENTRY  dir;
    fo->dirccls = fo->dirclus;
    if ((dir = CacheFileEntry( fo, fHandle, TRUE)) != NULL) {
#if defined(SUPPORT_LFN)
        if (fo->utf16LFNlength) {
            fileNameLength = fo->utf16LFNlength; a = fileNameLength % MAX_UTF16_CHARS_IN_LFN_ENTRY;
            numberOfFileEntries = fileNameLength/MAX_UTF16_CHARS_IN_LFN_ENTRY;
            if (a || (fileNameLength < MAX_UTF16_CHARS_IN_LFN_ENTRY)) { numberOfFileEntries++;}
            numberOfFileEntries = numberOfFileEntries + 1;
        } else
#endif // SUPPORT_LFN
        numberOfFileEntries = 1;
        while (status == NOT_FOUND) {
            amountfound = 0; bHandle = *fHandle;
            do {
                dir = CacheFileEntry( fo, fHandle, FALSE);
                if (dir != NULL) { a = dir->DIR_Name[0];}
                (*fHandle)++;
            } while((dir != (DIRENTRY)NULL) && ((a == DIR_DEL) || (a == DIR_EMPTY)) && (++amountfound < numberOfFileEntries));
            if (dir == NULL) {
                b = fo->dirccls;
                if (b == FatRootDirClusterValue) {
                    if (fo->dsk->type != FAT32) status = NO_MORE;
                    else {
                        fo->ccls = b;
                        if (FileAllocateNewCluster(fo, 1) == CE_DISK_FULL) status = NO_MORE;
                        else status = FOUND;
                    }
                } else {
                    fo->ccls = b;
                    if (FileAllocateNewCluster(fo, 1) == CE_DISK_FULL) status = NO_MORE;
                    else { status = FOUND;}
                }
            } else { if (amountfound == numberOfFileEntries) status = FOUND; }
        }
    }
    *fHandle = bHandle;
    return(status);
}
#endif // ALLOW_WRITES

#ifdef ALLOW_WRITES
uint8_t  SDC::PopulateEntries(FILEOBJ fo, uint16_t *fHandle, uint8_t mode) {
    uint8_t  error = CE_GOOD; DIRENTRY  dir;
    fo->dirccls = fo->dirclus; dir = CacheFileEntry( fo, fHandle, TRUE);
    if (dir == NULL) return CE_BADCACHEREAD;
    strncpy(dir->DIR_Name,fo->name,DIR_NAMECOMP);
    if (mode == DIRECTORY)  dir->DIR_Attr = ATTR_DIRECTORY;
    else                    dir->DIR_Attr   = ATTR_ARCHIVE;
    dir->DIR_NTRes = 0x00; dir->DIR_FstClusHI = 0x0000;
    dir->DIR_FstClusLO = 0x0000; dir->DIR_FileSize = 0x0;
#ifdef INCREMENTTIMESTAMP
    dir->DIR_CrtTimeTenth = 0xB2; dir->DIR_CrtTime = 0x7278; dir->DIR_CrtDate = 0x32B0;
    dir->DIR_LstAccDate = 0x32B0; dir->DIR_WrtTime = 0x7279; dir->DIR_WrtDate = 0x32B0;
#endif
#ifdef USEREALTIMECLOCK
    CacheTime(); dir->DIR_CrtTimeTenth = gTimeCrtMS; dir->DIR_CrtTime = gTimeCrtTime;
    dir->DIR_CrtDate = gTimeCrtDate; dir->DIR_LstAccDate = gTimeAccDate;
    dir->DIR_WrtTime = gTimeWrtTime; dir->DIR_WrtDate = gTimeWrtDate;
#endif
#ifdef USERDEFINEDCLOCK
    dir->DIR_CrtTimeTenth = gTimeCrtMS; dir->DIR_CrtTime = gTimeCrtTime;
    dir->DIR_CrtDate = gTimeCrtDate; dir->DIR_LstAccDate = gTimeAccDate;
    dir->DIR_WrtTime = gTimeWrtTime; dir->DIR_WrtDate = gTimeWrtDate;
#endif
    fo->size = dir->DIR_FileSize; fo->time = dir->DIR_CrtTime;
    fo->date = dir->DIR_CrtDate; fo->attributes = dir->DIR_Attr;
    fo->entry = *fHandle;
    if (WriteFileEntry(fo,fHandle) != TRUE) error = CE_WRITE_ERROR;
    return(error);
}
#endif /* ALLOW_WRITES */

#ifdef ALLOW_WRITES
CETYPE  SDC::CreateFirstCluster(FILEOBJ fo) {
    CETYPE  error; uint32_t  cluster,TempMsbCluster;
    uint16_t  fHandle; DIRENTRY  dir; fHandle =  fo->entry;
    if ((error = FileCreateHeadCluster(fo,&cluster)) == CE_GOOD) {
        dir = LoadDirAttrib(fo, &fHandle); dir->DIR_FstClusLO = (cluster & 0x0000FFFF);
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        TempMsbCluster = (cluster & 0x0FFF0000); TempMsbCluster = TempMsbCluster >> 16;
        dir->DIR_FstClusHI = TempMsbCluster;
#else // If FAT32 support not enabled
        TempMsbCluster = 0; dir->DIR_FstClusHI = 0;
#endif
        if(WriteFileEntry(fo, &fHandle) != TRUE) error = CE_WRITE_ERROR;
    }
    return(error);
}
#endif // ALLOW_WRITES

#ifdef ALLOW_WRITES
CETYPE  SDC::FileCreateHeadCluster( FILEOBJ fo, uint32_t *cluster) {
    DISK * disk; CETYPE error = CE_GOOD;
    disk = fo->dsk; *cluster = FATFindEmptyCluster(fo);
    if (*cluster == 0) { error = CE_DISK_FULL;}
    else {
#ifdef SUPPORT_FAT12 // If FAT12 supported.
        if (disk->type == FAT12) { if (WriteFAT( disk, *cluster, LAST_CLUSTER_FAT12, FALSE) == CLUSTER_FAIL_FAT16) { error = CE_WRITE_ERROR;}}
        else
#endif
        if (disk->type == FAT16) { if (WriteFAT( disk, *cluster, LAST_CLUSTER_FAT16, FALSE) == CLUSTER_FAIL_FAT16) { error = CE_WRITE_ERROR;}}
        else
#ifdef SUPPORT_FAT32 // If FAT32 supported.
        if (disk->type == FAT32) { if (WriteFAT( disk, *cluster, LAST_CLUSTER_FAT32, FALSE) == CLUSTER_FAIL_FAT32) { error = CE_WRITE_ERROR;}}
        else
#endif
        {}
        if (error == CE_GOOD) { error = EraseCluster(disk,*cluster);}
    }
    return(error);
}
#endif // ALLOW_WRITES

uint16_t  SDC::StreamRGBFrame(FSFILE *fileStream, uint16_t numSectors) {
    DISK * dsk; uint16_t  sectorsToRead; uint32_t  sec_sel, currentCluster, prevousCluster;
    static uint32_t add = START_ADD; dsk = (DISK *)fileStream->dsk; sec_sel = Cluster2Sector(dsk, fileStream->ccls);
    sec_sel += (uint16_t)fileStream->sec; currentCluster = fileStream->ccls;
    prevousCluster = currentCluster; sectorsToRead = (uint16_t)dsk->SecPerClus - (uint16_t)fileStream->sec;
    while (sectorsToRead < numSectors) {
        if (FileGetNextCluster( fileStream, 1) != CE_GOOD) return 0xFFFF;
        if ((prevousCluster + 1) != fileStream->ccls) { fileStream->ccls = prevousCluster; break;}
        prevousCluster++; sectorsToRead += dsk->SecPerClus;
    }
    if (sectorsToRead > numSectors) sectorsToRead = numSectors;
    if (!SDSectorDMARead(sec_sel,  add, sectorsToRead)) { fileStream->ccls = currentCluster; return 0;}
    else {
        add += (dsk->sectorSize * (uint32_t)sectorsToRead);
        if (add >= (FRAME_SIZE + START_ADD)) { add = START_ADD;}
        fileStream->seek += (dsk->sectorSize * sectorsToRead);
        if (fileStream->seek > fileStream->size) { fileStream->seek = fileStream->size; return 0xFFFF;}
    }
    currentCluster = fileStream->sec + sectorsToRead;
    while (currentCluster > dsk->SecPerClus) currentCluster -= dsk->SecPerClus;
    fileStream->sec = currentCluster;
    if (fileStream->sec == dsk->SecPerClus) {
        fileStream->sec = 0;
        if (FileGetNextCluster( fileStream, 1) != CE_GOOD) return 0xFFFF;
    }
    return sectorsToRead;
}

void  SDC::RGBReadFromSD(FSFILE * pFile, void * f) {
    uint16_t  sectorsToWrite; uint32_t  total_frames; int  i;
    uint8_t reg; int8_t (*pf)()=f; uint8_t af; reg=rdReg8(0x71); wrReg8(0x71,reg&0xBF);
    total_frames = (pFile->size / FRAME_SIZE);
    for (i = 0; i < total_frames; i++) {
        if(f != (0x0)) af=pf();
        if (!af) {
            sectorsToWrite = FRAME_SIZE / pFile->dsk->sectorSize;
            while (sectorsToWrite) {
                uint16_t result = StreamRGBFrame(pFile, sectorsToWrite);
                if (result==0xFFFF) break; sectorsToWrite-=result;
            }
        } else {
            if (af == 0x01) { break; }  // stop
            else { i--; }               // pause
        }
    }
    ClearDevice(); wrReg8(0x71, reg);
}

uint8_t  SDC::JPEGReadFromSD(JPEG_DECODE * jpeg_decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot)
{
    int err=TRUE; err=SDJPEGRegsSetup(jpeg_decode); err=SDJPEGHeader(jpeg_decode);
    if (err == NO_ERR) {
        SDJPEGResize(jpeg_decode, left, top, right, bot); wrReg8(0x382, 7); wrReg8(0x383, 0x30);
        wrReg8(0x38A, 1); err = SDJPEGData(jpeg_decode); wrReg8(0x380, 0x10); wrReg8(0x38A, 0);
    }
    return err;
}

int  SDC::SDJPEGRegsSetup(JPEG_DECODE * decode) {
    uint32_t  size; int err; decode->bytes_read = 0; wrReg8(0x41E, 0);
    wrReg8(0x3A4, (uint8_t) (JPEG_FIFO_BLK_NUMBER - 1)); decode->fifo_addr = JPEG_FIFO_START_ADDR;
    decode->fifo_addr_end = JPEG_FIFO_START_ADDR + (JPEG_FIFO_BLK_NUMBER * JPEG_FIFO_BLK_SIZE);
    size = JPEG_FIFO_START_ADDR / 4; wrReg32(0x414, size); wrReg8(0x380, 0x11);
    wrReg8(0x402, 0x80); delay(1); wrReg8(0x402, 0); wrReg8(0x41C, 2); wrReg8(0x41E, 0);
    err = FSfseek((FSFILE*)decode->stream, 0, SEEK_END); if (err) return err;
    size = FSftell((FSFILE*)decode->stream);
    err  = FSfseek((FSFILE*)decode->stream, size, SEEK_END);
    if (err)  return err; decode->size = size; wrReg32(0x3B8, size);
    wrReg8(0x400, 4); wrReg8(0x360, 1); wrReg8(0x402, 1);
    return err;
}

JPEG_ERR  SDC::SDJPEGHeader(JPEG_DECODE * decode) {
    uint8_t  cnt = 0;
    while (cnt++ < 250)
    {
        if (rdReg8(0x41E)) return (ERR_DECODE);
        if (rdReg8(0x385) & 0x10) {
            uint16_t   size;
            size = (uint16_t)rdReg8(0x3D8) | ((uint16_t)(rdReg8(0x3D8 + 1)))<<8;
            decode->image_width = size;
            size = (uint16_t)rdReg8(0x3DC) | ((uint16_t)(rdReg8(0x3DC + 1)))<<8;
            decode->image_height = size; decode->op_mode = rdReg8(0x401);
            return (NO_ERR);
        }
        if (!(rdReg8(0x384) & 0x01)) continue;
        if (decode->bytes_read < decode->size) {
            if (!SDFile2JPEGFIFO(JPEG_FIFO_START_ADDR, JPEG_FIFO_SIZE, decode->stream)) { }
            decode->bytes_read += JPEG_FIFO_SIZE;
        } else return (ERR_NO_DATA);
    }
    return (ERR_DECODE);
}

void  SDC::SDJPEGResize(JPEG_DECODE * decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot) {
    const uint32_t __align[] = { 8, 16, 16, 32}; uint16_t size; uint8_t div = 1;
    uint16_t _maxX=right-left, _maxY=bot-top;
    if (right<left) _maxX=left-right; if (bot<top) _maxY=top-bot;
    if ((decode->image_height>(_maxY))||(decode->image_width>(_maxX))) {
        div = 2;
        while (div<8) {
            if (((decode->image_height/div)<=_maxY)&&((decode->image_width/div)<=_maxX)) break;
            div<<=1;
        }
    }
    wrReg8(0x36C,div); wrReg8(0x36E,2); decode->display_height=div*(GetMaxY()+1);
    decode->display_width=div*(GetMaxX()+1); uint32_t destAddr; uint32_t x, y;
    x=(right+left-decode->image_width/div)/2; y=(bot+top-decode->image_height/div)/2;
    destAddr = ((y*(GetMaxX()+1)+x)>>1); destAddr &= ~0x0001; wrReg32(0x410, destAddr);
    decode->display_height=decode->display_height&(~(__align[decode->op_mode]-1));
    decode->display_width =decode->display_width &(~(__align[decode->op_mode]-1));
    size = decode->display_width-1; wrReg16(0x364, 0); wrReg16(0x368, size);
    size = decode->display_height-1; wrReg16(0x366, 0); wrReg16(0x36A, size);
}

JPEG_ERR  SDC::SDJPEGData(JPEG_DECODE * decode) {
    while (1) {
        if (rdReg8(0x41E)) return (ERR_DECODE);
        if (rdReg8(0x404)&0xF8) return (ERR_DECODE);
        if (rdReg8(0x385)==0x22) return (NO_ERR);
        if (!(rdReg8(0x384)&0x01)) continue;
        if (decode->bytes_read<decode->size) {
            if(!SDFile2JPEGFIFO(JPEG_FIFO_START_ADDR, JPEG_FIFO_SIZE, decode->stream)) { }
            decode->bytes_read += JPEG_FIFO_SIZE;
        } else  continue;
    }
}
