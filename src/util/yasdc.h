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
#ifndef _YASDC_H_
#define _YASDC_H_


/************************************************************
 * Defines
 ************************************************************/
#define SUPPORT_FAT32
#define ALLOW_WRITES
#define FS_MAX_FILES_OPEN                           1

// Summary: Macro indicating the length of a 8.3 file name
// Description: The TOTAL_FILE_SIZE_8P3 macro indicates the maximum number of characters in an 8.3 file name.  This value includes
//              8 characters for the name, three for the extentsion, and 1 for the radix ('.')
#define TOTAL_FILE_SIZE_8P3                   (8+3+1)
#define TOTAL_FILE_SIZE           TOTAL_FILE_SIZE_8P3
#define MAX_FILE_NAME_LENGTH_LFN                  256
#define MASK_MAX_FILE_ENTRY_LIMIT_BITS           0x0f
#define VALUE_BASED_ON_ENTRIES_PER_CLUSTER          4
#define VALUE_DOTDOT_CLUSTER_VALUE_FOR_ROOT         0
#define FILE_NAME_SIZE_8P3                         11
#define FILE_NAME_SIZE             FILE_NAME_SIZE_8P3
#define MAX_UTF16_CHARS_IN_LFN_ENTRY      (uint8_t)13
#define DIRECTORY                                0x12   // Value indicating that the Create_FileEntry function will be creating a directory
#define MAX_UTF16_CHARS_IN_LFN_ENTRY      (uint8_t)13
#define MAIN_PANEL                                  1
#define SUB_PANEL                                   0
// internal errors
#define CE_FAT_EOF                                 60   // Error that indicates an attempt to read FAT entries beyond the end of the file
#define CE_EOF                                     61   // Error that indicates that the end of the file has been reached

#define FOUND                                       0
#define NOT_FOUND                                   1
#define NO_MORE                                     2
#define FAT12                                       1
#define FAT16                                       2
#define FAT32                                       3
#define ATTR_READ_ONLY                           0x01
#define ATTR_HIDDEN                              0x02
#define ATTR_SYSTEM                              0x04
#define ATTR_VOLUME                              0x08
#define ATTR_LONG_NAME                           0x0f
#define ATTR_DIRECTORY                           0x10
#define ATTR_ARCHIVE                             0x20
#define ATTR_MASK                                0x3f
#define CLUSTER_EMPTY                          0x0000
#define LAST_CLUSTER_FAT12                     0x0ff8
#define END_CLUSTER_FAT12                      0x0FF7
#define LAST_CLUSTER_FAT16                     0xfff8
#define END_CLUSTER_FAT16                      0xFFF7
#define CLUSTER_FAIL_FAT16                     0xFFFF
#define LAST_CLUSTER_FAT32                 0x0FFFFFF8
#define END_CLUSTER_FAT32                  0x0FFFFFF7
#define CLUSTER_FAIL_FAT32                 0x0FFFFFFF
#define NUMBER_OF_BYTES_IN_DIR_ENTRY               32
#define DIR_DEL                                  0xE5
#define DIR_EMPTY                                   0
#define DIR_NAMESIZE                                8
#define DIR_EXTENSION                               3
#define DIR_NAMECOMP     (DIR_NAMESIZE+DIR_EXTENSION)
#define RAMwrite( a, f, d)                 *(a+f) = d
#define RAMread( a, f)                         *(a+f)
#define RAMreadW( a, f)            *(uint16_t *)(a+f)
#define RAMreadD( a, f)            *(uint32_t *)(a+f)

#ifndef SEEK_SET
    #define SEEK_SET                                0
#endif
#ifndef SEEK_CUR
    #define SEEK_CUR                                1
#endif
#ifndef SEEK_END
    #define SEEK_END                                2
#endif

#define FS_APPEND                                 "a"
#define FS_WRITE                                  "w"
#define FS_READ                                   "r"
#define FS_APPENDPLUS                            "a+"
#define FS_WRITEPLUS                             "w+"
#define FS_READPLUS                              "r+"

#define BSI_BPS                                    11
#define BSI_SPC                                    13
#define BSI_RESRVSEC                               14
#define BSI_FATCOUNT                               16
#define BSI_ROOTDIRENTS                            17
#define BSI_TOTSEC16                               19
#define BSI_SPF                                    22
#define BSI_TOTSEC32                               32
#define BSI_BOOTSIG                                38
#define BSI_FSTYPE                                 54
#define BSI_FATSZ32                                36
#define BSI_ROOTCLUS                               44
#define BSI_FAT32_BOOTSIG                          66
#define BSI_FAT32_FSTYPE                           82

#define FO_MBR                                     0L
#define FAT_GOOD_SIGN_0                          0x55
#define FAT_GOOD_SIGN_1                          0xAA

// JPEG FIFO Size and Location
#define JPEG_FIFO_BLK_SIZE      (uint32_t) (4 * 1024)
#define JPEG_FIFO_BLK_NUMBER    (uint32_t) 16
#define JPEG_FIFO_SIZE          JPEG_FIFO_BLK_SIZE * JPEG_FIFO_BLK_NUMBER
#define JPEG_FIFO_START_ADDR    (uint32_t) (0x30000)

typedef enum {
    NAME_8P3_ASCII_CAPS_TYPE, NAME_8P3_ASCII_MIXED_TYPE, NAME_8P3_UTF16_TYPE, NAME_8P3_UTF16_ASCII_CAPS_TYPE,
    NAME_8P3_UTF16_ASCII_MIXED_TYPE, NAME_8P3_UTF16_NONASCII_TYPE, NAME_LFN_TYPE, NAME_ERROR
} FILE_DIR_NAME_TYPE;
typedef enum { LOOK_FOR_EMPTY_ENTRY = 0, LOOK_FOR_MATCHING_ENTRY} SEARCH_TYPE;
typedef enum { DEMO_FILE_TYPE_JPEG, DEMO_FILE_TYPE_RGB, DEMO_FILE_TYPE_OTHER} DEMO_FILE_TYPE;
typedef enum { MEDIA_NO_ERROR, MEDIA_DEVICE_NOT_PRESENT, MEDIA_CANNOT_INITIALIZE} MEDIA_ERRORS;

// Summary: An enumeration used for various error codes.
// Description: The CETYPE enumeration is used to indicate different error conditions during device operation.
typedef enum _CETYPE {
    CE_GOOD = 0,                    // No error
    CE_ERASE_FAIL,                  // An erase failed
    CE_NOT_PRESENT,                 // No device was present
    CE_NOT_FORMATTED,               // The disk is of an unsupported format
    CE_BAD_PARTITION,               // The boot record is bad
    CE_UNSUPPORTED_FS,              // The file system type is unsupported
    CE_INIT_ERROR,                  // An initialization error has occured
    CE_NOT_INIT,                    // An operation was performed on an uninitialized device
    CE_BAD_SECTOR_READ,             // A bad read of a sector occured
    CE_WRITE_ERROR,                 // Could not write to a sector
    CE_INVALID_CLUSTER,             // Invalid cluster value > maxcls
    CE_FILE_NOT_FOUND,              // Could not find the file on the device
    CE_DIR_NOT_FOUND,               // Could not find the directory
    CE_BAD_FILE,                    // File is corrupted
    CE_DONE,                        // No more files in this directory
    CE_COULD_NOT_GET_CLUSTER,       // Could not load/allocate next cluster in file
    CE_FILENAME_2_LONG,             // A specified file name is too long to use
    CE_FILENAME_EXISTS,             // A specified filename already exists on the device
    CE_INVALID_FILENAME,            // Invalid file name
    CE_DELETE_DIR,                  // The user tried to delete a directory with FSremove
    CE_DIR_FULL,                    // All root dir entry are taken
    CE_DISK_FULL,                   // All clusters in partition are taken
    CE_DIR_NOT_EMPTY,               // This directory is not empty yet, remove files before deleting
    CE_NONSUPPORTED_SIZE,           // The disk is too big to format as FAT16
    CE_WRITE_PROTECTED,             // Card is write protected
    CE_FILENOTOPENED,               // File not opened for the write
    CE_SEEK_ERROR,                  // File location could not be changed successfully
    CE_BADCACHEREAD,                // Bad cache read
    CE_CARDFAT32,                   // FAT 32 - card not supported
    CE_READONLY,                    // The file is read-only
    CE_WRITEONLY,                   // The file is write-only
    CE_INVALID_ARGUMENT,            // Invalid argument
    CE_TOO_MANY_FILES_OPEN,         // Too many files are already open
    CE_UNSUPPORTED_SECTOR_SIZE      // Unsupported sector size
} CETYPE;

#ifndef _JPEG_DECODE_
#define _JPEG_DECODE_
typedef struct _JPEG_DECODE {
    void *stream; uint16_t image_width; uint16_t image_height; uint32_t display_width; uint32_t display_height;
    uint16_t op_mode; uint32_t fifo_addr; uint32_t fifo_addr_end; uint32_t size; uint32_t bytes_read;
} JPEG_DECODE;
#endif

#ifndef _JPEG_ERR_
#define _JPEG_ERR_	
typedef enum _JPEG_ERR {NO_ERR,ERR_TIMEOUT,ERR_IMCOMPLETE,ERR_NO_DATA,ERR_DECODE,ERR_CODEC_INT} JPEG_ERR;
#endif

/***************************************************************************
* Prototypes                                                               *
***************************************************************************/
typedef struct {
    char  filename[FILE_NAME_SIZE_8P3 + 2]; unsigned char attributes; unsigned long filesize; unsigned long timestamp;
#ifdef SUPPORT_LFN
    bool  AsciiEncodingType; unsigned short int *utf16LFNfound; unsigned short int utf16LFNfoundLength;
#endif
    unsigned int entry; char searchname[FILE_NAME_SIZE_8P3 + 2]; unsigned char searchattr;
    unsigned long cwdclus; unsigned char initialized;
} SearchRec;

typedef struct {
    uint8_t *buffer; uint32_t firsts; uint32_t fat; uint32_t root; uint32_t data; uint16_t  maxroot; uint32_t maxcls;
    uint32_t sectorSize; uint32_t fatsize; uint8_t fatcopy; uint8_t SecPerClus; uint8_t type; uint8_t mount;
} DISK;

typedef struct { unsigned long write:1; unsigned long read:1; unsigned long FileWriteEOF:1;} FILEFLAGS;

typedef struct {
    DISK *dsk; uint32_t cluster; uint32_t ccls; uint16_t sec; uint16_t pos; uint32_t seek; uint32_t size;
    FILEFLAGS flags; uint16_t time; uint16_t date; char name[FILE_NAME_SIZE_8P3];
#ifdef SUPPORT_LFN
    bool AsciiEncodingType; unsigned short int *utf16LFNptr; unsigned short int utf16LFNlength;
#endif
    uint16_t entry; uint16_t chk; uint16_t attributes; uint32_t dirclus; uint32_t dirccls;} FSFILE;

typedef FSFILE * FILEOBJ;

typedef struct {
    char DIR_Name[DIR_NAMESIZE]; char DIR_Extension[DIR_EXTENSION]; uint8_t DIR_Attr; uint8_t DIR_NTRes;
    uint8_t DIR_CrtTimeTenth; uint16_t DIR_CrtTime; uint16_t DIR_CrtDate; uint16_t DIR_LstAccDate;
    uint16_t DIR_FstClusHI; uint16_t DIR_WrtTime; uint16_t DIR_WrtDate; uint16_t DIR_FstClusLO; uint32_t DIR_FileSize;
}_DIRENTRY;

typedef _DIRENTRY * DIRENTRY;

typedef struct {
    uint8_t LFN_SequenceNo; uint8_t LFN_Part1[10]; uint8_t LFN_Attribute; uint8_t LFN_Type; uint8_t LFN_Checksum;
    unsigned short int LFN_Part2[6]; unsigned short int LFN_Reserved2; unsigned short int LFN_Part3[2];
}LFN_ENTRY;

// Summary: A 24-bit data type
typedef struct { unsigned char array[3];} __attribute__ ((packed)) SWORD;

typedef struct {
    SWORD BootSec_JumpCmd; uint8_t BootSec_OEMName[8]; uint16_t BootSec_BPS; uint8_t BootSec_SPC;
    uint16_t BootSec_ResrvSec; uint8_t BootSec_FATCount; uint16_t BootSec_RootDirEnts; uint16_t BootSec_TotSec16;
    uint8_t  BootSec_MDesc; uint16_t BootSec_SPF; uint16_t BootSec_SPT; uint16_t BootSec_HeadCnt;
    uint32_t BootSec_HiddenSecCnt; uint32_t BootSec_Reserved; uint8_t  BootSec_DriveNum; uint8_t BootSec_Reserved2;
    uint8_t  BootSec_BootSig; uint8_t BootSec_VolID[4]; uint8_t BootSec_VolLabel[11]; uint8_t BootSec_FSType[8];
} _BPB_FAT12;

typedef struct {
    SWORD BootSec_JumpCmd; uint8_t  BootSec_OEMName[8]; uint16_t BootSec_BPS; uint8_t  BootSec_SPC;
    uint16_t BootSec_ResrvSec; uint8_t  BootSec_FATCount; uint16_t BootSec_RootDirEnts; uint16_t BootSec_TotSec16;
    uint8_t  BootSec_MDesc; uint16_t BootSec_SPF; uint16_t BootSec_SPT; uint16_t BootSec_HeadCnt;
    uint32_t BootSec_HiddenSecCnt; uint32_t BootSec_TotSec32; uint8_t  BootSec_DriveNum; uint8_t  BootSec_Reserved;
    uint8_t  BootSec_BootSig; uint8_t  BootSec_VolID[4]; uint8_t  BootSec_VolLabel[11]; uint8_t  BootSec_FSType[8];
} _BPB_FAT16;

typedef struct {
    SWORD BootSec_jmpBoot; uint8_t  BootSec_OEMName[8]; uint16_t BootSec_BytsPerSec; uint8_t  BootSec_SecPerClus;
    uint16_t BootSec_RsvdSecCnt; uint8_t  BootSec_NumFATs; uint16_t BootSec_RootEntCnt; uint16_t BootSec_TotSec16;
    uint8_t  BootSec_Media; uint16_t BootSec_FATSz16; uint16_t BootSec_SecPerTrk; uint16_t BootSec_NumHeads;
    uint32_t BootSec_HiddSec; uint32_t BootSec_TotSec32; uint32_t BootSec_FATSz32; uint16_t BootSec_ExtFlags;
    uint16_t BootSec_FSVers; uint32_t BootSec_RootClus; uint16_t BootSec_FSInfo; uint16_t BootSec_BkBootSec;
    uint8_t  BootSec_Reserved[12]; uint8_t  BootSec_DrvNum; uint8_t  BootSec_Reserved1; uint8_t  BootSec_BootSig;
    uint8_t  BootSec_VolID[4]; uint8_t  BootSec_VolLab[11]; uint8_t  BootSec_FilSysType[8];
} _BPB_FAT32;

typedef struct { uint8_t PTE_BootDes;SWORD PTE_FrstPartSect;uint8_t PTE_FSDesc;SWORD PTE_LstPartSect;uint32_t PTE_FrstSect;uint32_t PTE_NumSect;} PTE_MBR;

typedef struct { uint8_t ConsChkRtn[446];PTE_MBR Partition0;PTE_MBR Partition1;PTE_MBR Partition2;PTE_MBR Partition3;uint8_t Signature0;uint8_t Signature1;} _PT_MBR;

typedef _PT_MBR *  PT_MBR;

typedef struct { union{ _BPB_FAT32  FAT_32; _BPB_FAT16  FAT_16; _BPB_FAT12  FAT_12;} FAT;
    uint8_t  Reserved[512-sizeof(_BPB_FAT32)-2]; uint8_t  Signature0; uint8_t  Signature1;} _BootSec;

typedef _BootSec * BootSec;

typedef struct {uint8_t errorCode; union{uint8_t value;struct{uint8_t sectorSize:1;uint8_t maxLUN:1;}bits;}validityFlags;uint16_t sectorSize;uint8_t maxLUN;} MEDIA_INFORMATION;

/******************************************************************************
 * User Defines
 *****************************************************************************/
#define SYS_CLOCK               (uint32_t)  (4000000)
#define SD_CLK_INIT             (uint32_t) (50000000)
#define SD_TIMEOUT              (uint32_t)  (3000000)

/******************************************************************************
 * SD Commands
 *****************************************************************************/
#define CMD_RESET                                   0
#define CMD_SEND_OCR                                1           // used exclusively in MMC
#define CMD_SEND_ALL_CID                            2           // R2: R136
#define CMD_SEND_RCA                                3           // R1 (MMC) or R6(SDMEM)
#define CMD_SET_DSR                                 4
#define CMD_IO_SEND_OCR                             5           // R4, unique to IO cards
#define CMD_SELECT_CARD                             7           // R1, arg=rca[31..16] or 0
#define CMD_SEND_IF_COND                            8
#define CMD_SEND_CSD                                9           // R2: R136
#define CMD_SEND_CID                               10           // R2: R136
#define CMD_STOP_TRANSMISSION                      12           // R1b: arg=stuff bits
#define CMD_SEND_STATUS                            13           // R1
#define CMD_GO_INACTIVE                            15           // None, arg=rca[31..16], stuff[15..0]
#define CMD_SET_BLKLEN                             16           // R1, arg=block len[31..0]
#define CMD_RD_SINGLE                              17           // R1, arg=block address[31..0]
#define CMD_RD_MULTIPLE                            18           // R1, arg=block address[31..0]
#define CMD_WR_SINGLE                              24           // R1, arg=block address[31..0]
#define CMD_WR_MULTIPLE                            25           // R1, arg=block address[31..0]
#define CMD_SET_WP                                 28           // R1b, arg=wp address[31..0]
#define CMD_CLR_WP                                 29           // R1b, arg=wp address[31..0]
#define CMD_SEND_WP                                30           // R1, DATA, arg=wp address[31..0]
#define CMD_ERASE_SADDR                            32           // R1, arg=block address[31..0]
#define CMD_ERASE_EADDR                            33           // R1, arg=block address[31..0]
#define CMD_ERASE_GRP_SADDR                        35           // R1, arg=block address[31..0]
#define CMD_ERASE_GRP_EADDR                        36           // R1, arg=block address[31..0]
#define CMD_ERASE                                  38           // R1b, arg=stuff bits[31..0]
#define CMD_IO_RW_DIRECT                           52           // R5
#define CMD_IO_RW_EXTENDED                         53           // R1, data transfer
#define CMD_APP_CMD                                55           // R1, arg=rca[31..16], stuff[15..0]
#define CMD_GEN_CMD                                56           // R1, data, arg=stuff[31..1], RD/WR[0]
#define ACMD_SET_BUS_WIDTH                          6           // R1, arg=[1..0] = bus width, [31:2] stuff bits
#define ACMD_SEND_STATUS                           13           // R1, DATA, arg=stuff bits [31..0]
#define ACMD_SEND_NUM_WR_BLK                       22           // R1, DATA, arg=stuff bits [31..0]
#define ACMD_SEND_OCR                              41        
#define ACMD_SEND_SCR                              51           // R1, arg=stuff bits[31..0]
                                                   
/******************************************************************************
 * Flags
 *****************************************************************************/
#define SD_CLK_CTRL_ON      (uint32_t) 0x80000000
#define SD_CLK_ENABLE       (uint32_t) 0x00000004
#define SD_INT_CLK_STABLE   (uint32_t) 0x00000002
#define SD_INT_CLK_ENABLE   (uint32_t) 0x00000001
#define SD_CLK_FLAGS        (SD_CLK_CTRL_ON | SD_CLK_ENABLE | SD_INT_CLK_ENABLE)
#define CMD_TYPE_ABORT                           0xC0
#define CMD_TYPE_RESUME                          0x80
#define CMD_TYPE_SUSPEND                         0x40
#define CMD_TYPE_NORMAL                          0x00
#define DATA_PRESENT                             0x20
#define CMD_IDX_CHK                              0x10
#define CMD_CRC_CHK                              0x08
#define NO_RESPONSE                              0x00
#define RESPONSE_136                             0x01
#define RESPONSE_48                              0x02
#define RESPONSE_48_BUSY                         0x03

#define CARD_DETECT                              0x04
#define CARD_STABLE                              0x02
#define CARD_INSERTED                            0x01
#define WRITE_PROTECT                            0x08
#define RESET_ALL                                0x01
#define RESET_CMD                                0x02
#define RESET_DATA                               0x04

#define WAIT_CNT                 (uint32_t) 10000000l


/*********************************************************************
* Class SDCard
*********************************************************************/
class  SDC:INTRFC {
  public:
    int         FSInit(void);
    int         FindFirst (const char * fileName, unsigned int attr, SearchRec * rec);
    DEMO_FILE_TYPE GetFileType(char *fileName);
    void        RGBReadFromSD(FSFILE * pFile, void * f);
    int         FindNext (SearchRec * rec);
    FSFILE *    FSfopen( const char * fileName, const char *mode );
    int         FSfseek(FSFILE *stream, long offset, int whence);
    long        FSftell (FSFILE * fo) { FSerrno = CE_GOOD; return (fo->seek);};
    int         FSfclose(FSFILE   *fo);
    size_t      FSfwrite(const void *data_to_write, size_t size, size_t n, FSFILE *stream);
    size_t      FSfread (void *ptr, size_t size, size_t n, FSFILE *stream);
    int         FSfeof(FSFILE * stream) { FSerrno = CE_GOOD; return( stream->seek == stream->size );};
    uint8_t     SDSectorRead(uint32_t sector_addr, uint8_t *buffer);
    uint8_t     SDFile2JPEGFIFO(uint32_t fifoAddress, uint32_t byteSize, FSFILE *stream);
    uint8_t     SDReset(uint8_t type) {uint32_t to=SD_TIMEOUT; wrReg8(0x112F,type); while(1){ if(!to--) return 0; if(!(rdReg8(0x112F)&type)) return 1;}};
    void        SDPower(uint8_t on) { if (on) wrReg8(0x1129, 0x0F); else wrReg8(0x1129, 0x0E);};
    uint8_t     SDWriteProtectState(void) { return (rdReg8(0x1126) & WRITE_PROTECT);};
    uint8_t     SDSetClock(uint32_t clockMax);
    uint8_t     JPEGReadFromSD(JPEG_DECODE * jpeg_decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot);
    DEMO_FILE_TYPE currentFileType;
    SearchRec   nextFile;
    uint32_t    maxBusClock;

  private:
    uint8_t     DISKmount(DISK *dsk);
    uint8_t     LoadMBR(DISK *dsk);
    uint8_t     LoadBootSector(DISK *dsk);
    uint8_t     FormatFileName( const char* fileName, FILEOBJ fptr, uint8_t mode);
    void        FileObjectCopy(FILEOBJ foDest,FILEOBJ foSource);
    FILE_DIR_NAME_TYPE ValidateChars(uint8_t mode);
    DIRENTRY    CacheFileEntry( FILEOBJ fo, uint16_t * curEntry, uint8_t ForceRead);
    uint8_t     FillFileObject(FILEOBJ fo, uint16_t *fHandle);
    CETYPE      FILEfind( FILEOBJ foDest, FILEOBJ foCompareTo, uint8_t cmd, uint8_t mode);
    CETYPE      FILEopen (FILEOBJ fo, uint16_t *fHandle, char type);
    uint8_t     FileGetNextCluster(FSFILE *fo, uint32_t n);
    uint32_t    Cluster2Sector(DISK * dsk, uint32_t cluster);
    uint8_t     ReadByte(uint8_t* pBuffer, uint16_t index) {return (pBuffer[index]);};
    uint16_t    ReadWord(uint8_t* pBuffer, uint16_t index) {uint8_t loByte, hiByte; uint16_t res; loByte = pBuffer[index];
                         hiByte=pBuffer[index+1]; res=hiByte; res*=0x100; res|=loByte; return(res);};
    uint32_t    ReadDWord(uint8_t* pBuffer, uint16_t index) {uint16_t loWord, hiWord; uint32_t result; loWord=ReadWord(pBuffer, index);
                          hiWord=ReadWord(pBuffer, index+2); result=hiWord; result*=0x10000; result|=loWord; return result;};
#if defined(SUPPORT_LFN)
    uint8_t     FillLFNObject(FILEOBJ fo, LFN_ENTRY *lfno, uint16_t *fHandle);
    bool        Alias_LFN_Object(FILEOBJ fo);
#endif /* (SUPPORT_LFN) */
    DIRENTRY    LoadDirAttrib(FILEOBJ fo, uint16_t *fHandle);
#ifdef ALLOW_WRITES
    CETYPE      CreateFileEntry(FILEOBJ fo, uint16_t *fHandle, uint8_t mode, bool createFirstCluster);
    uint8_t     FindEmptyEntries(FILEOBJ fo, uint16_t *fHandle);
    uint8_t     PopulateEntries(FILEOBJ fo, uint16_t *fHandle, uint8_t mode);
    CETYPE      CreateFirstCluster(FILEOBJ fo);
    CETYPE      FileCreateHeadCluster( FILEOBJ fo, uint32_t *cluster);
#endif // ALLOW_WRITES
    MEDIA_INFORMATION  *SDInitialize(void);
    uint8_t     SDSendCmd(uint8_t cmd_idx, uint8_t flags, uint32_t arg);
    uint8_t     SDInit(void);
    uint32_t    GetCmdResp(uint8_t idx);
    void        ReadCmdResp(uint32_t *rsp, uint32_t size) { uint32_t idx; for(idx=0; idx<size; idx++) {*rsp=GetCmdResp(idx); rsp++;}};
    uint8_t     SDSendAppCmd(uint8_t cmd_idx, uint8_t flags, uint32_t arg1, uint32_t arg2);
    uint8_t     FlushData(void);
    uint8_t     FileAllocateNewCluster( FILEOBJ fo, uint8_t mode);
    uint32_t    FATFindEmptyCluster(FILEOBJ fo);
    uint32_t    ReadFAT (DISK *dsk, uint32_t ccls);
    uint32_t    WriteFAT (DISK *dsk, uint32_t ccls, uint32_t value, uint8_t forceWrite);
    CETYPE      FileErase( FILEOBJ fo, uint16_t *fHandle, uint8_t EraseClusters);
    uint8_t     SDSectorWrite(uint32_t sector_addr, uint8_t *buffer, uint8_t allowWriteToZero);
    uint8_t     EraseCluster(DISK *disk, uint32_t cluster);
    uint8_t     WriteFileEntry( FILEOBJ fo, uint16_t * curEntry);
    uint8_t     FATEraseClusterChain(uint32_t cluster, DISK * dsk);
    uint32_t    GetFullClusterNumber(DIRENTRY entry);
    uint8_t     SDSectorDMARead(uint32_t sector_addr, uint32_t dma_addr, uint16_t num_blk);
    uint16_t    StreamRGBFrame(FSFILE *fileStream, uint16_t numSectors);
    int         SDJPEGRegsSetup(JPEG_DECODE * decode);
    JPEG_ERR    SDJPEGHeader(JPEG_DECODE * decode);
    void        SDJPEGResize(JPEG_DECODE * decode, uint16_t left, uint16_t top, uint16_t right, uint16_t bot);
    JPEG_ERR    SDJPEGData(JPEG_DECODE * decode);
    void        ClearDevice(void) { uint32_t i; for(i=0; i<(GetMaxY()+1); i++) { wrReg8(0x1E4, 0);
                                    wrReg8(0x1E5,0); wrReg8(0x1E8,i); wrReg8(0x1E9,i>>8); wrReg8(0x1EC,64);
                                    wrReg8(0x1ED,1); wrReg8(0x1F0,i); wrReg8(0x1F1,i>>8); wrReg8(0x1D4,0);
                                    wrReg8(0x1D5,0); wrReg8(0x1D6,0); wrReg8(0x1F4,0); wrReg8(0x1F5,0);
                                    wrReg8(0x1F6,0); wrReg8(0x1F8,64); wrReg8(0x1F9,1); wrReg8(0x1D8,240);
                                    wrReg8(0x1D9,0); wrReg8(0x1FE,0); wrReg8(0x1FD,0); wrReg8(0x1FC,0);
                                    wrReg8(0x1DD,0); wrReg8(0x1D1,1); wrReg8(0x1D2,1);}};
#ifdef SUPPORT_LFN
    unsigned short int fileFoundString[261];
    unsigned short int *utf16Filename;
    bool        utfModeFileName = FALSE;
    unsigned short int lfnData[FS_MAX_FILES_OPEN][257];
#endif // SUPPORT_LFN
    uint8_t     gDataBuffer[MEDIA_SECTOR_SIZE];
    uint8_t     gFATBuffer[MEDIA_SECTOR_SIZE];
    DISK        gDiskData;
    uint8_t     gFileSlotOpen[FS_MAX_FILES_OPEN];
    MEDIA_INFORMATION   mediaInformation;
    uint8_t     gBufferZeroed = FALSE;
    uint8_t     gNeedFATWrite = FALSE;
    uint32_t    gLastFATSectorRead = 0xFFFFFFFF;
    uint32_t    gLastDataSectorRead = 0xFFFFFFFF;
    FSFILE      gFileArray[FS_MAX_FILES_OPEN];
    uint16_t    hcMode;
    uint16_t    sectorSize;
    uint32_t    FatRootDirClusterValue;
    uint32_t    finalLBA;
    FSFILE *    gBufferOwner = NULL;
    uint8_t     gNeedDataWrite = FALSE;
    uint8_t     nextClusterIsLast = FALSE;
    uint8_t     FSerrno;
    FSFILE      gFileTemp;
    FSFILE      cwd;
    FSFILE    * cwdptr = &cwd;
    char      * asciiFilename;
    unsigned short int fileNameLength;
};

#endif // _YASDC_H_
