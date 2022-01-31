/**************************************************************************
MODULE:       FLASH_NVOL
CONTAINS:     Storage of non-volatile variables in Flash memory
DEVELOPED BY: Embedded Systems Academy, Inc. 2010
              www.esacademy.com
COPYRIGHT:    NXP Semiconductors, 2010. All rights reserved.
VERSION:      1.10
***************************************************************************/ 

#ifndef _FLASHNVOL_
#define _FLASHNVOL_

// Data Types
#define UNSIGNED8 unsigned char
#define UNSIGNED16 unsigned short
#define UNSIGNED32 unsigned int
#define INTEGER8 char
#define INTEGER16 short
#define INTEGER32 int
//#define BOOL int

#define TRUE 1
#define FALSE 0

#define WRITE_PAGE_SIZE 1024  
  
// start address and numbers for two sectors used for storing
// non-volatile variables

#define SECTOR1_STARTADDR 0x00070000
#define SECTOR1_NUM       28
#define SECTOR2_STARTADDR 0x00078000
#define SECTOR2_NUM       29
/*
#define SECTOR1_STARTADDR 0x00050000
#define SECTOR1_NUM       26
#define SECTOR2_STARTADDR 0x00058000
#define SECTOR2_NUM       27
*/
// size of sectors - they must be the same size
#define SECTOR_SIZE 0x8000
// CPU clock in kHz
#define CPU_CLK (SystemCoreClock / 1000)
#define DISABLEIRQ __disable_interrupt()
#define ENABLEIRQ  __enable_interrupt()
#define IAP_LOCATION 0x1FFF1FF1

// maximum number of variables supported
// must be less than ( (SECTOR_SIZE - 48) / (MAX_VARIABLE_SIZE + 4) )
#define MAX_VARIABLES 70
// max size of variable in bytes (16 * N - 4) 
#define MAX_VARIABLE_SIZE 412
// invalid variable offset into a sector
#define INVALID_VAR_OFFSET 0

// sector flags
#define SECTOR_EMPTY        0xFFFFFF
#define SECTOR_INITIALIZING 0xAAFFFF
#define SECTOR_VALID        0xAAAAFF
#define SECTOR_INVALID      0xAAAAAA

// defines a sector
typedef struct _Sector
{
  UNSIGNED8 *Addr;                                    // sector start address
  UNSIGNED8 Num;					   // sector number
} SECTOR;

// defines a sector record
// members must be byte aligned
// must be 48 bytes in size
typedef __packed struct _Sector_Record
{
  UNSIGNED8 Flags1;				             // flags indicate sector status
  UNSIGNED8 Reserved1[15];                                 // padding
  UNSIGNED8 Flags2;				            // flags indicate sector status
  UNSIGNED8 Reserved2[15];                                 // padding
  UNSIGNED8 Flags3;				             // flags indicate sector status
  UNSIGNED8 Reserved3[15];                                 // padding
} SECTOR_RECORD;

// defines a variable record
// members must be byte aligned
typedef __packed struct _Variable_Record
{
  UNSIGNED8 Flags;					         // flags indicate variable status
  UNSIGNED16 Id;					   // unique variable id
  UNSIGNED8 Data[MAX_VARIABLE_SIZE];				 // variable data
  UNSIGNED8 Checksum;						  // 2's complement checksum of id and data
} VARIABLE_RECORD;

// defines an entry in the variable lookup table
typedef struct _Lookup_Table_Entry
{
  UNSIGNED16 Id;						  // unique id of variable
  UNSIGNED32 Offset;						  // offset of variable record in currently valid sector
} LOOKUP_TABLE_ENTRY;

// IAP
#define CMD_SUCCESS 0
typedef void (*IAP)(unsigned int [], unsigned int []);

/**************************************************************************
DOES:    Module variables
**************************************************************************/

// allocate memory for non-volatile memory so it isn't used by the linker
// for something else
#pragma location = SECTOR1_STARTADDR
__no_init static UNSIGNED8 mSectorMemory1[SECTOR_SIZE];
#pragma location = SECTOR2_STARTADDR
__no_init static UNSIGNED8 mSectorMemory2[SECTOR_SIZE];

// define sectors
static SECTOR mSector1 = {mSectorMemory1, SECTOR1_NUM};
static SECTOR mSector2 = {mSectorMemory2, SECTOR2_NUM};

// the variable lookup table
static volatile LOOKUP_TABLE_ENTRY mLookupTable[MAX_VARIABLES];
// number of entries in the lookup table
static volatile UNSIGNED16 mNumVariables;
// the next free offset in the valid sector
static volatile UNSIGNED32 mNextFreeOffset;
// pointer to valid sector
static volatile SECTOR *mValidSector;

// IAP function
static IAP mIAPEntry = (IAP)IAP_LOCATION;

/**************************************************************************
DOES:    Initializes access to non-volatile memory
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_Init
  (
  void
  );

/**************************************************************************
DOES:    Sets the value of a variable
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SetVariable
  (
  UNSIGNED16 Id,			                          // id for variable
  UNSIGNED8 *Value,										   // variable data
  UNSIGNED16 Size										   // size of data in bytes
  );

/**************************************************************************
DOES:    Gets the value of a variable
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_GetVariable
  (
  UNSIGNED16 Id,						                   // id of variable
  UNSIGNED8 *Value,										   // location to store variable data
  UNSIGNED16 Size										   // size of variable in bytes
  );


/**************************************************************************
Module functions
**************************************************************************/

/**************************************************************************
DOES:    Identifies which sector is the valid one and completes any
         partially completed operations that may have been taking place
		 before the last reset
RETURNS: The valid sector or zero for error
**************************************************************************/
static SECTOR *NVOL_InitSectors
  (
  void
  );

/**************************************************************************
DOES:    Erases a sector
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
static BOOL NVOL_EraseSector
  (
  SECTOR *Sector                                           // sector to erase
  );

/**************************************************************************
DOES:    Updates the flags for a sector
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
static BOOL NVOL_SetSectorFlags
  (
  SECTOR *Sector,				                           // pointer to sector
  UNSIGNED32 Flags										   // new flags to write (SECTOR_xxx)
  );

/**************************************************************************
DOES:    Moves the data from one sector to another, removing old entries
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
static BOOL NVOL_SwapSectors
  (
  SECTOR *SrcSector,				                       // pointer to source sector
  SECTOR *DstSector 				                       // pointer to destination sector
  );

/**************************************************************************
DOES:    Constructs the lookup table from the valid sector
**************************************************************************/
static void NVOL_ConstructLookupTable
  (
  void
  );

/**************************************************************************
DOES:    Gets the offset of a variable into the valid sector
RETURNS: Offset or INVALID_VAR_OFFSET if not found
**************************************************************************/
static UNSIGNED32 NVOL_GetVariableOffset
  (
  UNSIGNED16 VariableId                                    // id of variable to look for
  );

/**************************************************************************
DOES:    Checks if a variable record is valid or not
RETURNS: TRUE if valid, FALSE if not valid
**************************************************************************/
static BOOL NVOL_IsVariableRecordValid
  (
  VARIABLE_RECORD *VarRec		                           // variable record to check
  );

/**************************************************************************
DOES:    Gets the offset of the next free location in a sector
RETURNS: Returns offset (offset = SECTOR_SIZE when sector is full)
**************************************************************************/
static UNSIGNED32 NVOL_GetNextFreeOffset
  (
  SECTOR *Sector	                                       // sector to search
  );

/**************************************************************************
DOES:    Writes a variable record into a specific sector at a specific
         offset
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SetVariableRecord
  (
  VARIABLE_RECORD *VarRec,								   // variable record to store
  SECTOR *Sector,										   // sector to write to
  UNSIGNED16 Offset										   // offset in sector for variable record
  );

/**************************************************************************
DOES:    выдает последовательные значения переменной
**************************************************************************/
BOOL NVOL_PrintDataTable 
(
UNSIGNED16 Id,						 // id of variable
UNSIGNED8 *Value,						   // location to store variable data
UNSIGNED16 Size						   // size of variable in bytes
);


void eraseFlash(void);

#endif
