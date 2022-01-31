/**************************************************************************
MODULE:       FLASH_NVOL
CONTAINS:     Storage of non-volatile variables in Flash memory
DEVELOPED BY: Embedded Systems Academy, Inc. 2010
              www.esacademy.com
COPYRIGHT:    NXP Semiconductors, 2010. All rights reserved.
VERSION:      1.10
***************************************************************************/ 

//#include "flash_nvol.h"
#include <includes.h>


VARIABLE_RECORD *VarRecTable = (VARIABLE_RECORD *)(SECTOR1_STARTADDR + sizeof(SECTOR_RECORD));

/**************************************************************************
Public functions
**************************************************************************/

/**************************************************************************
DOES:    Initializes access to non-volatile memory
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_Init
  (
  void
  )
{
  // initialize sectors
  if ((mValidSector = NVOL_InitSectors()) == 0) return FALSE;
  
  // generate lookup table
  NVOL_ConstructLookupTable();

  return TRUE;
}

/**************************************************************************
DOES:    Identifies which sector is the valid one and completes any
         partially completed operations that may have been taking place
		 before the last reset
RETURNS: The valid sector or null for error
**************************************************************************/
SECTOR *NVOL_InitSectors
  (
  void
  )
{
  volatile SECTOR_RECORD *mSec1Rec = (SECTOR_RECORD *)(mSector1.Addr);
  volatile SECTOR_RECORD *mSec2Rec = (SECTOR_RECORD *)(mSector2.Addr);
  UNSIGNED32 Sector1Flags, Sector2Flags;

  // compile sector flags into single values
  Sector1Flags = ((UNSIGNED32)(mSec1Rec->Flags1) << 16) | ((UNSIGNED32)(mSec1Rec->Flags2) << 8) | mSec1Rec->Flags3;
  Sector2Flags = ((UNSIGNED32)(mSec2Rec->Flags1) << 16) | ((UNSIGNED32)(mSec2Rec->Flags2) << 8) | mSec2Rec->Flags3;

  // if sector 1 has invalid flags then erase it
  if ((Sector1Flags != SECTOR_EMPTY)        &&
      (Sector1Flags != SECTOR_INITIALIZING) &&
	  (Sector1Flags != SECTOR_VALID)        &&
	  (Sector1Flags != SECTOR_INVALID))
  {
    NVOL_EraseSector(&mSector1);
	Sector1Flags = SECTOR_EMPTY;
  }

  // if sector 2 has invalid flags then erase it
  if ((Sector2Flags != SECTOR_EMPTY)        &&
      (Sector2Flags != SECTOR_INITIALIZING) &&
	  (Sector2Flags != SECTOR_VALID)        &&
	  (Sector2Flags != SECTOR_INVALID))
  {
    NVOL_EraseSector(&mSector2);
	Sector2Flags = SECTOR_EMPTY;
  }

  // what happens next depends on status of both sectors
  switch (Sector1Flags)
  {
    case SECTOR_EMPTY:
	  switch (Sector2Flags)
	  {
	    // sector 1 empty, sector 2 empty
	    case SECTOR_EMPTY:
	      // use sector 1
	      if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector1;
		// sector 1 empty, sector 2 initializing
		case SECTOR_INITIALIZING:
	      // use sector 2
	      if (!NVOL_SetSectorFlags(&mSector2, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
  	      return &mSector2;
		// sector 1 empty, sector 2 valid
		case SECTOR_VALID:
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
	      // sector 2 is already active
	      return &mSector2;
		// sector 1 empty, sector 2 invalid
		case SECTOR_INVALID:
	      // swap sectors 2 -> 1
	      if (!NVOL_SwapSectors(&mSector2, &mSector1)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
	      // use sector 1
	      return &mSector1;
	  }
	  break;

	case SECTOR_INITIALIZING:
	  switch (Sector2Flags)
	  {
	    // sector 1 initializing, sector 2 empty
	    case SECTOR_EMPTY:
	      // use sector 1
	      if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector1;
		// sector 1 initializing, sector 2 initializing
		case SECTOR_INITIALIZING:
	      // erase sector 2
	      if (!NVOL_EraseSector(&mSector2)) return 0;
	      // use sector 1
	      if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector1;
		// sector 1 initializing, sector 2 valid
		case SECTOR_VALID:
	      // erase sector 1
	      if (!NVOL_EraseSector(&mSector1)) return 0;
          // swap sectors 2 -> 1
	      if (!NVOL_SwapSectors(&mSector2, &mSector1)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
	      // use sector 1
	      return &mSector1;
		// sector 1 initializing, sector 2 invalid
		case SECTOR_INVALID:
	      // erase sector 2
	      if (!NVOL_EraseSector(&mSector2)) return 0;
	      // use sector 1
	      if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector1;
	  }
	  break;

	case SECTOR_VALID:
	  switch (Sector2Flags)
	  {
	    // sector 1 valid, sector 2 empty
	    case SECTOR_EMPTY:
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
	      // sector 1 is active
	      return &mSector1;
		// sector 1 valid, sector 2 initializing
		case SECTOR_INITIALIZING:
	      // erase sector 2
	      if (!NVOL_EraseSector(&mSector2)) return 0;
          // swap sectors 1 -> 2	  
	      if (!NVOL_SwapSectors(&mSector1, &mSector2)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
	      // use sector 2
	      return &mSector2;
		// sector 1 valid, sector 2 valid
		case SECTOR_VALID:
	      // erase sector 2 and use sector 1
	      if (!NVOL_EraseSector(&mSector2)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
	      return &mSector1;
		// sector 1 valid, sector 2 invalid
		case SECTOR_INVALID:
	      // erase sector 2 and use sector 1
	      if (!NVOL_EraseSector(&mSector2)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
	      return &mSector1;
	  }
	  break;

	case SECTOR_INVALID:
	  switch (Sector2Flags)
	  {
	    // sector 1 invalid, sector 2 empty
	    case SECTOR_EMPTY:
	      // swap sectors 1 -> 2
	      if (!NVOL_SwapSectors(&mSector1, &mSector2)) return 0;
	      // use sector 2
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
	      return &mSector2;
		// sector 1 invalid, sector 2 initializing
		case SECTOR_INITIALIZING:
	      // erase sector 1
	      if (!NVOL_EraseSector(&mSector1)) return 0;
	      // use sector 2
	      if (!NVOL_SetSectorFlags(&mSector2, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector2;
		// sector 1 invalid, sector 2 valid
		case SECTOR_VALID:
	      // erase sector 1
	      if (!NVOL_EraseSector(&mSector1)) return 0;
          mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
	      // use sector 2
	      return &mSector2;
		// sector 1 invalid, sector 2 invalid
		case SECTOR_INVALID:
	      // both sectors invalid so erase both and use sector 1
	      if (!NVOL_EraseSector(&mSector1)) return 0;
	      if (!NVOL_EraseSector(&mSector2)) return 0;
	      if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
	      mNextFreeOffset = sizeof(SECTOR_RECORD);
	      return &mSector1;
	  }
	  break;
  }

  return 0;
}

/**************************************************************************
DOES:    Erases a sector
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_EraseSector
  (
  SECTOR *Sector                                           // sector to erase
  )
{
  unsigned int Command[5], Result[5];

  // prepare sector
  Command[0] = 50;
  Command[1] = Sector->Num;
  Command[2] = Sector->Num;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE; 

  // erase sector
  Command[0] = 52;
  Command[1] = Sector->Num;
  Command[2] = Sector->Num;
  Command[3] = CPU_CLK;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE;
  
  return TRUE; 
}

/**************************************************************************
DOES:    Updates the flags for a sector
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SetSectorFlags
  (
  SECTOR *Sector,				                    // pointer to sector
  UNSIGNED32 Flags						   // new flags to write
  )
{
  UNSIGNED8 Buffer[WRITE_PAGE_SIZE];
  UNSIGNED32 Byte;
  SECTOR_RECORD *SecRec;
  unsigned int Command[5], Result[5];

  // set up buffer
  for (Byte = 0; Byte < WRITE_PAGE_SIZE; Byte++) Buffer[Byte] = 0xFF;

  // configure sector record
  SecRec = (SECTOR_RECORD *)Buffer;
  SecRec->Flags1 = (Flags >> 16) & 0xFF;
  SecRec->Flags2 = (Flags >> 8) & 0xFF;
  SecRec->Flags3 = Flags & 0xFF;

  // prepare sector
  Command[0] = 50;
  Command[1] = Sector->Num;
  Command[2] = Sector->Num;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE; 

  // write to sector
  Command[0] = 51;
  Command[1] = (UNSIGNED32)(Sector->Addr);
  Command[2] = (UNSIGNED32)Buffer;
  Command[3] = WRITE_PAGE_SIZE;
  Command[4] = CPU_CLK;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE;

  // verify
  Command[0] = 56;
  Command[1] = (UNSIGNED32)(Sector->Addr);
  Command[2] = (UNSIGNED32)Buffer;
  Command[3] = sizeof(SecRec);
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE;

  return TRUE;
}

/**************************************************************************
DOES:    Moves the data from one sector to another, removing old entries
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SwapSectors
  (
  SECTOR *SrcSector,			             // pointer to source sector
  SECTOR *DstSector 				     // pointer to destination sector
  )
{
  UNSIGNED16 Var;
  VARIABLE_RECORD *VarRec;
  UNSIGNED16 DstOffset;

  // make sure destination sector is erased
  if (DstSector->Addr[0] != 0xFF)
  {
    // erase it
    if (!NVOL_EraseSector(DstSector)) return FALSE;
  }

  // mark destination sector as being initialized
  if (!NVOL_SetSectorFlags(DstSector, SECTOR_INITIALIZING)) return FALSE; 
  DstOffset = sizeof(SECTOR_RECORD);

  // copy variables to destination sector
  for (Var = 0; Var < mNumVariables; Var++)
  {
    VarRec = (VARIABLE_RECORD *)(SrcSector->Addr + mLookupTable[Var].Offset);
	if (!NVOL_SetVariableRecord(VarRec, DstSector, DstOffset)) return FALSE;
	DstOffset += sizeof(VARIABLE_RECORD);
  }

  // mark source sector as being invalid
  if (!NVOL_SetSectorFlags(SrcSector, SECTOR_INVALID)) return FALSE; 
  // mark destination sector as being valid
  if (!NVOL_SetSectorFlags(DstSector, SECTOR_VALID)) return FALSE; 
  // erase source sector
  if (!NVOL_EraseSector(SrcSector)) return FALSE;

  // now using destination sector
  mValidSector = DstSector;
  // get next free location in destination sector
  mNextFreeOffset = NVOL_GetNextFreeOffset(DstSector);
  // regenerate lookup table
  NVOL_ConstructLookupTable();

  return TRUE;
}

/**************************************************************************
DOES:    Gets the offset of the next free location in a sector
RETURNS: Returns offset (offset = SECTOR_SIZE when sector is full)
**************************************************************************/
UNSIGNED32 NVOL_GetNextFreeOffset
  (
  SECTOR *Sector	                                   // sector to search
  )
{
  VARIABLE_RECORD *VarRec = (VARIABLE_RECORD *)(Sector->Addr + sizeof(SECTOR_RECORD));

  // loop through variable records
  while (VarRec->Flags != 0xFF)
  {
    VarRec++;
	// if reached end of sector then we are finished
	if ((UNSIGNED8 *)VarRec >= (Sector->Addr + SECTOR_SIZE)) return SECTOR_SIZE;
  }

  return (UNSIGNED32)((UNSIGNED8 *)VarRec - Sector->Addr);
}

/**************************************************************************
DOES:    Constructs the lookup table from the valid sector
**************************************************************************/
void NVOL_ConstructLookupTable
  (
  void
  )
  {
    VARIABLE_RECORD *VarRec = (VARIABLE_RECORD *)(mValidSector->Addr + sizeof(SECTOR_RECORD));
    UNSIGNED16 Var;
    BOOL Found;
    
    // no variables yet
    mNumVariables = 0;
    
    // loop through variable records in sector
    while (VarRec->Flags != 0xFF)
    {
      Found = FALSE;
      
      // if variable record is valid then add to lookup table
      if (NVOL_IsVariableRecordValid(VarRec))
      {
        // search for variable in lookup table. if already found then update existing entry with new offset
        for (Var = 0; Var < mNumVariables; Var++)
        {
          if (mLookupTable[Var].Id == VarRec->Id)
          {
            mLookupTable[Var].Offset = (UNSIGNED16)((UNSIGNED8 *)VarRec - mValidSector->Addr);
            Found = TRUE;
            break;
          }
        }
        
        // if variable not in lookup table then add it
        if (!Found)
        {
          mLookupTable[mNumVariables].Id = VarRec->Id;
          mLookupTable[mNumVariables++].Offset = (UNSIGNED16)((UNSIGNED8 *)VarRec - mValidSector->Addr);
        }
      }
      
      // move to next record
      VarRec++;
      
      // if reached end of sector then we are finished
      if ((UNSIGNED8 *)VarRec >= (mValidSector->Addr + SECTOR_SIZE)) break;
      // if reached maximum number of variables then we are finished
      if (mNumVariables == MAX_VARIABLES) break;
    }
  }

/**************************************************************************
DOES:    Checks if a variable record is valid or not
RETURNS: TRUE if valid, FALSE if not valid
**************************************************************************/
BOOL NVOL_IsVariableRecordValid
  (
  VARIABLE_RECORD *VarRec		                  // variable record to check
  )
{
  UNSIGNED8 Checksum;
  UNSIGNED16 Byte;

  // check flags
  if (VarRec->Flags != 0xAA) return FALSE;

  // check checksum
  Checksum = VarRec->Id & 0xFF;
  Checksum += (VarRec->Id >> 8) & 0xFF;
  for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++)
  {
	Checksum += VarRec->Data[Byte];
  }
  Checksum = 0x100 - Checksum;
  if (VarRec->Checksum != Checksum) return FALSE;

  return TRUE;
}

/**************************************************************************
DOES:    Gets the offset of a variable into the valid sector
RETURNS: Offset or INVALID_VAR_OFFSET if not found
**************************************************************************/
UNSIGNED32 NVOL_GetVariableOffset
  (
  UNSIGNED16 VariableId                               // id of variable to look for
  )
{
  UNSIGNED16 Var;

  for (Var = 0; Var < mNumVariables; Var++)
  {
    if (mLookupTable[Var].Id == VariableId) return mLookupTable[Var].Offset;
  }

  return INVALID_VAR_OFFSET;
}

/**************************************************************************
DOES:    Gets the value of a variable
RETURNS: TRUE for success, FALSE for error/not found
**************************************************************************/
BOOL NVOL_GetVariable
  (
  UNSIGNED16 Id,						 // id of variable
  UNSIGNED8 *Value,						   // location to store variable data
  UNSIGNED16 Size						   // size of variable in bytes
  )
{
  UNSIGNED32 Offset;
  VARIABLE_RECORD *VarRec;
  UNSIGNED16 Byte;

  // find offset for variable
  Offset = NVOL_GetVariableOffset(Id);
  if (Offset == INVALID_VAR_OFFSET) return FALSE;

  // get variable record
  VarRec = (VARIABLE_RECORD *)(mValidSector->Addr + Offset);

  // copy data
  for (Byte = 0; Byte < Size; Byte++)
  {
    Value[Byte] = VarRec->Data[Byte];
  }

  return TRUE;
}

/**************************************************************************
DOES:    Writes a variable record into a specific sector at a
         specific offset
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SetVariableRecord
  (
  VARIABLE_RECORD *VarRec,					  // variable record to store
  SECTOR *Sector,						   // sector to write to
  UNSIGNED16 Offset						   // offset in sector for variable record
  )
{
  UNSIGNED16 Byte;
  UNSIGNED8 Buffer[WRITE_PAGE_SIZE];
  unsigned int Command[5], Result[5];
  VARIABLE_RECORD *TmpVarRec;

  // set up buffer
  for (Byte = 0; Byte < WRITE_PAGE_SIZE; Byte++) Buffer[Byte] = 0xFF;

  // get offset of temporary variable record in 256 byte segment
  TmpVarRec = (VARIABLE_RECORD *)(Buffer + (Offset % 256));
  // copy variable record
  *TmpVarRec = *VarRec;
  for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++) TmpVarRec->Data[Byte] = VarRec->Data[Byte];

  // prepare sector
  Command[0] = 50;
  Command[1] = Sector->Num;
  Command[2] = Sector->Num;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE; 

  // write to sector
  Command[0] = 51;
  Command[1] = (UNSIGNED32)(Sector->Addr + Offset - (Offset % 256));
  Command[2] = (UNSIGNED32)Buffer;
  Command[3] = WRITE_PAGE_SIZE;
  Command[4] = CPU_CLK;
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE;

  // verify
  Command[0] = 56;
  Command[1] = (UNSIGNED32)(Sector->Addr + Offset);
  Command[2] = (UNSIGNED32)(Buffer + (Offset % 256));
  Command[3] = sizeof(VarRec);
  DISABLEIRQ;
  mIAPEntry(Command, Result);
  ENABLEIRQ;
  if (Result[0] != CMD_SUCCESS) return FALSE;

  return TRUE;
}

/**************************************************************************
DOES:    Sets the value of a variable
RETURNS: TRUE for success, FALSE for error
**************************************************************************/
BOOL NVOL_SetVariable
  (
  UNSIGNED16 Id,				              // id for variable
  UNSIGNED8 *Value,					   // variable data
  UNSIGNED16 Size					   // size of data in bytes
  )
{
  UNSIGNED16 Byte;
  VARIABLE_RECORD VarRec;
  UNSIGNED8 CurrentValue[MAX_VARIABLE_SIZE];
  UNSIGNED16 NumSameBytes = 0;
  
  if(Size > MAX_VARIABLE_SIZE) Size = MAX_VARIABLE_SIZE;

  // get current value for this variable, if one exists
  // and compare with new value
  if (NVOL_GetVariable(Id, CurrentValue, Size))
  {
    for (Byte = 0; Byte < Size; Byte++)
	{
	  if (Value[Byte] == CurrentValue[Byte]) NumSameBytes++;
	}
  }
  // if new value is the same as the current value then no need to store
  // the new value
  if (NumSameBytes == Size) return TRUE;

  // if sector is full then swap sectors
//  if (mNextFreeOffset >= SECTOR_SIZE)
  if ((mNextFreeOffset /*- sizeof(VARIABLE_RECORD)*/ + WRITE_PAGE_SIZE) >= SECTOR_SIZE)
  {
    if (mValidSector == &mSector1)
	{
	  if (!NVOL_SwapSectors(&mSector1, &mSector2)) return FALSE;
	}
	else if (mValidSector == &mSector2)
	{
	  if (!NVOL_SwapSectors(&mSector2, &mSector1)) return FALSE;
	}
	// if no space in new sector then no room for more variables
    if (mNextFreeOffset >= SECTOR_SIZE) return FALSE;
  }

  // assemble variable record
  VarRec.Flags = 0xAA;
  VarRec.Id = Id;
  VarRec.Checksum = Id & 0xFF;
  VarRec.Checksum += (Id >> 8) & 0xFF;
  for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++)
  {
    if (Byte < Size)
	  VarRec.Data[Byte] = Value[Byte];
	else
	  VarRec.Data[Byte] = 0x00;
	VarRec.Checksum += VarRec.Data[Byte];
  }
  VarRec.Checksum = 0x100 - VarRec.Checksum;

  // store record in sector
  if (!NVOL_SetVariableRecord(&VarRec, (SECTOR *)mValidSector, mNextFreeOffset)) return FALSE;

  // get offset of next free location
  mNextFreeOffset += sizeof(VARIABLE_RECORD);

  // add new variable record to lookup table
  NVOL_ConstructLookupTable();

  return TRUE;
}


/**************************************************************************
DOES:    выдает последовательные значения переменной
**************************************************************************/
BOOL NVOL_PrintDataTable 
(
UNSIGNED16 Id,						 // id of variable
UNSIGNED8 *Value,						   // location to store variable data
UNSIGNED16 Size						   // size of variable in bytes
)
{
  UNSIGNED16 Byte;
  static BOOL First = TRUE;
  
  if(First) VarRecTable = (VARIABLE_RECORD *)(mValidSector->Addr + sizeof(SECTOR_RECORD));
  
  // if reached end of sector then we are finished
  if ((UNSIGNED8 *)VarRecTable >= (mValidSector->Addr + SECTOR_SIZE))
  {
    First = TRUE;
    return FALSE;
  }
  
  // loop through variable records in sector
  if (VarRecTable->Flags != 0xFF)
  {
    // if variable record is valid then return its value
    if (NVOL_IsVariableRecordValid(VarRecTable)) //нашли переменную посылаем ее значение
    {
      if(VarRecTable->Id == NVRAMLOGBUF)  //проверяем ID переменной
      {
        // copy data
        for (Byte = 0; Byte < Size; Byte++)
        {
          Value[Byte] = VarRecTable->Data[Byte];
        }
      }
    }
    First = FALSE;
    // move to next record
    VarRecTable++;
    return TRUE;
  }
  else  //больше нет значений
  {
    First = TRUE;
    return FALSE;
  }
}

void eraseFlash(void)
{
  NVOL_EraseSector(&mSector1);
  NVOL_EraseSector(&mSector2);
}
