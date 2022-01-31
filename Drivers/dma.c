
#include "includes.h"

extern volatile PFV cb[MAX_IRQChannel];

#define PCGPDMA     (1<<29)

/***********************************************************************
* DMA driver private data
***********************************************************************/


/* DMAS driver data */
DMA_DRV_DATA_T DMADrvDat;



/***********************************************************************
*
* Function: DMAinit
*
* Purpose: Initial DMA controller and driver
*
* Processing:
*     This function sets up the DMA controller as initially disabled.
*     All DMA channels used by the driver are unallocated.
* 
* Parameters: None
*
* Outputs: None
*
* Returns:
*     _ERROR if the device was already initialized, otherside _NO_ERROR
*
* Notes: None
*
**********************************************************************/
INT32S DMAInit(void)
{
  INT32S idx;
  INT32S init = _ERROR;
  
  /* Only continue if driver has not been previously initialized */
  if (DMADrvDat.init == FALSE)
  {
    DMADrvDat.init = TRUE;
    DMADrvDat.numAllocCh = 0;
    
    /* Save base address of DMA controller registers */
    //DMADrvDat.pDMA = (DMAC_REGS_T *) DMA_BASE_ADDR;
    DMADrvDat.pDMA = (DMAC_REGS_T *) GPDMA;
    
    /* Enable clock to DMA controller (for now) */
    SC->PCONP |= PCGPDMA;
    
    /* Make sure DMA controller and all channels are disabled.
    Controller is in little-endian mode. Disable sync signals */
    DMADrvDat.pDMA->config = 0;
    DMADrvDat.pDMA->sync = 0;
    
    /* Clear interrupt and error INT32Ses */
    DMADrvDat.pDMA->intTCClear = 0xFF;
    DMADrvDat.pDMA->rawTCStat = 0xFF;
    
    /* All DMA channels are initially disabled and unallocated */
    for (idx = 0; idx < DMA_MAX_CHANNELS; idx++)
    {
      /* Channel is currently unallocated */
      DMADrvDat.allocCh[idx] = FALSE;
      DMADrvDat.cb[idx] = NULL;
      
      /* Make sure channel is disabled */
      DMADrvDat.pDMA->DMAChan[idx].control = 0;
      DMADrvDat.pDMA->DMAChan[idx].configCh = 0;
    }
    
    /* Disable clock to DMA controller. The clock will only be
    enabled when one or moer channels are active. */
    SC->PCONP &= ~PCGPDMA;
    NVIC_SetPriority(DMA_IRQn, DMA_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(DMA_IRQn);
    
    init = _NO_ERROR;
  }
  
  return init;
}

/***********************************************************************
*
* Function: DMAAllocChannel
*
* Purpose: Allocate a channel for DMA
*
* Processing:
*     If the passed channel is (-1), then a search loop is used to
*     find the first unallocated channel. The channel value is saved
*     and then checked to make sure it is unallocated. If it is
*     already allocated or not allocatable, then an error si return to
*     the caller. If the channel is not allocated, the channel is
*     marked as allocated and the channel ID is returned to the caller.
*     If at leasxt one channel is active, the DMA clock is enabled.
* 
* Parameters:
*     ch : Must be 0 (highest priority) to 7, or -1 for auto-allocation
*     cb : Pointer to user callback function when an interrupt occurs
*
* Outputs: None
*
* Returns: The channel index, or _ERROR if a channel wasn't allocated
*
* Notes: None
*
**********************************************************************/
INT32S DMAAllocChannel(INT32S ch, PFV cbk)
{
  INT32S challoc = ch;
  
  /* If auto-allocate is used, find the first available channel
  starting with the highest priority first */
  if (ch == -1)
  {
    ch = 0;
    challoc = _ERROR;
    while (ch < DMA_MAX_CHANNELS)
    {
      if (DMADrvDat.allocCh[ch] == FALSE)
      {
        /* Channel is free, use it */
        challoc = ch;
        ch = DMA_MAX_CHANNELS;
      }
      else 
      {
        /* Try next channel */
        ch++;
      }
    }
  }
  
  /* Only continue if channel is ok */
  if (challoc != _ERROR)
  {
    /* If the current channel is allocated already, then return an
    error instead */
    if (DMADrvDat.allocCh[challoc] == FALSE)
    {
      /* Channel is free, so use it */
      DMADrvDat.allocCh[challoc] = TRUE;
      DMADrvDat.cb[challoc] = cbk;
      DMADrvDat.numAllocCh++;
      
      /* Enable DMA clock if at least 1 DMA channel is used */
      if (DMADrvDat.numAllocCh == 1)
      {
        SC->PCONP |= PCGPDMA;
        
        /* Enable DMA controller */
        DMADrvDat.pDMA->config = DMAC_CTRL_ENABLE;
        
        /* Install DMA interrupt handler in interrupt controller
        and enable DMA interrupt */
        //				cb[DMA_IRQChannel] = (PFV)DMA_Handler; 
        //                vic_enable_ch(DMA_IRQChannel,1);
      }
    }
    else 
    {
      /* Selected channel is allocated, return an error */
      challoc = _ERROR;
    }
  }
  
  return challoc;
}

/***********************************************************************
*
* Function: DMAFreeChannel
*
* Purpose: Return (free) an allocated DMA channel
*
* Processing:
*     If the channel has been previously allocated, then deallocate
*     the channel and disable the channel in the DMA controller. If
*     no other DMA channels are enabled, the disable the DMA controller
*     along with the controller clock and DMA interrupts.
* 
* Parameters:
*     ch : Must be 0 to 7
*
* Outputs: None
*
* Returns: _NO_ERROR if the channel was freed, otherwise _ERROR
*
* Notes: None
*
**********************************************************************/
INT32S DMAFreeChannel(INT32S ch)
{
  INT32S status = _ERROR;
  if (DMADrvDat.allocCh[ch] == TRUE)
  {
    /* Deallocate channel */
    DMADrvDat.allocCh[ch] = FALSE;
    DMADrvDat.numAllocCh--;
    
    /* Shut down channel */
    DMADrvDat.pDMA->DMAChan[ch].control = 0;
    DMADrvDat.pDMA->DMAChan[ch].configCh = 0;
    DMADrvDat.pDMA->sync &= ~_BIT(ch);
    
    /* If no other DMA channels are enabled, then disable the DMA
    controller and disable the DMA clock */
    /*		if (DMADrvDat.numAllocCh == 0)
    {
    DMADrvDat.pDMA->config = 0;
    SC->PCONP &= ~PCGPDMA;
    
    // Disable DMA interrupt 
    //			cb[DMA_IRQChannel] = (PFV)DMA_Handler; 
    //			cb[DMA_IRQChannel] = (PFV)NULL; 
    //            vic_enable_ch(DMA_IRQChannel,0);
  }
    */
    status = _NO_ERROR;
  }
  
  return status;
}



/* extensions for test DMA to Periph */
INT32S DMAStartTransfer(INT32S ch,
                        INT32U transferType,   
                        void *src,
                        void *dest,
                        INT8U srcDevID, 
                        INT8U destDevID,
                        INT32S trans,
                        INT8U  enInt)
{
  INT32S sts = _ERROR;
  
  /* Verify that the selected channel has been allocated */
  if (DMADrvDat.allocCh[ch] == TRUE)
  {
    /* Setup source and destination and clear LLI */
    DMADrvDat.pDMA->DMAChan[ch].srcAddr = (INT32U) src;
    DMADrvDat.pDMA->DMAChan[ch].destAddr = (INT32U) dest;
    
    // обнулим конфигурацию канала, для смены размера
    DMADrvDat.pDMA->DMAChan[ch].configCh = 0;
    DMADrvDat.pDMA->DMAChan[ch].control = 0;
    
    switch (transferType){
    case DMAC_CHAN_FLOW_D_M2M:   
      DMADrvDat.pDMA->DMAChan[ch].control =
        (((enInt)?DMAC_CHAN_INT_TC_EN:0) | DMAC_CHAN_SRC_AUTOINC |	
         DMAC_CHAN_DEST_AUTOINC | DMAC_CHAN_SRC_WIDTH_8 |
           DMAC_CHAN_DEST_WIDTH_8 | DMAC_CHAN_SRC_BURST_1 |
             DMAC_CHAN_DEST_BURST_1 | DMAC_CHAN_TRANSFER_SIZE(trans));
      
      DMADrvDat.pDMA->DMAChan[ch].configCh =
        (DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_M2M | 
         DMAC_CHAN_ENABLE);
      break;                
      
    case DMAC_CHAN_FLOW_D_M2P:
      DMADrvDat.pDMA->DMAChan[ch].control =
        (((enInt)?DMAC_CHAN_INT_TC_EN:0) | DMAC_CHAN_SRC_AUTOINC | 
         DMAC_CHAN_SRC_WIDTH_8 |
           DMAC_CHAN_DEST_WIDTH_8 | DMAC_CHAN_SRC_BURST_1 |
             DMAC_CHAN_DEST_BURST_1 | DMAC_CHAN_TRANSFER_SIZE(trans));
      
      DMADrvDat.pDMA->DMAChan[ch].configCh =
        (DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_M2P | 
         DMAC_DEST_PERIP(destDevID) | DMAC_CHAN_ENABLE);                     
      break;  
      
    case DMAC_CHAN_FLOW_D_P2M:                         
      DMADrvDat.pDMA->DMAChan[ch].control =
        (((enInt)?DMAC_CHAN_INT_TC_EN:0) | DMAC_CHAN_DEST_AUTOINC	
         | DMAC_CHAN_SRC_WIDTH_8 |
           DMAC_CHAN_DEST_WIDTH_8 | DMAC_CHAN_SRC_BURST_1 |
             DMAC_CHAN_DEST_BURST_1 | DMAC_CHAN_TRANSFER_SIZE(trans));
      
      DMADrvDat.pDMA->DMAChan[ch].configCh =
        (DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_P2M | 
         DMAC_SRC_PERIP(srcDevID) | DMAC_CHAN_ENABLE); 
      break;                    
    case DMAC_CHAN_FLOW_D_P2P:
      DMADrvDat.pDMA->DMAChan[ch].control =
        (((enInt)?DMAC_CHAN_INT_TC_EN:0) | DMAC_CHAN_SRC_WIDTH_8 |
         DMAC_CHAN_DEST_WIDTH_8 | DMAC_CHAN_SRC_BURST_1 |
           DMAC_CHAN_DEST_BURST_1 | DMAC_CHAN_TRANSFER_SIZE(trans));
      
      DMADrvDat.pDMA->DMAChan[ch].configCh =
        (DMAC_CHAN_ITC | DMAC_CHAN_IE | DMAC_CHAN_FLOW_D_P2P | 
         DMAC_SRC_PERIP(srcDevID) | DMAC_DEST_PERIP(destDevID) | 
           DMAC_CHAN_ENABLE);   
      break;
    }
    
    
    sts = _NO_ERROR;
  }
  
  return sts;
}

