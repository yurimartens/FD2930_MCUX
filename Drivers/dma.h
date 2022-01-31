
#ifndef DMAC_H
#define DMAC_H

#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif


//typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
//typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

// Pointer to Function returning Void (any number of parameters) 
typedef void (*PFV)();
*/

/*  more */
#define _NO_ERROR           (INT32S)(0)
#define _ERROR              (INT32S)(-1)
//#define _BIT(n)	(((INT32U)(1)) << (n))    

#define MAX_IRQChannel               (0x21)

#define DMA_INTERRUPT_PRIORITY        12

/* Number of DMA channels */
#define DMA_MAX_CHANNELS 8

#define DMA_ENABLE_INTERRUPT    1
#define DMA_DISABLE_INTERRUPT   0

/**********************************************************************/
/* Macro for determining a bit position for a channel */
#define DMAC_GET_CHAN_POS(chan)     (0x1 << ((chan) & 0x7))

/**********************************************************************/
/* Peripheral DMA bit position for I2S0 DMA0 */
#define DMA_PER_I2S0_DMA0           _BIT(0)

/* Peripheral DMA bit position for NAND FLASH (same as 12) */
#define DMA_PER_NAND1               _BIT(1)

/* Peripheral DMA bit position for I2S1 DMA0 */
#define DMA_PER_I2S1_DMA0           _BIT(2)

/* Peripheral DMA bit position for SPI2 (RX and TX) */
#define DMA_PER_SPI2_TXRX           _BIT(3)

/* Peripheral DMA bit position for SSP1 (RX) */
#define DMA_PER_SSP1_RX             _BIT(3)

/* Peripheral DMA bit position for SD card */
#define DMA_PER_SDCARD              _BIT(4)

/* Peripheral DMA bit position for HSUART1 TX */
#define DMA_PER_HSUART1_TX          _BIT(5)

/* Peripheral DMA bit position for HSUART1 RX */
#define DMA_PER_HSUART1_RX          _BIT(6)

/* Peripheral DMA bit position for HSUART2 TX */
#define DMA_PER_HSUART2_TX          _BIT(7)

/* Peripheral DMA bit position for HSUART2 RX */
#define DMA_PER_HSUART2_RX          _BIT(8)

/* Peripheral DMA bit position for HSUART7 TX */
#define DMA_PER_HSUART7_TX          _BIT(9)

/* Peripheral DMA bit position for HSUART7 RX */
#define DMA_PER_HSUART7_RX          _BIT(10)

/* Peripheral DMA bit position for I2S1 DMA1 */
#define DMA_PER_I2S1_DMA1           _BIT(10)

/* Peripheral DMA bit position for SPI1 (RX and TX) */
#define DMA_PER_SPI1_TXRX           _BIT(11)

/* Peripheral DMA bit position for SSP1 (TX) */
#define DMA_PER_SSP1_TX             _BIT(11)

/* Peripheral DMA bit position for NAND FLASH (same as 1) */
#define DMA_PER_NAND2               _BIT(12)

/* Peripheral DMA bit position for I2S0 DMA1 */
#define DMA_PER_I2S0_DMA1           _BIT(13)

/* Peripheral DMA bit position for SSP0 (RX) */
#define DMA_PER_SSP0_RX             _BIT(14)

/* Peripheral DMA bit position for SSP0 (TX) */
#define DMA_PER_SSP0_TX             _BIT(15)

/**********************************************************************
* config register definitions
**********************************************************************/
/* Bit for enabling big endian mode on AHB 1 */
#define DMAC_BIG_ENDIAN_AHB1        _BIT(2)

/* Bit for enabling big endian mode on AHB 0 */
#define DMAC_BIG_ENDIAN_AHB0        _BIT(1)

/* Bit for enabling the DMA controller */
#define DMAC_CTRL_ENABLE            _BIT(0)

/**********************************************************************
* lli register definitions
**********************************************************************/
/* Bit for selecting AHB0 (0) or AHB1 (1) */
#define DMAC_CHAN_LLI_SEL_AHB1      _BIT(0)

/**********************************************************************
* control register definitions
**********************************************************************/
/* Bit for enabling a channel terminal count interrupt */
#define DMAC_CHAN_INT_TC_EN         _BIT(31)

/* Bit for indicating address is cacheable */
#define DMAC_CHAN_PROT3             _BIT(30)

/* Bit for indicating address is bufferable */
#define DMAC_CHAN_PROT2             _BIT(29)

/* Bit for indicating address is privelaged mode (1) or user
   mode (0) */
#define DMAC_CHAN_PROT1             _BIT(28)

/* Bit for enabling automatic destination increment */
#define DMAC_CHAN_DEST_AUTOINC      _BIT(27)

/* Bit for enabling automatic source increment */
#define DMAC_CHAN_SRC_AUTOINC       _BIT(26)

/* Destination data width selection defines */
#define DMAC_CHAN_DEST_WIDTH_8      0x0
#define DMAC_CHAN_DEST_WIDTH_16     _BIT(21)
#define DMAC_CHAN_DEST_WIDTH_32     _BIT(22)

/* Source data width selection defines */
#define DMAC_CHAN_SRC_WIDTH_8       0x0
#define DMAC_CHAN_SRC_WIDTH_16      _BIT(18)
#define DMAC_CHAN_SRC_WIDTH_32      _BIT(19)

/* Destination data burst size defines (in transfer width) */
#define DMAC_CHAN_DEST_BURST_1      0
#define DMAC_CHAN_DEST_BURST_4      _BIT(15)
#define DMAC_CHAN_DEST_BURST_8      _BIT(16)
#define DMAC_CHAN_DEST_BURST_16     (_BIT(16) | _BIT(15))
#define DMAC_CHAN_DEST_BURST_32     _BIT(17)
#define DMAC_CHAN_DEST_BURST_64     (_BIT(17) | _BIT(15))
#define DMAC_CHAN_DEST_BURST_128    (_BIT(17) | _BIT(16))
#define DMAC_CHAN_DEST_BURST_256    (_BIT(17) | _BIT(16) | _BIT(15))

/* Macro for direct loading of destination burst size field */
#define DMAC_CHAN_DEST_BURST_LOAD(n) (((n) & 0x7) << 15)

/* Source data burst size defines (in transfer width) */
#define DMAC_CHAN_SRC_BURST_1       0
#define DMAC_CHAN_SRC_BURST_4       _BIT(12)
#define DMAC_CHAN_SRC_BURST_8       _BIT(13)
#define DMAC_CHAN_SRC_BURST_16      (_BIT(13) | _BIT(12))
#define DMAC_CHAN_SRC_BURST_32      _BIT(14)
#define DMAC_CHAN_SRC_BURST_64      (_BIT(14) | _BIT(12))
#define DMAC_CHAN_SRC_BURST_128     (_BIT(14) | _BIT(13))
#define DMAC_CHAN_SRC_BURST_256     (_BIT(14) | _BIT(13) | _BIT(12))

/* Macro for direct loading of source burst size field */
#define DMAC_CHAN_SRC_BURST_LOAD(n) (((n) & 0x7) << 12)

/* Macro for loading transfer size */
#define DMAC_CHAN_TRANSFER_SIZE(n)  ((n) & 0xFFF)

/**********************************************************************
* config_ch register definitions
**********************************************************************/
/* Bit for halting a DMA transfer */
#define DMAC_CHAN_HALT              _BIT(18)

/* Bit for checking active INT32S of the DMA channel */
#define DMAC_CHAN_ACTIVE            _BIT(17)

/* Bit for enabling locked transfers */
#define DMAC_CHAN_LOCK              _BIT(16)

/* Terminal count interrupt mask bit */
#define DMAC_CHAN_ITC               _BIT(15)

/* Interrupt error mask bit */
#define DMAC_CHAN_IE                _BIT(14)

/* Defines for flow control with DMA as the controller */
#define DMAC_CHAN_FLOW_D_M2M        (0x0 << 11)
#define DMAC_CHAN_FLOW_D_M2P        (0x1 << 11)
#define DMAC_CHAN_FLOW_D_P2M        (0x2 << 11)
#define DMAC_CHAN_FLOW_D_P2P      (0x3 << 11)

/* Defines for flow control with destination peripheral as the
   controller */
#define DMAC_CHAN_FLOW_DP_SP2DP     (0x4 << 11)

/* Defines for flow control with peripheral as the controller */
#define DMAC_CHAN_FLOW_P_M2P        (0x5 << 11)
#define DMAC_CHAN_FLOW_P_P2M        (0x6 << 11)

/* Defines for flow control with source peripheral as the
   controller */
#define DMAC_CHAN_FLOW_SP_SP2DP     (0x7 << 11)

/* Macro for loading destination peripheral */
#define DMAC_DEST_PERIP(n)          (((n) & 0x1F) << 6)

/* Macro for loading source peripheral */
#define DMAC_SRC_PERIP(n)           (((n) & 0x1F) << 1)

/* Channel enable bit */
#define DMAC_CHAN_ENABLE            _BIT(0)

/* Macro pointing to DMA registers */
#define DMAC ((DMAC_REGS_T *)(0x50004000))

/* DMA Connections */
#define DMA_PERID_UART0_TX           8
#define DMA_PERID_UART0_RX           9
#define DMA_PERID_UART1_TX           10
#define DMA_PERID_UART1_RX           11
#define DMA_PERID_UART2_TX           12
#define DMA_PERID_UART2_RX           13
#define DMA_PERID_UART3_TX           14
#define DMA_PERID_UART3_RX           15

/**********************************************************************
* DMA controller register structures
**********************************************************************/

/* DMA controller channel register structure */
typedef struct {
    volatile INT32U srcAddr;           /* Source address reg */
    volatile INT32U destAddr;          /* Destination address reg */
    volatile INT32U LLI;                /* Linked list item reg */
    volatile INT32U control;            /* Channel control reg */
    volatile INT32U configCh;          /* Channel config reg */
    volatile INT32U reserved[3];
} DMAC_CHAN_T;

/* DMA controller register structures */ 
typedef struct {
    volatile INT32U intStat;           /* Interrupt INT32S reg */
    volatile INT32U intTCStat;        /* INT terminal count req sts */
    volatile INT32U intTCClear;       /* INT terminal count clear */
    volatile INT32U intErrStat;       /* Interrupt error sts reg */
    volatile INT32U intErrClear;      /* Interrupt error clear reg */
    volatile INT32U rawTCStat;        /* Raw terminal count reg */
    volatile INT32U rawErrStat;       /* Raw error INT32S reg */
    volatile INT32U chanEnable;        /* Enabled channel reg */
    volatile INT32U swBurstReq;       /* SW burst req reg */
    volatile INT32U swSingleReq;      /* SW single req reg */
    volatile INT32U swLastBurstReq;  /* SW last burst req reg */
    volatile INT32U swLastSingleReq; /* SW last single req reg */
    volatile INT32U config;             /* Config register */
    volatile INT32U sync;               /* Synchronization register */
    volatile INT32U reserved[50];
    DMAC_CHAN_T     DMAChan[8];       /* Individual DMA chan regs */
} DMAC_REGS_T;

/* DMA linked list structure used with a channel's LLI register */
typedef struct 
{
	volatile INT32U DMASrc;     /* Source for data transfer */
	volatile INT32U DMADest;    /* Destination for data transfer */
	volatile INT32U nextLLI;    /* Next linked list pointer or NULL */
	volatile INT32U nextCtrl;   /* DMA control word for list entry */
} DMAC_LL_T;

/* DMA driver control structure */
typedef struct 
{
	INT32S init;
	INT32S allocCh[DMA_MAX_CHANNELS];
	PFV    cb[DMA_MAX_CHANNELS];
	INT32S numAllocCh;    /* Number of allocated channels */
	DMAC_REGS_T *pDMA;
} DMA_DRV_DATA_T;


/***********************************************************************
 * DMA driver functions
 **********************************************************************/


/* Initial DMA controller and driver */
INT32S DMAInit(void);
extern void GPDMA_IRQHandler( void );

/* Allocate a channel for DMA, use ch for selected channel (0 to 7) or
   -1 to use the highest priority available channel. Also sets up the
   user callback function for the channel's DMA interrupt. */
INT32S DMAAllocChannel(INT32S ch, PFV cb);

/* Return (free) an allocated DMA channel */
INT32S DMAFreeChannel(INT32S ch);


INT32S DMAStartTransfer(INT32S ch,
                     INT32U transferType,   
                     void *src,
                     void *dest,
                     INT8U srcDevID, 
                     INT8U destDevID,
                     INT32S trans,
                     INT8U  enInt);


extern DMA_DRV_DATA_T DMADrvDat;


#ifdef __cplusplus
}
#endif

#endif 
