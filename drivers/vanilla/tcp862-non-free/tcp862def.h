/* $Header: /Tews/Device Driver/Linux/TDRV001/Code/tdrv001def.h 8     18.02.05 10:01 Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @ T D R V 0 0 1 D E F @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TDRV001 Device Driver                                 **
**                                                                           **
**                                                                           **
**    File             tdrv001def.h                                          **
**                                                                           **
**                                                                           **
**    Function         Driver header file                                    **
**                                                                           **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49 / (0)4101 / 4058-0                          **
**                     Fax.: +49 / (0)4101 / 4058-19                         **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2004-2005                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/
#ifndef __TDRV001DEF_H__
#define __TDRV001DEF_H__

#include "tcp862.h"
#if defined CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#else
typedef void* devfs_handle_t; /* avoid #if defined... inside structure declarations */
#endif /* CONFIG_DEVFS_FS */
#include <asm/semaphore.h>


/*
**  Various constants
*/

#define TDRV001_MAJOR                       0       /* use automatic generated major device number */

#define TP862_TRANSMIT_TIMEOUT              5       /* seconds */

#define CHIP_TIMEOUT                        15      /* chip command timeout (jiffies) */

#define BRR_DIVIDER_MAX                     64*0x00004000    /* Cf errata DS5 p.10 */

#define IRQ_RING_SIZE                       32
#define TP862_RX_IRQ_LEN                    IRQ_RING_SIZE
#define TP862_TX_IRQ_LEN                    IRQ_RING_SIZE
#define TP862_RX_IRQ_MEM_SIZE               (TP862_RX_IRQ_LEN * sizeof(unsigned long))
#define TP862_TX_IRQ_MEM_SIZE               (TP862_TX_IRQ_LEN * sizeof(unsigned long))

#define TP862_RX_BUFFER_SIZE                2000
#define TP862_TX_BUFFER_SIZE                50

#define RECEIVE_LENGTH_LIMIT                TP862_RX_BUFFER_SIZE
#define RX_DATA_QUEUE_SIZE                  30
#define TX_DATA_QUEUE_SIZE                  30


#define TP862_DEFAULT_TX_TIMEOUT            1000
#define TP862_DEFAULT_RX_TIMEOUT            10000000


#define SCC_MAX_REG                         0x58
#define GLOB_MAX_REG                        0x00FC
#define CPLD_MAX_REG                        0x11

#define DISCARD_CRCVALUE_IN_HDLC_MODE       1

#define TP862_NUM_BAR                       2       /* use 2 PCI base address registers */

#define DEV_NAME_LEN                        20      /* length of DevFS-device-name                        */

#define ID_VERSION_1_0                      0x01
#define ID_VERSION_1_1                      0x02



/*
** Standard initialization values
*/
#define STANDARD_BPS                        TP_BAUDRATE_14400   /* 14400bps  */
#define STANDARD_CCR0                       0x80000016          /* hdlc mode */
#define STANDARD_CCR1                       0x02248080          /* adress-mode */
#define STANDARD_CCR2                       0x08030000          /* transparent u-frames */
#define STANDARD_FIFOCR1                    0x42104000          /* FIFO-Size: 32 DWORDs per channel */
#define STANDARD_FIFOCR2                    0x08420800          /* FIFO-TxThreshold: 1 DWORDs */
#define STANDARD_FIFOCR3                    0x00000000          /* FIFO-RxThershold: 0 DWORDs */
#define STANDARD_FIFOCR4                    0x00000000


#define STANDARD_BAUDRATE                   TP_BAUDRATE_14400
#define STANDARD_TRANSCEIVERMODE            TRANSCEIVER_RS530
#define STANDARD_COMMTYPE                   COMMTYPE_ASYNC
#define STANDARD_CLOCKMODE                  CLOCKMODE_CM7B
#define STANDARD_DCEDTE                     DCEDTE_DCE
#define STANDARD_TXCLOCK                    ENABLED
#define STANDARD_OVERSAMPLING               ENABLED

#define STANDARD_IMR                        0xfffaef3d          /* interrupt mask register */

#define STANDARD_MODE_REG0                  0x55                /* ch0/ch1: RS530A      */
#define STANDARD_MODE_REG1                  0x55                /* ch1/ch2: RS530A      */
#define STANDARD_CLK_SEL_REG                0x00                /* ch0..3 : NO_SOURCE (14.7456MHz) */



/*
** values for transceiver mode registers
*/
#define TP862_TRANSCEIVER_NOT_USED          0x00
#define TP862_TRANSCEIVER_RS530A            0x01
#define TP862_TRANSCEIVER_RS530             0x02
#define TP862_TRANSCEIVER_X21               0x03
#define TP862_TRANSCEIVER_V35               0x04
#define TP862_TRANSCEIVER_RS449             0x05
#define TP862_TRANSCEIVER_V36               0x05
#define TP862_TRANSCEIVER_RS232             0x06
#define TP862_TRANSCEIVER_V28               0x06
#define TP862_TRANSCEIVER_NO_CABLE          0x07

#define TP862_DCEDTE_DCE                    0x01  /* TxClk is output */
#define TP862_DCEDTE_DTE                    0x00  /* TxClk is input */

#define CLOCKSOURCE_EXTERNAL                0x01
#define CLOCKSOURCE_ONBOARD                 0x02
#define CLOCKSOURCE_XTAL1                   0x03
#define CLOCKSOURCE_XTAL2                   0x04    /* same as ONBOARD */

#define TP862_CM9_EN                        0x20

#define TP862_TXC_INV                       (1 << 7)
#define TP862_RXC_INV                       (1 << 6)


/*
** TPMC862 build-options (register content)
*/
#define TP862_BUILDOPTION_00                0x00
#define TP862_BUILDOPTION_01                0x01
#define TP862_BUILDOPTION_02                0x02
#define TP862_BUILDOPTION_03                0x03
#define TP862_BUILDOPTION_04                0x04
#define TP862_BUILDOPTION_05                0x05
#define TP862_BUILDOPTION_06                0x06
#define TP862_BUILDOPTION_07                0x07
#define TP862_BUILDOPTION_08                0x08
#define TP862_BUILDOPTION_09                0x09
#define TP862_BUILDOPTION_10                0x0A
#define TP862_BUILDOPTION_11                0x0B
#define TP862_BUILDOPTION_12                0x0C
#define TP862_BUILDOPTION_13                0x0D
#define TP862_BUILDOPTION_14                0x0E
#define TP862_BUILDOPTION_15                0x0F

/*
** XTAL1 / XTAL2 default values for different build-options
*/
#define TP862_BUILDOPTION_00_XTAL1          14745600
#define TP862_BUILDOPTION_00_XTAL2          24000000
#define TP862_BUILDOPTION_01_XTAL1          14745600
#define TP862_BUILDOPTION_01_XTAL2          24000000
#define TP862_BUILDOPTION_02_XTAL1          14745600
#define TP862_BUILDOPTION_02_XTAL2          24000000
#define TP862_BUILDOPTION_03_XTAL1          14745600
#define TP862_BUILDOPTION_03_XTAL2          24000000
#define TP862_BUILDOPTION_04_XTAL1          14745600
#define TP862_BUILDOPTION_04_XTAL2          24000000
#define TP862_BUILDOPTION_05_XTAL1          14745600
#define TP862_BUILDOPTION_05_XTAL2          24000000
#define TP862_BUILDOPTION_06_XTAL1          14745600
#define TP862_BUILDOPTION_06_XTAL2          24000000
#define TP862_BUILDOPTION_07_XTAL1          14745600
#define TP862_BUILDOPTION_07_XTAL2          24000000
#define TP862_BUILDOPTION_08_XTAL1          14745600
#define TP862_BUILDOPTION_08_XTAL2          24000000
#define TP862_BUILDOPTION_09_XTAL1          14745600
#define TP862_BUILDOPTION_09_XTAL2          24000000
#define TP862_BUILDOPTION_10_XTAL1          14745600
#define TP862_BUILDOPTION_10_XTAL2          24000000
#define TP862_BUILDOPTION_11_XTAL1          14745600
#define TP862_BUILDOPTION_11_XTAL2          24000000
#define TP862_BUILDOPTION_12_XTAL1          14745600
#define TP862_BUILDOPTION_12_XTAL2          24000000
#define TP862_BUILDOPTION_13_XTAL1          14745600
#define TP862_BUILDOPTION_13_XTAL2          24000000
#define TP862_BUILDOPTION_14_XTAL1          14745600
#define TP862_BUILDOPTION_14_XTAL2          24000000
#define TP862_BUILDOPTION_15_XTAL1          14745600
#define TP862_BUILDOPTION_15_XTAL2          24000000

/*-----------------------------------------------------------------------
  Definitions
  -----------------------------------------------------------------------*/

/*
** internal error codes
*/
#define TP862_ERROR_COMMTYPE                0x01
#define TP862_ERROR_CLOCKMODE               0x02
#define TP862_ERROR_TXCLOCK_OUT             0x03
#define TP862_ERROR_TRANSCEIVER             0x04
#define TP862_ERROR_DCEDTE                  0x05
#define TP862_ERROR_OVERSAMPLING            0x06
#define TP862_ERROR_USETERMCHAR             0x07
#define TP862_ERROR_BAUDRATE                0x08
#define TP862_ERROR_CLOCKINVERSION          0x09

/*
** Transmit Descriptor
*/
typedef struct
{
	unsigned long state;
	unsigned long next;
	unsigned long data;
	unsigned long complete;
    unsigned long txbuf_virt;   /* used to free the buffer */
    unsigned long txbuf_dma;    /* used to free the buffer */
    unsigned long size;         /* used to free the buffer */
    unsigned long finished;     /* used for blocking write */
} TP862_TxFD;

/*
** Receive Descriptor
*/
typedef struct
{
	unsigned long state1;
	unsigned long next;
	unsigned long data;
	unsigned long state2;
	unsigned long end;
} TP862_RxFD;


typedef struct
{
    struct list_head   node;
    TP862_TxFD  TxFD;
} TX_QUEUE_ENTRY;


typedef struct
{
    unsigned long   xtal1_hz;
    unsigned long   xtal2_hz;
} BUILDOPTION_STRUCT;


/*
** structure to read data from the internal channel buffer
*/
typedef struct
{
    unsigned long   NumberOfBytes;                      /* number of valid bytes inside the returned buffer */
    unsigned long   Valid;                              /* indication if the buffer contains valid data */
    unsigned long   Overflow;                           /* TRUE if an internal buffer overflow happened.    */
    unsigned long   blocking;                           /* unused   */
    unsigned long   timeout;                            /* unused   */
    unsigned char   pData[TP862_RX_BUFFER_SIZE];        /* fixed data buffer size   */
} TP862_INTERNAL_RX_BUFFER;

/*
** internal ringbuffer (old WinNT style)
*/
typedef struct
{
    int                         buffer_overrun;
    unsigned long               get_idx;
    unsigned long               put_idx;
    TP862_INTERNAL_RX_BUFFER    entry[RX_DATA_QUEUE_SIZE];
} TP862_INTERNAL_RINGBUFFER;



/*
** This data structure is stored in the device extension.
** It contains the information by the driver for this IP.
*/
typedef struct {
    struct list_head                node;               /* used to manage the list of attached channels */
    struct pci_dev                  *dev;


    int                             ChannelNumber;
    int                             BoardNumber;
    int                             Version;

    unsigned long                   scc_regstart;
    unsigned long                   cpld_space;
    unsigned long                   dscc4_space;

    /*
    ** rx interrupt queue
    */
    u32*                            rx_irq;
    dma_addr_t                      rx_irq_dma;
    unsigned long                   rx_irq_index;
    int                             rx_irq_index_outofsync;

    /*
    ** tx interrupt queue
    */
    u32*                            tx_irq;
    dma_addr_t                      tx_irq_dma;
    unsigned long                   tx_irq_index;
    int                             tx_irq_index_outofsync;

    /*
    ** tx descriptor list
    */
    TP862_TxFD*                     tx_fd;
    dma_addr_t                      tx_fd_dma;
    dma_addr_t                      tx_fd_first_dma;
    dma_addr_t                      tx_fd_last_dma;
    unsigned long                   tx_fd_index;
    unsigned long                   tx_fd_last_index;
    unsigned long                   tx_fd_index_dpc;

    /*
    ** rx descriptor list
    */
    TP862_RxFD*                     rx_fd;
    dma_addr_t                      rx_fd_dma;
    unsigned long                   rx_fd_index;

    TP862_RxFD*                     rx_fd_internal;
    dma_addr_t                      rx_fd_external_dma;
    TP862_RxFD*                     rx_fd_external;
    unsigned long                   rx_data_queue_size;

    /*
    ** rx/tx buffers (debug)
    */
    unsigned long                   rx_buf;
    dma_addr_t                      rx_buf_dma;
    unsigned char*                  tx_buf;
    dma_addr_t                      tx_buf_dma;


    TP862_INTERNAL_RINGBUFFER*      pInternalRingBuffer;
    dma_addr_t                      InternalRingBuffer_dma;
    unsigned long                   ReadOffset;  /* indicate remaining bytes of one packet */


    devfs_handle_t                  devfs_handle;       /* devfs device handle                          */
    int                             minor;              /* device's minor number                        */
    int                             OpenCount;


    /* transmit queue handling */
    int                             tx_free;        /* number of free transmit descriptors (counting semaphore) */
    unsigned long                   tx_insert;      /* actual tx insert index (for write) */


    /*
    ** timeout handling
    */
    int                             TxTimeRemaining;
    unsigned long                   DeviceWriteTimeout;    /* in seconds */

    unsigned long                   RxTimeout;              /* in jiffies */
    unsigned long                   TxTimeout;              /* in jiffies */

    unsigned long                   scc_regs_local[23];   // local copy of SCC registers

    unsigned long                   xtal1_hz;
    unsigned long                   xtal2_hz;
    unsigned long                   xtalExt_hz;
    unsigned long                   osc_hz;

    int                             is_in_reset_state;
    int                             is_in_init_state;

    int                             discardCrcValue;

    /*
    **  operation mode information
    */
    TP862_OPERATION_MODE_STRUCT     OperationMode;

    wait_queue_head_t               rx_waitqueue;  /* indicates that new data has arrived */
    wait_queue_head_t               tx_waitqueue;  /* indicates that a tx descriptor is finished  */


    struct list_head                tx_entry_queue;

    struct _TP862_DCB*              pDCB;           /* link back to our device control block */

    spinlock_t                      lock;
    struct semaphore                sema;       /* mutex semaphore */

    /*
    ** Error statistics
    */
    unsigned long                   tx_count_error;
    unsigned long                   tx_count_ok;

    struct timer_list               timer;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    /* timeout bottom-half for Kernel 2.4.x */
    struct tq_struct                taskQ;
#else
    /* timeout bottom-half for Kernel 2.6.x */
    struct work_struct              Work;
#endif

} TP862_CCB, *PTP862_CCB;






typedef struct _TP862_DCB {

    struct list_head  node;
        /* used to manage the list of known tpmc862 boards           */

    struct pci_dev  *dev;
        /* pointer to the attached pci device object                 */

    unsigned long  cpld_space;
        /* pointer to the cpld register space */

    unsigned long  dscc4_space;
        /* pointer to the DSCC4 register space        */

    struct resource *bar[TP862_NUM_BAR];
        /* allocated resources by this driver                        */


    u32*                cfg_irq;
    dma_addr_t          cfg_irq_dma;
    unsigned long       cfg_irq_index;
    int                 cfg_irq_index_outofsync;


    TP862_CCB*          pCCB[4];

    int                 BoardNumber;
    int                 Version;

    unsigned long                   xtal1_hz;
    unsigned long                   xtal2_hz;

    int                 CfgIntOk;

} TP862_DCB;


#endif      /* __TDRV001DEF_H__ */
