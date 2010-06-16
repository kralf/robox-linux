/* $Header: /Tews/Device Driver/Linux/TDRV001/Code/tdrv001.h 4     18.02.05 10:03 Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   T D R V 0 0 1     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - TDRV001 Driver                                **
**                                                                           **
**                                                                           **
**    File             tdrv001.h                                             **
**                                                                           **
**                                                                           **
**    Function         TDRV001 LowLevel driver defintions                    **
**                                                                           **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49(0)4101/4058-0                               **
**                     Fax.: +49(0)4101/4058-19                              **
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


#ifndef __TDRV_001_H__
#define __TDRV_001_H__

#include <linux/ioctl.h>

#define TP862_IOC_MAGIC     0x42

#define TP862_NUM_CHANS             4


/* 
** Register-Offset definitions for onboard CPLD 
*/
#define TP862_CPLD_MODE_REG0            0x00
#define TP862_CPLD_MODE_REG1            0x01
#define TP862_CPLD_TX_CLK_SEL_REG       0x02
#define TP862_CPLD_DTR3_CTRL_REG        0x03
#define TP862_CPLD_RESET_REG            0x04
#define TP862_CPLD_ID_REG               0x05
#define TP862_CPLD_BUILDOPTION_REG      0x06

#define TP862_CPLD_OSC_SEL_REG          0x09
#define TP862_CPLD_0_SPECIAL_REG_A      0x0A
#define TP862_CPLD_0_SPECIAL_REG_B      0x0B
#define TP862_CPLD_1_SPECIAL_REG_A      0x0C
#define TP862_CPLD_1_SPECIAL_REG_B      0x0D
#define TP862_CPLD_2_SPECIAL_REG_A      0x0E
#define TP862_CPLD_2_SPECIAL_REG_B      0x0F
#define TP862_CPLD_3_SPECIAL_REG_A      0x10
#define TP862_CPLD_3_SPECIAL_REG_B      0x11


/*
** definitions for receiver state
*/
#define TP_RECEIVERSTATE_ON         0x01
#define TP_RECEIVERSTATE_OFF        0x00

/*
** bitmasks for programmable clock inversion
*/
#define CLOCKINV_NONE               0x00
#define CLOCKINV_TXC                0x01
#define CLOCKINV_RXC                0x02


/*
** set of standard async baud rates (with 16times oversampling)
*/
/* clockmode master1 (14.7456MHz) */
#define TP_BAUDRATE_9600            9600
#define TP_BAUDRATE_14400           14400
#define TP_BAUDRATE_19200           19200
#define TP_BAUDRATE_25600           25600
#define TP_BAUDRATE_28800           28800
#define TP_BAUDRATE_57600           57600
#define TP_BAUDRATE_115200          115200
#define TP_BAUDRATE_230400          230400
#define TP_BAUDRATE_460800          460800
#define TP_BAUDRATE_921600          921600

/* clockmode master2 (24MHz) */
#define TP_BAUDRATE_15000           15000
#define TP_BAUDRATE_25000           25000
#define TP_BAUDRATE_30000           30000
#define TP_BAUDRATE_50000           50000
#define TP_BAUDRATE_60000           60000
#define TP_BAUDRATE_75000           75000
#define TP_BAUDRATE_93750           93750
#define TP_BAUDRATE_100000          100000
#define TP_BAUDRATE_125000          125000
#define TP_BAUDRATE_250000          250000
#define TP_BAUDRATE_500000          500000
#define TP_BAUDRATE_750000          750000
#define TP_BAUDRATE_1500000         1500000


/*
** configuration values for communication type
** (used for SET_OPERATION_MODE)
*/    
typedef enum
{
    COMMTYPE_NO_CHANGES,
    COMMTYPE_HDLC,              /* HDLC  operation mode (unnumbered frames) */
    COMMTYPE_ASYNC              /* ASYNC operation mode */
} COMM_TYPE;


/*
** configuration values for clock mode
** (used for SET_OPERATION_MODE)
*/    
typedef enum
{
    CLOCKMODE_NO_CHANGES,
    CLOCKMODE_CM0A,             /*                  external clock, no division             */
    CLOCKMODE_CM0B,             /* HDLC             Tx derived from 14.7456MHz, Rx external */
    CLOCKMODE_CM3B,             /* ASYNC            derived from external clock             */
    CLOCKMODE_CM4,              /* HDLC             high-speed with external tx/rx clocks   */
    CLOCKMODE_CM7B,             /* ASYNC            derived from XTAL1 (default 14.7456MHz) */
    CLOCKMODE_CM8,              /* identical to CM8B (Xtal2 on RxC)                         */
    CLOCKMODE_CM8A,             /* Xtal1 on RxC (14.7456MHz)                                */
    CLOCKMODE_CM8B,             /* Xtal2 on RxC (24.0000MHz)                                */             
    CLOCKMODE_CM9               /* HDLC             high-speed, tx clock 10MHz (Xtal3)      */
} CLOCK_MODE;


/*
** configuration values transceiver mode
** (used for SET_OPERATION_MODE)
*/    
typedef enum
{
    TRANSCEIVER_NO_CHANGES,
    TRANSCEIVER_NOT_USED,
    TRANSCEIVER_RS530A,
    TRANSCEIVER_RS530,
    TRANSCEIVER_X21,
    TRANSCEIVER_V35,
    TRANSCEIVER_RS449,
    TRANSCEIVER_V36,
    TRANSCEIVER_RS232,
    TRANSCEIVER_V28,
    TRANSCEIVER_NO_CABLE
} TRANSCEIVER_MODE;

/*
** configuration values for dce_dte
** (used for SET_OPERATION_MODE)
*/    
typedef enum
{
    DCEDTE_NO_CHANGES,
    DCEDTE_DCE,
    DCEDTE_DTE
} DCEDTE;

/*
** configuration values for oversampling / txclk_out
** (used for SET_OPERATION_MODE)
*/    
typedef enum
{
    DISABLED = 0,
    ENABLED,
    NO_CHANGES
} ENABLE_DISABLE;

/*
** structure to select the global oscillator
*/
typedef enum
{
    OSCSOURCE_XTAL1,
    OSCSOURCE_XTAL2
} OSC_SOURCE;

/*
** structure to configure a channel (set it's operation mode)
*/
typedef struct
{
    COMM_TYPE           commtype;               /* HDLC or ASYNC    */
    CLOCK_MODE          clockmode;              /* desired clockmode    */
    ENABLE_DISABLE      txclk_out;              /* transmit clock output enable */
    TRANSCEIVER_MODE    transceivermode;        /* used mode for multiprotocol transceivers */
    DCEDTE              dce_dte;                /* DCE or DTE   */
    ENABLE_DISABLE      oversampling;           /* 16times oversampling for ASYNC mode  */
    ENABLE_DISABLE      usetermchar;            /* use a termination character for ASNYC commtype   */
    unsigned char       termchar;               /* programmable termination character   */
    unsigned long       baudrate;               /* used datarate for communication  */
    unsigned char       clockinversion;         // only 362/862 v1.1
} TP862_OPERATION_MODE_STRUCT;



/*
** structure to directly read/write specific board registers
*/
typedef struct 
{
    unsigned long   offset;             /* offset for specific TPMC862 register (DSCC4 or CPLD) */
    unsigned long   value;              /* value to write into the specified register or read value */
} TP862_REGISTER_STRUCT;


/*
** S means "Set" through a ptr,
** T means "Tell" directly with the argument value
** G means "Get": reply by setting through a pointer
** Q means "Query": response is on the return value
** X means "eXchange": G and S atomically
** H means "sHift": T and Q atomically
*/
#define TP862_IOCS_SET_OPERATION_MODE           _IOWR(TP862_IOC_MAGIC, 2, TP862_OPERATION_MODE_STRUCT)
#define TP862_IOCG_GET_OPERATION_MODE           _IOWR(TP862_IOC_MAGIC, 3, TP862_OPERATION_MODE_STRUCT)
#define TP862_IOCT_SET_BAUDRATE                 _IOWR(TP862_IOC_MAGIC, 4, unsigned long)
#define TP862_IOCT_SET_RECEIVER_STATE           _IOWR(TP862_IOC_MAGIC, 5, unsigned long)
#define TP862_IOC_CLEAR_RX_BUFFER               _IO(TP862_IOC_MAGIC, 6)
#define TP862_IOCT_SET_EXT_XTAL                 _IOWR(TP862_IOC_MAGIC, 7, unsigned long)
#define TP862_IOCT_SET_READ_TIMEOUT             _IOWR(TP862_IOC_MAGIC, 8, unsigned long)
#define TP862_IOCQ_GET_TX_COUNT_ERROR           _IOWR(TP862_IOC_MAGIC, 9, unsigned long)
#define TP862_IOCQ_GET_TX_COUNT_OK              _IOWR(TP862_IOC_MAGIC,10, unsigned long)
#define TP862_IOC_DSCC4_RESET                   _IO(TP862_IOC_MAGIC, 1)
#define TP862_IOCT_SET_OSC_SRC                  _IOWR(TP862_IOC_MAGIC, 18, OSC_SOURCE)

#define TP862_IOCS_SCC_REG_WRITE                _IOWR(TP862_IOC_MAGIC,12, TP862_REGISTER_STRUCT)
#define TP862_IOCG_SCC_REG_READ                 _IOWR(TP862_IOC_MAGIC,13, TP862_REGISTER_STRUCT)
#define TP862_IOCS_GLOB_REG_WRITE               _IOWR(TP862_IOC_MAGIC,14, TP862_REGISTER_STRUCT)
#define TP862_IOCG_GLOB_REG_READ                _IOWR(TP862_IOC_MAGIC,15, TP862_REGISTER_STRUCT)
#define TP862_IOCS_CPLD_REG_WRITE               _IOWR(TP862_IOC_MAGIC,16, TP862_REGISTER_STRUCT)
#define TP862_IOCG_CPLD_REG_READ                _IOWR(TP862_IOC_MAGIC,17, TP862_REGISTER_STRUCT)


#endif      /* __TDRV_001_H__ */
