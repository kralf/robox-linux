/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.2.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI HW parameters
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/

 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
------------------ 
000,02oct00,bt   created
*/


#ifndef  INCicpMioHWh
#define  INCicpMioHWh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include <linux/ioctl.h>
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
/****************************
** general parameters
*****************************/
#define ICP_MIO_DEV_NAME                 "mio"

#define ICP_MIO_BAD_IOCTL                0x80000000
#define ICP_MIO_BAD_MINOR                0x80000000
#define ICP_MIO_PCI_MEM_WINDOW_SIZE      0x10000 /* 64K */

/****************************
** minor number assignment
*****************************/
/* mio board */
#define ICP_MIO_MINOR_OFFS               0

/* analog input channels */
#define MINOR_AI_OFFS                    10

/* analog output channels */
#define MINOR_AO_OFFS                    50

/* digital input channel */
#define DI_MINOR_NR_OFFS                 80

/* digital output channel */
#define DO_MINOR_NR_OFFS                 90

/* counter channels */
#define CTR_MINOR_NR_OFFS                100

/***************************
** IRQ definitions
***************************/
/* analog input channels */
#define ADC_READY_IRQ                    0x0001

/* analog output channels */
#define DAC_READY_IRQ                    0x0002

/* digital input channel */
#define DIN_STATUS_CHANGE_IRQ            0x0008

/* digital output channel */
#define DOUT_ERROR_IRQ                   0x0004

/* counter channels */
#define CTR_OVERFLOW_IRQ( ch_nr )        0x10 << (ch_nr)

/******************************
** Register offsets
******************************/
/* general registers */
#define IRQ_ENABLE_REG_OFFS              0x0C
#define IRQ_STATUS_REG_OFFS              0x0E

/* analog input channels */
#define ADC_CMD_STAT_REG_OFFS            0x00
#define AI_DATA_REG_OFFS                 0x02

/* analog output channels */
#define DAC_CMD_STAT_REG_OFFS            0x04
#define AO_DATA_REG_OFFS                 0x06

/* digital in channel */
#define DI_REG_OFFS                      0x08

/* digital output channel */
#define DO_REG_OFFS                      0x0A

/* counter channels */
#define CTR_CH_REG_OFFS( ch_nr )         0x10 + 2*(ch_nr)

/*****************************
** HW parameters 
*****************************/
/* analog input channels  */
#define ADC_NR_OF_CH                     0x10
#define ADC_CMD_REG_BUSY                 0x0001
#define ADC_TIMEOUT                      1000

/* analog output channels */
#define NR_OF_DO_CH                      0x04
#define DAC_CMD_REG_BUSY                 0x0001
#define DAC_TIMEOUT                      1000

/* digital input channel */

/* digital output channel */

/* counter channels */
#define NR_OF_CTR                        4  /* Number of counters */

/* General board Interrupt object */
/*
** number of interrupt sources = 8
** - analog in
** - analog out
** - digital in
** - digital out
** - counter * 4
*/
#define NR_OF_IRQ_SOURCES                0x08 

/* EEPROM */
#define ICP_MIO_EEP_DEV_SEL              0xA0 /* EEPROM Device address on 
                                              ** I2C bus */


/******************************************************************************
**                               IOCTL Codes
******************************************************************************/
#define ICP_MIO_MAGIC                     'q'
/* general( mio board ) */
#define ICP_MIO_IOCTL_GET_NR              _IOR(ICP_MIO_MAGIC, 0, int)

/* analog input channels */
#define MIO_IOCTL_AI_CH_READ_PERM_MODE    _IOW(ICP_MIO_MAGIC, 10, int)
#define MIO_IOCTL_AI_ENABLE_IRQ           _IOW(ICP_MIO_MAGIC, 11, int)
#define MIO_IOCTL_AI_CH_ADC_INP_RANGE     _IOW(ICP_MIO_MAGIC, 12, int)
#define MIO_IOCTL_AI_CH_ADC_BIP_INP_RANGE _IOW(ICP_MIO_MAGIC, 13, int)
#define MIO_IOCTL_AI_CH_ADC_DIFF_INP_MODE _IOW(ICP_MIO_MAGIC, 14, int)
#define MIO_IOCTL_AI_CH_ADC_UNITS         _IOW(ICP_MIO_MAGIC, 15, int)
/*
** possible unit values for MIO_IOCTL_AI_CH_ADC_UNITS 
*/
#define ADC_100uV_UNIT                    10000
#define ADC_mV_UNIT                       1000
#define ADC_10mV_UNIT                     100
#define ADC_100mV_UNIT                    10 
#define ADC_V_UNIT                        1

/* analog output channels */
#define MIO_IOCTL_AO_ENABLE_IRQ           _IOW(ICP_MIO_MAGIC, 20, int)
#define MIO_IOCTL_AO_CH_DAC_INP_RANGE     _IOW(ICP_MIO_MAGIC, 21, int)
#define MIO_IOCTL_AO_CH_DAC_BIP_INP_RANGE _IOW(ICP_MIO_MAGIC, 22, int)
#define MIO_IOCTL_AO_CH_DAC_UNITS         _IOW(ICP_MIO_MAGIC, 23, int)
/*
** possible unit values for MIO_IOCTL_AO_CH_DAC_UNITS 
*/
#define DAC_100uV_UNIT                    10000
#define DAC_mV_UNIT                       1000
#define DAC_10mV_UNIT                     100
#define DAC_100mV_UNIT                    10 
#define DAC_V_UNIT                        1

/* digital input channel */
#define MIO_IOCTL_DI_READ_MODE            _IOW(ICP_MIO_MAGIC, 30, int)
#define MIO_IOCTL_DI_ENABLE_IRQ           _IOW(ICP_MIO_MAGIC, 31, int)
/* direct read return value is the read data */
#define MIO_IOCTL_DI_DIRECT_READ          _IOR(ICP_MIO_MAGIC, 32, unsigned short)

/* digital output channel */
#define MIO_IOCTL_DO_ENABLE_IRQ           _IOW(ICP_MIO_MAGIC, 40, int)
#define MIO_IOCTL_DO_WRITE_MODE           _IOW(ICP_MIO_MAGIC, 41, int)
 /* output irq value to write */
#define MIO_IOCTL_DO_ERROR_OUT_VAL        _IOW(ICP_MIO_MAGIC, 42, unsigned char)
/* direct access */
#define MIO_IOCTL_DO_DIRECT_WRITE         _IOW(ICP_MIO_MAGIC, 43, unsigned char)

/*  counter */
#define MIO_IOCTL_CTR_CH_MODE             _IOW(ICP_MIO_MAGIC, 50, int)
#define MIO_IOCTL_CTR_CH_ENABLE_IRQ       _IOW(ICP_MIO_MAGIC, 51, int)
#define MIO_IOCTL_CTR_CH_PRESET_VALUE     _IOW(ICP_MIO_MAGIC, 52, u16)

#if PERIODIC_TIMER_SUPPORT
  #define MIO_IOCTL_CTR_CH_TIMER_FUNC       _IOW(ICP_MIO_MAGIC, 53, void*)
  #define MIO_IOCTL_CTR_CH_TIMER_PARAM      _IOW(ICP_MIO_MAGIC, 54, void*)
#endif

/* General board Interrupt object */
#define MIO_IOCTL_IRQ_GEN_ENABLE          _IOW(ICP_MIO_MAGIC, 60, int)

#if IRQ_SET_PRIO_SUPPORT
  #define MIO_IOCTL_IRQ_SET_PRIO            _IOW(ICP_MIO_MAGIC, 61, void*)
#endif



/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)


#else /* __STDC__ */


#endif /* __STDC__ */

#ifdef __cplusplus
  }
#endif

#endif /* INCicpMioHwh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/