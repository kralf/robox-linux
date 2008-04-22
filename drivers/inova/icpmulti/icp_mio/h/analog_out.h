/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.30.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI analog out Object header
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
000,30sep00,bt   created
*/


#ifndef  INCanalogOuth
#define  INCanalogOuth

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include "../h/icp_mio_hw.h"
#include "../h/ao_ch.h"
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/******************************************** 
** D/A converter object
*********************************************/
typedef struct _DAC_PARAMETERS
{
  int          active_ch;
  BOOL         dac_busy;
} DAC_PARAMETERS;


/******************************************** 
** Analog Out object
*********************************************/
typedef enum
{
  ao_initialized,
  ao_uninitialized,
  ao_opened
} AO_STATE;


typedef struct _MIO_ANALOG_OUT
{
  AO_STATE        state;
  u8*             dac_cmd_stat_reg;
  u8*             ao_data_reg;
  u8*             interrupt_enable_reg;
  u8*             interrupt_status_reg;
  DAC_PARAMETERS  dac;
  AO_CHANNEL      ao_ch[NR_OF_DO_CH];
  void*           irq_ident; /* needed for irq registration */
} MIO_ANALOG_OUT;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_analog_out_init(MIO_ANALOG_OUT* p_ao, 
                                    u8*   mem_base_addr,
                                    void* irq_ident,
                                    void* p_ai);

  extern void   mio_analog_out_cleanup(MIO_ANALOG_OUT* p_ao);

  extern int    mio_analog_out_open(struct inode* p_inode, 
                                    struct file* p_file, 
                                    MIO_ANALOG_OUT* p_ao);

  extern int    mio_analog_out_ioctl(struct file* p_file, 
                                     unsigned int cmd, 
                                     unsigned long param, 
                                     MIO_ANALOG_OUT* p_ao);
#else /* __STDC__ */

  extern STATUS mio_analog_out_init();
  extern void   mio_analog_out_cleanup();
  extern int    mio_analog_out_open();
  extern int    mio_analog_out_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCananlogOuth  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/