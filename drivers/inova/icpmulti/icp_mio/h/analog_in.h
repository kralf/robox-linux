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
** Title:   ICP-MULTI analog In Object header
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


#ifndef  INCanalogInh
#define  INCanalogInh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include "../h/icp_mio_hw.h"
#include "../h/ai_ch.h"
#include "../h/ai_adc.h"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
/******************************************** 
** Analog In object
*********************************************/
typedef enum
{
  ai_initialized,
  ai_uninitialized,
  ai_opened
} AI_STATE;


typedef struct _MIO_ANALOG_IN
{
  AI_STATE        state;
  AI_ADC          adc;
  AI_CHANNEL      ai_ch[16];
  void*           irq_ident; /* needed for irq registration */
} MIO_ANALOG_IN;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_analog_in_init(MIO_ANALOG_IN* p_ai, 
                                   u8*   mem_base_addr,
                                   void* irq_ident,
                                   u32 factory_calib);

  extern void   mio_analog_in_cleanup(MIO_ANALOG_IN* p_ai);

  extern int    mio_analog_in_open(struct inode* p_inode, 
                                   struct file* p_file, 
                                   MIO_ANALOG_IN* p_ai);

  extern int    mio_analog_in_ioctl(struct file* p_file, 
                                    unsigned int cmd, 
                                    unsigned long param, 
                                    MIO_ANALOG_IN* p_ai);
#else /* __STDC__ */

  extern STATUS mio_analog_in_init();
  extern void   mio_analog_in_cleanup();
  extern int    mio_analog_in_open();
  extern int    mio_analog_in_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCanalogInh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/