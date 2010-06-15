/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.27.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI analog In channel Object header
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
000,27oct00,bt   created
*/


#ifndef  INCai_Chh
#define  INCai_Chh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include "../h/icp_mio_hw.h"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
/************************************************
** Analog Input channel object
*************************************************/
typedef enum
{
  ai_ch_initialized,
  ai_ch_uninitialized,
  ai_ch_opened,
  ai_ch_to_read
} AI_CH_STATE;

typedef enum
{
  ai_ch_permanent_read = 0,
  ai_ch_ondemand_read = 1
} AI_CH_MODE;

typedef struct _AI_CH_ADC_ADJ_PAR
{
  long  units;
  long  cb_i;
  long  cm_i;
  long  tgt_voltage_i;
  BOOL  b_4mA_20mA;
} AI_CH_ADC_ADJ_PAR;


typedef struct _AI_CHANNEL
{
  AI_CH_STATE            state;
  AI_CH_MODE             mode;
  u16                    adc_reg;
  AI_CH_ADC_ADJ_PAR      adc_adj;
  void*                  adc_parent;       /* parent object for adc params */
  u16                    immediate_data;
  int                    nr;               /* channel numbner */
  struct file_operations fops;             /* channel's file ops */
  struct wait_queue*     wq;               /* wait queue for blocking I/O */
  void (*write_data)(struct _AI_CHANNEL*, u16); /*  data write func, used 
                                                ** by irq service routine */
  STATUS (*irq_start_func)(void*, int);    /* adc IRQ start routine */
  void*                  irq_start_par;    /* adc IRQ start routine's param */
} AI_CHANNEL;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS ai_ch_init_ai_ch(AI_CHANNEL* p_ai_ch, 
                                 int ch_nr, 
                                 void* irq_start_par,
                                 STATUS (*irq_start_func)(void*, int), 
                                 void* adc_adj_param);
  extern void   ai_ch_cleanup_ai_ch(AI_CHANNEL* p_ai_ch);
  extern int    ai_ch_open(struct file* p_file, AI_CHANNEL* p_ai);

#else /* __STDC__ */

  STATUS ai_ch_init_ai_ch();
  void   ai_ch_cleanup_ai_ch();
  int    ai_ch_open();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCai_Chh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/