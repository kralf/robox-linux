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
** Title:   ICP-MULTI counter Object header
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


#ifndef  INCcounterh
#define  INCcounterh

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
** Counter channel object
*************************************************/
typedef enum
{
  ctr_ch_initialized,
  ctr_ch_uninitialized,
  ctr_ch_opened,
  ctr_ch_wait_for_overflow
} CTR_CH_STATE;

typedef enum
{
  ctr_ch_normal_operation = 0,
  ctr_ch_wakeup = 1

#if PERIODIC_TIMER_SUPPORT
  ,ctr_ch_auto_load
#endif

} CTR_CH_MODE;

#if PERIODIC_TIMER_SUPPORT
  typedef void(void*)  MIO_CTR_CH_TIMER;
#endif

typedef struct _CTR_CHANNEL
{
  CTR_CH_STATE           state;
  CTR_CH_MODE            mode;
  u8*                    ctr_reg;
  u8*                    interrupt_enable_reg;
  u8*                    interrupt_status_reg;
  u16                    preset_data;      /* data to write after overflow */
  int                    nr;               /* channel numbner */
  struct file_operations fops;             /* channel's file ops */
  struct wait_queue*     wq;               /* wait queue for blocking I/O */
  void*                  irq_ident;        /* needed for irq registration */

#if PERIODIC_TIMER_SUPPORT
  MIO_CTR_CH_TIMER*      timer_func;       /* periodic timer func */
  void*                  timer_param;      /* periodic timer param */
#endif

} CTR_CHANNEL;



/******************************************** 
** Counter object
*********************************************/
typedef enum
{
  ctr_initialized,
  ctr_uninitialized,
} COUNTER_STATE;


typedef struct _MIO_COUNTER
{
  COUNTER_STATE   state;
  CTR_CHANNEL     ctr_ch[NR_OF_CTR];
} MIO_COUNTER;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_counter_init(MIO_COUNTER* p_ctr, 
                                 u8*   mem_base_addr,
                                 void* irq_ident);

  extern void   mio_counter_cleanup(MIO_COUNTER* p_ctr);

  extern int    mio_counter_open(struct inode* p_inode, 
                                 struct file* p_file, 
                                 MIO_COUNTER* p_ctr);

  extern int    mio_counter_ioctl(struct file* p_file, 
                                  unsigned int cmd, 
                                  unsigned long param, 
                                  MIO_COUNTER* p_ctr);
#else /* __STDC__ */

  extern STATUS mio_counter_init();
  extern void   mio_counter_cleanup();
  extern int    mio_counter_open();
  extern int    mio_counter_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCcounterh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/