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
** Title:   ICP-MULTI digital out Object header
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


#ifndef  INCdigitalOuth
#define  INCdigitalOuth

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
** Digital Output object
*************************************************/
typedef enum
{
  do_blocking_write_with_update = 0,
  do_blocking_write_no_update = 1,
  do_normal_write = 2
} DO_MODE;

typedef enum
{
  do_initialized,
  do_uninitialized,
  do_opened,
  do_wait_for_error
} DO_STATE;


typedef struct _MIO_DIGITAL_OUT
{
  DO_STATE               state;
  DO_MODE                mode;
  u8*                    do_reg;
  u8*                    interrupt_enable_reg;
  u8*                    interrupt_status_reg;
  u8                     data;  /* variable to hold the data written by irq */
  struct file_operations fops;   /* channel's file ops */
  struct wait_queue*     wq;     /* wait queue for blocking I/O */
  void*                  irq_ident; /* needed for irq registration */
} MIO_DIGITAL_OUT;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_digital_out_init(MIO_DIGITAL_OUT* p_do, 
                                     u8*   mem_base_addr,
                                     void* irq_ident);

  extern void   mio_digital_out_cleanup(MIO_DIGITAL_OUT* p_do);

  extern int    mio_digital_out_open(struct inode* p_inode, 
                                     struct file* p_file, 
                                     MIO_DIGITAL_OUT* p_do);

  extern int    mio_digital_out_ioctl(struct file* p_file, 
                                      unsigned int cmd, 
                                      unsigned long param, 
                                      MIO_DIGITAL_OUT* p_do);
#else /* __STDC__ */

  extern STATUS mio_digital_out_init();
  extern void   mio_digital_out_cleanup();
  extern int    mio_digital_out_open();
  extern int    mio_digital_out_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCdigitalOuth  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/