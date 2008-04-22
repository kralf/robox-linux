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
** Title:   ICP-MULTI digital In Object header
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


#ifndef  INCdigitalInh
#define  INCdigitalInh

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

/******************************************** 
** Digital In object
*********************************************/
typedef enum
{
  di_initialized,
  di_uninitialized,
  di_opened,
  di_wait_for_change
} DI_STATE;

typedef enum
{
  di_normal_read = 0,
  di_blocking_read = 1
} DI_MODE;


typedef struct _MIO_DIGITAL_IN
{
  DI_STATE               state;
  DI_MODE                mode;
  u8*                    di_reg;
  u8*                    interrupt_enable_reg;
  u8*                    interrupt_status_reg;
  u16                    data;   /* variable to hold the data read by irq */
  struct file_operations fops;   /* channel's file ops */
  struct wait_queue*     wq;     /* wait queue for blocking I/O */
  void*                  irq_ident; /* needed for irq registration */
} MIO_DIGITAL_IN;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_digital_in_init(MIO_DIGITAL_IN* p_di, 
                                    u8*   mem_base_addr,
                                    void* irq_ident);

  extern void   mio_digital_in_cleanup(MIO_DIGITAL_IN* p_di);

  extern int    mio_digital_in_open(struct inode* p_inode, 
                                   struct file* p_file, 
                                   MIO_DIGITAL_IN* p_di);

  extern int    mio_digital_in_ioctl(struct file* p_file, 
                                    unsigned int cmd, 
                                    unsigned long param, 
                                    MIO_DIGITAL_IN* p_di);
#else /* __STDC__ */

  extern STATUS mio_digital_in_init();
  extern void   mio_digital_in_cleanup();
  extern int    mio_digital_in_open();
  extern int    mio_digital_in_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCdigitalInh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/