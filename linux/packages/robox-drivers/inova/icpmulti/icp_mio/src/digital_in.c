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
** Project: ICP-MULTI Linux Driver
** Title:   Digital Input Object
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI */

 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,30sep00,bt   created
*/
 
/*---------------------------------------------------------------------------
** PRAGMA
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#ifndef __KERNEL__
  #define __KERNEL__
#endif

#define __NO_VERSION__ /* don't define kernel_verion in module.h */
#include <linux/module.h> /* MOD_DEC_USE_COUNT */

#include <linux/kernel.h> /* printk()... */
#include <linux/malloc.h> /* kmalloc() */

#include <linux/types.h>  /* size_t */

#include <asm/io.h>       /* writew(), readw()...*/
#include <asm/uaccess.h>  /* put_user() */
#include <asm/string.h>

#include <icp_drv_gen.h>
/* 
** local module headers
*/
#include "../h/interrupt.h"

#include "../h/digital_in.h"
#include "../h/icp_mio.msg"
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/
static char irq_service_rt_name[] = "digital in interrupt service routine";

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static inline void clear_din_irq_status(MIO_DIGITAL_IN* p_di);
static inline void enable_din_irq(MIO_DIGITAL_IN* p_di);
static inline void disable_din_irq(MIO_DIGITAL_IN* p_di);
static void   irq_service_rt(void* param);
static int    din_close(struct inode* p_inode, struct file* p_file);
static ssize_t din_read(struct file* p_file, char* buff, size_t count, 
                        loff_t* offset);
static int    din_ioctl(struct inode *inode, struct file *filp,  
                        unsigned int cmd, unsigned long arg);
static int di_dummy_open(struct inode* p_inode, struct file* p_file);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* clear_din_irq_status - clears the DIN status change Irq status bit
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void clear_din_irq_status(MIO_DIGITAL_IN* p_di)
{
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  writew(DIN_STATUS_CHANGE_IRQ,p_di->interrupt_status_reg);
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* enable_din_irq - enables the DIN status change Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void enable_din_irq(MIO_DIGITAL_IN* p_di)
{
  u16            val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_di->interrupt_enable_reg);

  if ( !(val & DIN_STATUS_CHANGE_IRQ) )
  {
    /*
    ** Irq not enabled, enable it
    */
    val |= DIN_STATUS_CHANGE_IRQ;
    writew(val,p_di->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* disable_din_irq - disables the DIN status change Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void disable_din_irq(MIO_DIGITAL_IN* p_di)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_di->interrupt_enable_reg);

  if ( (val & DIN_STATUS_CHANGE_IRQ) )
  {
    /*
    ** Irq enabled, disable it
    */
    val &= ~DIN_STATUS_CHANGE_IRQ;
    writew(val,p_di->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* irq_service_rt - DIN error Irq service routine
*
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static void irq_service_rt(void* param)
{
  MIO_DIGITAL_IN* p_di  = (MIO_DIGITAL_IN*)param;
  /*
  ** check if the DIN_STATUS_CHANGE_IRQ is enabled
  */
  if ( !(readw(p_di->interrupt_enable_reg) & DIN_STATUS_CHANGE_IRQ) )
  {
    /*
    ** Irq not enabled, return
    */
    return;
  }
  /*
  ** check if it is an DIN_STATUS_CHANGE_IRQ
  */
  if ( !(readw(p_di->interrupt_status_reg) & DIN_STATUS_CHANGE_IRQ) )
  {
    /*
    ** not an DIN_STATUS_CHANGE_IRQ
    */
    return;
  }
  /*
  ** service the IRQ
  ** 1. read data from the input
  ** 2. set state to opened (it is finished with writing)
  ** 3. disable irq
  ** 4. clear IR status
  ** 5  wakeup the wait queue (it is made by the write routine)
  */
  p_di->data = readw(p_di->di_reg);
  /*
  **  disable irq, clear status irq, wake up
  */
  disable_din_irq(p_di);
  clear_din_irq_status(p_di);
  wake_up_interruptible(&(p_di->wq));
  
  return;
}
/******************************************************************************
*
* din_close - close routine for an Digital Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int din_close(struct inode* p_inode, struct file* p_file)
{
  MIO_DIGITAL_IN* p_di = (MIO_DIGITAL_IN*)p_file->private_data;

  /*
  ** deinitialize interrupt
  ** 1. disable din interrupt
  ** 2. clear din interrupt status bit
  */
  disable_din_irq(p_di);
  clear_din_irq_status(p_di);
  p_di->mode  = di_normal_read;
  p_di->state = di_initialized;
  /*
  ** decrement usage count
  */
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* din_read - read routine for the Digital Input channel
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t din_read(struct file* p_file, char* buff, size_t count, 
                        loff_t* offset)
{
  MIO_DIGITAL_IN*  p_di = (MIO_DIGITAL_IN*)p_file->private_data;
  unsigned long    flags;
  ssize_t          ret_val = 0;
  /*
  ** 1. check if a read is pending -> yes return
  ** 3. check mode
  ** 4. normal mode -> read data
  ** 5. blocking modes -> enable irq, go sleep, put the data to th user, 
  **                      set  state to opened
  ** 6. return with success
  */
#ifdef DEBUG_DI
  printk("<1> digital in read count:%i\n", count);
#endif
  
  /* check if a read is pending, synchronize with the irq */
  save_flags(flags);
  cli();
  if (p_di->state == di_wait_for_change)
  {
    restore_flags(flags);
    return -EBUSY;
  }
  restore_flags(flags);
  /* check params */
  if (count <= 0)
  {
    return -EINVAL;
  }
  else
  {
    /*
    ** read data
    */
    switch (p_di->mode)
    {
      case di_normal_read:
      {
        /*
        ** just write the data
        */
        p_di->data = readw(p_di->di_reg);
        break;
      }
      case di_blocking_read:
      {
        /*
        ** set new state clear status irq, enbable irq, go sleep
        ** synchronize with the irq
        */
        save_flags(flags);
        cli();
        p_di->state = di_wait_for_change;
        clear_din_irq_status(p_di);
        enable_din_irq(p_di);
        /* interruptible_sleep_on enables irq */
        interruptible_sleep_on(&(p_di->wq));
        restore_flags(flags);
        break;
      }
      default:
      {
        icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FATAL_ERROR);
        return -EINVAL;
      }
    }
  }
  /*
  ** p_di->data has the actual value
  */

#ifdef DEBUG_DI
  printk("<1> digital in read data:%hi\n", p_di->data);
#endif
  
  if (count == 1)
  {
    if (put_user((u8)p_di->data, (u8*)buff) != 0)
    {
      ret_val = -EFAULT;
    }
    else
    {
      ret_val = sizeof(u8);
    }
  }
  else
  {
    if (put_user(p_di->data, (u16*)buff) != 0)
    {
      ret_val = -EFAULT;
    }
    else
    {
      ret_val = sizeof(u16);
    }
  }
  p_di->state = di_opened;
  return ret_val;
}
/******************************************************************************
*
* din_ioctl - ioctl for an Digital Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int din_ioctl(struct inode *inode, struct file *p_file,  
                       unsigned int cmd, unsigned long arg)
{
  MIO_DIGITAL_IN* p_di = (MIO_DIGITAL_IN*)p_file->private_data;

  switch(cmd)
  {
    case MIO_IOCTL_DI_ENABLE_IRQ:
    {
      if (arg)
      {
        enable_din_irq(p_di);
      }
      else
      {
        disable_din_irq(p_di);
      }
      break;
    }
    case MIO_IOCTL_DI_READ_MODE:
    {
      switch(arg)
      {
        case di_blocking_read:
        case di_normal_read:
        {
          p_di->mode = (DI_MODE)arg;
          break;
        }
        default:
        {
          return -EINVAL;
        }
      }
      break;
    }
    case MIO_IOCTL_DI_DIRECT_READ:
    {
      return (int) (readw(p_di->di_reg));
    }
    default:
    {
      return -EINVAL;
    }
  }
  return 0; /* success */
}
/******************************************************************************
*
* di_dummy_open - dummy open the digital input 
*
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static int di_dummy_open(struct inode* p_inode, struct file* p_file)
{
  /* 
  ** opeartion not permitted
  */
  return -EPERM;
}

/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_digital_in_init - Initialize the Digital Input object of a MIO card 
*                        (constructor)
*
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
STATUS mio_digital_in_init(MIO_DIGITAL_IN* p_di, u8* mem_base_addr,
                           void* irq_ident)
{
  STATUS status = OK;
  /* 
  ** initialize the MIO_DIGITAL_IN stucture
  */
  p_di->state                = di_uninitialized;
  p_di->mode                 = di_normal_read;
  p_di->di_reg               = mem_base_addr + DI_REG_OFFS;
  p_di->interrupt_enable_reg = mem_base_addr + IRQ_ENABLE_REG_OFFS;
  p_di->interrupt_status_reg = mem_base_addr + IRQ_STATUS_REG_OFFS;
  p_di->irq_ident            = irq_ident;

  /*
  ** initialize interrupt
  ** 1. register irq routine
  ** 2. disable din interrupt
  ** 3. clear din interrupt status bit
  */
  if ((status = mio_interrupt_register_routine(irq_ident, 
                                               irq_service_rt, 
                                               (void*)p_di, 
                                               irq_service_rt_name)) != OK)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DI_IRQ_INIT_ERR);
    return status;
  }
  
  disable_din_irq(p_di);
  clear_din_irq_status(p_di);
  
  p_di->wq    = NULL;                 /* wait queue for blocking I/O */
  /*
  ** fill in the file_operations structure
  */
  p_di->fops.llseek              = NULL; /* NOT USED for input channel */
  p_di->fops.read                = din_read; /* read */
  p_di->fops.write               = NULL; /* NOT USED for input channel */
  p_di->fops.readdir             = NULL; /* NOT USED for input channel */
  p_di->fops.poll                = NULL; /* NOT USED for input channel */

  p_di->fops.ioctl               = din_ioctl; /* ioct() for setting 
                                                ** input channel params 
                                                */
  p_di->fops.mmap                = NULL; /* NOT USED for input channel */
  p_di->fops.open                = di_dummy_open; /* dummy open routine */
  p_di->fops.flush               = NULL; /* NOT USED for input channel */
  p_di->fops.release             = din_close; /* close routine */
  p_di->fops.fsync               = NULL; /* NOT USED for input channel */
  p_di->fops.fasync              = NULL; /* NOT USED for input channel */
  p_di->fops.check_media_change  = NULL; /* NOT USED for input channel */
  p_di->fops.revalidate          = NULL; /* NOT USED for input channel */
  p_di->fops.lock                = NULL; /* NOT USED for input channel */

  p_di->state = di_initialized;

#ifdef DEBUG_DI
  printk("<1> digital in init OK\n");
#endif

  return OK;
}
/******************************************************************************
*
* mio_digital_in_cleanup - cleanup the Digital Input object of a MIO card 
*                         (destructor)
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
void mio_digital_in_cleanup(MIO_DIGITAL_IN* p_di)
{

  /*
  ** deinitialize interrupt
  ** 1. disable din interrupt
  ** 2. clear din interrupt status bit
  ** 3. deregister irq routine
  */
  disable_din_irq(p_di);
  clear_din_irq_status(p_di);
  (void)mio_interrupt_deregister_routine(p_di->irq_ident,
                                         irq_service_rt, 
                                         irq_service_rt_name );

  if (p_di->state != di_initialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DI_CLEANUP_ERR);
  }
  p_di->state = di_uninitialized;

#ifdef DEBUG_DI
  printk("<1> digital in deinit OK\n");
#endif

}
/******************************************************************************
*
* mio_digital_in_open - open routine for the Digital Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_digital_in_open(struct inode* p_inode, struct file* p_file, 
                        MIO_DIGITAL_IN* p_di)
{
  int ch_nr = MINOR(p_inode->i_rdev) - DI_MINOR_NR_OFFS;

  if( ch_nr != 0 )
  {
    /*
    ** bad param
    */
/*    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DI_OPEN_ERR);*/
    return ICP_MIO_BAD_MINOR;
  }

  switch (p_di->state)
  {
    case di_uninitialized:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DI_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case di_opened:
    case di_wait_for_change:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DI_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case di_initialized:
    {
      /*
      ** open the file
      */
      disable_din_irq(p_di);
      clear_din_irq_status(p_di);

      p_file->f_op         = &(p_di->fops);
      p_file->private_data = (void*)p_di;
      p_di->mode           = di_normal_read;
      p_di->state          = di_opened;
      MOD_INC_USE_COUNT;
      return 0; /* success */
    }
    default:
    {
      /* FATAL Error */
      return -EPERM; /* operation not permitted */
    }
  }
}
/******************************************************************************
*
* mio_digital_in_ioctl - ioctl routine for the Digital Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_digital_in_ioctl(struct file* p_file, unsigned int cmd, 
                         unsigned long param, MIO_DIGITAL_IN* p_di)
{
  return ICP_MIO_BAD_IOCTL;
}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/