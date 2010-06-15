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
** Title:   Digital Output Object
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

#include "../h/digital_out.h"
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
static char irq_service_rt_name[] = "digital out interrupt service routine";

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static inline void clear_dout_irq_status(MIO_DIGITAL_OUT* p_do);
static inline void enable_dout_irq(MIO_DIGITAL_OUT* p_do);
static inline void disable_dout_irq(MIO_DIGITAL_OUT* p_do);
static void    irq_service_rt(void* param);
static int     dout_close(struct inode* p_inode, struct file* p_file);
static ssize_t dout_write(struct file* p_file, const char* buff, size_t count,
                          loff_t* offset);
static int     dout_ioctl(struct inode *inode, struct file *filp,  
                          unsigned int cmd, unsigned long arg);
static int     dout_dummy_open(struct inode* p_inode, struct file* p_file);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* clear_dout_irq_status - clears the DOUT error Irq status bit
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void clear_dout_irq_status(MIO_DIGITAL_OUT* p_do)
{
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  writew(DOUT_ERROR_IRQ,p_do->interrupt_status_reg);
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* enable_dout_irq - enables the DOUT error Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void enable_dout_irq(MIO_DIGITAL_OUT* p_do)
{
  u16            val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_do->interrupt_enable_reg);

  if ( !(val & DOUT_ERROR_IRQ) )
  {
    /*
    ** Irq not enabled, enable it
    */
    val |= DOUT_ERROR_IRQ;
    writew(val,p_do->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* disable_dout_irq - enables the DOUT error Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void disable_dout_irq(MIO_DIGITAL_OUT* p_do)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_do->interrupt_enable_reg);

  if ( (val & DOUT_ERROR_IRQ) )
  {
    /*
    ** Irq enabled, disable it
    */
    val &= ~DOUT_ERROR_IRQ;
    writew(val,p_do->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* irq_service_rt - DOUT error Irq service routine
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
  MIO_DIGITAL_OUT* p_do  = (MIO_DIGITAL_OUT*)param;
  /*
  ** check if the DOUT_ERROR_IRQ is enabled
  */
  if ( !(readw(p_do->interrupt_enable_reg) & DOUT_ERROR_IRQ) )
  {
    /*
    ** Irq not enabled, return
    */
    return;
  }
  /*
  ** check if it is an DOUT_ERROR_IRQ
  */
  if ( !(readw(p_do->interrupt_status_reg) & DOUT_ERROR_IRQ) )
  {
    /*
    ** not an DOUT_ERROR_IRQ
    */
    return;
  }
  /*
  ** service the IRQ
  ** 1. write data to the output if necessary
  ** 2. set state to opened (it is finished with writing)
  ** 3. disable irq
  ** 4. clear IR status
  ** 5  wakeup the wait queue (it is made by the write routine)
  */
  switch (p_do->mode)
  {
    case do_blocking_write_with_update:
    {
      writew((u16)p_do->data, p_do->do_reg);
    }
    case do_blocking_write_no_update:
    {
      /*
      ** set new state , disable irq, clear status irq, wake up
      */
      p_do->state = do_opened;
      disable_dout_irq(p_do);
      clear_dout_irq_status(p_do);
      wake_up_interruptible(&(p_do->wq));
      break;
    }
    default:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FATAL_ERROR);
      return;
    }
  }
  return;
}
/******************************************************************************
*
* dout_close - close routine for an Digital Output channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int dout_close(struct inode* p_inode, struct file* p_file)
{
  MIO_DIGITAL_OUT* p_do = (MIO_DIGITAL_OUT*)p_file->private_data;


  /*
  ** deinitialize interrupt
  ** 1. disable dout interrupt
  ** 2. clear dout interrupt status bit
  */
  disable_dout_irq(p_do);
  clear_dout_irq_status(p_do);
  p_do->mode  = do_normal_write;
  p_do->state = do_initialized;
  /*
  ** decrement usage count
  */
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* dout_write - write routine for the Digital Output channel
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t dout_write(struct file* p_file, const char* buff, size_t count,
                          loff_t* offset)
{
  MIO_DIGITAL_OUT* p_do = (MIO_DIGITAL_OUT*)p_file->private_data;
  unsigned long    flags;
  u8               data;

  /*
  ** 1. check if a write is pending -> yes return
  ** 3. check mode
  ** 4. normal mode -> write data
  ** 5. blocking modes -> enable irq, go sleep
  ** 6. return with success
  */
  
  /* check if a write is pending, synchronize with the irq */
  save_flags(flags);
  cli();
  if (p_do->state == do_wait_for_error)
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
    if (get_user(data, (u8*)buff) != 0)
    {
      return -EFAULT;
    }
  }
  
  switch (p_do->mode)
  {
    case do_normal_write:
    {
      /*
      ** just write the data
      */
      writew((u16)data, p_do->do_reg);
      break;
    }
    case do_blocking_write_with_update:
    case do_blocking_write_no_update:
    {
      /*
      ** set new state clear status irq, enbable irq, go sleep,
      ** synchronize with the irq
      */
      save_flags(flags);
      cli();
      p_do->state = do_wait_for_error;
      clear_dout_irq_status(p_do);
      enable_dout_irq(p_do);
      /* interruptible_sleep_on enables irq */
      interruptible_sleep_on(&(p_do->wq));
      restore_flags(flags);
      break;
    }
    default:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FATAL_ERROR);
      return -EINVAL;
    }
  }
  return 1;
}
/******************************************************************************
*
* dout_ioctl - ioctl for an Digital Output channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int dout_ioctl(struct inode *inode, struct file *p_file,  
                       unsigned int cmd, unsigned long arg)
{
  MIO_DIGITAL_OUT* p_do = (MIO_DIGITAL_OUT*)p_file->private_data;

  switch(cmd)
  {
    case MIO_IOCTL_DO_ENABLE_IRQ:
    {
      if (arg)
      {
        enable_dout_irq(p_do);
      }
      else
      {
        disable_dout_irq(p_do);
      }
      break;
    }
    case MIO_IOCTL_DO_WRITE_MODE:
    {
      switch(arg)
      {
        case do_blocking_write_with_update:
        case do_blocking_write_no_update:
        case do_normal_write:
        {
          p_do->mode = (DO_MODE)arg;
          break;
        }
        default:
        {
          return -EINVAL;
        }
      }
      break;
    }
    case MIO_IOCTL_DO_ERROR_OUT_VAL:
    {
      p_do->data = (u8)arg;
      break;
    }
    case MIO_IOCTL_DO_DIRECT_WRITE:
    {
      writew((u16)arg, p_do->do_reg);
      break;
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
* do_dummy_open - dummy open the digital output 
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
static int dout_dummy_open(struct inode* p_inode, struct file* p_file)
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
* mio_digital_out_init - Initialize the Digital Output object of a MIO card 
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
STATUS mio_digital_out_init(MIO_DIGITAL_OUT* p_do, u8* mem_base_addr,
                            void* irq_ident)
{
  STATUS status = OK;
  /* 
  ** initialize the MIO_DIGITAL_OUT stucture
  */
  p_do->state                = do_uninitialized;
  p_do->mode                 = do_normal_write;
  p_do->do_reg               = mem_base_addr + DO_REG_OFFS;
  p_do->interrupt_enable_reg = mem_base_addr + IRQ_ENABLE_REG_OFFS;
  p_do->interrupt_status_reg = mem_base_addr + IRQ_STATUS_REG_OFFS;
  p_do->irq_ident            = irq_ident;

  /*
  ** initialize interrupt
  ** 1. register irq routine
  ** 2. disable dout interrupt
  ** 3. clear dout interrupt status bit
  */
  if ( (status = mio_interrupt_register_routine(irq_ident, 
                                                irq_service_rt, 
                                                (void*)p_do, 
                                                irq_service_rt_name )) != OK)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DO_IRQ_INIT_ERR);
    return status;
  }
  
  disable_dout_irq(p_do);
  clear_dout_irq_status(p_do);
  
  p_do->wq    = NULL;                 /* wait queue for blocking I/O */
  /*
  ** fill out the file_operations structure
  */
  p_do->fops.llseek              = NULL; /* NOT USED for output channel */
  p_do->fops.read                = NULL; /* NOT USED for output channel */
  p_do->fops.write               = dout_write; /* write */
  p_do->fops.readdir             = NULL; /* NOT USED for output channel */
  p_do->fops.poll                = NULL; /* NOT USED for output channel */

  p_do->fops.ioctl               = dout_ioctl; /* ioct() for setting 
                                                ** output channel params 
                                                */
  p_do->fops.mmap                = NULL; /* NOT USED for output channel */
  p_do->fops.open                = dout_dummy_open; /* dummy open routine */
  p_do->fops.flush               = NULL; /* NOT USED for output channel */
  p_do->fops.release             = dout_close; /* close routine */
  p_do->fops.fsync               = NULL; /* NOT USED for output channel */
  p_do->fops.fasync              = NULL; /* NOT USED for output channel */
  p_do->fops.check_media_change  = NULL; /* NOT USED for output channel */
  p_do->fops.revalidate          = NULL; /* NOT USED for output channel */
  p_do->fops.lock                = NULL; /* NOT USED for output channel */

  p_do->state = do_initialized;

#ifdef DEBUG_DO
  printk("<1> digital out init OK\n");
#endif

  return OK;
}
/******************************************************************************
*
* mio_digital_out_cleanup - cleanup the Digital Output object of a MIO card (destructor)
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
void mio_digital_out_cleanup(MIO_DIGITAL_OUT* p_do)
{
  /*
  ** deinitialize interrupt
  ** 1. disable dout interrupt
  ** 2. clear dout interrupt status bit
  ** 3. deregister irq routine
  */
  disable_dout_irq(p_do);
  clear_dout_irq_status(p_do);
  (void)mio_interrupt_deregister_routine(p_do->irq_ident,
                                         irq_service_rt, 
                                         irq_service_rt_name );

  if (p_do->state != do_initialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DO_CLEANUP_ERR);
  }
  p_do->state = do_uninitialized;

#ifdef DEBUG_DO
  printk("<1> digital out deinit OK\n");
#endif

}
/******************************************************************************
*
* mio_digital_out_open - open routine for the Digital Output object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_digital_out_open(struct inode* p_inode, struct file* p_file, 
                         MIO_DIGITAL_OUT* p_do)
{
  int ch_nr = MINOR(p_inode->i_rdev) - DO_MINOR_NR_OFFS;

  if( ch_nr != 0 )
  {
    /*
    ** bad param
    */
/*    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DO_OPEN_ERR);*/
    return ICP_MIO_BAD_MINOR;
  }

  switch (p_do->state)
  {
    case do_uninitialized:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DO_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case do_opened:
    case do_wait_for_error:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_DO_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case do_initialized:
    {
      /*
      ** open the file
      */
      disable_dout_irq(p_do);
      clear_dout_irq_status(p_do);

      p_file->f_op = &(p_do->fops);
      p_file->private_data = (void*)p_do;
      p_do->mode           = do_normal_write;
      p_do->state = do_opened;
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
* mio_digital_out_ioctl - ioctl routine for the Digital Output object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_digital_out_ioctl(struct file* p_file, unsigned int cmd, 
                          unsigned long param, MIO_DIGITAL_OUT* p_do)
{
  return ICP_MIO_BAD_IOCTL;
}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/