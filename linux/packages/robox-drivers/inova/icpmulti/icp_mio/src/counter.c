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
** Title:   Counter Object
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI card */

 
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

#include "../h/counter.h"
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
static char irq_service_rt_name[4][80] = { "counter0 interrupt service routine",
                                           "counter1 interrupt service routine",
                                           "counter2 interrupt service routine",
                                           "counter3 interrupt service routine"
                                          };

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static inline void clear_ctr_ch_irq_status(CTR_CHANNEL* p_ctr_ch);
static inline void enable_ctr_ch_irq(CTR_CHANNEL* p_ctr_ch);
static inline void disable_ctr_ch_irq(CTR_CHANNEL* p_ctr_ch);
static void    irq_service_rt(void* param);
static int     ctr_ch_close(struct inode* p_inode, struct file* p_file);
static ssize_t ctr_ch_read(struct file* p_file, char* buff, size_t count,
                           loff_t* offset);
static ssize_t ctr_ch_write(struct file* p_file, const char* buff, 
                            size_t count, loff_t* offset);
static int     ctr_ch_ioctl(struct inode *inode, struct file *filp,  
                            unsigned int cmd, unsigned long arg);
static STATUS  init_ctr_ch(CTR_CHANNEL* p_ctr_ch, int ch_nr, 
                           u8* mem_base_addr, void* irq_ident);
static void    ctr_ch_cleanup(CTR_CHANNEL* p_ctr_ch);
static int     ctr_ch_open(struct file* p_file, CTR_CHANNEL* p_ctr_ch);
static int     ctr_ch_dummy_open(struct inode* p_inode, struct file* p_file);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* clear_ctr_ch_irq_status - clears the CTR overflow Irq status bit
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void clear_ctr_ch_irq_status(CTR_CHANNEL* p_ctr_ch)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = (u16)CTR_OVERFLOW_IRQ(p_ctr_ch->nr);
  writew(val,p_ctr_ch->interrupt_status_reg);
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* enable_ctr_ch_irq - enables the CTR overflow Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void enable_ctr_ch_irq(CTR_CHANNEL* p_ctr_ch)
{
  u16            val;
  u16            irq_bit = (u16)CTR_OVERFLOW_IRQ(p_ctr_ch->nr);
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_ctr_ch->interrupt_enable_reg);

  if ( !(val & irq_bit) )
  {
    /*
    ** Irq not enabled, enable it
    */
    val |= irq_bit;
    writew(val,p_ctr_ch->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* disable_ctr_ch_irq - disables the CTR overflow Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void disable_ctr_ch_irq(CTR_CHANNEL* p_ctr_ch)
{
  u16            val;
  u16            irq_bit = (u16)CTR_OVERFLOW_IRQ(p_ctr_ch->nr);
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_ctr_ch->interrupt_enable_reg);

  if ( (val & irq_bit) )
  {
    /*
    ** Irq enabled, disable it
    */
    val &= ~irq_bit;
    writew(val,p_ctr_ch->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* irq_service_rt - CTR error Irq service routine
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
  CTR_CHANNEL* p_ctr_ch = (CTR_CHANNEL*)param;
  u16          irq_bit  = (u16)CTR_OVERFLOW_IRQ(p_ctr_ch->nr);
  /*
  ** check if the CTR_OVERFLOW_IRQ is enabled
  */
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i irq enter\n", p_ctr_ch->nr);
#endif
  if ( !(readw(p_ctr_ch->interrupt_enable_reg) & irq_bit) )
  {
    /*
    ** Irq not enabled, return
    */
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i irq not enabled\n", p_ctr_ch->nr);
#endif
    return;
  }
  /*
  ** check if it is an CTR_OVERFLOW_IRQ
  */
  if ( !(readw(p_ctr_ch->interrupt_status_reg) & irq_bit) )
  {
    /*
    ** not an CTR_OVERFLOW_IRQ
    */
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i irq not an overflow irq\n", p_ctr_ch->nr);
#endif
    return;
  }
  /*
  ** service the IRQ
  ** 1. branch according to mode
  **  a1. wake_up_mode
  **   a2. disable IRQ
  **   a3. clear IR status
  **   a4. set new state to ctr_ch_opened
  **   a5. wakeup the wait queue 
  **  b1. auto_load_mode
  **   b2. write preset value
  **   b3. clear IR status
  **   b4. wakeup the wait queue 
  */
  
  if (p_ctr_ch->mode == ctr_ch_wakeup)
  {
    disable_ctr_ch_irq(p_ctr_ch);
    clear_ctr_ch_irq_status(p_ctr_ch);
    p_ctr_ch->state = ctr_ch_opened;
    wake_up_interruptible(&(p_ctr_ch->wq));
  }
  else
  {
#if PERIODIC_TIMER_SUPPORT
    /*
    ** at the moment do nothing, later periodic timer
    **
    */
    writew(p_ctr_ch->preset_data, p_ctr_ch->ctr_reg);
    clear_ctr_ch_irq_status(p_ctr_ch);
    wake_up_interruptible(&(p_ctr_ch->wq));
    
#endif
  }
  return;
}
/******************************************************************************
*
* ctr_ch_close - close routine for an Counter channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ctr_ch_close(struct inode* p_inode, struct file* p_file)
{
  CTR_CHANNEL* p_ctr_ch = (CTR_CHANNEL*)p_file->private_data;

  /*
  ** deinitialize interrupt
  ** 1. disable ctr interrupt
  ** 2. clear ctr interrupt status bit
  */
  disable_ctr_ch_irq(p_ctr_ch);
  clear_ctr_ch_irq_status(p_ctr_ch);
  p_ctr_ch->state = ctr_ch_initialized;
  /*
  ** decrement usage count
  */
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* ctr_ch_read - read routine for the Counter channel
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t ctr_ch_read(struct file* p_file, char* buff, size_t count,
                           loff_t* offset)
{
  CTR_CHANNEL*     p_ctr_ch = (CTR_CHANNEL*)p_file->private_data;
  u16              data;

#ifdef DEBUG_CTR
  printk("<1> ctr ch %i count:%i read\n", p_ctr_ch->nr, count);
#endif
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
    data = readw(p_ctr_ch->ctr_reg);
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i read data %hu addr %lx\n", p_ctr_ch->nr, data, (unsigned long)p_ctr_ch->ctr_reg);
#endif
    if (count == 1)
    {
      if (put_user((u8)data, (u8*)buff) != 0)
      {
        return -EFAULT;
      }
      else
      {
        return sizeof(u8);
      }
    }
    else
    {
      if (put_user(data, (u16*)buff) != 0)
      {
        return -EFAULT;
      }
      else
      {
        return sizeof(u16);
      }
    }
  }
}
/******************************************************************************
*
* ctr_ch_write - write routine for a counter channel
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t ctr_ch_write(struct file* p_file, const char* buff, 
                            size_t count, loff_t* offset)
{
  CTR_CHANNEL*     p_ctr_ch = (CTR_CHANNEL*)p_file->private_data;
  unsigned long    flags;
  u16              data = 0;
  int              ret_val;

  /*
  ** 1. check if a write is pending -> yes return
  ** 2. fill out the data param
  ** 3  branch according to mode
  **   a1. normal or auto_load mode just write the data
  **
  **   b1. write the data
  **   b2. set state to ctr_ch_wait_for_overflow
  **   b3. clear irq status
  **   b4. enable irq
  **   b5. go to sleep
  ** 4. return with success
  */
  
  /* check if a write is pending, synchronize with the irq */
  save_flags(flags);
  cli();
  if (p_ctr_ch->state == ctr_ch_wait_for_overflow)
  {
    restore_flags(flags);
    return -EBUSY;
  }
  restore_flags(flags);
  /* check params */

#ifdef DEBUG_CTR
  printk("<1> ctr ch %i count:%i write\n", p_ctr_ch->nr, count);
#endif

  if (count <= 0)
  {
    return -EINVAL;
  }
  else if (count == 1)
  {
    if (get_user((u8)data, (u8*)buff) != 0)
    {
      return -EFAULT;
    }
    ret_val = sizeof(u8);
  }
  else
  {
    if (get_user(data, (u16*)buff) != 0)
    {
      return -EFAULT;
    }
    ret_val = sizeof(u16);
  }
  
  /* 
  ** check mode
  */
  switch (p_ctr_ch->mode)
  {
    case ctr_ch_normal_operation:
#if PERIODIC_TIMER_SUPPORT
    case ctr_ch_auto_load:
#endif
    {
      /*
      ** just write the data
      */
      writew(data, p_ctr_ch->ctr_reg);
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i write data %hu addr %lx\n", p_ctr_ch->nr, data, (unsigned long)p_ctr_ch->ctr_reg);
#endif
      break;
    }
    case ctr_ch_wakeup:
    {
      /*
      ** write the data
      */
      writew(data, p_ctr_ch->ctr_reg);
      /*
      ** set new state, synchronize with the irq
      */
      cli();
      p_ctr_ch->state = ctr_ch_wait_for_overflow;
      clear_ctr_ch_irq_status(p_ctr_ch);
      enable_ctr_ch_irq(p_ctr_ch);
      /* interruptible_sleep_on enables irq */
      interruptible_sleep_on(&(p_ctr_ch->wq));
      restore_flags(flags);
      break;
    }
    default:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FATAL_ERROR);
      return -EINVAL;
    }
  }
  return ret_val;
}
/******************************************************************************
*
* ctr_ch_ioctl - ioctl for an Counter channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ctr_ch_ioctl(struct inode *inode, struct file *p_file,  
                        unsigned int cmd, unsigned long arg)
{
  CTR_CHANNEL* p_ctr_ch = (CTR_CHANNEL*)p_file->private_data;

  switch(cmd)
  {
    case MIO_IOCTL_CTR_CH_ENABLE_IRQ:
    {
      if (arg)
      {
        enable_ctr_ch_irq(p_ctr_ch);
      }
      else
      {
        disable_ctr_ch_irq(p_ctr_ch);
      }
      break;
    }
    case MIO_IOCTL_CTR_CH_MODE:
    {
      switch(arg)
      {
        case ctr_ch_normal_operation:
        case ctr_ch_wakeup:
        {
          p_ctr_ch->mode = (CTR_CH_MODE)arg;
          break;
        }
        default:
        {
          return -EINVAL;
        }
      }
      break;
    }
#if PERIODIC_TIMER_SUPPORT
    case MIO_IOCTL_CTR_CH_TIMER_PARAM:
    {
      p_ctr_ch->timer_param = (void*)arg;
    }
    case MIO_IOCTL_CTR_CH_TIMER_FUNC:
    {
      p_ctr_ch->timer_func = (MI_CTR_CH_TIMER*)arg;
    }
#endif
    case MIO_IOCTL_CTR_CH_PRESET_VALUE:
    {
      p_ctr_ch->preset_data = (u16)arg;
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
* init_ctr_ch - Initialize the Counter channel object of a MIO card 
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
static STATUS init_ctr_ch(CTR_CHANNEL* p_ctr_ch, int ch_nr, 
                          u8* mem_base_addr, void* irq_ident)
{
  STATUS status = OK;
  
  if ( (ch_nr < 0) || (ch_nr >= NR_OF_CTR) )
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_CH_INIT_ERR1);
    return -EINVAL;
  }

  /* 
  ** initialize the CTR_CHANNEL stucture
  */
  p_ctr_ch->nr                   = ch_nr;
  p_ctr_ch->state                = ctr_ch_uninitialized;
  p_ctr_ch->mode                 = ctr_ch_normal_operation;
  p_ctr_ch->ctr_reg              = mem_base_addr + CTR_CH_REG_OFFS(ch_nr);
  p_ctr_ch->interrupt_enable_reg = mem_base_addr + IRQ_ENABLE_REG_OFFS;
  p_ctr_ch->interrupt_status_reg = mem_base_addr + IRQ_STATUS_REG_OFFS;

  p_ctr_ch->wq    = NULL;                 /* wait queue for blocking I/O */
  /*
  ** fill in the file_operations structure
  */
  p_ctr_ch->fops.llseek              = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.read                = ctr_ch_read;  /* read */
  p_ctr_ch->fops.write               = ctr_ch_write; /* write */
  p_ctr_ch->fops.readdir             = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.poll                = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.ioctl               = ctr_ch_ioctl; /* ioct() for setting 
                                                     ** counter channel params 
                                                     */
  p_ctr_ch->fops.mmap                = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.open                = ctr_ch_dummy_open; /*  open routine */
  p_ctr_ch->fops.flush               = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.release             = ctr_ch_close; /* close routine */
  p_ctr_ch->fops.fsync               = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.fasync              = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.check_media_change  = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.revalidate          = NULL; /* NOT USED for input channel */
  p_ctr_ch->fops.lock                = NULL; /* NOT USED for input channel */

  p_ctr_ch->irq_ident                = irq_ident;

  /*
  ** initialize interrupt
  ** 1. register irq routine
  ** 2. disable ctr interrupt
  ** 3. clear ctr interrupt status bit
  */
  if ( (status = mio_interrupt_register_routine(irq_ident,
                                                irq_service_rt, 
                                                (void*)p_ctr_ch, 
                                                irq_service_rt_name[ch_nr])) != OK)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_CH_IRQ_INIT_ERR);
    return status;
  }
  
  disable_ctr_ch_irq(p_ctr_ch);
  clear_ctr_ch_irq_status(p_ctr_ch);

  p_ctr_ch->state = ctr_ch_initialized;

#ifdef DEBUG_CTR
  printk("<1> ctr ch %i init OK\n", ch_nr);
#endif

  return OK;
}
/******************************************************************************
*
* ctr_ch_cleanup - clean up the Counter channel object of a MIO card 
*                        (destructor)
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
static void ctr_ch_cleanup(CTR_CHANNEL* p_ctr_ch)
{
  /*
  ** deinitialize interrupt
  ** 1. disable ctr interrupt
  ** 2. clear ctr interrupt status bit
  ** 3. deregister irq routine
  */
  disable_ctr_ch_irq(p_ctr_ch);
  clear_ctr_ch_irq_status(p_ctr_ch);
  (void)mio_interrupt_deregister_routine(p_ctr_ch->irq_ident,
                                         irq_service_rt, 
                                         irq_service_rt_name[p_ctr_ch->nr]);

  if (p_ctr_ch->state != ctr_ch_initialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_CH_CLEANUP_ERR);
  }
  p_ctr_ch->state = ctr_ch_uninitialized;

#ifdef DEBUG_CTR
  printk("<1> ctr ch %i deinit OK\n", p_ctr_ch->nr);
#endif

}
/******************************************************************************
*
* ctr_ch_open - open the Counter channel 
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
static int ctr_ch_open(struct file* p_file, CTR_CHANNEL* p_ctr_ch)
{
#ifdef DEBUG_CTR
  printk("<1> ctr ch %i open\n", p_ctr_ch->nr);
#endif

  switch (p_ctr_ch->state)
  {
    case ctr_ch_uninitialized:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_CH_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case ctr_ch_opened:
    case ctr_ch_wait_for_overflow:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_CH_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case ctr_ch_initialized:
    {
      /*
      ** open the file
      */
      disable_ctr_ch_irq(p_ctr_ch);
      clear_ctr_ch_irq_status(p_ctr_ch);

      p_ctr_ch->mode  = ctr_ch_normal_operation;
      p_file->f_op    = &(p_ctr_ch->fops);
      p_file->private_data = (void*)p_ctr_ch;
      p_ctr_ch->state = ctr_ch_opened;
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
* ctr_ch_dummy_open - dummy open the Counter channel 
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
static int ctr_ch_dummy_open(struct inode* p_inode, struct file* p_file)
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
* mio_counter_init - Initialize the Counter object of a MIO card 
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
STATUS mio_counter_init(MIO_COUNTER* p_ctr, u8* mem_base_addr,
                        void* irq_ident)
{
  int    i;
  STATUS status = OK;
  /* 
  ** initialize the MIO_COUNTER stucture
  */
  p_ctr->state               = ctr_uninitialized;

  /*
  ** init the counter channels
  */
  for (i = 0; i < NR_OF_CTR; i++)
  {
    I_CHK_STATUSRS( init_ctr_ch(&(p_ctr->ctr_ch[i]), i, mem_base_addr, 
                                irq_ident) );
  }

  p_ctr->state = ctr_initialized;

#ifdef DEBUG_CTR
  printk("<1> gen counter init OK\n");
#endif

  return status;
}

/******************************************************************************
*
* mio_counter_cleanup - cleanup the Counter object of a MIO card 
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
void mio_counter_cleanup(MIO_COUNTER* p_ctr)
{
  int    i;

  /*
  ** clean up the counter channels
  */
  for (i = 0; i < NR_OF_CTR; i++)
  {
    ctr_ch_cleanup(&(p_ctr->ctr_ch[i]));
  }

  p_ctr->state = ctr_uninitialized;

#ifdef DEBUG_CTR
  printk("<1> gen counter deinit OK\n");
#endif

}

/******************************************************************************
*
* mio_counter_open - open routine for the Counter object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_counter_open(struct inode* p_inode, struct file* p_file, 
                     MIO_COUNTER* p_ctr)
{
  int ch_nr = MINOR(p_inode->i_rdev) - CTR_MINOR_NR_OFFS;

  if ( (ch_nr < 0) || (ch_nr >= NR_OF_CTR) )
  {
    /*
    ** bad param
    */
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_CTR_OPEN_ERR);
    return ICP_MIO_BAD_MINOR;
  }

  return (ctr_ch_open(p_file, &(p_ctr->ctr_ch[ch_nr])));
}

/******************************************************************************
*
* mio_counter_ioctl - ioctl routine for the Counter object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_counter_ioctl(struct file* p_file, unsigned int cmd, 
                      unsigned long param, MIO_COUNTER* p_ctr)
{
  return ICP_MIO_BAD_IOCTL;
}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/