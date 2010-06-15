/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.25.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux Driver
** Title:   Analog Input Channel Object
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
000,25oct00,bt   created
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

#include <linux/types.h>  /* size_t */
#include <asm/io.h>       /* writew(), readw()...*/
#include <asm/uaccess.h>  /* put_user() */
#include <linux/delay.h>
#include <asm/string.h>

#include <icp_drv_gen.h>
/* 
** local module headers
*/

#include "../h/ai_ch.h"
#include "../h/ai_adc.h"
#include "../h/icp_mio.msg"
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define AI_ADC_RANGE      0x00001000L    /* 12 Bit resolution */

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static void    ai_ch_write_data(AI_CHANNEL* p_ai_ch, u16 data);
static int     ai_ch_close(struct inode* p_inode, struct file* p_file);
static ssize_t ai_ch_read_perm(struct file* p_file, char* buff, size_t count,
                               loff_t* offset);
static ssize_t ai_ch_read_ondem(struct file* p_file, char* buff, size_t count,
                               loff_t* offset);
static int     ai_ch_ioctl(struct inode *inode, struct file *filp,  
                           unsigned int cmd, unsigned long arg);
static int     ai_ch_dummy_open(struct inode* p_inode, struct file* p_file);
static void    calc_data(AI_CHANNEL* p_ai_ch, long* data);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* calc_data - calculate the data
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static void calc_data(AI_CHANNEL* p_ai_ch, long* data)
{
  long l_data = *data;
  long l_temp;

#ifdef DEBUG_AI
  printk("<1> analog in calc data: l_data: %li, cb: %li cm: %li tgt_volt: %li \n", l_data, 
    p_ai_ch->adc_adj.cb_i, p_ai_ch->adc_adj.cm_i, p_ai_ch->adc_adj.tgt_voltage_i);
#endif

  l_temp = ((l_data << (AI_C_PARAM-4)) - p_ai_ch->adc_adj.cb_i) / 
            p_ai_ch->adc_adj.cm_i;
  
  l_data = (p_ai_ch->adc_adj.tgt_voltage_i*l_temp) / AI_ADC_RANGE;


  /*
  ** Currency measurment
  */
  if (p_ai_ch->adc_adj.b_4mA_20mA)
  {
    /* 0x10000         X
    ** -------  = ----------           (0xCCCD = 4/5 * 0x10000)
    ** 0xCCCD     lInputData - 0x3333  (0x3333 = 1/5 * 0x10000)
    */
    if ( l_data < 0x3333 )
    {
      if ( l_data < 0x28F6 ) /* < 0.8V is an open ciruit error */
      {
        l_data=0;
      }
    }
    else
    {
      l_data -= 0x3333;
      l_data =((unsigned long)(l_data * 0x10000)) / 0xCCCD; /* adjust data value */
    }
  }
  *data = l_data;
}
/******************************************************************************
*
* ai_ch_write_data - data write routine of an analog in channel
*
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static void ai_ch_write_data(AI_CHANNEL* p_ai_ch, u16 data)
{
  switch(p_ai_ch->mode)
  {
    case ai_ch_permanent_read:
    {
#ifdef DEBUG_AI
  printk("<1> analog in ch FATAL ERROR");
#endif
      /*
      ** write the data in the ring buffer if permanent read was specified
      */
      break;
    }
    case ai_ch_ondemand_read:
    {
#ifdef DEBUG_AI
  printk("<1> analog in ch %i write data, data: %hx\n", p_ai_ch->nr, data);
#endif
      /*
      ** write the data in the immediate buffer if on demand read was specified
      */
      p_ai_ch->immediate_data = data;
      /*
      ** set state back to opened, prevent coming irqs
      */
      p_ai_ch->state = ai_ch_opened;
      break;
    }
    default:
    {
      /*
      ** FATAL ERROR
      */
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_FATAL_ERROR);
      return;
    }
  }
  /*
  ** Wakeup the read process
  */
  wake_up_interruptible(&(p_ai_ch->wq));

  return;
}
/******************************************************************************
*
* ai_ch_close - close routine for an Analog Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ai_ch_close(struct inode* p_inode, struct file* p_file)
{
  AI_CHANNEL* p_ai_ch = (AI_CHANNEL*)p_file->private_data;

  
  p_ai_ch->adc_reg = 0;
  SET_ADC_CH(p_ai_ch->adc_reg, p_ai_ch->nr);
  SET_ADC_BUSY(p_ai_ch->adc_reg, 1);
  ai_adc_set_adc_conv_adj_param(p_ai_ch->adc_parent, &(p_ai_ch->adc_adj), 
                                p_ai_ch->adc_reg, ADC_mV_UNIT);
 
  p_ai_ch->state = ai_ch_initialized;
  /*
  ** decrement usage count
  */
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* ai_ch_read_perm - read routine for an Analog Input channel
*                   permananent read mode
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t ai_ch_read_perm(struct file* p_file, char* buff, size_t count,
                               loff_t* offset)
{
/*  AI_CHANNEL* p_ai = (AI_CHANNEL*)p_file->private_data; */

  /*
  ** try to read the Ring Buffer
  */
  return -EPERM;

}
/******************************************************************************
*
* ai_ch_read_ondem - read routine for an Analog Input channel
*                    on demand read mode
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t ai_ch_read_ondem(struct file* p_file, char* buff, size_t count,
                               loff_t* offset)
{
  AI_CHANNEL*   p_ai_ch = (AI_CHANNEL*)p_file->private_data;
  unsigned long flags;
  int           ret_val;
  long          l_data;


  /*
  ** check params
  */
  if (count <= 0)
  {
    return -EINVAL;
  }
  /*
  ** 1. check if a read is pending -> yes return
  */
  /*  synchronize with the irq */
  save_flags(flags);
  cli();
  if (p_ai_ch->state == ai_ch_to_read)
  {
    restore_flags(flags);
    return -EBUSY;
  }
  restore_flags(flags);

  /*
  ** 2. set state to ao_ch_to_read
  ** 3. get the conversion adjusment params
  ** 4. start the conversion
  ** 6. go to sleep
  ** 6. put the data to the user
  ** 7. change state to ai_ch_opened
  */
  /* synchronize with the irq */
  save_flags(flags);
  cli();
  p_ai_ch->state = ai_ch_to_read;
  /*
  ** start A/D conversion
  */
  p_ai_ch->irq_start_func(p_ai_ch->irq_start_par, p_ai_ch->nr);
  /* interruptible_sleep_on enables irq */
  interruptible_sleep_on(&(p_ai_ch->wq));
  restore_flags(flags);
  /*
  ** put the data to the user function
  */
  /* 
  ** calculate the data
  */
  l_data = (long)p_ai_ch->immediate_data;
  calc_data(p_ai_ch, &l_data);
  switch (count)
  {
    case 1:
    {
      if (put_user((u8)l_data, (u8*)buff) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = sizeof(u8);
      }
      break;
    }
    case 2:
    {
      if (put_user((u16)l_data, (u16*)buff) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = sizeof(u16);
      }
      break;
    }
    case 3:
    {
      if (copy_to_user(buff, &l_data, 3) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = 3;
      }
      break;
    }
    default:
    {
      if (put_user(l_data, (long*)buff) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = sizeof(long);
      }
      break;
    }
  }
  p_ai_ch->state = ai_ch_opened;
  return ret_val;
}
/******************************************************************************
*
* ai_ch_ioctl - read routine for an Analog Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ai_ch_ioctl(struct inode *inode, struct file *filp,  
                       unsigned int cmd, unsigned long arg)
{
  AI_CHANNEL*   p_ai_ch = (AI_CHANNEL*)(filp->private_data);
  unsigned long flags;

  switch(cmd)
  {
    case MIO_IOCTL_AI_CH_READ_PERM_MODE:
    {
      if (arg)
      {
        return -EINVAL; /* not supported at the moment */
        /*
        ** set to permanent mode, synchronize with the irq
        */
        save_flags(flags);
        cli();
        p_ai_ch->mode  = ai_ch_permanent_read;
        p_ai_ch->state = ai_ch_to_read;
        restore_flags(flags);
      }
      else
      {
        /*
        ** set to on-demand mode, synchronize with the irq
        */
        save_flags(flags);
        cli();
        p_ai_ch->mode  = ai_ch_ondemand_read;
        p_ai_ch->state = ai_ch_opened;
        restore_flags(flags);
      }
      break;
    }
    case MIO_IOCTL_AI_CH_ADC_INP_RANGE:
    {
      if(arg)
      {
        SET_ADC_INP_RANGE(p_ai_ch->adc_reg, 1);
      }
      else
      {
        SET_ADC_INP_RANGE(p_ai_ch->adc_reg, 0);
      }
      ai_adc_set_adc_conv_adj_param(p_ai_ch->adc_parent, &(p_ai_ch->adc_adj), 
                                p_ai_ch->adc_reg, 0);
      break;
    }
    case MIO_IOCTL_AI_CH_ADC_BIP_INP_RANGE:
    {
      if(arg)
      {
        SET_ADC_BIP_INP_RANGE(p_ai_ch->adc_reg, 1);
      }
      else
      {
        SET_ADC_BIP_INP_RANGE(p_ai_ch->adc_reg, 0);
      }
      ai_adc_set_adc_conv_adj_param(p_ai_ch->adc_parent, &(p_ai_ch->adc_adj), 
                                p_ai_ch->adc_reg, 0);
      break;
    }
    case MIO_IOCTL_AI_CH_ADC_DIFF_INP_MODE:
    {
      if(arg)
      {
        /*
        ** check if it is possible last 8 channels can't be used for it
        */
        if (p_ai_ch->nr >= 8)
        {
          return -EINVAL;
        }
        else
        {
          SET_ADC_CH(p_ai_ch->adc_reg, (p_ai_ch->nr << 1));
          SET_ADC_DIFF_INP_MODE(p_ai_ch->adc_reg, 1);
        }
      }
      else
      {
        SET_ADC_CH(p_ai_ch->adc_reg, p_ai_ch->nr);
        SET_ADC_DIFF_INP_MODE(p_ai_ch->adc_reg, 0);
      }
      break;
    }
    case MIO_IOCTL_AI_CH_ADC_UNITS:
    {
      switch(arg)
      {
        case ADC_100uV_UNIT:
        case ADC_mV_UNIT:
        case ADC_10mV_UNIT:
        case ADC_100mV_UNIT:
        case ADC_V_UNIT:
        {
          ai_adc_set_adc_conv_adj_param(p_ai_ch->adc_parent, &(p_ai_ch->adc_adj), 
                                p_ai_ch->adc_reg, arg);
          break;
        }
        default:
        {
          return -EINVAL; /* Invalid parameter */
        }
      }
      break;
    }

    default:
    {
      return -EINVAL; /* Invalid parameter */
    }
  }
  return 0;
}
/******************************************************************************
*
* ai_ch_dummy_open - dummy open the Counter channel 
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
static int ai_ch_dummy_open(struct inode* p_inode, struct file* p_file)
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
* init_ai_ch - Initialize an Analog input channel object (constructor)
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
STATUS ai_ch_init_ai_ch(AI_CHANNEL* p_ai_ch, int ch_nr, void* irq_start_par,
                        STATUS (*irq_start_func)(void*, int), 
                        void* adc_adj_param)
{
  /*
  ** Ring buffer init is not supported now
  */

  p_ai_ch->mode  = ai_ch_ondemand_read; /* ondemand read if it is opened */
  p_ai_ch->wq    = NULL;                 /* wait queue for blocking I/O */

  p_ai_ch->write_data = ai_ch_write_data; /* data write func, used by irq 
                                           ** service routine */
  p_ai_ch->nr   = ch_nr;
  /*
  ** A/D conversion IRQ start parameters
  */
  p_ai_ch->irq_start_func = irq_start_func;
  p_ai_ch->irq_start_par  = irq_start_par;
  /*
  ** fill out the file_operations structure
  */
  p_ai_ch->fops.llseek              = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.read                = ai_ch_read_perm; /* read */
  p_ai_ch->fops.write               = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.readdir             = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.poll                = NULL; /* NOT USED for input channel */

  p_ai_ch->fops.ioctl               = ai_ch_ioctl; /* ioct() for setting input 
                                                   ** channel params 
                                                   */
  p_ai_ch->fops.mmap                = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.open                = ai_ch_dummy_open;/* dummy open routine */
  p_ai_ch->fops.flush               = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.release             = ai_ch_close; /* close routine */
  p_ai_ch->fops.fsync               = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.fasync              = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.check_media_change  = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.revalidate          = NULL; /* NOT USED for input channel */
  p_ai_ch->fops.lock                = NULL; /* NOT USED for input channel */

  /* set up the adc params, BIP=0,DIFF=0,RANGE=5V */
  p_ai_ch->adc_parent = adc_adj_param;
  p_ai_ch->adc_reg = 0;
  SET_ADC_CH(p_ai_ch->adc_reg, ch_nr);
  SET_ADC_BUSY(p_ai_ch->adc_reg, 1);
  ai_adc_set_adc_conv_adj_param(p_ai_ch->adc_parent, &(p_ai_ch->adc_adj), 
                                p_ai_ch->adc_reg, ADC_mV_UNIT);
  p_ai_ch->state = ai_ch_initialized;

#ifdef DEBUG_AI
  printk("<1> analog in ch %i init OK\n", p_ai_ch->nr);
#endif

  return OK;
}
/******************************************************************************
*
* cleanup_ai_ch - cleanup an Analog input channel object (destructor)
*
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void ai_ch_cleanup_ai_ch(AI_CHANNEL* p_ai_ch)
{
  /*
  ** Ring buffer cleanup, not supported now
  */
  p_ai_ch->state = ai_ch_uninitialized;
  p_ai_ch->mode  = ai_ch_ondemand_read; /* ondemand read if it is opened */

#ifdef DEBUG_AI
  printk("<1> analog in ch %i cleanup OK\n", p_ai_ch->nr);
#endif

}
/******************************************************************************
*
* ai_ch_open - open routine for an Analog Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int ai_ch_open(struct file* p_file, AI_CHANNEL* p_ai)
{
  switch (p_ai->state)
  {
    case ai_ch_uninitialized:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AI_CH_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case ai_ch_opened:
    case ai_ch_to_read:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AI_CH_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case ai_ch_initialized:
    {
      /*
      ** open the file
      */
      p_ai->mode = ai_ch_ondemand_read;
      
        /* set up the adc params, BIP=0,DIFF=0,RANGE=5V */
      p_ai->adc_reg = 0;
      SET_ADC_CH(p_ai->adc_reg, p_ai->nr);
      SET_ADC_BUSY(p_ai->adc_reg, 1);
      ai_adc_set_adc_conv_adj_param(p_ai->adc_parent, &(p_ai->adc_adj), 
                                    p_ai->adc_reg, ADC_mV_UNIT);

      p_file->f_op = &(p_ai->fops);
      p_file->private_data = (void*)p_ai;

      p_ai->state = ai_ch_opened;
      p_ai->fops.read = ai_ch_read_ondem;

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


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/