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
** Title:   Analog Output Channel Object
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

#include "../h/ao_ch.h"
#include "../h/icp_mio.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define DAC_BIP_INP       0x0010
#define AO_DAC_RANGE      0x00001000L    /* 12 Bit DAC resolution */

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static int     ao_ch_close(struct inode* p_inode, struct file* p_file);
static ssize_t ao_ch_write(struct file* p_file, const char* buff, size_t count,
                           loff_t* offset);
static int     ao_ch_ioctl(struct inode* p_inode, struct file* p_file,  
                           unsigned int cmd, unsigned long arg);
static int     ao_ch_dummy_open(struct inode* p_inode, struct file* p_file);
static inline void calc_data(AO_CHANNEL* p_ao_ch, long l_data);
static inline void set_dac_conv_adj_param(DAC_CALIB* p_cal, u16 dac_reg, 
                                          long units);
static inline void calc_data(AO_CHANNEL* p_ao_ch, long l_data);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* set_dac_conv_adj_param - sets the ADC-adjustment parameters
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void set_dac_conv_adj_param(DAC_CALIB* p_cal, u16 dac_reg, 
                                          long units)
{
  u16           cal_index;

  if (units != 0)
  {
    p_cal->units = units;
  }

  cal_index      = (dac_reg >> 4) & 0x03;
  p_cal->cbi      = p_cal->cb[cal_index];
  p_cal->cmi      = p_cal->cm[cal_index];
  p_cal->offsi    = p_cal->offs[cal_index] * p_cal->units;
  p_cal->rangei   = p_cal->range[cal_index] * p_cal->units;

  p_cal->c1       = 0;
  while ( (unsigned long)((p_cal->rangei*AO_DAC_RANGE) << (++p_cal->c1)) 
                                                               < 0x80000000UL);
  p_cal->c1--;
  if (p_cal->c1 > AO_C_PARAM)
  {
    p_cal->c1 = AO_C_PARAM;
  }
  p_cal->c2       = AO_C_PARAM - p_cal->c1;
  p_cal->bipolar = (dac_reg & DAC_BIP_INP);
  return;
}

/******************************************************************************
*
* calc_data- calculate the data for the output
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void calc_data(AO_CHANNEL* p_ao_ch, long l_data)
{
  long l_temp;

  /* check if data is within defined range */
  if ( (!p_ao_ch->dac_calib.bipolar) && (l_data < 0))
  {
    l_data = 0;
  }
  /*
  ** calculate the data x = (c*y-c*b)/c*m
  ** first we need the y value
  **
  ** if unipolar:
  ** y = (Y in mV)*0x1000/RANGE
  **
  ** bipolar:
  ** y = (Y in mV + Offset)*0x1000/RANGE
  ** 
  ** what we need is actually c*y =>
  **
  ** c*y = c*(Y in mV)*0x1000/RANGE or 
  ** c*y = c*(Y in mV + offset)*0x1000/RANGE
  **
  */
#ifdef DEBUG_AO
  if (l_data == 0)
  {
    printk("<1> analog out ch %i cmi %li cbi %li rangei %li offsi: %li c1 %i c2 %i \n",p_ao_ch->nr, p_ao_ch->dac_calib.cmi, p_ao_ch->dac_calib.cbi, p_ao_ch->dac_calib.rangei, p_ao_ch->dac_calib.offsi, p_ao_ch->dac_calib.c1, p_ao_ch->dac_calib.c2);
  }
#endif

  /*
  ** check for range
  */
  if (l_data < (-1*p_ao_ch->dac_calib.offsi))
  {
    l_data = -1*p_ao_ch->dac_calib.offsi;
  }

  if (l_data > (p_ao_ch->dac_calib.rangei - p_ao_ch->dac_calib.offsi))
  {
    l_data = (p_ao_ch->dac_calib.rangei - p_ao_ch->dac_calib.offsi);
  }

  /* 
  ** y = ((Y + offset)*0x1000)/RANGE
  */
  l_temp  = ( ((((l_data + p_ao_ch->dac_calib.offsi)*AO_DAC_RANGE) 
                                                      << p_ao_ch->dac_calib.c1) / 
             p_ao_ch->dac_calib.rangei) << p_ao_ch->dac_calib.c2 ); /* c*y */


  l_temp  -= p_ao_ch->dac_calib.cbi; /* c*y - c*b */
  l_temp /= p_ao_ch->dac_calib.cmi;  /* (c*y - c*b)/c*m */
  
  p_ao_ch->data = (u16)(l_temp);

#ifdef DEBUG_AO
  printk("<1> analog out write on ch%i with 16 bit data:%hi %hx\n", p_ao_ch->nr, p_ao_ch->data, p_ao_ch->data);
#endif

}
/******************************************************************************
*
* ao_ch_close - close routine for an Analog Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ao_ch_close(struct inode* p_inode, struct file* p_file)
{
  AO_CHANNEL* p_ao_ch = (AO_CHANNEL*)p_file->private_data;

  p_ao_ch->dac_reg = 0;
  SET_DAC_CH(p_ao_ch->dac_reg, p_ao_ch->nr);
  SET_DAC_BUSY(p_ao_ch->dac_reg, 1);
  set_dac_conv_adj_param(&(p_ao_ch->dac_calib), p_ao_ch->dac_reg, DAC_mV_UNIT);

  p_ao_ch->state = ao_ch_initialized;
  /*
  ** decrement usage count
  */
  MOD_DEC_USE_COUNT;
  return 0;
}
/******************************************************************************
*
* ao_ch_write - write routine for an Analog Output channel
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static ssize_t ao_ch_write(struct file* p_file, const char* buff, size_t count,
                           loff_t* offset)
{
  AO_CHANNEL*   p_ao_ch = (AO_CHANNEL*)p_file->private_data;
  unsigned long flags;
  ssize_t       ret_val;
  long          l_data;

  /*
  ** 1. check if a write is pending -> yes return
  ** 2. fill out the data param
  ** 3. set state to ao_ch_to_write
  ** 4. start the conversion
  ** 5. go to sleep
  ** 6. return with success
  */
  
  /* check if a write is pending, synchronize with the irq */
  save_flags(flags);
  cli();
  if (p_ao_ch->state == ao_ch_to_write)
  {
    restore_flags(flags);
    return -EBUSY;
  }
  restore_flags(flags);

#ifdef DEBUG_AO
//  printk("<1> analog out write on ch%i data length in bytes:%i\n", p_ao_ch->nr, count);
#endif

  /* check params */
  p_ao_ch->data = 0;
  if (count <= 0)
  {
    return -EINVAL;
  }

  switch (count)
  {
    case 1:
    {
      if (get_user((u8)l_data, (u8*)buff) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = 1;
      }
      break;
    }
    case 2:
    {
      if (get_user((u16)l_data, (u16*)buff) != 0)
      {
        ret_val = -EFAULT;
      }
      else
      {
        ret_val = 2;
      }
      break;
    }
    case 3:
    {
      if (copy_from_user(&l_data, buff, 3) != 0)
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
      if (get_user(l_data, (long*)buff) != 0)
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

#ifdef DEBUG_AO
//  printk("<1> analog out write on ch%i with data:%lx, ret_val:%i\n", p_ao_ch->nr, l_data, ret_val);
#endif

  /*
  ** calculate the data, the function fills the data memeber of the 
  ** AO_CHANNEL struct
  */
  calc_data(p_ao_ch, l_data);
  /* start a conversion synchronize with the irq */
  save_flags(flags);
  cli();
  /* change state from opened to ao_ch_to_write */
  p_ao_ch->state = ao_ch_to_write;
  /*
  ** start D/A conversion
  */
  p_ao_ch->irq_start_func(p_ao_ch->irq_start_par, p_ao_ch->nr);
  /* interruptible_sleep_on enables irq */
  interruptible_sleep_on(&(p_ao_ch->wq));
  restore_flags(flags);
  /*
  ** return to the user function
  */
  return ret_val;
}
/******************************************************************************
*
* ao_ch_ioctl - read routine for an Analog Input channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
static int ao_ch_ioctl(struct inode* p_inode, struct file* p_file,  
                       unsigned int cmd, unsigned long arg)
{
  AO_CHANNEL* p_ao_ch = (AO_CHANNEL*)p_file->private_data;

  switch(cmd)
  {
    case MIO_IOCTL_AO_CH_DAC_INP_RANGE:
    {
      if(arg)
      {
        SET_DAC_INP_RANGE(p_ao_ch->dac_reg , 1);
      }
      else
      {
        SET_DAC_INP_RANGE(p_ao_ch->dac_reg , 0);
      }
      /*
      ** don't change the units param (3d param = 0)
      */
      set_dac_conv_adj_param(&(p_ao_ch->dac_calib), p_ao_ch->dac_reg, 0);
      break;
    }
    case MIO_IOCTL_AO_CH_DAC_BIP_INP_RANGE:
    {
      if(arg)
      {
        SET_DAC_BIP_INP_RANGE(p_ao_ch->dac_reg , 1);
      }
      else
      {
        SET_DAC_BIP_INP_RANGE(p_ao_ch->dac_reg , 0);
      }
      /*
      ** don't change the units param (3d param = 0)
      */
      set_dac_conv_adj_param(&(p_ao_ch->dac_calib), p_ao_ch->dac_reg, 0);
      break;
    }
    case MIO_IOCTL_AO_CH_DAC_UNITS:
    {
      switch(arg)
      {
        case DAC_100uV_UNIT:
        case DAC_mV_UNIT:
        case DAC_10mV_UNIT:
        case DAC_100mV_UNIT:
        case DAC_V_UNIT:
        {
          set_dac_conv_adj_param(&(p_ao_ch->dac_calib), p_ao_ch->dac_reg, arg);
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
  return 0; /* success */
}
/******************************************************************************
*
* ao_ch_dummy_open - dummy open the analog output channel 
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
static int ao_ch_dummy_open(struct inode* p_inode, struct file* p_file)
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
* init_ao_ch - Initialize an Analog input channel object (constructor)
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
STATUS init_ao_ch(AO_CHANNEL* p_ao_ch, int ch_nr, void* irq_start_par,
                  STATUS (*irq_start_func)(void*, int))
{
  p_ao_ch->wq    = NULL;                 /* wait queue for blocking I/O */

  p_ao_ch->nr   = ch_nr;
  /*
  ** D/A conversion IRQ start parameters
  */
  p_ao_ch->irq_start_func = irq_start_func;
  p_ao_ch->irq_start_par  = irq_start_par;
  /*
  ** fill out the file_operations structure
  */
  p_ao_ch->fops.llseek              = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.read                = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.write               = ao_ch_write; /* write */
  p_ao_ch->fops.readdir             = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.poll                = NULL; /* NOT USED for output channel */

  p_ao_ch->fops.ioctl               = ao_ch_ioctl; /* ioct() for setting 
                                                   ** output channel params 
                                                   */
  p_ao_ch->fops.mmap                = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.open                = ao_ch_dummy_open;/* dummy open routine */
  p_ao_ch->fops.flush               = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.release             = ao_ch_close; /* close routine */
  p_ao_ch->fops.fsync               = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.fasync              = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.check_media_change  = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.revalidate          = NULL; /* NOT USED for output channel */
  p_ao_ch->fops.lock                = NULL; /* NOT USED for output channel */
  
  /*
  ** Initial settings, 5V range not bipolar, mV units 
  */
  p_ao_ch->dac_reg = 0;
  SET_DAC_CH(p_ao_ch->dac_reg, ch_nr);
  SET_DAC_BUSY(p_ao_ch->dac_reg, 1);
  set_dac_conv_adj_param(&(p_ao_ch->dac_calib), p_ao_ch->dac_reg, DAC_mV_UNIT);
  p_ao_ch->state = ao_ch_initialized;

#ifdef DEBUG_AO
  printk("<1> analog out ch %i init OK\n", ch_nr);
#endif

  return OK;
}
/******************************************************************************
*
* cleanup_ao_ch - cleanup an Analog input channel object (destructor)
*
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void cleanup_ao_ch(AO_CHANNEL* p_ao_ch)
{
  /*
  ** Ring buffer cleanup
  */
  p_ao_ch->state = ao_ch_uninitialized;

#ifdef DEBUG_AO
  printk("<1> analog out ch %i init OK\n", p_ao_ch->nr);
#endif

}
/******************************************************************************
*
* ao_ch_open - open routine for an Analog Output channel
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int ao_ch_open(struct file* p_file, AO_CHANNEL* p_ao)
{
  switch (p_ao->state)
  {
    case ao_ch_uninitialized:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AO_CH_OPEN_ERR1);
      return -ENODEV; /* No such device */
    }
    case ao_ch_opened:
    case ao_ch_to_write:
    {
      icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AO_CH_OPEN_ERR2);
      return -EMFILE; /* Too many open files */
    }
    case ao_ch_initialized:
    {
      /*
      ** open the file
      */
      /*
      ** Initial settings, 5V range not bipolar, mV units 
      */
      p_ao->dac_reg = 0;
      SET_DAC_CH(p_ao->dac_reg, p_ao->nr);
      SET_DAC_BUSY(p_ao->dac_reg, 1);
      set_dac_conv_adj_param(&(p_ao->dac_calib), p_ao->dac_reg, DAC_mV_UNIT);

      p_file->f_op = &(p_ao->fops);
      p_file->private_data = (void*)p_ao;
      p_ao->state = ao_ch_opened;
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