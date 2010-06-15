/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.29.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux Driver
** Title:   Analog Input Object
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
000,29sep00,bt   created
00a,25oct00,bt   cleanup
00b 27oct00,bt   split up into 3 files ai_ch.c, ai_adc.c, analog_in.c
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
#include "../h/interrupt.h"

#include "../h/analog_in.h"
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
static char irq_service_rt_name[] = "analog in interrupt service routine";

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static STATUS  start_adc_irq(void* param, int ch_nr);
static void    irq_service_rt(void* param);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* irq_service_rt - A/D conversion Irq service routine
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
  MIO_ANALOG_IN* p_ai        = (MIO_ANALOG_IN*)param;
  AI_CHANNEL*    p_act_ai_ch;
  int            i;
  u16            data;
  /*
  ** check if the ADC IRQ is enabled
  */
  if ( !(readw(p_ai->adc.ai_adc_interrupt_enable_reg) & ADC_READY_IRQ) )
  {
    /*
    ** Irq not enabled, return
    */
#ifdef DEBUG_AI
  printk("<1> analog in ch %i irq ADC irq not enabled\n", p_ai->ai_ch[p_ai->adc.active_ch].nr);
#endif
    return;
  }
  /*
  ** check if it is an ADC ready IRQ 
  */
  if ( !(readw(p_ai->adc.ai_adc_interrupt_status_reg) & ADC_READY_IRQ) )
  {
    /*
    ** not an ADC ready irq
    */
#ifdef DEBUG_AI
  printk("<1> analog in ch %i irq not an ADC irq\n", p_ai->ai_ch[p_ai->adc.active_ch].nr);
#endif
    return;
  }
  /*
  ** service the IRQ
  ** 0. clear irq status
  ** 1. write the data to the channel
  ** 2  wakeup the wait queue (it is made by the write routine)
  ** 3. select the new active channel
  ** 4. clear IRQ status
  ** 5. start the next A/D conversion
  */
  /* clear IRQ status */
  writew(ADC_READY_IRQ,p_ai->adc.ai_adc_interrupt_status_reg);

  p_ai->adc.adc_busy = FALSE;

  p_act_ai_ch = &(p_ai->ai_ch[p_ai->adc.active_ch]);

  /*
  ** read the data
  */
  data = readw(p_ai->adc.ai_adc_data_reg);
  /* write the data to the caller */
  p_act_ai_ch->write_data( p_act_ai_ch, data );
  
  for(i = (p_ai->adc.active_ch + 1) % ADC_NR_OF_CH; 
      i != p_ai->adc.active_ch; 
      i = (i + 1) % ADC_NR_OF_CH)
  {
    if( p_ai->ai_ch[i].state == ai_ch_to_read)
    {
      /*
      ** we've found the next active channel
      */
#ifdef DEBUG_AI
  printk("<1> analog in ch %i irq next channel for ADC %i\n", p_ai->ai_ch[p_ai->adc.active_ch].nr, i);
#endif
      p_ai->adc.active_ch = i;
      p_act_ai_ch = &(p_ai->ai_ch[i]);
      break;
    }
  }
  /* 
  ** if nothing found we can use the old channel, otherwise just return
  */
  if (p_act_ai_ch->state == ai_ch_to_read)
  {
    /* start the conversion , adc_reg is prepared*/
    p_ai->adc.adc_busy = TRUE;
    writew(p_act_ai_ch->adc_reg, p_ai->adc.ai_adc_cmd_stat_reg);
  }
  return;
}


/******************************************************************************
*
* start_adc_irq - starts the A/D conversion Irq
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
static STATUS start_adc_irq(void* param, int ch_nr)
{
  MIO_ANALOG_IN* p_ai = (MIO_ANALOG_IN*)param;
  unsigned long  flags;
  /*
  ** check if irqs are started
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  if(p_ai->adc.adc_busy)
  {
    restore_flags(flags);
    return OK;
  }
  /*
  ** clear status and enable adc irq
  */
  ai_adc_clear_adc_irq_status(&(p_ai->adc));
  ai_adc_enable_adc_irq(&(p_ai->adc));
  /*
  ** set the active channel to ch_nr and start the conversion
  */
  p_ai->adc.active_ch = ch_nr;

  /* start the conversion , dac_reg is prepared*/
  writew(p_ai->ai_ch[ch_nr].adc_reg, p_ai->adc.ai_adc_cmd_stat_reg);
  p_ai->adc.adc_busy  = TRUE;

  restore_flags(flags);
  return OK;
}


/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_analog_in_init - Initialize the Analog Input object of a MIO card (constructor)
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
STATUS mio_analog_in_init(MIO_ANALOG_IN* p_ai, u8* mem_base_addr,
                          void* irq_ident, u32 factory_calib)
{
  int    i;
  STATUS status = OK;

#ifdef DEBUG_AI
  printk("<1> analog in init start\n");
#endif
  /* 
  ** initialize the MIO_ANALOG_IN stucture
  */
  p_ai->state                = ai_uninitialized;
  p_ai->irq_ident            = irq_ident;
  /*
  ** init ADC
  */
  I_CHK_STATUSRS(ai_adc_init_adc(&(p_ai->adc), factory_calib, mem_base_addr));

#ifdef DEBUG_AI
  printk("<1> analog in adc init ok\n");
#endif
  
  for (i = 0; i < ADC_NR_OF_CH; i++)
  {
    I_CHK_STATUSRS( ai_ch_init_ai_ch(&(p_ai->ai_ch[i]), i, (void*)p_ai,
                                     start_adc_irq, 
                                     (void*)(&(p_ai->adc.adc_calib)) ) );
  }
  /*
  ** initialize interrupt
  ** 1. register irq routine
  ** 2. clear adc interrupt status bit
  ** 3. enable adc interrupt
  */
  if (mio_interrupt_register_routine(irq_ident,
                                     irq_service_rt, 
                                     (void*)p_ai, 
                                     irq_service_rt_name ) != OK)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AI_IRQ_INIT_ERR);
    mio_analog_in_cleanup(p_ai);
    return -EAGAIN; /* Try again */
  }
  
  ai_adc_clear_adc_irq_status(&(p_ai->adc));
  ai_adc_enable_adc_irq(&(p_ai->adc));
  
  p_ai->state = ai_initialized;

#ifdef DEBUG_AI
  printk("<1> analog in init OK\n");
#endif

  return status;
}
/******************************************************************************
*
* mio_analog_in_cleanup - cleanup the Analog Input object of a MIO card (destructor)
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
void mio_analog_in_cleanup(MIO_ANALOG_IN* p_ai)
{
  int i;
  /*
  ** deinitialize interrupt
  ** 1. disable adc interrupt
  ** 2. clear adc interrupt status bit
  ** 3. deregister irq routine
  */
  ai_adc_disable_adc_irq(&(p_ai->adc));
  ai_adc_clear_adc_irq_status(&(p_ai->adc));
  (void)mio_interrupt_deregister_routine(p_ai->irq_ident,
                                         irq_service_rt, 
                                         irq_service_rt_name );

  if (p_ai->state != ai_initialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AI_CLEANUP_ERR);
  }
  for (i = 0; i < ADC_NR_OF_CH; i++)
  {
    ai_ch_cleanup_ai_ch( &(p_ai->ai_ch[i]) );
  }
  ai_adc_cleanup_adc(&(p_ai->adc));

  p_ai->state = ai_uninitialized;

#ifdef DEBUG_AI
  printk("<1> analog in cleanup OK\n");
#endif

}
/******************************************************************************
*
* mio_analog_in_open - open routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_in_open(struct inode* p_inode, struct file* p_file, 
                   MIO_ANALOG_IN* p_ai)
{
  int ch_nr = MINOR(p_inode->i_rdev) - MINOR_AI_OFFS;

  if ( (ch_nr >= ADC_NR_OF_CH) || (ch_nr < 0) )
  {
    /*
    ** bad param
    */
/*    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AI_OPEN_ERR); */
    return ICP_MIO_BAD_MINOR;
  }

  return (ai_ch_open(p_file, &(p_ai->ai_ch[ch_nr])));
}

#if 0
/******************************************************************************
*
* mio_analog_in_close - close routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_in_close()
{


}
#endif

/******************************************************************************
*
* mio_analog_in_ioctl - ioctl routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_in_ioctl(struct file* p_file, unsigned int cmd, 
                        unsigned long param, MIO_ANALOG_IN* p_ai)
{
  switch(cmd)
  {
    case MIO_IOCTL_AI_ENABLE_IRQ:
    {
      if (param)
      {
        /*
        ** It would belong to ai_adc.c
        */
        ai_adc_enable_adc_irq(&(p_ai->adc));
      }
      else
      {
        ai_adc_disable_adc_irq(&(p_ai->adc));
      }
      break;
    }
    default:
    {
      return ICP_MIO_BAD_IOCTL;
    }
  }
  return OK;
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/