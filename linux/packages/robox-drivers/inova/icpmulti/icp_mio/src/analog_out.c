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
** Title:   Analog Output Object
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
#include <linux/delay.h>
#include <asm/string.h>

#include <icp_drv_gen.h>
/* 
** local module headers
*/
#include "../h/interrupt.h"

#include "../h/analog_out.h"
#include "../h/icp_mio.msg"
/*---------------------------------------------------------------------------
** FRIEND Functions
**-------------------------------------------------------------------------*/
extern STATUS mio_ai_adc_calib_ao(void* param, u16 ch_nr, BOOL bip, 
                                  BOOL range, long* p_data);

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
/*
** for the calibration
*/
#define DAC_10_PERC       0x1999
#define DAC_90_PERC       0xE666
#define AO_X2_X1_DIFF     0x0000CCCDL    /* x2 -x1 in calibration E666 - 1999 */

#define AO_C_PARAM        15

#define SET_DAC_CH( dest, ch_nr)                    \
                  (dest) = ( (dest) & 0xFCFF ) | ( (ch_nr) << 8 )

#define SET_DAC_INP_RANGE( dest, inp_range)         \
                  (dest) = ( (dest) & 0xFFDF ) | ( (inp_range) << 5 )

#define SET_DAC_BIP_INP_RANGE( dest, bip_inp_range) \
                 (dest) = ( (dest) & 0xFFEF ) | ( (bip_inp_range) << 4 )

#define SET_DAC_BUSY( dest, busy) \
                 (dest) = ( (dest) & 0xFFFE ) | ( busy )
/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/
static char irq_service_rt_name[] = "analog out interrupt service routine";

/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static inline void clear_dac_irq_status(MIO_ANALOG_OUT* p_ao);
static inline void enable_dac_irq(MIO_ANALOG_OUT* p_ao);
static inline void disable_dac_irq(MIO_ANALOG_OUT* p_ao);
static STATUS  start_dac_irq(void* param, int ch_nr);
static void    irq_service_rt(void* param);
static STATUS  init_dac_params(MIO_ANALOG_OUT* p_ao, void* p_ai);
static void    cleanup_dac_params(MIO_ANALOG_OUT* p_ao);
static STATUS  ao_ch_calib_write_output(MIO_ANALOG_OUT* p_ao, u16 ch_nr, 
                                        BOOL bip, BOOL range, u16 w_data);
static STATUS  ao_ch_calib(void* p_ai, MIO_ANALOG_OUT* p_ao, u16 ch_nr,
                           BOOL bip, BOOL range, u16 w_data, long* data);
static STATUS  ao_auto_calib(void* p_ai, MIO_ANALOG_OUT* p_ao);
static inline BOOL dac_irq_enabled(MIO_ANALOG_OUT* p_ao);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* ao_ch_calib_write_output - write calibration data to analog output
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static STATUS ao_ch_calib_write_output(MIO_ANALOG_OUT* p_ao, u16 ch_nr, 
                                       BOOL bip, BOOL range, u16 w_data)
{
  STATUS ret_val = OK;
  int    j;
  BOOL   enable_irq   = FALSE;
  BOOL   dac_busy_save;
  u16    dac_reg_save;
  u16    ao_data_save;
  u16    dac_reg = 0;

  /*
  ** first set the channel nr. param
  */
  SET_DAC_CH(dac_reg,ch_nr);
  if(bip)
  {
    SET_DAC_BIP_INP_RANGE(dac_reg,1);
  }
  else
  {
    SET_DAC_BIP_INP_RANGE(dac_reg,0);
  }

  if(range)
  {
    SET_DAC_INP_RANGE(dac_reg,1);
  }
  else
  {
    SET_DAC_INP_RANGE(dac_reg,0);
  }
  SET_DAC_BUSY(dac_reg,1);

  /*
  ** if dac IRQ enabled, disable it
  */
  if (dac_irq_enabled(p_ao))
  {
    disable_dac_irq(p_ao);
    enable_irq = TRUE;
  }
  dac_busy_save      = p_ao->dac.dac_busy;
  p_ao->dac.dac_busy = FALSE;
  ao_data_save = readw(p_ao->ao_data_reg);
  dac_reg_save = readw(p_ao->dac_cmd_stat_reg);

  /*
  ** write ADC data and then ADC_COMMAND_REG
  */
  writew(w_data, p_ao->ao_data_reg);
  writew(dac_reg, p_ao->dac_cmd_stat_reg);
  j = 0;
  do
  {
    udelay(100); /* wait 100 us */
    j++;
  } while ( (readw(p_ao->dac_cmd_stat_reg) & DAC_CMD_REG_BUSY) &&
            (j < DAC_TIMEOUT) );

  if ( j == DAC_TIMEOUT )
  {
    ret_val = -EBUSY;
  }

  clear_dac_irq_status(p_ao);
  writew(ao_data_save, p_ao->ao_data_reg);
  p_ao->dac.dac_busy = dac_busy_save;
  writew(dac_reg_save, p_ao->dac_cmd_stat_reg);

  if (enable_irq)
  {
    enable_dac_irq(p_ao);
  }

  return ret_val;
}

/******************************************************************************
*
* ao_ch_calib - calibration of one channel with a given value
*
 *
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static STATUS ao_ch_calib(void* p_ai, MIO_ANALOG_OUT* p_ao, u16 ch_nr,
                          BOOL bip, BOOL range, u16 w_data, long* data)
{
  STATUS status;
  /*
  ** first write the data to convert
  */
  if ( (status = ao_ch_calib_write_output(p_ao, ch_nr, bip, range, w_data)) 
                                                                         != OK)
  {
    return status;
  }
  /* 
  ** wait 1000us
  */
  udelay(1000);
  /* 
  ** get the read input data
  */
  return mio_ai_adc_calib_ao(p_ai, ch_nr, bip, range, data);
}

/******************************************************************************
*
* ao_auto_calib - auto calibration of the ao channels
*
*  unfortunatelly the DAC is not perfect (y = output, x = input)
*  y = m*x + b;
*  to be more accurate we use the following form:
*  c*y = c*m*x + c*b;
*  and so: which x value I need to get the right y value:
*  x = (c*y-c*b) / c*m;
*  
*  calcualtion:
*  2 measurments: at x1 = 0.1*Range and x2 = 0.9*Range
*
*  c*y1 = c*m*x1 + c*b;
*  c*y2 = c*m*x2 + c*b;
* ---------------------
* 
* 1. build the difference of the two equation (2-1)
*    c*y2 - c*y1 = c*m*(x2-x1) 
*    => c*m = c*(y2 - y1) / (x2 - x1)
*    
* 2. multiply the first equation with (x2/x1) and then build the diference (2 - 1)
*   - c*y1*(x2/x1) = c*m*x2 + c*b*(x2/x1);
*   + c*y2         = c*m*x2 + c*b;
*   ------------------------------
*    c*y2 - c*y1*(x2/x1) = c*b - c*b*(x2/x1)
*    c*(y2 - y1*(x2/x1)) = c*(1 - (x2/x1))*b   
*    => c*b = (c*(y2 - y1*(x2/x1))) / (1 - (x2/x1))
*
* x1 = 0.1*RANGE
* x2 = 0.9*RANGE
* x2/x1 = 9
*
* => c*m = (c*y2 - c*y1) / 0.8*RANGE
* => c*b = c*(y2 - 9*y1) / (-8)
*
* if R = 0-5V 
*             => x1 = 0.1*R = 0.1*0xFFFF = 0x1999
*             => x2 = 0.9*R = 0.9*0xFFFF = 0xE666
*
* if R = 0-10V 
*             => x1 = 0.1*R = 0.1*0xFFFF = 0x1999
*             => x2 = 0.9*R = 0.9*0xFFFF = 0xE666
*
* if R = -5-5V 
*             => x1 = 0.1*R = 0.1*0xFFFF = 0x1999
*             => x2 = 0.9*R = 0.9*0xFFFF = 0xE666
*
* if R = -10-10V 
*             => x1 = 0.1*R = 0.1*0xFFFF = 0x1999
*             => x2 = 0.9*R = 0.9*0xFFFF = 0xE666
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static STATUS ao_auto_calib(void* p_ai, MIO_ANALOG_OUT* p_ao)
{
  STATUS      status = OK;
  AO_CHANNEL* p_ao_ch = NULL;
  int   i;
  long  y1;
  long  y2;
  u16   w_data;
  BOOL  bip;
  BOOL  range;

  /*
  ** do it for all channel
  */
  for (i = 0; i < NR_OF_DO_CH; i++)
  {
    p_ao_ch = &(p_ao->ao_ch[i]);
    /*
    ** 1. bip = 0, range = 5V, 10% then 90 %
    */
    bip = FALSE;
    range = FALSE;
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_10_PERC, &y1))
                                                                         != OK)
    {
      return status;
    }
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_90_PERC, &y2))
                                                                         != OK)
    {
      return status;
    }
    p_ao_ch->dac_calib.cb[0] = ((y2 - 9*y1) << (AO_C_PARAM)) / (-8L);
    p_ao_ch->dac_calib.cm[0] =  ((y2 - y1) << (AO_C_PARAM)) / AO_X2_X1_DIFF; /* x2 -x1 = (0x0000E666L - 0x00001999L) */
    if ( p_ao_ch->dac_calib.cm[0] == 0 )
    {
      p_ao_ch->dac_calib.cm[0] = 1;
    }
    p_ao_ch->dac_calib.offs[0]  = 0;
    p_ao_ch->dac_calib.range[0] = 5;
    /*
    ** 2. bip = 1, range = 5V, 10% then 90 %
    */
    bip = TRUE;
    range = FALSE;
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_10_PERC, &y1))
                                                                         != OK)
    {
      return status;
    }
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_90_PERC, &y2))
                                                                         != OK)
    {
      return status;
    }
    p_ao_ch->dac_calib.cb[1] = ((y2 - 9*y1) << (AO_C_PARAM)) / (-8L);
    p_ao_ch->dac_calib.cm[1] = ((y2 - y1) << (AO_C_PARAM)) / AO_X2_X1_DIFF; /* x2 -x1 = (0x0000E666L - 0x00001999L) */
    if ( p_ao_ch->dac_calib.cm[1] == 0 )
    {
      p_ao_ch->dac_calib.cm[1] = 1;
    }
    p_ao_ch->dac_calib.offs[1]  = 5;
    p_ao_ch->dac_calib.range[1] = 10;
    /*
    ** 3. bip = 0, range = 10V, 10% then 90 %
    */
    bip = FALSE;
    range = TRUE;
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_10_PERC, &y1))
                                                                         != OK)
    {
      return status;
    }
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_90_PERC, &y2))
                                                                         != OK)
    {
      return status;
    }
    p_ao_ch->dac_calib.cb[2] = ((y2 - 9*y1) << (AO_C_PARAM)) / (-8L);
    p_ao_ch->dac_calib.cm[2]  =  ((y2 - y1) << (AO_C_PARAM)) / AO_X2_X1_DIFF; /* x2 -x1 = (0x0000E666L - 0x00001999L) */
    if ( p_ao_ch->dac_calib.cm[2] == 0 )
    {
      p_ao_ch->dac_calib.cm[2] = 1;
    }
    p_ao_ch->dac_calib.offs[2]  = 0;
    p_ao_ch->dac_calib.range[2] = 10;
    /*
    ** 4. bip = 1, range = 10V, 10% then 90 %
    */
    bip = TRUE;
    range = TRUE;
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_10_PERC, &y1))
                                                                         != OK)
    {
      return status;
    }
    if ( (status = ao_ch_calib(p_ai, p_ao, (u16)i, bip, range, DAC_90_PERC, &y2))
                                                                         != OK)
    {
      return status;
    }
#ifdef DEBUG_AO
   printk("<1> analog out ch %i y1: %li %lx y2: %li %lx\n", i, y1, y1,y2,y2);
#endif
    p_ao_ch->dac_calib.cb[3] = ((y2 - 9*y1) << (AO_C_PARAM)) / (-8L);
    p_ao_ch->dac_calib.cm[3] = ((y2 - y1) << (AO_C_PARAM)) / AO_X2_X1_DIFF; /* x2 -x1 = (0x0000E666L - 0x00001999L) */
    if ( p_ao_ch->dac_calib.cm[3] == 0 )
    {
      p_ao_ch->dac_calib.cm[3] = 1;
    }
    p_ao_ch->dac_calib.offs[3]  = 10;
    p_ao_ch->dac_calib.range[3] = 20;
    /*
    ** set the output to 0
    */
    w_data = (u16)((-1*p_ao_ch->dac_calib.cb[0]) / p_ao_ch->dac_calib.cm[0]);

#ifdef DEBUG_AO
  printk("<1> analog out ch %i w_data: %hx\n", i, w_data);
#endif
    if ( (status = ao_ch_calib_write_output(p_ao, i, FALSE, FALSE, 
                                            w_data)) != OK)
    {
      return status;
    }
  }
  return OK;
}

/******************************************************************************
*
* clear_dac_irq_status - clears the A/D conversion Irq status bit
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void clear_dac_irq_status(MIO_ANALOG_OUT* p_ao)
{
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  writew(DAC_READY_IRQ,p_ao->interrupt_status_reg);
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* dac_irq_enabled - 
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline BOOL dac_irq_enabled(MIO_ANALOG_OUT* p_ao)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_ao->interrupt_enable_reg);

  restore_flags(flags);
  return ( (val & DAC_READY_IRQ) );
}
/******************************************************************************
*
* enable_dac_irq - enables the A/D conversion Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void enable_dac_irq(MIO_ANALOG_OUT* p_ao)
{
  u16            val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_ao->interrupt_enable_reg);

  if ( !(val & DAC_READY_IRQ) )
  {
    /*
    ** Irq not enabled, enable it
    */
    val |= DAC_READY_IRQ;
    writew(val,p_ao->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* disable_dac_irq - enables the A/D conversion Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline void disable_dac_irq(MIO_ANALOG_OUT* p_ao)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_ao->interrupt_enable_reg);

  if ( (val & DAC_READY_IRQ) )
  {
    /*
    ** Irq enabled, disable it
    */
    val &= ~DAC_READY_IRQ;
    writew(val,p_ao->interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* start_dac_irq - starts the A/D conversion Irq
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
static STATUS start_dac_irq(void* param, int ch_nr)
{
  MIO_ANALOG_OUT* p_ao = (MIO_ANALOG_OUT*)param;
  unsigned long  flags;
  /*
  ** check if irqs are started
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  if(p_ao->dac.dac_busy)
  {
    restore_flags(flags);
    return OK;
  }
  /*
  ** clear status and enable dac irq
  */
  clear_dac_irq_status(p_ao);
  enable_dac_irq(p_ao);
  /*
  ** set the active channel to ch_nr and start the conversion
  */
  p_ao->dac.active_ch = ch_nr;

  /* write the data to convert */
  writew( p_ao->ao_ch[ch_nr].data, p_ao->ao_data_reg );
  /* start the conversion , dac_reg is prepared*/
  writew(p_ao->ao_ch[ch_nr].dac_reg, p_ao->dac_cmd_stat_reg);
  p_ao->dac.dac_busy = TRUE;
  restore_flags(flags);
  return OK;
}
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
  MIO_ANALOG_OUT* p_ao        = (MIO_ANALOG_OUT*)param;
  AO_CHANNEL*    p_act_ao_ch;
  int            i;
  /*
  ** check if the DAC IRQ is enabled
  */
  if ( !(readw(p_ao->interrupt_enable_reg) & DAC_READY_IRQ) )
  {
    /*
    ** Irq not enabled, return
    */
    return;
  }
  /*
  ** check if it is an DAC ready IRQ 
  */
  if ( !(readw(p_ao->interrupt_status_reg) & DAC_READY_IRQ) )
  {
    /*
    ** not an DAC ready irq
    */
    return;
  }
  /*
  ** service the IRQ
  ** -1. clear IRQ status
  ** 0. set the actual channel state to opened (it is finished with writing)
  ** 1  wakeup the wait queue (it is made by the write routine)
  ** 2. select the new active channel
  ** 3. write data to the channel
  ** 4. clear IR status
  ** 5. start the next D/A conversion
  */
  /* clear IRQ status */
  writew(DAC_READY_IRQ,p_ao->interrupt_status_reg);
  p_ao->dac.dac_busy = FALSE;
  p_act_ao_ch = &(p_ao->ao_ch[p_ao->dac.active_ch]);

  p_act_ao_ch->state = ao_ch_opened;
  wake_up_interruptible(&(p_act_ao_ch->wq));

  /* select the new active channel */
  for(i = (p_ao->dac.active_ch + 1) % NR_OF_DO_CH; 
      i != p_ao->dac.active_ch; 
      i = (i + 1) % NR_OF_DO_CH)
  {
    if( p_ao->ao_ch[i].state == ao_ch_to_write)
    {
      /*
      ** we've found the next active channel
      */
      p_ao->dac.active_ch = i;
      p_act_ao_ch = &(p_ao->ao_ch[i]);
      break;
    }
  }
  /* 
  ** if something found, start conversion
  */
  if (p_act_ao_ch->state == ao_ch_to_write)
  {
    /* write the data to convert */
    writew( p_act_ao_ch->data, p_ao->ao_data_reg );
    /* start the conversion , dac_reg is prepared*/
    p_ao->dac.dac_busy = TRUE;
    writew(p_act_ao_ch->dac_reg, p_ao->dac_cmd_stat_reg);
  }
  return;
}
/******************************************************************************
*
* init_dac_params - Initialize the DAC object of a MIO card (constructor)
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
static STATUS init_dac_params(MIO_ANALOG_OUT* p_ao, void* p_ai)
{
  STATUS          status  = OK;
  DAC_PARAMETERS* p_dac   = &(p_ao->dac);
  
  if ( (status = ao_auto_calib(p_ai, p_ao)) != OK)
  {
    return status;
  }

  writew(0, p_ao->dac_cmd_stat_reg);
  p_dac->dac_busy = FALSE;
  return OK;
}
/******************************************************************************
*
* cleanup_dac_params - cleanup the DAC object of a MIO card (destructor)
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
static void cleanup_dac_params(MIO_ANALOG_OUT* p_ao)
{
  DAC_PARAMETERS* p_dac   = &(p_ao->dac);

  writew(0, p_ao->dac_cmd_stat_reg);
  p_dac->dac_busy = FALSE;
}
/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_analog_out_init - Initialize the Analog Input object of a MIO card (constructor)
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
STATUS mio_analog_out_init(MIO_ANALOG_OUT* p_ao, u8* mem_base_addr,
                           void* irq_ident, void* p_ai)
{
  int    i;
  STATUS status = OK;
  /* 
  ** initialize the MIO_ANALOG_OUT stucture
  */
  p_ao->state                = ao_uninitialized;
  p_ao->dac_cmd_stat_reg     = mem_base_addr + DAC_CMD_STAT_REG_OFFS;
  p_ao->ao_data_reg          = mem_base_addr + AO_DATA_REG_OFFS;
  p_ao->interrupt_enable_reg = mem_base_addr + IRQ_ENABLE_REG_OFFS;
  p_ao->interrupt_status_reg = mem_base_addr + IRQ_STATUS_REG_OFFS;
  p_ao->irq_ident            = irq_ident;
  /*
  ** init DAC params
  */
  I_CHK_STATUSRS( init_dac_params(p_ao, p_ai) );
  
  for (i = 0; i < NR_OF_DO_CH; i++)
  {
    I_CHK_STATUSRS( init_ao_ch(&(p_ao->ao_ch[i]), i, (void*)p_ao,
                               start_dac_irq) );
  }
  /*
  ** initialize interrupt
  ** 1. register irq routine
  ** 2. clear dac interrupt status bit
  ** 3. enable dac interrupt
  */
  if (mio_interrupt_register_routine(irq_ident, 
                                     irq_service_rt, 
                                     (void*)p_ao, 
                                     irq_service_rt_name ) != OK)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AO_IRQ_INIT_ERR);
    mio_analog_out_cleanup(p_ao);
    return -EAGAIN; /* Try again */
  }
  
  clear_dac_irq_status(p_ao);
  enable_dac_irq(p_ao);
  
  p_ao->state = ao_initialized;

#ifdef DEBUG_AO
  printk("<1> analog out init OK\n");
#endif
  return status;
}
/******************************************************************************
*
* mio_analog_out_cleanup - cleanup the Analog Input object of a MIO card (destructor)
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
void mio_analog_out_cleanup(MIO_ANALOG_OUT* p_ao)
{
  int i;

  /*
  ** deinitialize interrupt
  ** 1. disable dac interrupt
  ** 2. clear dac interrupt status bit
  ** 3. deregister irq routine
  */
  disable_dac_irq(p_ao);
  clear_dac_irq_status(p_ao);
  (void)mio_interrupt_deregister_routine(p_ao->irq_ident,
                                         irq_service_rt, 
                                         irq_service_rt_name );

  if (p_ao->state != ao_initialized)
  {
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AO_CLEANUP_ERR);
  }
  for (i = 0; i < NR_OF_DO_CH; i++)
  {
    cleanup_ao_ch( &(p_ao->ao_ch[i]) );
  }
  cleanup_dac_params(p_ao);

  p_ao->state = ao_uninitialized;

#ifdef DEBUG_AO
  printk("<1> analog out deinit OK\n");
#endif

}
/******************************************************************************
*
* mio_analog_out_open - open routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_out_open(struct inode* p_inode, struct file* p_file, 
                        MIO_ANALOG_OUT* p_ao)
{
  int ch_nr = MINOR(p_inode->i_rdev) - MINOR_AO_OFFS;

  if ( (ch_nr >= NR_OF_DO_CH) || (ch_nr < 0) )
  {
    /*
    ** bad param
    */
/*    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_AO_OPEN_ERR); */
    return ICP_MIO_BAD_MINOR;
  }

  return (ao_ch_open(p_file, &(p_ao->ao_ch[ch_nr])));
}

#if 0
/******************************************************************************
*
* mio_analog_out_close - close routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_out_close()
{


}
#endif

/******************************************************************************
*
* mio_analog_out_ioctl - ioctl routine for the Analog Input object
*
*
* Parameters:
*         
*
* RETURNS:-
*   
* SEE ALSO: 
******************************************************************************/
int mio_analog_out_ioctl(struct file* p_file, unsigned int cmd, 
                        unsigned long param, MIO_ANALOG_OUT* p_ao)
{
  switch(cmd)
  {
    case MIO_IOCTL_AO_ENABLE_IRQ:
    {
      if (param)
      {
        enable_dac_irq(p_ao);
      }
      else
      {
        disable_dac_irq(p_ao);
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