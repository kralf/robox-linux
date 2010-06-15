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
** Title:   Analog Input channel ADC Object
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
#include "../h/interrupt.h"

#include "../h/ai_ch.h"
#include "../h/ai_adc.h"
#include "../h/icp_mio.msg"
/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define BIP_OFFS          0x00000800L /* bipolar offset half of the 12 bit 
                                         resolution */
#define SET_4mA_20mA      0x0840

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static inline BOOL   adc_irq_enabled(AI_ADC* p_adc);
static        STATUS adc_calib(AI_ADC* p_adc, u16 adc_reg, long* data);
static        STATUS adc_autocalib(AI_ADC* p_adc, u32 factory_calib);
/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* adc_irq_enabled - is the A/D conversion Irq enabled ?
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static inline BOOL adc_irq_enabled(AI_ADC* p_adc)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_adc->ai_adc_interrupt_enable_reg);

  restore_flags(flags);
  return ( (val & ADC_READY_IRQ) );
}

/******************************************************************************
*
* adc_calib - calibration of the ADC with specific params
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static STATUS adc_calib(AI_ADC* p_adc, u16 adc_reg, long* data)
{
  STATUS ret_val     = OK;
  int  i             = 0;
  int  j             = 0;
  u16  adc_data      = 0;
  long cum_data      = 0;
  BOOL enable_irq    = FALSE;
  BOOL adc_busy_save = FALSE;
  u16  adc_reg_save  = 0;

  /*
  ** if adc IRQ enabled, disable it
  */
  if (adc_irq_enabled(p_adc))
  {
    ai_adc_disable_adc_irq(p_adc);
    enable_irq = TRUE;
  }

  adc_busy_save      = p_adc->adc_busy;
  p_adc->adc_busy    = FALSE;
  adc_reg_save       = readw(p_adc->ai_adc_cmd_stat_reg);

  for(i = 0; i < 16; i++)
  {
    /*
    ** write ADC_COMMAND_REG
    */
    writew(adc_reg, p_adc->ai_adc_cmd_stat_reg);
    j = 0;
    do
    {
      udelay(100); /* wait 100 us */
      j++;
    }while ( (readw(p_adc->ai_adc_cmd_stat_reg) & ADC_CMD_REG_BUSY) &&
             (j < ADC_TIMEOUT) );

    if ( j == ADC_TIMEOUT )
    {
      ret_val = -EBUSY;
    }
    /*
    ** read the data
    */
    adc_data = readw(p_adc->ai_adc_data_reg);
    cum_data += (long)adc_data;
  }
  cum_data /= 16;
  *data = cum_data;

  ai_adc_clear_adc_irq_status(p_adc);
  p_adc->adc_busy = adc_busy_save;
  writew(adc_reg_save, p_adc->ai_adc_cmd_stat_reg);

  if (enable_irq)
  {
    ai_adc_enable_adc_irq(p_adc);
  }

  return ret_val;
}
/******************************************************************************
*
* adc_autocalib - automatic calibration of the ADC
*
*  unfortunatelly the ADC is not linear:
*  y = m*x +b
* 
*  we have to fix x values:
*  GND => x1 = 0
*  Vref => Factory calibration value in EEPROM
*
*  it means:
*  y1 = m*x1 + b => y1 = m*0 + b =>   b = y1
*  y2 = m*x2 + b => y2 = m*x2 + y1 => m = (y2 - y1) / x2
*
*  to be more accurate we are using long variables with a left shift of 16 bit, so
*  we can calculate the input values in mV:
* 
*  max range: bipolar -10V - 10 V => 20 V => 20.000 mV
*
*  which means  x [in mV] = 20.000/x, but we have only integer values, that's why:
*  we use the following form: 20.000*c/x*c => x in mV
*  to be in the 32bit long type: 20.000*c < 2^31 => c < 107374.1824 => 
*  c = 2^16 = 65536 < 107374.1824
*      
*  and so we use the following form calculating x:
*  x = (y - b)/m => x = (c*y - c*b)/c*m
*
*  to get more accurate values for b and m:
*  c*y1 = c*m*x1 + c*b => x1 = 0 => c*y1 = c*b (Caution y1 already y1*2^4,
*                                               the 4 LSB's are 0)
*
*  c*y2 = c*m*x2 + c*b => x2 = fact_calib => c*b = c*y1 => 
*  c*m = (c*y2 - c*y1)/x2                      (Caution y2 already y2*2^4,
*                                               the 4 LSB's are 0)
*        
* factory_calib is given in uVs => to get the for the above form fitting value:
* x2 = factory_calib = factory_calib*0x1000 / Range in uV, but it would 
* be too long for a long value: factory_calib < 2^19 = 524288 so if it is given
* in 10 uV units it fits till 524288 * 10uV, so the form for getting x2:
*
* x2 = fact_calib 
* normally we should use: fact_calib_n = ((fact_calib/10)*2^12)/ Range in 10uV units, 
* ==>
* x2 = fact_calib_n = ((fact_calib/10)*2^12)/ (Range in 10uV)
* 
* and now the concrete values:
* 
* 1.Unipolar, 5V Range
* -) c*b = 2^16*y1, but y1 = 2^4*(real)y1 => c*b = 2^12*y1
* -) c*m = (c*y2 -c*y1)/x2 => where x2 = ((fact_calib/10)*2^12)/ (500000)
*                                   x2 = ((fact_calib*(2^11/5))/(500000) 
*                                   x2 = fact_calib*((2^11/5)/(500000))
*                                   x2 = fact_calib*(2^11/2500000)
*
* and so c*m = (2^12*y2 - 2^12*y1)/ (fact_calib*2^11/2500000)
*
* and the x calcualting form: x = (c*y - c*b)/c*m => x in mV = x*5000/0x1000
*
*  x in mV = 5000*((c*y - c*b)/c*m)/0x1000
* 
* 2.Unipolar, 10V Range
* -) c*b = 2^16*y1, but y1 = 2^4*(real)y1 => c*b = 2^12*y1
* -) c*m = (c*y2 -c*y1)/x2 => where x2 = ((fact_calib/10)*2^12)/ (1000000)
*                                   x2 = ((fact_calib*(2^11/5))/(1000000) 
*                                   x2 = fact_calib*((2^11/5)/(1000000))
*                                   x2 = fact_calib*(2^11/5000000)
*
* and so c*m = (2^12*y2 - 2^12*y1)/ (fact_calib*(2^11/5000000))
*
* and the x calcualting form: x = (c*y - c*b)/c*m => x in mV = 10000*x/0x1000
*
*  x in mV = 10000*((c*y - c*b)/c*m)/0x1000
* 
* 3.Bipolar, 5V Range
* y = m(x+off) + b
* x = (y - m*off -b)/m and (m*off + b) = B
* so c*x = (c*y - c*B)/m
* => x = (c*y - c*B)/c*m
* and c*B = c*y1
* and c*m = (c*y2 - c*B)/x2
*
* -) c*B = 2^16*y1, but y1 = 2^4*(real)y1 => c*B = 2^12*y1
* -) c*m = (c*y2 -c*B)/x2 => where  c*B = c*y1 and
*                                   x2 = (((fact_calib/10)*2^11)/ (500000))
*                                   x2 = (((fact_calib*(2^10/5))/(500000)) 
*                                   x2 = (fact_calib*((2^10/5)/(500000)))
*                                   x2 = (fact_calib*(2^10/2500000))
*
* and so c*m = (2^12*y2 - 2^12*y1)/ ((fact_calib*(2^10/2500000)))
*
* and the x calcualting form: x = (c*y - c*B)/c*m => x in mV = 5000*x/0x800
*                                                 => x in mV = 10000*x/0x1000
*
*  x in mV = 10000* ((((c*y - c*b)/c*m))) / 0x1000
* 
* 3.Bipolar, 10V Range
* y = m(x+off) + b
* x = (y - m*off -b)/m and (m*off + b) = B
* so c*x = (c*y - c*B)/m
* and c*B = c*y1
* and c*m = (c*y2 - c*B)/x2
*
* -) c*B = 2^16*y1, but y1 = 2^4*(real)y1 => c*B = 2^12*y1
* -) c*m = (c*y2 -c*B)/x2 => where  c*B = c*y1 and
*                                   x2 = ((fact_calib/10)*2^11)/ (1000000)
*                                   x2 = ((fact_calib*(2^10/5))/(1000000) 
*                                   x2 = (fact_calib*((2^10/5)/(1000000)))
*                                   x2 = (fact_calib*(2^10/5000000))
*
* and so c*m = (2^12*y2 - 2^12*y1)/ ((fact_calib*(2^10/5000000)))
*
* and the x calcualting form: x = (c*y - c*b)/c*m => x in mV = 10000*x/0x800
*                                                 => x in mV = 20000*x/0x1000
*
*  x in mV = 20000 * (((c*y - c*b)/c*m)) / 0x1000
* 
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
static STATUS adc_autocalib(AI_ADC* p_adc, u32 factory_calib)
{
  ADC_CALIB* p_cal   = &(p_adc->adc_calib);
  u16        adc_reg;
  STATUS     status;
  long       fact_calib_l = (long)factory_calib;
  long       data;
  
  /*
  ** 1. Unipolar 5V
  */
  /*
  ** Set the values for GND
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 3);
  SET_ADC_INP_RANGE(adc_reg, 0);
  SET_ADC_BIP_INP_RANGE(adc_reg, 0);
  SET_ADC_BUSY(adc_reg, 1);

  if ( (status = adc_calib( p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
  /*
  ** store calibration value
  */
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by GND for 5V Unipolar: %li\n", data);
#endif
  p_cal->cb[0] = (data << (AI_C_PARAM-4));
  /*
  ** Set the values for VRef
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 1);
  SET_ADC_INP_RANGE(adc_reg, 0);
  SET_ADC_BIP_INP_RANGE(adc_reg, 0);
  SET_ADC_BUSY(adc_reg, 1);
  if ( (status = adc_calib(p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by VREF for 5V unipolar: %li\n", data);
#endif
  /*
  ** store calibration value
  */
  p_cal->cm[0] = ((data << (AI_C_PARAM-4)) - p_cal->cb[0]) / 
                 (((fact_calib_l/5) << 11)/500000);

  if ( p_cal->cm[0] == 0 )
  {
    p_cal->cm[0] = 1;
  }

  /*
  ** 2. Bipolar 5V
  */
  /*
  ** Set the values for GND
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 3);
  SET_ADC_INP_RANGE(adc_reg, 0);
  SET_ADC_BIP_INP_RANGE(adc_reg, 1);
  SET_ADC_BUSY(adc_reg, 1);

  if ( (status = adc_calib( p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
  /*
  ** store calibration value
  */
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by GND for 5V Bipolar: %li\n", data);
#endif
  p_cal->cb[1] = (data << (AI_C_PARAM-4));
  /*
  ** Set the values for VRef
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 1);
  SET_ADC_INP_RANGE(adc_reg, 0);
  SET_ADC_BIP_INP_RANGE(adc_reg, 1);
  SET_ADC_BUSY(adc_reg, 1);
  if ( (status = adc_calib(p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by VREF for 5V bipolar: %li\n", data);
#endif
  /*
  ** store calibration value
  */
  /* c*m = (2^12*y2 - 2^12*y1)/ ((fact_calib/5)*(2^10/500000))) */
  p_cal->cm[1] = ((data << (AI_C_PARAM-4)) - p_cal->cb[1]) / 
                 (((fact_calib_l/5) << 10)/500000);

  if ( p_cal->cm[1] == 0 )
  {
    p_cal->cm[1] = 1;
  }

  /*
  ** 3. Unipolar 10V
  */
  /*
  ** Set the values for GND
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 3);
  SET_ADC_INP_RANGE(adc_reg, 1);
  SET_ADC_BIP_INP_RANGE(adc_reg, 0);
  SET_ADC_BUSY(adc_reg, 1);

  if ( (status = adc_calib( p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
  /*
  ** store calibration value
  */
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by GND for 10V unipolar: %li\n", data);
#endif
  p_cal->cb[2] = (data << (AI_C_PARAM-4));
  /*
  ** Set the values for VRef
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 1);
  SET_ADC_INP_RANGE(adc_reg, 1);
  SET_ADC_BIP_INP_RANGE(adc_reg, 0);
  SET_ADC_BUSY(adc_reg, 1);
  if ( (status = adc_calib(p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by VREF for 10V unipolar: %li\n", data);
#endif
  /*
  ** store calibration value
  */
  /* and so c*m = (2^12*y2 - 2^12*y1)/ (fact_calib*(2^11/5000000)) */
  p_cal->cm[2] = ((data << (AI_C_PARAM-4)) - p_cal->cb[2]) / 
                 (((fact_calib_l/5) << 11)/1000000);

  if ( p_cal->cm[2] == 0 )
  {
    p_cal->cm[2] = 1;
  }

  /*
  ** 4. Bipolar 10V
  */
  /*
  ** Set the values for GND
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 3);
  SET_ADC_INP_RANGE(adc_reg, 1);
  SET_ADC_BIP_INP_RANGE(adc_reg, 1);
  SET_ADC_BUSY(adc_reg, 1);

  if ( (status = adc_calib( p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
  /*
  ** store calibration value
  */
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by GND for 10V bipolar: %li\n", data);
#endif
  p_cal->cb[3] = (data << (AI_C_PARAM-4));
  /*
  ** Set the values for VRef
  */
  adc_reg = 0;
  SET_ADC_DIFF_INP_MODE(adc_reg, 1);
  SET_ADC_CH(adc_reg, 1);
  SET_ADC_INP_RANGE(adc_reg, 1);
  SET_ADC_BIP_INP_RANGE(adc_reg, 1);
  SET_ADC_BUSY(adc_reg, 1);
  if ( (status = adc_calib(p_adc, adc_reg, &data)) != OK )
  {
    return status;
  }
#ifdef DEBUG_AI
  printk("<1> ANALOG IN MEASURED cb by VREF for 10V bipolar: %li\n", data);
#endif
  /*
  ** store calibration value
  */
  /* and so c*m = (2^12*y2 - 2^12*y1)/ (((fact_calib/5)*(2^10/1000000))) */
  p_cal->cm[3] = ((data << (AI_C_PARAM-4)) - p_cal->cb[3]) / 
                 (((fact_calib_l/5) << 10)/1000000);

  if ( p_cal->cm[3] == 0 )
  {
    p_cal->cm[3] = 1;
  }

  /*
  ** 3. Set the tgt_voltage
  */
  p_cal->tgt_voltage[0] = 5; /* 5V, unipolar */
  p_cal->tgt_voltage[1] = 10; /* 5V, bipolar */
  p_cal->tgt_voltage[2] = 10; /* 10V, unipolar */
  p_cal->tgt_voltage[3] = 20; /* 10V, bipolar */

  return OK;
}
/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* ai_adc_clear_adc_irq_status - clears the A/D conversion Irq status bit
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
inline void ai_adc_clear_adc_irq_status(AI_ADC* p_adc)
{
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  writew(ADC_READY_IRQ,p_adc->ai_adc_interrupt_status_reg);
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* enable_adc_irq - enables the A/D conversion Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
inline void ai_adc_enable_adc_irq(AI_ADC* p_adc)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_adc->ai_adc_interrupt_enable_reg);

  if ( !(val & ADC_READY_IRQ) )
  {
    /*
    ** Irq not enabled, enable it
    */
    val |= ADC_READY_IRQ;
    writew(val,p_adc->ai_adc_interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}
/******************************************************************************
*
* disable_adc_irq - disables the A/D conversion Irq
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
inline void ai_adc_disable_adc_irq(AI_ADC* p_adc)
{
  u16 val;
  unsigned long  flags;
  /*
  ** synchronize with the irq routine
  */
  save_flags(flags);
  cli();
  val = readw(p_adc->ai_adc_interrupt_enable_reg);

  if ( (val & ADC_READY_IRQ) )
  {
    /*
    ** Irq enabled, disable it
    */
    val &= ~ADC_READY_IRQ;
    writew(val,p_adc->ai_adc_interrupt_enable_reg);
  }
  restore_flags(flags);
  return;
}

/******************************************************************************
*
* ai_adc_set_adc_conv_adj_param - sets the ADC-adjustment parameters
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
inline void ai_adc_set_adc_conv_adj_param(void* p_adc_calib_parent, 
                                          AI_CH_ADC_ADJ_PAR* p_cal, 
                                          u16 adc_reg, long units)
{
  ADC_CALIB*    p_adc_calib = (ADC_CALIB*)p_adc_calib_parent;
  u16           cal_index;

  if (units != 0)
  {
    p_cal->units = units;
  }

  cal_index             = (adc_reg >> 4) & 0x03;
  p_cal->cb_i           = p_adc_calib->cb[cal_index];
  p_cal->cm_i           = p_adc_calib->cm[cal_index];
  p_cal->tgt_voltage_i  = p_adc_calib->tgt_voltage[cal_index] * p_cal->units;
  
  p_cal->b_4mA_20mA     = (adc_reg == SET_4mA_20mA);
  return;
}


/******************************************************************************
*
* init_adc_params - Initialize the ADC object of a MIO card (constructor)
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
STATUS ai_adc_init_adc(AI_ADC* p_adc, u32 factory_calib, u8* mem_base_addr )
{
  STATUS          status;

  /* 
  ** initialize the AI_ADC stucture
  */
  p_adc->state                       = ai_adc_uninitialized;
  p_adc->ai_adc_cmd_stat_reg         = mem_base_addr + ADC_CMD_STAT_REG_OFFS;
  p_adc->ai_adc_data_reg             = mem_base_addr + AI_DATA_REG_OFFS;
  p_adc->ai_adc_interrupt_enable_reg = mem_base_addr + IRQ_ENABLE_REG_OFFS;
  p_adc->ai_adc_interrupt_status_reg = mem_base_addr + IRQ_STATUS_REG_OFFS;
  p_adc->adc_calib.factory_calib     = factory_calib;

  
  /*
  ** make auto calibration
  */
  if ( (status = adc_autocalib(p_adc, factory_calib)) != OK)
  {
    /* 
    ** icpErrorMsg(Error during autocalibration )
    */
    return status;
  }
  p_adc->state    = ai_adc_calibrated;
  /* 
  ** set active channel to 0
  */
  p_adc->active_ch = 0;
  /*
  ** all other params are 0
  */
  p_adc->adc_busy = FALSE;
  writew(0, p_adc->ai_adc_cmd_stat_reg);

  return OK;
}
/******************************************************************************
*
* cleanup_adc_params - cleanup the ADC object of a MIO card (destructor)
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
void ai_adc_cleanup_adc(AI_ADC* p_adc)
{
  /* 
  ** set active channel to 0
  */
  p_adc->active_ch = 0;
  /*
  ** all params are 0
  */
  p_adc->adc_busy = FALSE;
  writew(0, p_adc->ai_adc_cmd_stat_reg);
  p_adc->state    = ai_adc_uninitialized;
}

/******************************************************************************
*
* mio_ai_adc_calib_ao - calibration of the ADC with specific params
*
*
* Parameters:
*         
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
STATUS mio_ai_adc_calib_ao(void* param, u16 ch_nr, BOOL bip, 
                           BOOL range, long* p_data)
{
  AI_ADC*        p_adc   = (AI_ADC*)param;
  u16            adc_reg = 0;
  int            idx     = 0;
  STATUS         status;
  long           l_temp;

#if 0
  if ( ch_nr > 3 )
  {
    icpErrMessage(.....);
    return -EBADPARAM;
  }

  if (p_adc->state != ai_adc_calibrated)
  {
    icpErrMessage(.....);
    return -EBADPARAM;
  }
#endif

  /*
  ** settings for calibration:
  ** DIFF_INP = 1
  ** ch_nr = 9  => AOUT0 (9 + ch_nr*2)
  ** ch_nr = 11 => AOUT1
  ** ch_nr = 13 => AOUT2
  ** ch_nr = 15 => AOUT3
  **
  ** No ch_nr param check !!!
  */
  SET_ADC_CH(adc_reg, (9 + 2*ch_nr));
  SET_ADC_DIFF_INP_MODE(adc_reg,1);
  if (bip)
  {
    SET_ADC_BIP_INP_RANGE(adc_reg,1);
    idx = 1;
  }
  else
  {
    SET_ADC_BIP_INP_RANGE(adc_reg,0);
    idx = 0;
  }

  if (range)
  {
    SET_ADC_INP_RANGE(adc_reg,1);
    idx += 2;
  }
  else
  {
    SET_ADC_INP_RANGE(adc_reg,0);
  }
  
  SET_ADC_BUSY(adc_reg,1);

  if ((status = adc_calib(p_adc, adc_reg, p_data)) != OK)
  {
    return status;
  }

  /* 
  ** use the calibration values from channel 0
  */
  l_temp = ((*p_data << (AI_C_PARAM-4)) - p_adc->adc_calib.cb[idx]) / 
            p_adc->adc_calib.cm[idx];
  
  *p_data = l_temp;
  if( bip )
  {
    *p_data += BIP_OFFS;
  }
  
  return OK;
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/