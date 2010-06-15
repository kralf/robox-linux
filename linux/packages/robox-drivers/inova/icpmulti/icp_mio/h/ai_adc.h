/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.27.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI analog In ADC Object header
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
000,27oct00,bt   created
*/


#ifndef  INCai_adch
#define  INCai_adch

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
#define AI_C_PARAM        16

#define SET_ADC_CH( dest, ch_nr)                    \
                (dest) = ( (dest) & 0xF0FF ) | ( (ch_nr) << 8 )

#define SET_ADC_DIFF_INP_MODE(dest, diff_inp_mode)   \
                (dest) = ( (dest) & 0xFFBF ) | ( (diff_inp_mode) << 6 )

#define SET_ADC_INP_RANGE( dest, inp_range)         \
                (dest) = ( (dest) & 0xFFDF ) | ( (inp_range) << 5 )

#define SET_ADC_BIP_INP_RANGE( dest, bip_inp_range) \
                (dest) = ( (dest) & 0xFFEF ) | ( (bip_inp_range) << 4 )

#define SET_ADC_BUSY( dest, busy)                   \
                (dest) = ( (dest) & 0xFFFE ) | ( busy )


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/

/******************************************** 
** A/D converter object
*********************************************/
typedef enum
{
  ai_adc_calibrated,
  ai_adc_uninitialized,
} ADC_STATE;

typedef struct _ADC_CALIB
{
  long  cb[4];
  long  cm[4];
  long  tgt_voltage[4];
  u32   factory_calib;
} ADC_CALIB;

typedef struct _ADC_PARAMETERS
{
  ADC_STATE state;
  u8*       ai_adc_cmd_stat_reg;
  u8*       ai_adc_data_reg;
  u8*       ai_adc_interrupt_enable_reg;
  u8*       ai_adc_interrupt_status_reg;
  ADC_CALIB adc_calib;
  int       active_ch;
  BOOL      adc_busy;
} AI_ADC;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern inline void   ai_adc_clear_adc_irq_status(AI_ADC* p_adc);
  extern inline void   ai_adc_enable_adc_irq(AI_ADC* p_adc);
  extern inline void   ai_adc_disable_adc_irq(AI_ADC* p_adc);
  extern inline void   ai_adc_set_adc_conv_adj_param(void* p_adc_calib_parent, 
                                            AI_CH_ADC_ADJ_PAR* p_cal, 
                                            u16 adc_reg, long units);
  extern        STATUS ai_adc_init_adc(AI_ADC* p_adc, u32 factory_calib, 
                                       u8* mem_base_addr);
  extern        void   ai_adc_cleanup_adc(AI_ADC* p_adc);
  extern        STATUS mio_ai_adc_calib_ao(void* param, u16 ch_nr, BOOL bip, 
                                           BOOL range, long* p_data);

#else /* __STDC__ */

  void   ai_adc_clear_adc_irq_status();
  void   ai_adc_enable_adc_irq();
  void   ai_adc_disable_adc_irq();
  void   ai_adc_set_adc_conv_adj_param();
  STATUS ai_adc_init_adc();
  void   ai_adc_cleanup_adc();
  STATUS mio_ai_adc_calib_ao();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCai_adch  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/