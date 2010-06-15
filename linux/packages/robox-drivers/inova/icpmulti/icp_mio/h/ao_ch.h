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
** Title:   ICP-MULTI analog out channel Object header
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


#ifndef  INCao_chh
#define  INCao_chh

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
/************************************************
** Analog Output channel object
*************************************************/
typedef enum
{
  ao_ch_initialized,
  ao_ch_uninitialized,
  ao_ch_opened,
  ao_ch_to_write
} AO_CH_STATE;

typedef struct _DAC_CALIB
{
  long  cb[4];
  long  cm[4];
  long  offs[4];
  long  range[4];
  long  cbi;
  long  cmi;
  long  offsi;
  long  rangei;
  long  units;
  int   c1;
  int   c2;
  BOOL  bipolar;
} DAC_CALIB;



typedef struct _AO_CHANNEL
{
  AO_CH_STATE            state;
  u16                    dac_reg;
  DAC_CALIB              dac_calib;
  u16                    data;
  int                    nr;               /* channel numbner */
  struct file_operations fops;             /* channel's file ops */
  struct wait_queue*     wq;               /* wait queue for blocking I/O */
  STATUS (*irq_start_func)(void*, int);    /* dac IRQ start routine */
  void*                  irq_start_par;    /* dac IRQ start routine's param */
} AO_CHANNEL;



/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)
  extern STATUS  init_ao_ch(AO_CHANNEL* p_ao_ch, int ch_nr, void* irq_start_par,
                            STATUS (*irq_start_func)(void*, int));
  extern void    cleanup_ao_ch(AO_CHANNEL* p_ao_ch);
  extern int     ao_ch_open(struct file* p_file, AO_CHANNEL* p_ao);

#else /* __STDC__ */

  STATUS init_ao_ch();
  void   cleanup_ao_ch();
  int    ao_ch_open();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCao_chh */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/