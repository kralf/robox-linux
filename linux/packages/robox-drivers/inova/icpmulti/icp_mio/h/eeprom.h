/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.3.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI EEPROM Object header
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
000,03oct00,bt   created
*/


#ifndef  INCeepromh
#define  INCeepromh

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


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
/******************************************** 
** EEPROM object
*********************************************/
typedef enum
{
  eeprom_initialized,
  eeprom_uninitialized
} EEPROM_STATE;


typedef struct _MIO_EEPROM
{
  struct pci_dev* pci;
  u32             fact_calib;
  EEPROM_STATE    state;
} MIO_EEPROM;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_eeprom_init(MIO_EEPROM* p_eeprom, struct pci_dev* p_pci);

  extern void   mio_eeprom_cleanup(MIO_EEPROM* p_eeprom);

  extern int    mio_eeprom_ioctl(struct file* p_file, 
                                 unsigned int cmd, 
                                 unsigned long param, 
                                 MIO_EEPROM* p_eeprom);
#else /* __STDC__ */

  STATUS mio_eeprom_init();
  void   mio_eeprom_cleanup();
  int    mio_eeprom_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCeepromh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/