/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.27.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI
** Title:   ICP-MULTI Linux driver
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
000,27sep00,bt   created
*/


#ifndef  INCicpMioh
#define  INCicpMioh

#ifdef __cplusplus
  extern "C" {
#endif

/*---------------------------------------------------------------------------
** INCLUDE
**-------------------------------------------------------------------------*/
#include "../h/mio_pci.h"
#include "../h/eeprom.h"
#include "../h/analog_in.h"
#include "../h/analog_out.h"
#include "../h/digital_in.h"
#include "../h/digital_out.h"
#include "../h/counter.h"
#include "../h/interrupt.h"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/
/******************************************** 
** Linux file system object
*********************************************/
typedef enum 
{ 
  registered, 
  unregistered,
  opened
} MIO_FS_STATE;


typedef struct _LINUX_FS_MIO
{
  int                     major_number;
  char                    devName[80];
  struct file_operations  mio_fops;
  MIO_FS_STATE            state;
} LINUX_FS_MIO;

/************************************************
** ICP-MULTI I/O board object
*************************************************/
  
typedef enum 
{ 
  initialized,
  uninitialized 
} MIO_DEV_STATE;


typedef struct _ICP_MIO_STRUCT
{
  MIO_PCI         pci;
  MIO_EEPROM      eeprom;
  MIO_DEV_STATE   state;
  LINUX_FS_MIO    fs;
  MIO_ANALOG_IN   a_i;
  MIO_ANALOG_OUT  a_o;
  MIO_DIGITAL_IN  d_i;
  MIO_DIGITAL_OUT d_o;
  MIO_INTERRUPT   irq;
  MIO_COUNTER     ctr;
} ICP_MIO_STRUCT;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS icp_mio_init(void);    /* init the whole icp_mio list */
  extern void   icp_mio_cleanup(void); /* cleans up the whole icp_mio list */

#else /* __STDC__ */

  STATUS icp_mio_init();
  void   icp_mio_cleanup();

#endif /* __STDC__ */

#ifdef __cplusplus
  }
#endif

#endif /* INCicpMioh  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/