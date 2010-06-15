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
** Project: ICP-MULTI Linux driver
** Title:   ICP-MULTI Interrupt Object header
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
000,30sep00,bt   created
*/


#ifndef  INCinterrupth
#define  INCinterrupth

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
** Interrupt object
*********************************************/
typedef enum
{
  interrupt_initialized,
  interrupt_uninitialized //,
//  interrupt_enabled,
//  interrupt_disabled
} INTERRUPT_STATE;


typedef struct _IRQ_SERVICE_RT
{
  void (*service_routine)(void*);
  void* parameter;
  char* identifier;
} IRQ_SERVICE_RT;

typedef struct _MIO_INTERRUPT
{
  INTERRUPT_STATE state;
  struct pci_dev* pci;
  IRQ_SERVICE_RT  routine[NR_OF_IRQ_SOURCES];
} MIO_INTERRUPT;


/*---------------------------------------------------------------------------
** EXTERN DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** EXTERN FUNCTION
**-------------------------------------------------------------------------*/

#if defined(__STDC__) || defined(__cplusplus)

  extern STATUS mio_interrupt_init(MIO_INTERRUPT* p_irq,
                                   struct pci_dev* p_pci,
                                   char*  irq_name);

  extern void   mio_interrupt_cleanup(MIO_INTERRUPT* p_irq);

  extern int    mio_interrupt_register_routine(void* irq_ident, 
                                               void (*srv_routine)(void*), 
                                               void* param, 
                                               char* ident);


  extern int    mio_interrupt_deregister_routine(void* irq_ident,
                                                 void (*srv_routine)(void*),
                                                 char* ident);

  extern int    mio_interrupt_ioctl(struct file* p_file, 
                                    unsigned int cmd, 
                                    unsigned long param, 
                                    MIO_INTERRUPT* p_irq);
#else /* __STDC__ */

  extern STATUS mio_interrupt_init();
  extern void   mio_interrupt_cleanup();
  extern int    mio_interrupt_register_routine();
  extern int    mio_interrupt_deregister_routine();
  extern int    mio_interrupt_ioctl();

#endif /* __STDC__ */


#ifdef __cplusplus
  }
#endif

#endif /* INCinterrupth  */
/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/