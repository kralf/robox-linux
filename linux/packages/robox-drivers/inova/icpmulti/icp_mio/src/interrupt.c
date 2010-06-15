/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Oct.2.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI Linux Driver
** Title:   Interrupt Object
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI card
 */


 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,02Oct00,bt   created

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

#include <linux/kernel.h> /* printk()... */
#include <linux/fs.h>
#include <linux/types.h>  /* size_t */
#include <linux/pci.h>
#include <linux/sched.h>


#include <icp_carr_def.h>
#include <icp_drv_gen.h>

/*
** local module headers
*/
#include "../h/interrupt.h"
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


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static void set_gen_interrupt_params(struct pci_dev* p_pci);
static void enable_gen_interrupt(struct pci_dev* p_pci);
static void disable_gen_interrupt(struct pci_dev* p_pci);
static void gen_interrupt_handler(int irq, void* param, 
                                  struct pt_regs* p_regs);

/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* set_gen_interrupt_params - short descr
*
* Description
*
* Parameters:
*         
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
static void set_gen_interrupt_params(struct pci_dev* p_pci)
{
  u32 pciConfigValue;

  /*
  ** Set the AUX_INT_POL irq polarity bit to low active
  */
  (void)pci_read_config_dword(p_pci, ICP_CARR_LOCAL_BUS_CTRL_REG, 
                              &pciConfigValue);
  pciConfigValue &= ~ICP_CARR_AUX_INT_POL;  
 (void)pci_write_config_dword(p_pci, ICP_CARR_LOCAL_BUS_CTRL_REG,
                              pciConfigValue);

  /*
  **  Set the AUX_INT_STATUS back, if there would be any
  */
  (void)pci_read_config_dword(p_pci, ICP_CARR_IRQSTATUS, &pciConfigValue);
  
  (void)pci_write_config_dword(p_pci, ICP_CARR_IRQSTATUS, pciConfigValue);
}
/******************************************************************************
*
* enable_gen_interrupt - enable the general interrupt for a mio board
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static void enable_gen_interrupt(struct pci_dev* p_pci)
{
  u32         pciConfigValue;

 (void)pci_read_config_dword(p_pci, ICP_CARR_IRQENABLE, &pciConfigValue);

  if ( pciConfigValue & ICP_CARR_IRQ_ON)
  {
    /*
    ** it is enabled 
    */
    return;
  }
  else
  {
    pciConfigValue |= ICP_CARR_IRQ_ON;
    (void)pci_write_config_dword(p_pci, ICP_CARR_IRQENABLE, pciConfigValue);
  }
}

/******************************************************************************
*
* disable_gen_interrupt - disable the general interrupt for a mio board
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static void disable_gen_interrupt(struct pci_dev* p_pci)
{
  u32         pciConfigValue;

 (void)pci_read_config_dword(p_pci, ICP_CARR_IRQENABLE, &pciConfigValue);

  if ( !(pciConfigValue & ICP_CARR_IRQ_ON) )
  {
    /*
    ** it is disabled 
    */
    return;
  }
  else
  {
    pciConfigValue &= ~ICP_CARR_IRQ_ON;
    (void)pci_write_config_dword(p_pci, ICP_CARR_IRQENABLE, pciConfigValue);
  }
}

/******************************************************************************
*
* gen_interrupt_handler - general interrupt handler for a mio board
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
static void gen_interrupt_handler(int irq, void* param, struct pt_regs* p_regs)
{
  MIO_INTERRUPT*  p_irq = (MIO_INTERRUPT*)param;
  struct pci_dev* p_pci = p_irq->pci;
  int             i;
  u32             pciConfigValue;
  
  /*
  ** Check if this board caused the Irq
  */
  (void)pci_read_config_dword(p_pci, ICP_CARR_IRQSTATUS, &pciConfigValue);

#ifdef INOVA_DEBUG
  LOGMSG("Carr General Irq for %x nesting count:%d\n", (unsigned int)p_pci->irq, intCount());
#endif INOVA_DEBUG

  if ( (pciConfigValue & ICP_CARR_AUX_INT) == 0)
  {
    /*
    ** we have nothing to do with the IRQ
    */
#ifdef INOVA_DEBUG
  LOGMSG("Carr General Irq for %x, not from the carrier board\n", (unsigned int)p_pci->irq);
#endif INOVA_DEBUG
    return;
  }
  /* 
  ** I'm the interrupter, call the registered Irq routines
  */

#ifdef DEBUG_INTR
  printk("mio irq\n");
#endif


#ifdef INOVA_DEBUG
/*  LOGMSG("Carr General Irq for %x irq from carrier board\n", (unsigned int)pCarr->irqNr);*/
#endif INOVA_DEBUG

  for(i = 0; i < NR_OF_IRQ_SOURCES; i++)
  {
    if (p_irq->routine[i].service_routine != NULL)
    {
      /*
      ** there is a registered routine, call it
      */
#ifdef INOVA_DEBUG
     LOGMSG("General Irq call %i. irq\n", i);
#endif INOVA_DEBUG
     (*(p_irq->routine[i].service_routine))(p_irq->routine[i].parameter);
    }
  }
  /*
  ** General interrupt acknowledge on the board
  */
#ifdef INOVA_DEBUG
/*  LOGMSG("Carr General Irq for %x acknowledge irq\n", (unsigned int)pCarr->irqNr);*/
#endif INOVA_DEBUG

  (void)pci_read_config_dword(p_pci, ICP_CARR_IRQSTATUS, &pciConfigValue);
  (void)pci_write_config_dword(p_pci, ICP_CARR_IRQSTATUS, pciConfigValue);

#ifdef INOVA_DEBUG
/*  LOGMSG("Carr General Irq for %x exit\n", (unsigned int)pCarr->irqNr); */
#endif INOVA_DEBUG
  return;
}
/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_interrupt_init - Initialization of the interrupt object
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
STATUS mio_interrupt_init(MIO_INTERRUPT* p_irq, struct pci_dev* p_pci,
                          char* irq_name)
{
  int             result = OK;
  int             i;
 
  /*
  ** Init the MIO_INTERRUPT structure
  */
  p_irq->state                = interrupt_uninitialized;
  p_irq->pci                  = p_pci;
  
  for(i = 0; i < NR_OF_IRQ_SOURCES; i++)
  {
    p_irq->routine[i].service_routine = NULL;
    p_irq->routine[i].parameter       = NULL;
    p_irq->routine[i].identifier      = NULL;
  }
  /*
  ** install the handler
  */
  result = request_irq(p_pci->irq, gen_interrupt_handler, 
                       SA_INTERRUPT | SA_SHIRQ ,
                       irq_name, (void*)p_irq);
  if (result)
  {
    /*
    ** there was an error
    */
    icpErrorMsg(ICP_MIO_NAME, ICP_GEN_IRQ_REG_ERR);
    return result;
  }
  /*
  ** configure the pci params (for the irq)
  */
  disable_gen_interrupt(p_pci);
  set_gen_interrupt_params(p_pci);
  enable_gen_interrupt(p_pci);
  p_irq->state = interrupt_initialized;

#ifdef DEBUG_INTR
  printk("<1> interrupt init OK\n");
#endif

  return result;
}

/******************************************************************************
*
* mio_interrupt_cleanup - Cleanup of the Interrupt object
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
void mio_interrupt_cleanup(MIO_INTERRUPT* p_irq)
{
  int    i;
  struct pci_dev* p_pci    = p_irq->pci;
  
  /*
  ** deconfigure the pci params
  */
  disable_gen_interrupt(p_pci);

  /*
  ** deinstall the handler
  */
  free_irq(p_pci->irq, (void*)p_irq);

  /*
  ** cleanup the MIO_INTERRUPT structure
  */
  for(i = 0; i < NR_OF_IRQ_SOURCES; i++)
  {
    p_irq->routine[i].service_routine = NULL;
    p_irq->routine[i].parameter       = NULL;
    p_irq->routine[i].identifier      = NULL;
  }
  p_irq->state = interrupt_uninitialized;

#ifdef DEBUG_INTR
  printk("<1> interrupt deinit OK\n");
#endif

}

/******************************************************************************
*
* mio_interrupt_register_routine - register an irq service routine
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
int mio_interrupt_register_routine(void* irq_ident, void (*srv_routine)(void*), 
                                   void* param, char* ident)
{
  MIO_INTERRUPT* p_irq = (MIO_INTERRUPT*)irq_ident;
  int            i;
  int            ret_val = -ENXIO; /* No such device or address, ret_val if 
                                   ** there is no free irq "slot" */

  /*
  ** find the next free irq routine "slot"
  */
  for(i = 0; i < NR_OF_IRQ_SOURCES; i++)
  {
    if (p_irq->routine[i].service_routine == NULL)
    {
      /*
      ** we've found it, register
      */
      p_irq->routine[i].service_routine = srv_routine;
      p_irq->routine[i].parameter       = param;
      p_irq->routine[i].identifier      = ident;
      ret_val = OK;  
      break;
    }
  }
  return ret_val;
}

/******************************************************************************
*
* mio_interrupt_deregister_routine - deregister an irq service routine
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
int mio_interrupt_deregister_routine(void* irq_ident, 
                                     void (*srv_routine)(void*), char* ident)
{
  MIO_INTERRUPT* p_irq = (MIO_INTERRUPT*)irq_ident;
  int            i;
  int            ret_val = -ENXIO; /* No such device or address, ret_val if 
                                   ** there is no such irq "slot" */

  /*
  ** find the irq routine "slot"
  */
  for(i = 0; i < NR_OF_IRQ_SOURCES; i++)
  {
    if ( (p_irq->routine[i].service_routine == srv_routine) &&
         (p_irq->routine[i].identifier      == ident) )
    {
      /*
      ** we've found it, deregister
      */
      p_irq->routine[i].service_routine = NULL;
      p_irq->routine[i].parameter       = NULL;
      p_irq->routine[i].identifier      = NULL;
      ret_val = OK;  
      break;
    }
  }
  return ret_val;
}

/******************************************************************************
*
* mio_interrupt_ioctl - ioctl routine for the interrupt object
*
* Description
*
* Parameters:
*         
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/  
int mio_interrupt_ioctl(struct file* p_file, unsigned int cmd, 
                        unsigned long arg, MIO_INTERRUPT* p_irq)
{
  struct pci_dev* p_pci   = p_irq->pci;
  int             ret_val = OK;

  switch(cmd)
  {
    case MIO_IOCTL_IRQ_GEN_ENABLE:
    {
      if (arg)
      {
        enable_gen_interrupt(p_pci);
      }
      else
      {
        disable_gen_interrupt(p_pci);
      }
      ret_val = OK;
      break;
    }
    default:
    {
      ret_val = ICP_MIO_BAD_IOCTL;
    }
  }
  return ret_val;
}

/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
