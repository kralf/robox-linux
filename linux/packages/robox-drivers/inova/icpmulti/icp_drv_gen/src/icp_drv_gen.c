/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.25.2000
**
** Operating System: Linux Kernel version 2.2.12
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: General CPCI library for the K6-CPU
** Title:   
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the common driver code of the general CPCI library
 */


 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,25sep00,bt   created from the VxWorks code

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

#ifndef MODULE
  #define MODULE
#endif

#define __NO_VERSION__ /* don't define kernel_verion in module.h */
#define EXPORT_SYMTAB
#include <linux/module.h>
#include <linux/version.h>
char kernel_version [] = UTS_RELEASE;


#include <linux/kernel.h> /* printk()... */
#include <linux/malloc.h> /* kmalloc() */

#include <linux/types.h>  /* size_t */
#include <linux/pci.h>

#include <asm/string.h>

#include <icp_drv_gen.h>
#include "../h/icp_drv_gen.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
MODULE_AUTHOR("Balazs Toth");
MODULE_DESCRIPTION("General Inova CPCI Library");

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* icpFindAllPciDev - find all PCI device with the given Sub IDs
*
* This routine finds all PCI devices in the system with the given vendor, 
* device, SubSystem Vendor and sysbsystem device IDs and a create a single 
* linked list of them.
*
* Parameters:
*      unsigned int  vendorId              : vendor ID to find
*      unsigned int  deviceId              : device ID to find
*      u16           subVendorId           : subsystem vendor ID to find
*      u16           subSystemId           : subsystem device ID to find
*      void**        ppListBegin           : address of the first element of the 
*                                           device list
*      size_t        listElementSize       : size of an element of the device list
*      unsigned int* NrOfDevs              : number of founded devices
*      STATUS (*CallBackFunc)(void*)      : callback function to execute device 
*                                           specific initialization
*
* RETURNS: ERROR, in case of an error (Out of memory, No device found)
*          OK   , if everything was OK.
*   
* SEE ALSO: 
******************************************************************************/
STATUS icpFindAllPciDev(unsigned int vendorId, unsigned int deviceId, 
                        u16 subVendorId, u16 subSystemId,
                        void** ppListBegin, size_t listElementSize,
                        unsigned int* NrOfDevs, STATUS (*CallBackFunc)(void*) )
{
  PCI_MIN_STRUCT** ppList;
  STATUS           result;
  unsigned int     nrOfFoundedDev = 0;
  u16              pciSubVendorId;
  u16              pciSubSystemId;
  struct pci_dev*  dev            = NULL;


  /*
  ** Check parameters
  */
  if (listElementSize < sizeof(PCI_MIN_STRUCT) )
  {
    icpErrorMsg(ICP_DRV_GEN_MODNAME, ICP_DRVGEN_BADPARAM);
    return -EINVAL;
 /* Invalid argument */
  }

  /*
  ** Initialize the variables 
  */
  ppList     = (PCI_MIN_STRUCT**)ppListBegin;
  *ppList    = NULL;

  /*
  ** Find all devices with the given vendor and device ID 
  */
  while ( (dev = pci_find_device(vendorId, deviceId, dev)) != NULL)
  {
    /*
    ** Check if the subVendor and SubSystem ID is OK 
    ** ( The return value of the function is not checked, because we can reach
    **   this code only if the pciLibrary already initialized )
    */
    (void)pci_read_config_word(dev, PCI_SUBSYSTEM_VENDOR_ID, &pciSubVendorId);
    if ( pciSubVendorId == subVendorId )
    {
      /*
      ** SubVendorId is OK
      */
      (void)pci_read_config_word(dev, PCI_SUBSYSTEM_ID, &pciSubSystemId);
      if ( pciSubSystemId == subSystemId )
      {
        /*
        ** We've found a device, allocate memory and add a list element
        */
        if ( (*ppList = (PCI_MIN_STRUCT*)kmalloc(listElementSize, GFP_KERNEL)) == NULL )
        {
          icpErrorMsg(ICP_DRV_GEN_MODNAME, ICP_DRVGEN_OUT_OF_MEM);
          return -ENOMEM; /* Out of Memory */
        }
        (*ppList)->pciDev  = dev;
        (*ppList)->next    = NULL;
        /*
        ** Call the callback function - this function can be used by the caller
        ** to arrange device specific initialization, and initialize other 
        ** elements of it's dev specific structure
        */
        if (CallBackFunc != NULL)
        {
          if ( (result = (*CallBackFunc)((void*)(*ppList))) != OK )
          {
            icpErrorMsg(ICP_DRV_GEN_MODNAME, ICP_DRV_GEN_INIT_ERROR);
            /*
            ** Initialization error delete the last list element
            */
            kfree(*ppList);
            *ppList = NULL; /* set the end of the list to NULL */
            return result;
          }
        }
        ppList = (PCI_MIN_STRUCT**)( &((*ppList)->next) );
        nrOfFoundedDev++;
      }
    }
  }
  *NrOfDevs = nrOfFoundedDev;
  return OK;
}


/******************************************************************************
*
* icpFreeListWithCallback - 
*
* This routine deallocates the given device list and calls a cleanup callback 
* function
*
* Parameters:
*      void** ppListBegin                 : address of the first element 
*                                           of the list to free
*      void (*CleanupCallBackFunc)(void*) : callback function to execute 
*                                           device 
specific cleanup
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void icpFreeListWithCallback(void** ppListBegin, 
                             void (*CleanupCallBackFunc)(void*))
{
  PCI_MIN_STRUCT* pList;
  PCI_MIN_STRUCT* pTmp;
  
  if ( ppListBegin == NULL )
  {
    return;
  }
  
  pList = (PCI_MIN_STRUCT*)(*ppListBegin);
  while ( pList != NULL )
  {
    /*
    ** call cleanup callback function
    */
    if (CleanupCallBackFunc != NULL)
    {
      CleanupCallBackFunc((void*)pList);
    }
    pTmp = pList->next;
    kfree(pList);
    pList = pTmp;
  }
}

/******************************************************************************
*
* icpFreeList - 
*
* This routine deallocates the given device list
*
* Parameters:
*      void** ppListBegin : address of the first element of the list to free
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void icpFreeList(void** ppListBegin)
{
  PCI_MIN_STRUCT* pList;
  PCI_MIN_STRUCT* pTmp;
  
  if ( ppListBegin == NULL )
  {
    return;
  }
  
  pList = (PCI_MIN_STRUCT*)(*ppListBegin);
  while ( pList != NULL )
  {
    pTmp = pList->next;
    kfree(pList);
    pList = pTmp;
  }
}


/******************************************************************************
*
* icpGetNthDev - get the Nth element of a device list
*
* This routine gets the Nth element of the given device list
*
* Parameters:
*     void* pListBegin: pointer to the first element of the list 
*     int   nthElement: which element to get
*
* RETURNS: pointer to the Nth list element
*   
* SEE ALSO: 
******************************************************************************/
void* icpGetNthDev(void* pListBegin, int nthElement)
{
  PCI_MIN_STRUCT* pList = (PCI_MIN_STRUCT*)pListBegin;

  for( ;( (nthElement > 0) && (pList != NULL) ); nthElement--)
  {
    pList = pList->next;
  }
  return (void*)pList;
}


/******************************************************************************
*
* icpErrorMsg - error message output
*
* This routine prints an error message
*
* NOTE: 
*    The ICP_ERRORLOG macro defines how to output the error string, at 
*    the moment ICP_ERRORLOG = printf();
*
* Parameters:
*           char* modName: Module name of the module caused the error 
*     const char* pErrStr: error string (printf format string)
*                 ...    : error string parameters
*
* RETURNS: -
*   
* SEE ALSO: 
******************************************************************************/
void icpErrorMsg(char* modName, const char* pErrStr, ... )
{ 
  char       buf1[ICP_ERRMSG_MAX_LENGTH];
  char       errorBuff[ICP_ERRMSG_MAX_LENGTH + ICP_MODNAME_MAX_LENGTH + 8];
  va_list    args;

  va_start( args, pErrStr );


  /*
  ** format the text
  */
  vsprintf(buf1, pErrStr, args);

  /*
  ** Build the error message -> MODNAME: Error message
  */
  if ( strlen(modName) >= ICP_MODNAME_MAX_LENGTH )
  {
    modName[ICP_MODNAME_MAX_LENGTH - 1] = 0;
  }
  sprintf(errorBuff, "%s: %s\n", modName, buf1 );

  /*
  ** Display the Error message
  */
  ICP_ERRORLOG(errorBuff);
}

/******************************************************************************
**
** Export the Functions of the module to the Kernel space
**
******************************************************************************/
#if 1
EXPORT_SYMBOL_NOVERS(icpErrorMsg);
EXPORT_SYMBOL_NOVERS(icpGetNthDev);
EXPORT_SYMBOL_NOVERS(icpFreeList);
EXPORT_SYMBOL_NOVERS(icpFindAllPciDev);
EXPORT_SYMBOL_NOVERS(icpFreeListWithCallback);
#endif

#ifdef MODULE
/******************************************************************************
*
* init_module - module init func 
*
* This routine is a dummy function needed by the Linux Kernel
*
* Parameters:
*
* RETURNS: always 0
*   
* SEE ALSO: 
******************************************************************************/
int init_module(void)
{
#ifdef DEBUG
  printk("<1> icpDrvGen loaded\n");
#endif
  return 0;
}

/******************************************************************************
*
* cleanup_module - module cleanup func 
*
* This routine is a dummy function needed by the Linux Kernel
*
* Parameters:
*
* RETURNS: 
*   
* SEE ALSO: 
******************************************************************************/
void cleanup_module(void)
{
#ifdef DEBUG
  printk("<1> icpDrvGen cleanup\n");
#endif

}
#endif












#if 0
/*******************************************************************************
*
* icpDelay - wait function 
*
* This routine waits for the by cycle given time (1 cycle is about 720ns)
*
* Parameters:
*           int cyclye: Nr of wait cycles
*
* RETURNS: -
******************************************************************************/
void icpDelay(uint32_t cycle)
{
  while(cycle--) 
  {
    sysDelay();
  }
}
#endif


/*---------------------------------------------------------------------------
** END of FILE
**-------------------------------------------------------------------------*/
