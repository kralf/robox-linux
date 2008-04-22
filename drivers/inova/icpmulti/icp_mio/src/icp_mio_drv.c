/*---------------------------------------------------------------------------
** Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
**
** (c) Copyright 1998-2000. All Rights Reserved
**
** Date: Sep.26.2000
**
** Operating System: Linux
**
** Compiler : Gnu C/C++ Compiler 
**
**---------------------------------------------------------------------------
** Project: ICP-MULTI
** Title:   "ICP-MULTI Linux Driver Object" file
**-------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
** DESCRIPTION
**-------------------------------------------------------------------------*/
/* This module contains the driver code of the ICP-MULTI CompactPCI
 * Adapter
 */


 
/*---------------------------------------------------------------------------
** MODIFICATION HISTORY
**-------------------------------------------------------------------------*/
/*
modification history
-------------------- 
000,26sep00,bt   created

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
#include <linux/module.h>
#include <linux/version.h>
char kernel_version [] = UTS_RELEASE;


#include <linux/kernel.h> /* printk()... */
#include <linux/malloc.h> /* kmalloc() */

#include <linux/types.h>  /* size_t */
#include <linux/pci.h>

#include <asm/string.h>

#include <icp_drv_gen.h>
/*
** local module headers
*/
#include "../h/icp_mio.h"
#include "../h/icp_mio.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
MODULE_AUTHOR("Balazs Toth");
MODULE_DESCRIPTION("ICP-MULTI Linux driver");


/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/
static BOOL             mioInitialized = FALSE;


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
  int result;

  if (mioInitialized)
  {
    /*
    ** module already loaded, and initialized
    */
    icpErrorMsg(ICP_MIO_NAME, ICP_MIO_ALREADY_INIT);
    return -EPERM; /* Operation not permitted */
  }

  /* 
  ** The following function call creates the list of the ICP_MULTI devices
  */
  if ( (result = icp_mio_init()) < 0)
  {
#ifdef DEBUG
    printk("<1> icpMulti init error\n");
#endif
    return result; 
  }

  mioInitialized = TRUE;

  /*
  ** register a file in the /proc folder (future)
  */

#ifdef DEBUG
  printk("<1> icpMulti loaded\n");
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
  /*
  ** Clear the list of the ICP_MULTI devices
  */
  icp_mio_cleanup();
  
  mioInitialized = FALSE;

#ifdef DEBUG
  printk("<1> icpMulti cleanup\n");
#endif
}




/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
