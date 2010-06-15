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
** Title:   PCI Object
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
#include <linux/delay.h>
#include <asm/io.h>
#include <icp_carr_def.h>
#include <icp_drv_gen.h>

/*
** local module headers
*/
#include "../h/icp_mio_hw.h"
#include "../h/mio_pci.h"
#include "../h/icp_mio.msg"

/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DEFINE
**-------------------------------------------------------------------------*/
#define ICP_CARR_RESET_TIME            0x5

/*---------------------------------------------------------------------------
** TYPEDEF
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** DATA
**-------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
** FORWARD DECLARATIONS
**-------------------------------------------------------------------------*/
static void board_reset(struct pci_dev* p_pci);

/*---------------------------------------------------------------------------
** STATIC FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* board_reset - short descr
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
static void board_reset(struct pci_dev* p_pci)
{
  u32 pciConfigValue;

  (void)pci_read_config_dword(p_pci, ICP_CARR_LOCAL_BUS_CTRL_REG,
                              &pciConfigValue);

  pciConfigValue |= ICP_CARR_RESET;
  (void)pci_write_config_dword(p_pci, ICP_CARR_LOCAL_BUS_CTRL_REG,
                               pciConfigValue);
  /*
  ** Wait ICP_CARR_RESET_TIME is given in miliseconds
  */
  udelay(ICP_CARR_RESET_TIME*1000);

  pciConfigValue &= ~ICP_CARR_RESET;
  (void)pci_write_config_dword(p_pci, ICP_CARR_LOCAL_BUS_CTRL_REG,
                         pciConfigValue);
}

/*---------------------------------------------------------------------------
** FUNCTION
**-------------------------------------------------------------------------*/
/******************************************************************************
*
* mio_pci_init - Initialization of the PCI params
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
STATUS mio_pci_init(MIO_PCI* p_pci, void** mapped_addr)
{
  struct pci_dev* pPci = p_pci->pci_min.pciDev;
  u32             pciConfigValue;
  u16             pciConfigWord;

  /*
  ** Configure the PCI parameters of the founded MIO_DEVICE
  **
  ** 1. reset the board
  */
  board_reset(pPci);
  /*
  ** 2. enable memory space
  */
  (void)pci_read_config_word(pPci, PCI_COMMAND, &pciConfigWord);

  pciConfigWord |= PCI_COMMAND_MEMORY;
  (void)pci_write_config_word(pPci, PCI_COMMAND, pciConfigWord);
  /*
  ** 3. Set the data access width + wait states
  **
  ** clear the wait states & data width fields
  */
  (void)pci_read_config_dword(pPci, ICP_CARR_LOCAL_BUS_CTRL_REG, 
                              &pciConfigValue);

  pciConfigValue &= ~(ICP_CARR_SOCK1WAITSATES(0x0F) | 
                      ICP_CARR_SOCK1DATAWIDTH(1));
  /*
  ** set the new values
  */
  pciConfigValue |= ICP_CARR_SOCK1DATAWIDTH(ICP_MIO_ACCESS_WIDTH);
  pciConfigValue |= ICP_CARR_SOCK1WAITSATES(ICP_MIO_WAIT_STATES);

  (void)pci_write_config_dword(pPci, ICP_CARR_LOCAL_BUS_CTRL_REG, 
                               pciConfigValue);

  /*
  ** remap the pci memory base address[2] 64K
  ** We can directly access only the 640k-1MB area, 
  ** so anything else has to be remapped.
  */
  p_pci->mapped_addr = ioremap(pPci->base_address[2], 
                               ICP_MIO_PCI_MEM_WINDOW_SIZE);
  if (p_pci->mapped_addr == NULL)
  {
    *mapped_addr = NULL;
    return -ENOMEM;
  }
  *mapped_addr = p_pci->mapped_addr;
  return OK;
}

/******************************************************************************
*
* mio_pci_cleanup - Cleanup of the PCI params
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
void mio_pci_cleanup(MIO_PCI* p_pci)
{
  iounmap(p_pci->mapped_addr);
}


/*---------------------------------------------------------------------------
** End of File
**-------------------------------------------------------------------------*/
