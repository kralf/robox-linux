/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * CPCI-100A linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: cpci100a.c,v 1.3 2003/11/26 07:55:31 fred Exp $
 *-------------------------------------------------------------*/

/** \file cpci100a.c
 * \brief cpci100a (IP-PCI) device driver (kernel module, CURRENTLY NOT USED)
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/errno.h>

#ifndef CONFIG_PCI
#error "This driver needs PCI support to be available"
#endif

#define CPCI100A_VENDORID 0x124b
#define CPCI100A_DEVICEID 0x40

#define CPCI100A_NAME "cpci100a"

#define CPCI100A_BAR 0

#define CPCI100A_BAR2 0x18

MODULE_AUTHOR ("Frederic Pont");
MODULE_DESCRIPTION ("Linux driver for CPCI-100A");
MODULE_LICENSE ("EPFL");


int cpci100a_init(void)
{

  struct pci_dev *dev;
  unsigned int ip_iobase = 0;
  
  printk(KERN_ALERT "%s: init useless driver\n", CPCI100A_NAME);
  dev = pci_find_device(CPCI100A_VENDORID,CPCI100A_DEVICEID,NULL);
  
  if (!dev) {
    printk(KERN_ALERT "%s: device not found\n", CPCI100A_NAME);
    return -ENODEV;
  }
  
  if (pci_read_config_dword(dev,CPCI100A_BAR2,&ip_iobase)) {
    printk(KERN_ALERT "%s: could not find IP io base address\n", 
	   CPCI100A_NAME);
    return -ENODEV;
    
  }
  
  if (pci_enable_device(dev))
    return -ENODEV;


  printk(KERN_ALERT "%s: IP iobase: 0x%8x\n", CPCI100A_NAME,ip_iobase);
    
  return 0;
    
  
}


void cpci100a_exit(void)
{
  
  printk(KERN_ALERT "%s: exit driver\n", CPCI100A_NAME);


}



module_init(cpci100a_init);
module_exit(cpci100a_exit);
