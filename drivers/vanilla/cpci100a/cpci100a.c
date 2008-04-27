/*-------------------------------------------------------------
 * Linux on RoboX project - ASL EPFL
 *-------------------------------------------------------------
 *
 * CPCI-100A linux driver for kernel 2.6
 *
 * modifications by Ralf Kaestner
 *
 *-------------------------------------------------------------
 * $Id: cpci100a.c,v 1.3 2003/11/26 07:55:31 fred Exp $
 *-------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/errno.h>

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// IDs and names
#define CPCI100A_VENDORID       0x124b
#define CPCI100A_DEVICEID       0x40
#define CPCI100A_NAME           "Greenspan SBS cPCI-100A carrier board"
#define CPCI100A_DRVNAME        "cpci100a"

#define CPCI100A_BAR 0
#define CPCI100A_BAR2 0x18

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define cpci100a_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", CPCI100A_DRVNAME, message); \
}
#define cpci100a_alertk(fmt, arg...) \
  cpci100a_printk(KERN_ALERT fmt, ## arg)
#define cpci100a_debugk(fmt, arg...) \
  if (debug) cpci100a_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR ("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION ("Linux driver for Greenspan SBS cPCI-100A carrier board");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

static struct pci_device_id cpci100a_pci_table[] = {
  { PCI_DEVICE(CPCI100A_VENDORID, CPCI100A_DEVICEID) },
  { 0, } // Terminating entry
};
MODULE_DEVICE_TABLE(pci, cpci100a_pci_table);

int cpci100a_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
void cpci100a_remove(struct pci_dev *pdev);

static struct pci_driver cpci100a_driver = {
  .name = CPCI100A_DRVNAME,
  .owner = THIS_MODULE,
  .id_table = cpci100a_pci_table,
  .probe = cpci100a_probe,
  .remove = cpci100a_remove
};

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

int cpci100a_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
  int result = -ENODEV;

  if ((pdev->vendor == CPCI100A_VENDORID) &&
    (pdev->device == CPCI100A_DEVICEID)) {
    cpci100a_printk("%s detected\n", CPCI100A_NAME);

    result = pci_enable_device(pdev);

    if (result) cpci100a_alertk("Failed to enable device\n");
  }

  return result;
}

void cpci100a_remove(struct pci_dev *pdev) {
  cpci100a_printk("%s removed\n", CPCI100A_NAME);
}

static int __init cpci100a_init(void) {
  // find devices and register driver
  return pci_module_init(&cpci100a_driver);
}

static void __exit cpci100a_exit(void) {
  // unregister driver
  pci_unregister_driver(&cpci100a_driver);
}

module_init(cpci100a_init);
module_exit(cpci100a_exit);
