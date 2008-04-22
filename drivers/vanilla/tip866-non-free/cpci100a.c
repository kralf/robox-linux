/* $Id: carrier_sbs_pci.c 110 2007-09-18 13:52:57Z Hesse $" */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   CARRIER_SBS_PCI   @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**    File             carrier_sbs_pci.c                                     **
**                                                                           **
**    Function         Driver source for SBS PCI carrier boards              **
**                                                                           **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES                                     **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                                                                           **
**                     Tel.: +49(0)4101/4058-0                               **
**                     Fax.: +49(0)4101/4058-19                              **
**                     e-mail: support@tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2006-2007                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2007-09-18 15:52:57 +0200 (Di, 18 Sep 2007) $   $Rev: 110 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#undef DEBUG_CARRIER
#define DEBUG_NAME          "SBS PCI  : "

#define DRIVER_NAME         "TEWS TECHNOLOGIES - SBS (Compact)PCI IPAC Carrier"

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <asm/system.h>
#include <asm/io.h>

#include "tpmodule.c"
#include "ipac.h"
#include "cpci100a.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif


#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("SBS (Compact)PCI IPAC Carrier Driver");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS Technologies <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


struct carrier_board_info {

    struct list_head  node;
        /* used to manage the list of known carrier boards           */

    struct pci_dev  *dev;
        /* pointer to the attached pci device object                 */

    char  *ip_space;
        /* pointer to the carrier board CNTRL, IO, ID and INT spaces */

    char  *local_control;
        /* pointer to the local configuration space (PLX register)   */

    unsigned long saved_intcsr;

    struct resource *bar[SxPCI_NUM_BAR];
        /* allocated resources by this driver                        */

    struct carrier_slot  slot[SxPCI_MAX_SLOT];
};


static unsigned short ipac_clk[] = {
    SxPCI_CLK32A,
    SxPCI_CLK32B,
    SxPCI_CLK32C,
    SxPCI_CLK32D
};

extern PciDeviceStruct rejectedPciDevices[MAX_REJECT_PCI_DEVICES];

static struct list_head carrier_board_root;

static struct carrier_driver carrier_car_driver;

static void cleanup_device(struct carrier_board_info *info);
static int info_sanity_check(struct carrier_board_info *info);




/*****************************************************************************
 *
 *  info_sanity_check - perform a sanity check of the specified info pointer
 *
 *  Returns 0 if pointer is okay, otherwise -1
 *****************************************************************************/
static int info_sanity_check(struct carrier_board_info *info)
{
    struct list_head  *ptr;


    if (info == NULL) return -1;

    list_for_each(ptr, &carrier_board_root) {
        if (info == list_entry(ptr, struct carrier_board_info, node)) return 0;
    }

    return -1;  /* unknown info pointer */
}


/*****************************************************************************
 *  in_use      announce specified slot is used by an IPAC port driver
 *
 *  This function will be called by the carrier class
 *  driver every time an IPAC module was allocated by an
 *  IPAC port driver. This function must increment the
 *  module "used count" and whatever necessary to do when
 *  an IPAC module was allocated. Every carrier port driver
 *  must export this function.
 *****************************************************************************/
static void in_use(struct carrier_slot  *slot)

{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    try_module_get(THIS_MODULE);
#else
    MOD_INC_USE_COUNT;
#endif
}


/*****************************************************************************
 *  out_of_use      announce specified slot is no longer used
 *
 *  This function will be called by the carrier class
 *  driver every time an IPAC module was no longer used by
 *  an IPAC port driver. This function must decrement the
 *  module "used count" and whatever necessary to do when
 *  an IPAC module was no longer allocated. Every carrier
 *  port driver must export this function.
 *****************************************************************************/
static void out_of_use(struct carrier_slot  *slot)

{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    module_put(THIS_MODULE);
#else
    MOD_DEC_USE_COUNT;
#endif
}


/*****************************************************************************
 *  enable_module_intr       enable specified IPAC interrupt line
 *
 *  Not supported for single slots, returns always -1
 *****************************************************************************/
static int enable_module_intr(struct carrier_slot *slot, int intr_line)
{
    return -1;
}


/*****************************************************************************
 *  disable_module_intr       disable specified IPAC interrupt line
 *
 *  Not supported for single slots, returns always -1
 *****************************************************************************/
static int disable_module_intr(struct carrier_slot *slot, int intr_line)
{
    return -1;
}


/*****************************************************************************
 *  setup_intr_detection        configure level or edge sensitive interrupts
 *
 *  Not supported returns always -1
 *****************************************************************************/
static int setup_intr_detection (struct carrier_slot *slot, int intr_line, int sensitive)
{
    return -1;
}


/*****************************************************************************
 *  setup_clock_rate        setup new IPAC clock rate (8 MHz or 32 MHz)
 *
 *  Returns on success or -1 on failure
 *****************************************************************************/
static int setup_clock_rate (struct carrier_slot *slot, int clock_rate)
{
    struct carrier_board_info  *info = (struct carrier_board_info *)slot->private_data;
    unsigned char value;

    /*  first check if the info pointer is valid */
    if (info_sanity_check(info) == -1) return -1;

    value = readb((unsigned char*)(info->ip_space + SxPCI_CNTL0_REG));


    if (clock_rate & IPAC_CLK_32MHZ)
        value |= ipac_clk[slot->slot_index];
    else
        value &= ~ipac_clk[slot->slot_index];

    writeb(value, (unsigned char*)(info->ip_space + SxPCI_CNTL0_REG));

    return 0;
}


/*****************************************************************************
 *  enable_error_intr        enable error interrupt
 *
 *  Not supported returns always -1
 *****************************************************************************/
static int enable_error_intr (struct carrier_slot *slot)
{
    return -1;
}


/*****************************************************************************
 *  disable_error_intr        disable error interrupt
 *
 *  Not supported returns always -1
 *****************************************************************************/
static int disable_error_intr (struct carrier_slot *slot)
{
    return -1;
}


/*****************************************************************************
 *  check_error_status        check and clear error status
 *
 *  Not supported returns always 0
 *****************************************************************************/
static int check_error_status (struct carrier_slot *slot)
{
    return 0;   /* can't check, assume no error is pending */
}


/*****************************************************************************
 *  enable_timeout_intr        enable timeout interrupt
 *
 *  Not supported returns always -1
 *****************************************************************************/
static int enable_timeout_intr (struct carrier_slot *slot)
{
    return -1;
}


/*****************************************************************************
 *  disable_timeout_intr        disable timeout interrupt
 *
 *  Not supported returns always -1
 *****************************************************************************/
static int disable_timeout_intr (struct carrier_slot *slot)
{
    return -1;
}


/*****************************************************************************
 *  check_timeout_status        check and clear timeout status
 *
 *  Returns 1 if a timeout has occurred or 0 if not
 *****************************************************************************/
static int check_timeout_status (struct carrier_slot *slot)
{
    return 0;  /* can't check access, assume it was okay */
}


/*****************************************************************************
 *  interrupt_ack      acknowledge a pending interrupt request
 *
 *  This function clear an edge sensitive interrupt request and performs an
 *  IACK cycle for the specified interrupt line (0/1)
 *
 *  Returns the interrupt vector or -1 on error
 *****************************************************************************/
static int interrupt_ack (struct carrier_slot *slot, int intr_line)
{
    struct carrier_board_info  *info = (struct carrier_board_info *)slot->private_data;


	/* because this function is only called in interrupt context we did not */
	/* perform a sanity check (drops performence). The caller should known  */
	/* what he does                                                         */
    switch (intr_line) {
    case 0:
    	readb((unsigned char*)(info->ip_space + SxPCI_INT_SPACE_OFF + SxPCI_INT_SPACE_GAP * slot->slot_index));
    	return readb((unsigned char*)(info->ip_space + SxPCI_INT_SPACE_OFF + SxPCI_INT_SPACE_GAP * slot->slot_index));
		break;

	case 1:
    	readb((unsigned char*)(info->ip_space + 2 + SxPCI_INT_SPACE_OFF + SxPCI_INT_SPACE_GAP * slot->slot_index));
    	return readb((unsigned char*)(info->ip_space + 2 + SxPCI_INT_SPACE_OFF + SxPCI_INT_SPACE_GAP * slot->slot_index));
		break;

	default:
		return -1;
	}
}


/*****************************************************************************
 *  reset_carrier      perform a reset of the carrier board (all slots)
 *
 *  Not supported return always -1
 *****************************************************************************/
static int reset_carrier (struct carrier_slot *slot)
{
    return -1;
}


/*****************************************************************************
 *  reset_slot      perform a reset of the specified slot
 *
 *  Not supported return always -1
 *****************************************************************************/
static int reset_slot (struct carrier_slot *slot)
{
    return -1;
}




/*****************************************************************************
 *  carrier_request_irq      install an ISR for a specified interrupt vector number
 *
 *  This function returns 0 on success or -1 on error
 *****************************************************************************/
static int carrier_request_irq(struct carrier_slot *slot, unsigned int irq, int_handler_t (*handler), unsigned long flags, const char *dev_name, void *dev_id)
{
	/*return request_irq(slot->system_interrupt_vector, handler, flags, dev_name, dev_id);*/
	return tpmodule_request_irq(slot->system_interrupt_vector, handler, dev_name, dev_id);
}




/*****************************************************************************
 *  carrier_free_irq         remove an ISR from the IVT
 *
 *
 *****************************************************************************/
static void carrier_free_irq(struct carrier_slot *slot, unsigned int irq, void *dev_id)
{
	free_irq(slot->system_interrupt_vector, dev_id);
}




/*****************************************************************************
 *  read_uchar,         read from the specified IPAC space
 *  read_ushort,
 *  read_ulong
 *
 *  All raed operations are relative to the base address of the specified space.
 *  By using the kernel functions read..() the access is hardware independent.
 *****************************************************************************/
static unsigned char read_uchar (struct addr_space_desc *space,  unsigned long offset)
{
    return readb((unsigned char*)((unsigned char*)space->virtual_address + offset));
}

static unsigned short read_ushort (struct addr_space_desc *space,  unsigned long offset)
{
    return readw((unsigned short*)((unsigned char*)space->virtual_address + offset));
}

static unsigned long read_ulong (struct addr_space_desc *space,  unsigned long offset)
{
    return readl((unsigned long*)((unsigned char*)space->virtual_address + offset));
}


/*****************************************************************************
 *  write_uchar,         write a value to the specified IPAC space
 *  write_ushort,
 *  write_ulong
 *
 *  All write operations are relative to the base address of the specified space.
 *  By using the kernel functions write..() the access is hardware independent.
 *****************************************************************************/
static void write_uchar (struct addr_space_desc *space,  unsigned long offset, unsigned char value)
{
    writeb(value, (unsigned char*)((unsigned char*)space->virtual_address + offset));
}

static void write_ushort (struct addr_space_desc *space,  unsigned long offset, unsigned short value)
{
    writew(value, (unsigned short*)((unsigned char*)space->virtual_address + offset));
}

static void write_ulong (struct addr_space_desc *space,  unsigned long offset, unsigned long value)
{
    writel(value, (unsigned long*)((unsigned char*)space->virtual_address + offset));
}






/*****************************************************************************
 *
 *  carrier_init_one - initialize specified carrier board device
 *
 *
 *
 *****************************************************************************/
static int carrier_init_one(struct pci_dev *dev, const struct pci_device_id  *id)
{
    int  i;
    struct carrier_board_info  *info;
    unsigned long  ipac_base;



#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Probe new device (vendor=0x%04X, device=0x%04X, #slots=%ld)\n",
        DEBUG_NAME, id->vendor, id->device, id->driver_data);
#endif

    /*
    ** Check if the found device is listed in the reject-list.
    ** If so, don't init this device.
    */
    for (i=0; i<MAX_REJECT_PCI_DEVICES; i++)
    {
        if ( (dev->bus->number == rejectedPciDevices[i].busNo) &&
             (PCI_SLOT(dev->devfn) == rejectedPciDevices[i].devNo) &&
             (PCI_FUNC(dev->devfn) == rejectedPciDevices[i].funcNo) )
        {
            printk(KERN_WARNING "%s carrier board rejected (bus=0x%.2X, dev=0x%.2X, func=0x%.2X)\n", DEBUG_NAME,
                dev->bus->number, PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn));
            return -1;
        }
    }


    /* allocate kernel memory for carrier board info structure */
    if (!(info = kmalloc(sizeof(struct carrier_board_info), GFP_KERNEL))) {
        printk(KERN_WARNING "\n%s unable to allocate memory for carrier board info\n", DEBUG_NAME);
        return -1;
    }

    memset(info, 0, sizeof(struct carrier_board_info));

    /* add info structure to the list of known carrier board */
    list_add_tail(&info->node, &carrier_board_root);


    /* try to occupy memory resource of the carrier board */
    if ((info->bar[0] = request_mem_region(pci_resource_start(dev, 0), pci_resource_len(dev, 0), "SxPCI")) == 0) {
        printk(KERN_WARNING "\n%s BAR[%d] memory resource already occupied!?\n", DEBUG_NAME, 0);
        list_del(&info->node);
        cleanup_device(info);
        return -1;
    }

    if ((info->bar[2] = request_mem_region(pci_resource_start(dev, 2), pci_resource_len(dev, 2), "SxPCI")) == 0) {
        printk(KERN_WARNING "\n%s BAR[%d] memory resource already occupied!?\n", DEBUG_NAME, 2);
        list_del(&info->node);
        cleanup_device(info);
        return -1;
    }


    /* make pci device available for access */
    if (pci_enable_device(dev) < 0)
	{
        list_del(&info->node);
        cleanup_device(info);
        return -1;
	};


    /* get physical addresses of used base address registers */
    ipac_base = pci_resource_start(dev, 2);

    /* initialize carrier board info structure */
    info->dev = dev;
    info->ip_space  = (char*)ioremap(pci_resource_start(dev, 2), SxPCI_IOIDINT_SIZE);


    /* setup carrier board control register */
    writeb(SxPCI_AUTO_ACK | SxPCI_CLR_AUTO, info->ip_space + SxPCI_CNTL0_REG);
    writeb(SxPCI_INTEN | SxPCI_AUTO_ACK, info->ip_space + SxPCI_CNTL0_REG);


    /* calculate address and control information for all support slots */
    /* Note. driver_data contains the number of slots (see also array  */
    /*       carrier_pci_table)!                                       */
    for (i=0; i<id->driver_data; i++) {
        info->slot[i].slot_index = i;
        info->slot[i].private_data = (unsigned long)info;
        info->slot[i].module_interrupt_vector = dev->irq;
        info->slot[i].system_interrupt_vector = dev->irq;
        info->slot[i].interrupt_level_INT0 = 0;
        info->slot[i].interrupt_level_INT1 = 0;

        info->slot[i].IO_space.physical_address     = (void*)(ipac_base + SxPCI_IO_SPACE_OFF + SxPCI_IO_SPACE_GAP*i);
        info->slot[i].IO_space.space_size           = SxPCI_IO_SPACE_SIZE;
        info->slot[i].ID_space.physical_address     = (void*)(ipac_base + SxPCI_ID_SPACE_OFF + SxPCI_ID_SPACE_GAP*i);
        info->slot[i].ID_space.space_size           = SxPCI_ID_SPACE_SIZE;
        info->slot[i].MEM8_space.physical_address   = NULL;     /* not supported */
        info->slot[i].MEM8_space.space_size         = 0;
        info->slot[i].MEM16_space.physical_address  = (void*)(ipac_base + SxPCI_MEM16_OFF + SxPCI_MEM16_GAP*i);
        info->slot[i].MEM16_space.space_size        = SxPCI_MEM16_SIZE;
    }

    /* register all slots after the structure are completely initialized */
    for (i=0; i<id->driver_data; i++) {
        if (carrier_register_slot(&carrier_car_driver, &info->slot[i]) == -1) {
            printk(KERN_WARNING "\n%s unable to register IPAC slot [%d]\n", DEBUG_NAME, i);
            list_del(&info->node);
            cleanup_device(info);
            return -1;
        }
    }

    /*
    **  PLX9080 Interrupt Control/Status Register [INTCSR]
    **
    **  - enable PLX PCI interrupts
    **  - enable local interrupts
    */
    info->local_control = (char*)ioremap(pci_resource_start(dev, 0), 0x80);

    info->saved_intcsr = readl(info->local_control+0x68);
    writel(readl(info->local_control+0x68) | 0x000D0900, info->local_control+0x68);


    return 0;
}


/*****************************************************************************
 *
 *  carrier_remove_one - shutdown specified carrier board device
 *
 *****************************************************************************/
static void carrier_remove_one(struct pci_dev *dev)
{
#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Remove device (vendor=0x%04X, device=0x%04X)\n",
        DEBUG_NAME, dev->vendor, dev->device);
#endif
}


/*****************************************************************************
 *
 *  cleanup_device - release resources allocated by the specified carrier board
 *
 *****************************************************************************/
static void cleanup_device(struct carrier_board_info *info)

{
    int  i;

    if (info->local_control) {
        /* disable PLX interrupts to the PCI bus */
        writel(info->saved_intcsr, info->local_control+0x68);
        iounmap(info->local_control);
    }

    if (info->ip_space) {
        /* disable IPAC interrupts */
        writeb(0, info->ip_space + SxPCI_CNTL0_REG);
        iounmap(info->ip_space);
    }

    for (i=0; i<SxPCI_NUM_BAR; i++) {
        if (info->bar[i]) release_mem_region(pci_resource_start(info->dev, i), pci_resource_len(info->dev, i));
    }

    kfree(info);
}



/*
**  The following definitions and the array carrier_pci_table[] describes the PCI
**  devices we are interested in.
**  The number of IP slots will be passed by the last parameter "driver_data" to
**  the probe function.
*/
#define SxPCI_VENDOR_ID       0x124B
#define SxPCI_DEVICE_ID       0x0040
#define SxPCI_SUBVENDOR_ID    PCI_ANY_ID
#define SxPCI_SUBDEVICE_ID    PCI_ANY_ID


static struct pci_device_id  carrier_pci_table[] = {
    { SxPCI_VENDOR_ID, SxPCI_DEVICE_ID, SxPCI_SUBVENDOR_ID, SxPCI_SUBDEVICE_ID, 0, 0, 4 },
    {0, }
};

MODULE_DEVICE_TABLE(pci, carrier_pci_table);


/*
**  The 'carrier_pci_driver' structure describes our driver and provide the PCI subsystem
**  with callback function pointer. The id_table contains PCI device ID's of devices we
**  are interested in.
*/
static struct pci_driver carrier_pci_driver = {
    name:       DRIVER_NAME,
    probe:      carrier_init_one,
    remove:     carrier_remove_one,
    id_table:   carrier_pci_table,
};


/*
**  This structure is used to register our driver at the carrier class driver. They contains
**  a description of our driver and a set of callback function pointer, which will be used
**  used by the carrier class driver.
*/
static struct carrier_driver carrier_car_driver = {
    name:                   DRIVER_NAME,
    version:                IPAC_CARRIER_DRIVER_BIN_VERSION,
    in_use:                 in_use,
    out_of_use:             out_of_use,
    enable_module_intr:     enable_module_intr,
    disable_module_intr:    disable_module_intr,
    setup_intr_detection:   setup_intr_detection,
    setup_clock_rate:       setup_clock_rate,
    enable_error_intr:      enable_error_intr,
    disable_error_intr:     disable_error_intr,
    check_error_status:     check_error_status,
    enable_timeout_intr:    enable_timeout_intr,
    disable_timeout_intr:   disable_timeout_intr,
    check_timeout_status:   check_timeout_status,
    interrupt_ack:          interrupt_ack,
    reset_carrier:          reset_carrier,
    reset_slot:             reset_slot,
    carrier_request_irq:    carrier_request_irq,
    carrier_free_irq:       carrier_free_irq,
    read_uchar:             read_uchar,
    read_ushort:            read_ushort,
    read_ulong:             read_ulong,
    write_uchar:            write_uchar,
    write_ushort:           write_ushort,
    write_ulong:            write_ulong,
};


/*****************************************************************************
 *
 *  carrier_init - module initialization function
 *
 *  This module will be called during module initialization. The init function
 *  allocates necessary resources and initializes internal data structures.
 *
 *****************************************************************************/
static int carrier_init(void)
{
    int retval;

    printk(KERN_INFO "\n%s version %s (%s)\n", DRIVER_NAME, IPAC_CARRIER_DRIVER_VERSION, IPAC_CARRIER_DRIVER_REVDATE);


    INIT_LIST_HEAD(&carrier_board_root);

    if (carrier_register_driver(&carrier_car_driver) == 0) {
        retval = tpmodule_pci_register_driver(&carrier_pci_driver);
        if (retval != 0)
        {
            /* no device found */
            carrier_unregister_driver(&carrier_car_driver);
        }
        return retval;
    }
    else {
#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Register carrier driver failed\n", DEBUG_NAME);
#endif
        return -ENODEV;
    }
}

/*****************************************************************************
 *
 *  carrier_cleanup - module cleanup
 *
 *  This module will be called before module removal. The cleanup function
 *  free all allocated resources.
 *
 *****************************************************************************/
static void carrier_cleanup(void)
{
    struct list_head  *ptr, *next;
    struct carrier_board_info  *info;

#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Carrier port driver removed\n", DEBUG_NAME);
#endif

    carrier_unregister_driver(&carrier_car_driver);

    for (ptr=carrier_board_root.next; ptr != &carrier_board_root; ) {

        next = ptr->next;
        info = list_entry(ptr, struct carrier_board_info, node);

        /*  remove the board info structure from the list */
        list_del(ptr);

        /*  release all allocated resource and free the memory (kfree) allocated by the info structure */
        cleanup_device(info);

        ptr = next;
    }

    tpmodule_pci_unregister_driver(&carrier_pci_driver);
}



module_init(carrier_init);
module_exit(carrier_cleanup);


