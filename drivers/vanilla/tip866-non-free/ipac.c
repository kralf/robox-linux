/* $Id: carrier_class.c 94 2007-02-27 09:49:24Z Harland $" */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @    CARRIER_CLASS    @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**    File             carrier_class.c                                       **
**                                                                           **
**    Function         Carrier class driver source                           **
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
**                     Copyright (c) 2006                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2007-02-27 10:49:24 +0100 (Di, 27 Feb 2007) $   $Rev: 94 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif

/*
**  If automatic installation of carrier port and IPAC port drivers isn't desired
**  please #undef the macro CARRIER_PnP.
*/
#undef CARRIER_PnP

/*
**  Usually slots with IP modules with incorrect IDROM CRC will be handled like
**  empty slots.
**  If the macro IGNORE_CRC_ERROR is defined also IP modules with bad IDROM contents
**  will be recognized.
*/
#undef IGNORE_CRC_ERROR

#undef DEBUG_CC
#define DEBUG_NAME  "IPAC_CC : "


#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#ifdef CARRIER_PnP
#include <linux/kmod.h>
#include <linux/pci.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include <linux/workqueue.h>
#endif
#endif
#include <asm/system.h>
#include <asm/io.h>
#include <linux/proc_fs.h>	/* Necessary because we use the proc fs */
#include <linux/seq_file.h>	/* for seq_file */

#include "tpmodule.c"
#include "ipac.h"

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif

static char *ipac_class_name    = "TEWS TECHNOLOGIES - IPAC Carrier Class Driver";
static char *ipac_class_version = IPAC_CARRIER_DRIVER_VERSION;
static char *ipac_class_revdate = IPAC_CARRIER_DRIVER_REVDATE;

#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("IPAC Carrier Class Driver");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS Technologies <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


#ifdef CARRIER_PnP
#include "pnpinf.h"
#endif
/*
**  Pattern and masks to determine the allignment for byte, word and long word access
**  Note.
*/
static unsigned char  IPAC_UCHAR_PATTERN_OK[]   = { 0x00, 0x49, 0x00, 0x50 };
static unsigned char  IPAC_UCHAR_MASK_OK[]      = { 0x00, 0xFF, 0x00, 0xFF };
static unsigned char  IPAC_UCHAR_PATTERN_SWAP[] = { 0x49, 0x00, 0x50, 0x00 };
static unsigned char  IPAC_UCHAR_MASK_SWAP[]    = { 0xFF, 0x00, 0xFF, 0x00 };

static unsigned short IPAC_USHORT_PATTERN_OK[]  = { 0x0049, 0x0050 };
static unsigned short IPAC_USHORT_MASK_OK[]     = { 0x00FF, 0x00FF };
static unsigned short IPAC_USHORT_PATTERN_SWAP[]= { 0x4900, 0x5000 };
static unsigned short IPAC_USHORT_MASK_SWAP[]   = { 0xFF00, 0xFF00 };

static unsigned long  IPAC_ULONG_PATTERN_OK     = 0x00490050;
static unsigned long  IPAC_ULONG_MASK_OK        = 0x00FF00FF;
static unsigned long  IPAC_ULONG_PATTERN_BSWAP  = 0x49005000;
static unsigned long  IPAC_ULONG_MASK_BSWAP     = 0xFF00FF00;
static unsigned long  IPAC_ULONG_PATTERN_WSWAP  = 0x00500049;
static unsigned long  IPAC_ULONG_MASK_WSWAP     = 0x00FF00FF;
static unsigned long  IPAC_ULONG_PATTERN_BWSWAP = 0x50004900;
static unsigned long  IPAC_ULONG_MASK_BWSWAP    = 0xFF00FF00;

union alignment_check {
    unsigned char  b[4];
    unsigned short w[2];
    unsigned long  l;
};

extern PciDeviceStruct rejectedPciDevices[MAX_REJECT_PCI_DEVICES];

static struct list_head ipac_module_root;
static struct list_head ipac_driver_root;
static struct list_head carrier_driver_root;

static void init_ipac_slot(struct ipac_module *ipac);
static void cleanup_ipac_slot(struct ipac_module *ipac);
static int allocate_ipac_module(struct ipac_module *ipac, struct ipac_driver *drv, struct ipac_module_id  *ipac_id);
static int check_IDPROM(struct addr_space_desc  *space);

/*****************************************************************************
 *
 * Lock and unlock functions for multiprocessing systems
 *
 ****************************************************************************/
static spinlock_t cc_spinlock;
static unsigned long cc_flags;

static void cc_lock(void)
{
	spin_lock_irqsave(&cc_spinlock, cc_flags);
}

static void cc_unlock(void)
{
	spin_unlock_irqrestore(&cc_spinlock, cc_flags);
}

/*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/*                  Carrier Port Driver Interface Functions                  */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************
 *
 *  ipac_register_driver - Register a new carrier port driver
 *
 *  This function inserts the requesting carrier port driver in a linked list
 *  that is maintained by the carrier class driver. The function argument
 *  passes a pointer to a structure which describes the driver and his export
 *  functions.
 *  The function returns always (0)
 *****************************************************************************/
int carrier_register_driver(struct carrier_driver  *drv)
{
    if (drv == NULL) {
        printk(KERN_WARNING "\n%s <carrier_register_driver> invalid drv pointer\n", DEBUG_NAME);
        return -1;
    }

#ifdef DEBUG_CC
    printk("\n%s carrier driver <%s> registered\n", DEBUG_NAME, drv->name);
#endif

    /* add the new driver entry at the end of the list */
    cc_lock();
    list_add_tail(&drv->node, &carrier_driver_root);
    cc_unlock();

    TP_MOD_INC_USE_COUNT;

    return 0;

}


/*****************************************************************************
 *  carrier_unregister_driver - Un-register a carrier port driver
 *
 *  This function must be called by the carrier port driver if he exits to
 *  free all allocated resources. The function returns (0) on success and (-1)
 *  otherwise.
 *****************************************************************************/
int carrier_unregister_driver(struct carrier_driver  *drv)

{
    struct carrier_driver  *drv_expect=NULL;
    struct list_head  *ptr, *next;
    struct ipac_module  *entry;


    /*  check if the carrier driver is already registered */
    list_for_each(ptr, &carrier_driver_root) {
        drv_expect = list_entry(ptr, struct carrier_driver, node);
        if (drv_expect == drv) break;
    }

    if (drv_expect != drv) {
        printk(KERN_WARNING "\n%s unregister of unknown carrier driver not possible\n", DEBUG_NAME);
        return -1;
    }


    /* walk through the list of known IPAC modules */
    for (ptr=ipac_module_root.next; ptr != &ipac_module_root; ) {

        next = ptr->next;
        /*
        **  map the list_head structure pointer back into a pointer
        **  to structure that contains it
        */
        entry = list_entry(ptr, struct ipac_module, node);

        if (entry->carrier_drv == drv) {
            /*
            **  Remove the IPAC module (slot) from the list. We don't need to check
            **  whether the slot is occupied or not, because it's only possible to call
            **  this function when all slots are released.
            */
            cc_lock();
            list_del(&entry->node);
            cc_unlock();

            /*  if necessary unmap IPAC spaces */
            if (entry->slot.IO_space.virtual_address) iounmap(entry->slot.IO_space.virtual_address);
            if (entry->slot.ID_space.virtual_address) iounmap(entry->slot.ID_space.virtual_address);
            if (entry->slot.MEM8_space.virtual_address) iounmap(entry->slot.MEM8_space.virtual_address);
            if (entry->slot.MEM16_space.virtual_address) iounmap(entry->slot.MEM16_space.virtual_address);

            /*  release kernel memory */
            kfree(entry);
        }

        ptr = next;
    }

    /*  remove the carrier port driver from the list */
    cc_lock();
    list_del(&drv->node);
    cc_unlock();

#ifdef DEBUG_CC
    printk("\n%s carrier driver <%s> removed\n", DEBUG_NAME, drv->name);
#endif

    TP_MOD_DEC_USE_COUNT;

    return 0;
}


/*****************************************************************************
 *  carrier_register_slot - Register new IPAC slot
 *
 *  The carrier port driver must call this function for each IPAC slot found
 *  on an IPAC carrier to register the resources. The function returns (0) on
 *  success and (-1) otherwise.
 *****************************************************************************/
int carrier_register_slot(struct carrier_driver  *drv, struct carrier_slot  *slot)
{
    struct carrier_driver  *drv_expect=NULL;
    struct ipac_module  *entry;
    struct ipac_driver  *ipac_drv;
    struct list_head  *ptr;
    struct ipac_module_id  *id_table;
    struct addr_space_desc  *id_space;
    union alignment_check  align;
    int  i;
    int  ip_okay=FALSE;
    int  driver_found = FALSE;


    /*  check if the carrier driver is already registered */
    list_for_each(ptr, &carrier_driver_root) {
        drv_expect = list_entry(ptr, struct carrier_driver, node);
        if (drv_expect == drv) break;
    }

    if (drv_expect != drv) {
        if (drv != 0) {
            printk(KERN_WARNING "\n%s carrier driver <%s> not registered\n", DEBUG_NAME, drv->name);
        }
        else {
            printk(KERN_WARNING "\n%s <carrier_register_slot> invalid drv pointer\n", DEBUG_NAME);
        }
        return -1;
    }

    if (slot == NULL) {
        printk(KERN_WARNING "\n%s <carrier_register_slot> invalid slot pointer\n", DEBUG_NAME);
        return -1;
    }

    /*  allocate kernel memory for the new slot */
    if (!(entry = kmalloc(sizeof(struct ipac_module), GFP_KERNEL))) return -1;
    memset(entry, 0, sizeof(struct ipac_module));

    entry->carrier_drv = drv;
    entry->slot = *slot;

    /*  setup backward linking of each memory space descriptor */
    entry->slot.IO_space.ipac = entry;
    entry->slot.ID_space.ipac = entry;
    entry->slot.MEM8_space.ipac = entry;
    entry->slot.MEM16_space.ipac = entry;

    /*  initialize memory mapping information */
    entry->slot.IO_space.virtual_address = NULL;
    entry->slot.IO_space.mapped_size = 0;
    entry->slot.IO_space.range_error = 0;
    entry->slot.IO_space.attribute = 0;
    entry->slot.ID_space.virtual_address = NULL;
    entry->slot.ID_space.mapped_size = 0;
    entry->slot.ID_space.range_error = 0;
    entry->slot.ID_space.attribute = 0;
    entry->slot.MEM8_space.virtual_address = NULL;
    entry->slot.MEM8_space.mapped_size = 0;
    entry->slot.MEM8_space.range_error = 0;
    entry->slot.MEM8_space.attribute = 0;
    entry->slot.MEM16_space.virtual_address = NULL;
    entry->slot.MEM16_space.mapped_size = 0;
    entry->slot.MEM16_space.range_error = 0;
    entry->slot.MEM16_space.attribute = 0;


    /*  add the new slot to the list of known slots */
    cc_lock();
    list_add_tail(&entry->node, &ipac_module_root);
    cc_unlock();

    /*  map ID space and check if IPAC module is mounted */
    id_space = ipac_map_space(entry, IPAC_ID_SPACE);    /* not necessary to check for errors */

    (void)ipac_check_access(id_space);      /* dummy check to reset error status information */
    (void)ipac_read_uchar(id_space, 0);

    if (ipac_check_access(id_space)) {

        while(TRUE) {

            /*
            **  check byte alignment
            */
            for (i=0; i<4; i++) align.b[i] = ipac_read_uchar(id_space, i);

            if ((align.l & *((unsigned long*)IPAC_UCHAR_MASK_OK)) == *((unsigned long*)IPAC_UCHAR_PATTERN_OK)) {
                /* byte lanes are correct */
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s UCHAR access  [byte lanes correct]\n", DEBUG_NAME);
#endif
            }
            else if ((align.l & *((unsigned long*)IPAC_UCHAR_MASK_SWAP)) == *((unsigned long*)IPAC_UCHAR_PATTERN_SWAP)) {
                /* We must swap byte lanes because the 'IPAC' string doesn't appear */
                /* at odd addresses (default for TEWS IP's)                         */
                entry->slot.IO_space.attribute |= IPAC_ATTR_UCHAR_SWAP;
                entry->slot.ID_space.attribute |= IPAC_ATTR_UCHAR_SWAP;
                entry->slot.MEM8_space.attribute |= IPAC_ATTR_UCHAR_SWAP;
                entry->slot.MEM16_space.attribute |= IPAC_ATTR_UCHAR_SWAP;
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s UCHAR access  [byte lanes must be swapped]\n", DEBUG_NAME);
#endif
            }
            else {
                /* Illegal pattern read. IP seems to be bad */
                ip_okay = FALSE;
                break;
            }


            /*
            **  check word alignment
            */
            for (i=0; i<2; i++) align.w[i] = ipac_read_ushort(id_space, i*2);

            if ((align.l & *((unsigned long*)IPAC_USHORT_MASK_OK)) == *((unsigned long*)IPAC_USHORT_PATTERN_OK)) {
                /* byte lanes are correct */
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s USHORT access [byte lanes are correct]\n", DEBUG_NAME);
#endif
            }
            else if ((align.l & *((unsigned long*)IPAC_USHORT_MASK_SWAP)) == *((unsigned long*)IPAC_USHORT_PATTERN_SWAP)) {
                /* We must swap byte lanes during word (16-bit) access */
                entry->slot.IO_space.attribute |= IPAC_ATTR_USHORT_SWAP;
                entry->slot.ID_space.attribute |= IPAC_ATTR_USHORT_SWAP;
                entry->slot.MEM8_space.attribute |= IPAC_ATTR_USHORT_SWAP;
                entry->slot.MEM16_space.attribute |= IPAC_ATTR_USHORT_SWAP;
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s USHORT access [byte lanes must be swapped]\n", DEBUG_NAME);
#endif
            }
            else {
                /* Illegal pattern read. IP seems to be bad */
                ip_okay = FALSE;
                break;
            }


            /*
            **  check long word alignment
            */
            align.l = ipac_read_ulong(id_space, 0);

            if ((align.l & IPAC_ULONG_MASK_OK) == IPAC_ULONG_PATTERN_OK) {
                /* byte and word lanes are correct */
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s ULONG access  [byte and word lanes are correct]\n", DEBUG_NAME);
#endif
            }
            else if ((align.l & IPAC_ULONG_MASK_BSWAP) == IPAC_ULONG_PATTERN_BSWAP) {
                /* We must swap only byte lanes the word lanes are correct */
                entry->slot.IO_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP;
                entry->slot.ID_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP;
                entry->slot.MEM8_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP;
                entry->slot.MEM16_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP;
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s ULONG access  [byte lanes must be swapped]\n", DEBUG_NAME);
#endif
            }
            else if ((align.l & IPAC_ULONG_MASK_WSWAP) == IPAC_ULONG_PATTERN_WSWAP) {
                /* We must swap only word lanes the byte lanes are correct */
                entry->slot.IO_space.attribute |= IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.ID_space.attribute |= IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.MEM8_space.attribute |= IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.MEM16_space.attribute |= IPAC_ATTR_ULONG_WORD_SWAP;
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s ULONG access  [word lanes must be swapped]\n", DEBUG_NAME);
#endif
            }
            else if ((align.l & IPAC_ULONG_MASK_BWSWAP) == IPAC_ULONG_PATTERN_BWSWAP) {
                /* We must swap word and byte lanes*/
                entry->slot.IO_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP | IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.ID_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP | IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.MEM8_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP | IPAC_ATTR_ULONG_WORD_SWAP;
                entry->slot.MEM16_space.attribute |= IPAC_ATTR_ULONG_BYTE_SWAP | IPAC_ATTR_ULONG_WORD_SWAP;
                ip_okay = TRUE;
#ifdef DEBUG_CC
    printk("\n%s ULONG access  [byte and word lanes must be swapped]\n", DEBUG_NAME);
#endif
            }
            else {
                /* Illegal pattern read. IP seems to be bad */
                ip_okay = FALSE;
                break;
            }

            break;
        }


        if (ip_okay) {
            /*  now since the access mechanism seems to clear we check the contents of the IDPROM */

#if defined (IGNORE_CRC_ERROR)
            if (TRUE) {
#else
            if (check_IDPROM(id_space)) {
#endif
                /*
                **  Now we are sure that the IPAC is correct.
                **  Identify the mounted IPAC module and check whether a IPAC port driver
                **  is waiting for this module.
                */
                entry->module_id.manufacturer = ipac_read_uchar(id_space, IPAC_OFF_MANUFACTURER_ID);
                entry->module_id.model_number = ipac_read_uchar(id_space, IPAC_OFF_MODEL);

#ifdef DEBUG_CC
    printk("\n%s IPAC (manufacturer=%02lX, model=%02lX) recognized (slot=%d, carrier=<%s>) \n", DEBUG_NAME,
        entry->module_id.manufacturer, entry->module_id.model_number, entry->slot.slot_index, drv->name);
#endif

                /*
                **  The last step is to check if any IPAC port driver is waiting for this IPAC module.
                **  Now we walk through the list of known IPAC port driver and check each ID table
                **  entry matching this IPAC module.
                */
                driver_found = FALSE;

                list_for_each(ptr, &ipac_driver_root) {
                    /*
                    **  map the list_head structure pointer back into a pointer
                    **  to structure that contains it
                    */
                    ipac_drv = list_entry(ptr, struct ipac_driver, node);

                    for (id_table = ipac_drv->id_table; id_table->manufacturer != 0; id_table++) {

                        if (entry->module_id.manufacturer == id_table->manufacturer
                            && entry->module_id.model_number == id_table->model_number) {
                            /*  this entry match */
                            allocate_ipac_module(entry, ipac_drv, id_table);
                            driver_found = TRUE;
                            break;
                        }
                    }

                    if (driver_found) break;
                }

#if (defined MODULE) && (defined CARRIER_PnP)
                if (!driver_found) {
                    /*  necessary IPAC port driver is currently not loaded, try to load it */
                    for (i=0; ipac_PnP_list[i].manufacturer != 0; i++) {
                        if (entry->module_id.manufacturer == ipac_PnP_list[i].manufacturer
                            && entry->module_id.model_number == ipac_PnP_list[i].model_number) {
                            /*  this entry match, load the driver */
                            request_module(ipac_PnP_list[i].driver_name);
                            break;  /* all done */
                        }
                    }
                }
#endif
            }
            else {
#ifdef DEBUG_CC
    printk("\n%s illegal IPAC CRC (slot=%d, carrier=<%s>) \n", DEBUG_NAME, entry->slot.slot_index, drv->name);
#endif
            }
        }
    }
    else {
#ifdef DEBUG_CC
    printk("\n%s IPAC access failed (slot=%d, carrier=<%s>) \n", DEBUG_NAME, entry->slot.slot_index, drv->name);
#endif

    }
    return 0;
}



/*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/*                  IPAC Port Driver Interface Functions                     */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************
 *  The static functions init_ipac_slot() and cleanup_ipac_slot will be used
 *  for initialization and cleanup of the specified IPAC slot
 *****************************************************************************/
static void init_ipac_slot(struct ipac_module *ipac)
{
    /*  use slot_config information to prepare the IPAC slot */
    if (ipac->module_id.slot_config & IPAC_INT0_EN) (ipac->carrier_drv->enable_module_intr)(&ipac->slot, 0);
    if (ipac->module_id.slot_config & IPAC_INT1_EN) (ipac->carrier_drv->enable_module_intr)(&ipac->slot, 1);

    if (ipac->module_id.slot_config & IPAC_EDGE_SENS) {
        (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 0, IPAC_EDGE_SENS);
        (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 1, IPAC_EDGE_SENS);
    }
    else {
        (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 0, IPAC_LEVEL_SENS);    /* default */
        (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 1, IPAC_LEVEL_SENS);    /* default */
    }

    if (ipac->module_id.slot_config & IPAC_CLK_32MHZ)
        (ipac->carrier_drv->setup_clock_rate)(&ipac->slot, IPAC_CLK_32MHZ);
    else
        (ipac->carrier_drv->setup_clock_rate)(&ipac->slot, IPAC_CLK_8MHZ);         /* default */
}

static void cleanup_ipac_slot(struct ipac_module *ipac)
{
    /*  setup default settings */
    (ipac->carrier_drv->disable_module_intr)(&ipac->slot, 0);
    (ipac->carrier_drv->disable_module_intr)(&ipac->slot, 1);
    (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 0, IPAC_EDGE_SENS);
    (ipac->carrier_drv->setup_intr_detection)(&ipac->slot, 1, IPAC_EDGE_SENS);
    (ipac->carrier_drv->setup_clock_rate)(&ipac->slot, IPAC_CLK_8MHZ);
}


/*****************************************************************************
 *  allocate_ipac_module  -  allocate an IPAC module to a IPAC port driver
 *****************************************************************************/
static int allocate_ipac_module(struct ipac_module *ipac, struct ipac_driver *drv, struct ipac_module_id  *ipac_id)
{
    int init_ok = FALSE;

    ipac->module_id.mem_size = ipac_id->mem_size;
    ipac->module_id.slot_config = ipac_id->slot_config;
    ipac->module_id.private_data = ipac_id->private_data;

    /*  prepare the IPAC slot */
    init_ipac_slot(ipac);

    /*  occupy this module for the requesting driver */
    ipac->ipac_drv = drv;

#ifdef DEBUG_CC
    printk("\n%s Call probe function of <%s> for module [%ld/%ld]\n",
		DEBUG_NAME, drv->name, ipac->module_id.manufacturer, ipac->module_id.model_number);
#endif

    /*  call IPAC driver probe function to initialize the device */
    if ((drv->probe)(ipac, ipac_id) < 0) {
        /* device initialization failed, free this entry */
        cleanup_ipac_slot(ipac);
        ipac->ipac_drv = NULL;
        init_ok = FALSE;
    }
    else {
        /* announce usage of the slot to the carrier driver */
        (ipac->carrier_drv->in_use)(&ipac->slot);
        init_ok = TRUE;
    }

    return init_ok;
}



/*****************************************************************************
 *  ipac_register_driver - Register a new IPAC port driver
 *
 *  This function inserts the requesting driver in a linked list that is
 *  maintained by the carrier driver. The function argument passes a pointer
 *  to a structure which describes the driver and the requested IPAC module(s).
 *****************************************************************************/
int ipac_register_driver(struct ipac_driver  *drv)

{
    struct ipac_module_id  *id_table;
    struct list_head  *ptr;
    struct ipac_module  *entry;
    int  num_devices=0;


    if (drv->id_table && drv->probe) {
        /* add the new driver entry at the end of the list */
        cc_lock();
        list_add_tail(&drv->node, &ipac_driver_root);
        cc_unlock();

#ifdef DEBUG_CC
    printk("\n%s IPAC driver <%s> registered\n", DEBUG_NAME, drv->name);
#endif
        /*
        **  walk through the list IPAC module types supported by this driver
        **  and check if there is any corresponding free entry in the list
        **  of IPAC known modules
        */
        for (id_table = drv->id_table; id_table->manufacturer != 0; id_table++) {

            /* walk through the list of known IPAC modules */
            list_for_each(ptr, &ipac_module_root) {

                /*
                **  map the list_head structure pointer back into a pointer
                **  to structure that contains it
                */
                entry = list_entry(ptr, struct ipac_module, node);

                if (entry->ipac_drv == NULL
                    && entry->module_id.manufacturer == id_table->manufacturer
                    && entry->module_id.model_number == id_table->model_number) {
                    /*  this free entry match */

                    if (allocate_ipac_module(entry, drv, id_table)) num_devices++;
                }
            }
        }

        TP_MOD_INC_USE_COUNT;

        return num_devices;
    }
    else {
        return -1;
    }
}


/*****************************************************************************
 *  ipac_unregister_driver - Un-register an IPAC port driver
 *
 *  This function must be called by the IPAC Port driver if he exits to free
 *  all allocated resources
 *****************************************************************************/
int ipac_unregister_driver(struct ipac_driver  *drv)

{
    struct list_head  *ptr;
    struct ipac_module  *entry;


    /* walk through the list of known IPAC modules */
    list_for_each(ptr, &ipac_module_root) {

        /*
        **  map the list_head structure pointer back into a pointer
        **  to structure that contains it
        */
        entry = list_entry(ptr, struct ipac_module, node);

        if (entry->ipac_drv == drv) {

            /*  force default setting */
            cleanup_ipac_slot(entry);

            /*  this slot is no longer used by an IPAC port driver */
            (entry->carrier_drv->out_of_use)(&entry->slot);

            entry->ipac_drv = NULL;
        }
    }

#ifdef DEBUG_CC
    printk("\n%s IPAC driver <%s> unregistered\n", DEBUG_NAME, drv->name);
#endif

    /*  remove the IPAC port driver from the list */
    cc_lock();
    list_del(&drv->node);
    cc_unlock();


    TP_MOD_DEC_USE_COUNT;

    return 0;
}


/*****************************************************************************
 *  ipac_map_space - Map IPAC space
 *
 *  This function returns an address space descriptor for the specified IPAC
 *  space. This descriptor must be used as argument for all IPAC access
 *  functions. For valid space ID's please refer to 4.2.1.2.
 *****************************************************************************/
struct addr_space_desc *ipac_map_space(struct ipac_module  *ipac, int  space_id)
{
    struct addr_space_desc  *desc;

    if (ipac == NULL) {
        printk(KERN_WARNING "\n%s <addr_space_desc> invalid ipac pointer\n", DEBUG_NAME);
		return NULL;
    }

    switch (space_id) {
    case IPAC_IO_SPACE:
        desc = &ipac->slot.IO_space;
        if (desc->virtual_address == 0) {
            /*  map physical address into the virtual kernel space */
            if (desc->space_size > 0) {
                desc->mapped_size = desc->space_size;
                desc->virtual_address = ioremap((unsigned long)desc->physical_address, desc->mapped_size);
            }
            else {
                /* IO IPAC space not available */
                desc = NULL;
            }
        }
        break;

    case IPAC_ID_SPACE:
        desc = &ipac->slot.ID_space;
        if (desc->virtual_address == 0) {
            /*  map physical address into the virtual kernel space */
            if (desc->space_size > 0) {
                desc->mapped_size = desc->space_size;
                desc->virtual_address = ioremap((unsigned long)desc->physical_address, desc->mapped_size);
            }
            else {
                /* ID IPAC space not available */
                desc = NULL;
            }
        }
        break;

    case IPAC_MEM_SPACE:
        if (ipac->module_id.slot_config & IPAC_MEM_8BIT) {
            if (ipac->slot.MEM8_space.physical_address) {
                /*  this IPAC slot supports a linear 8-bit wide MEM space */
                desc = &ipac->slot.MEM8_space;
                desc->attribute |= IPAC_ATTR_MEM_8BIT;
            }
            else {
                /*  this IPAC only has a 16-bit wide MEM space */
                desc = &ipac->slot.MEM16_space;
                desc->attribute |= IPAC_ATTR_MEM_8BIT;
            }
        }
        else {
            /*  default memory size is 16-bit */
            desc = &ipac->slot.MEM16_space;
            desc->attribute |= IPAC_ATTR_MEM_16BIT;
        }

        if (desc->virtual_address == 0) {
            if (desc->physical_address) {
                /*
                **  Map physical address into the virtual kernel space.
                **  For IPAC MEM space we map only the space required by the module and
                **  not the full physical MEM space
                */
                desc->mapped_size = ipac->module_id.mem_size;
                desc->virtual_address = ioremap((unsigned long)desc->physical_address, desc->mapped_size);
            }
            else {
                /*  no MEM space available */
                desc = NULL;
            }
        }
        break;

    default:
        /*  illegal space ID */
        desc = NULL;
        break;
    }

#ifdef DEBUG_CC
	if (!desc) printk("\n%s IPAC space <%d> mapping failed\n", DEBUG_NAME, space_id);
#endif

    return desc;
}


/*****************************************************************************
 *  ipac_swap_space/ipac_unswap_space - control swapping within a space
 *
 *  This functions are used to control big-endian<->little-endian swapping
 *  during byte/word/and long word accesses
 *****************************************************************************/
void ipac_swap_space(struct addr_space_desc *space)
{
    space->attribute |= IPAC_ATTR_USER_SWAP;
}

void ipac_unswap_space(struct addr_space_desc *space)
{
    space->attribute &= ~IPAC_ATTR_USER_SWAP;
}

/*****************************************************************************
 *  ipac_read_xxxxx - Read one item from the specified space
 *****************************************************************************/

/* SWAPS() swaps byte lanes of a word (AABB -> BBAA) */
#define	SWAPS(x)	(((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00))

/* SWAPL() swaps word and byte lanes of a long word (AABBCCDD -> DDCCBBAA) */
#define	SWAPL(x)	(((x >> 24) & 0x000000FF) | ((x >> 8) & 0x0000FF00) | ((x << 8) & 0x00FF0000) | ((x << 24) & 0xFF000000))

/* SWAPLB() swaps byte lanes within word lanes of a long word (AABBCCDD -> BBAADDCC) */
#define	SWAPLB(x)	(((x >> 8) & 0x00FF00FF) | ((x << 8) & 0xFF00FF00))

/* SWAPLW() swaps word lanes of a long word (AABBCCDD -> CCDDAABB) */
#define SWAPLW(x)	(((x >> 16) & 0x0000FFFF) | ((x << 16) & 0xFFFF0000))


unsigned char ipac_read_uchar(struct addr_space_desc *space, unsigned long  offset)
{
    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
		return 0xff;
    }

    if ((space->mapped_size == 0) || (offset >= space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return 0xff;
	}

    if (space->attribute & IPAC_ATTR_UCHAR_SWAP) {
        /* swap byte lanes */
        return (space->ipac->carrier_drv->read_uchar)(space, offset^1);
    }
    else {
        /* don't swap */
        return (space->ipac->carrier_drv->read_uchar)(space, offset);
    }
}

unsigned short ipac_read_ushort(struct addr_space_desc *space, unsigned long  offset)
{
    unsigned short value;

    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
		return 0xffff;
    }

    if ((space->mapped_size == 0) || ((offset+2) > space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return 0xffff;
	}

    if ((space->attribute & IPAC_ATTR_USHORT_SWAP) && (space->attribute & IPAC_ATTR_USER_SWAP)) {
        /* don't swap attributes eleminates each other */
        return (space->ipac->carrier_drv->read_ushort)(space, offset);
    }
    else if ((space->attribute & IPAC_ATTR_USHORT_SWAP) || (space->attribute & IPAC_ATTR_USER_SWAP)) {
        /* swap byte lanes */
        value = (space->ipac->carrier_drv->read_ushort)(space, offset);
        return SWAPS(value);
    }
    else {
        /* don't swap */
        return (space->ipac->carrier_drv->read_ushort)(space, offset);
    }
}

unsigned long ipac_read_ulong(struct addr_space_desc *space, unsigned long  offset)
{
    unsigned long  value;

    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
		return 0xffffffff;
    }

    if ((space->mapped_size == 0) || ((offset+4) > space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return 0xffffffff;
	}

    value = (space->ipac->carrier_drv->read_ulong)(space, offset);

    if ((space->attribute & IPAC_ATTR_ULONG_BYTE_SWAP) && (space->attribute & IPAC_ATTR_ULONG_WORD_SWAP)) {
        /* swap byte and word lanes (normal swap)*/
        value = SWAPL(value);
    }
    else if (space->attribute & IPAC_ATTR_ULONG_BYTE_SWAP) {
        /* swap byte lanes within word lanes of a long word */
        value = SWAPLB(value);
    }
    else if (space->attribute & IPAC_ATTR_ULONG_WORD_SWAP) {
        /* swap word lanes of a long word */
        value = SWAPLW(value);
    }

    /*
    **  Swap because of IPAC driver configuration.
    */
    if (space->attribute & IPAC_ATTR_USER_SWAP) {
        value = SWAPL(value);
    }

    return value;
}


/*****************************************************************************
 *  ipac_write_xxxxx - Write one item to the specified space
 *****************************************************************************/
void ipac_write_uchar(struct addr_space_desc *space, unsigned long offset, unsigned char value)
{
    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
        return;
    }

    if ((space->mapped_size == 0) || (offset >= space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return;
	}

    if (space->attribute & IPAC_ATTR_UCHAR_SWAP) {
        /* swap byte lanes */
        (space->ipac->carrier_drv->write_uchar)(space, offset^1, value);
    }
    else {
        /* don't swap */
        (space->ipac->carrier_drv->write_uchar)(space, offset, value);
    }
}

void ipac_write_ushort(struct addr_space_desc *space, unsigned long offset, unsigned short value)
{
    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
        return;
    }

    if ((space->mapped_size == 0) || ((offset+2) > space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return;
	}

    if ((space->attribute & IPAC_ATTR_USHORT_SWAP) && (space->attribute & IPAC_ATTR_USER_SWAP)) {
        /* don't swap attributes eleminates each other */
        (space->ipac->carrier_drv->write_ushort)(space, offset, value);
    }
    else if ((space->attribute & IPAC_ATTR_USHORT_SWAP) || (space->attribute & IPAC_ATTR_USER_SWAP)) {
        /* swap byte lanes */
        (space->ipac->carrier_drv->write_ushort)(space, offset, SWAPS(value));
    }
    else {
        /* don't swap */
        (space->ipac->carrier_drv->write_ushort)(space, offset, value);
    }
}

void ipac_write_ulong(struct addr_space_desc *space, unsigned long offset, unsigned long value)
{
    if (space == NULL) {
        printk(KERN_WARNING "\n%s Block access via an invalid IPAC space desc\n", DEBUG_NAME);
        return;
    }

    if ((space->mapped_size == 0) || ((offset+4) > space->mapped_size)) {
		/* if this space isn't mapped or the offset is out of range we generate this error */
		space->range_error = TRUE;
		return;
	}

    if ((space->attribute & IPAC_ATTR_ULONG_BYTE_SWAP) && (space->attribute & IPAC_ATTR_ULONG_WORD_SWAP)) {
        /* swap byte and word lanes (normal swap)*/
        value = SWAPL(value);
    }
    else if (space->attribute & IPAC_ATTR_ULONG_BYTE_SWAP) {
        /* swap byte lanes within word lanes of a long word */
        value = SWAPLB(value);
    }
    else if (space->attribute & IPAC_ATTR_ULONG_WORD_SWAP) {
        /* swap word lanes of a long word */
        value = SWAPLW(value);
    }

    /*
    **  Swap because of IPAC driver configuration.
    */
    if (space->attribute & IPAC_ATTR_USER_SWAP) {
        (space->ipac->carrier_drv->write_ulong)(space, offset, SWAPL(value));
    }
    else {
        (space->ipac->carrier_drv->write_ulong)(space, offset, value);
    }
}


/*****************************************************************************
 *  ipac_check_access - check the last access to the specified IPAC slot
 *
 *  Returns TRUE if the last access to the specified IPAC space was okay or
 *  FALSE otherwise. This Function checks for range errors and access timeouts
 *  which can be determined by certain carrier boards.
 *  Please note that this function also clears the error state. To make sure
 *  that no previous access error is pending perform a dummy check before
 *  accessing the critical memory location.
 *****************************************************************************/
int ipac_check_access(struct addr_space_desc  *space)
{
    if (space == NULL) {
        printk(KERN_WARNING "\n%s <ipac_check_access> invalid IPAC space desc\n", DEBUG_NAME);
        return 0;
    }

    if (space->range_error) {
        space->range_error = 0;
        return 0;
    }

    if ((space->ipac->carrier_drv->check_timeout_status)(&space->ipac->slot) == 0) {
        /*  no timeout, access was okay */
        return 1;
    }
    else {
        return 0;
    }
}


/*****************************************************************************
 *  ipac_enable_error - Enable the error interrupt of the IP
 *
 *  This function returns (0) if the error interrupt was successful enabled
 *  and (-1) if the carrier does not support this feature.
 *****************************************************************************/
int ipac_enable_error(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->enable_error_intr)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_disable_error - Disable the error interrupt of the IP
 *
 *  This function returns (0) if the error interrupt was successful disabled
 *  and (-1) if the carrier does not support this feature.
 *****************************************************************************/
int ipac_disable_error(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->disable_error_intr)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_check_error - Check the IP ERROR# line and clear a pending IP
 *                     error interrupt and status bit
 *
 *  This function return (1) if the IP error line is active or (0) otherwise.
 *****************************************************************************/
int ipac_check_error(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->check_error_status)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_enable_timeout - Enable the access timeout interrupt
 *
 *  This function returns (0) if the timeout interrupt was successful enabled
 *  and (-1) if the carrier does not support this feature.
 *****************************************************************************/
int ipac_enable_timeout(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->enable_timeout_intr)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_disable_timeout - Disable the access timeout interrupt
 *
 *  This function returns (0) if the timeout interrupt was successful disabled
 *  and (-1) if the carrier does not support this feature.
 *****************************************************************************/
int ipac_disable_timeout(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->disable_timeout_intr)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_check_error - Check if a timeout has occurred and clear a pending
 *                     timeout error and status bit.
 *
 *  This function return (1) if a timeout has occurred or (0) if not.
 *****************************************************************************/
int ipac_check_timeout(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->check_timeout_status)(&ipac->slot);
}


/*****************************************************************************
 *  ipac_check_error - Clear a pending IP interrupt.
 *
 *  This is necessary either for edge sensitive interrupts or for IP's which
 *  requires an interrupt acknowledge cycle to clear the interrupt. This
 *  function returns the interrupt vector form the IPAC module or 0 if this
 *  feature is not supported.
 *****************************************************************************/
int ipac_interrupt_ack(struct ipac_module *ipac, int intr_line)
{
    return (ipac->carrier_drv->interrupt_ack)(&ipac->slot, intr_line);
}


/*****************************************************************************
 *  ipac_reset - Assert an IP RESET# at the specified IP slot
 *
 *  Assert an IP RESET# at the specified IP slot if the IP carrier board
 *  supports this feature, otherwise return with error (-1).
 *****************************************************************************/
int ipac_reset(struct ipac_module *ipac)
{
    return (ipac->carrier_drv->reset_slot)(&ipac->slot);
}



/*****************************************************************************
 *  ipac_request_irq - install an ISR for a specified interrupt number
 *
 *
 *****************************************************************************/
int ipac_request_irq(struct ipac_module *ipac, unsigned int irq, int_handler_t *handler, unsigned long flags, const char *dev_name, void *dev_id)
{
    return (ipac->carrier_drv->carrier_request_irq)(&ipac->slot, irq, handler, flags, dev_name, dev_id);
}



/*****************************************************************************
 *  ipac_free_irq - uninstall an ISR for a specified interrupt number
 *
 *
 *****************************************************************************/
void ipac_free_irq(struct ipac_module *ipac, unsigned int irq, void *dev_id)
{
	ipac->carrier_drv->carrier_free_irq(&ipac->slot, irq, dev_id);
}







/*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/*                     IPAC Class Driver Static Functions                    */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************
 *  check_IDPROM  - Check the CRC of the ID PROM
 *
 *  RETURNS
 *      TRUE if CRC is ok
 *
 *****************************************************************************/

static int check_IDPROM(struct addr_space_desc  *space)

{
    unsigned int i, len, offset, crc = 0xffff;
    unsigned char mask, data;


    len = ipac_read_uchar(space, IPAC_OFF_NUM_BYTES);

    offset = IPAC_OFF_I;

    for (i=0; i<len; i++) {

        if (offset == IPAC_OFF_CRC) {
            /*
            **  do not include the stored CRC in the CRC calculation
            */
            data = 0;
        }
        else {
            /*
            **  read next item from the ID space
            */
            data = ipac_read_uchar(space, offset);
        }

        mask = 0x80;

        while (mask) {

            if (data & mask) crc ^= 0x8000;

            crc += crc;

            if (crc > 0xffff) crc = (crc & 0xffff) ^ 0x1021;

            mask = mask >> 1;

        }

        offset += 2;
    }

  return (~crc & 0xff) == ipac_read_uchar(space, IPAC_OFF_CRC);
}



/*****************************************************************************
 * Work queue thread (Kernel 2.6 and higher only)
 *
 * Due to changes in modprobe its not possible to load other modules (e.g
 * carrier_default) in the module init function, when these modules uses symbols
 * exported by the requesting module (carrier_class).
 * This is because modprobe registers exported symbols after the module has
 * successful initialized.
 * Using work queues will delay loading of carrier port drivers after the
 * carrier init function has finished.
 *
 * The carrier_thread scans the PCI bus for supported carrier board and loads
 * the appropriate carrier port driver.
 *****************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if (defined MODULE) && (defined CARRIER_PnP)

static struct workqueue_struct *wq;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void carrier_thread( void *ignored )
#else
static void carrier_thread( struct work_struct *ignored )
#endif
{
    int i;

#ifdef DEBUG_CC
    printk(KERN_INFO "%s work queue thread started...\n", DEBUG_NAME);
#endif

#if (defined MODULE) && (defined CARRIER_PnP)
    if (request_module("carrier_default") == -ENOSYS) {
        printk(KERN_WARNING "%s KMOD not installed; can't load carrier port driver\n", DEBUG_NAME);
    }
    else {

        for (i=0; carrier_PnP_list[i].vendor != 0; i++) {
            if (pci_get_subsys(carrier_PnP_list[i].vendor, carrier_PnP_list[i].device,
                carrier_PnP_list[i].ss_vendor, carrier_PnP_list[i].ss_device, NULL) != NULL) {
                request_module(carrier_PnP_list[i].driver_name);
            }
        }
    }
#endif

    return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static DECLARE_WORK( work_obj, carrier_thread, NULL );
#else
static DECLARE_WORK( work_obj, carrier_thread );
#endif
#endif
#endif



/*****************************************************************************
 * This function is called at the beginning of a sequence.
 * ie, when:
 *	- the /proc file is read (first time)
 *	- after the function stop (end of sequence)
 *
 *****************************************************************************/
static void *carrier_seq_start(struct seq_file *s, loff_t *pos)
{
	static unsigned long counter = 0;

	/* beginning a new sequence ? */
	if ( *pos == 0 ) {
		/* yes => return a non null value to begin the sequence */
		return &counter;
	}
	else {
		/* no => it's the end of the sequence, return end to stop reading */
		*pos = 0;
		return NULL;
	}
}


/*****************************************************************************
 * This function is called after the beginning of a sequence.
 * It's called untill the return is NULL (this ends the sequence).
 *
 *****************************************************************************/
static void *carrier_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return NULL;
}


/*****************************************************************************
 * This function is called at the end of a sequence
 *
 *****************************************************************************/
static void carrier_seq_stop(struct seq_file *s, void *v)
{
	/* nothing to do, we use a static value in start() */
}


/*****************************************************************************
 * This function is called for each "step" of a sequence
 *
 *****************************************************************************/
static int carrier_seq_show(struct seq_file *s, void *v)
{
    struct list_head  *ptr;
    struct ipac_module  *entry;
    struct carrier_driver  *drv_entry;


   seq_printf(s, "\n%s version %s (%s)\n", ipac_class_name, ipac_class_version, ipac_class_revdate);
   seq_printf(s, "\nRegistered IP slots:\n");

    /* walk through the list of known IPAC modules */
    list_for_each(ptr, &ipac_module_root) {

        /*
        **  map the list_head structure pointer back into a pointer
        **  to structure that contains it
        */
        entry = list_entry(ptr, struct ipac_module, node);

        /*
        **  Output information to the /proc file system
        */
        seq_printf(s, "\n[%s - Slot %d]\n", entry->carrier_drv->name,  entry->slot.slot_index);
        if (entry->module_id.manufacturer == 0 && entry->module_id.model_number == 0) {
            seq_printf(s, "    Plugged Module     EMPTY\n");
        }
        else {
            seq_printf(s, "    Plugged Module     Vendor=0x%02lX, Modul=0x%02lX\n",
                entry->module_id.manufacturer, entry->module_id.model_number);

            if (entry->ipac_drv)
                seq_printf(s, "    Installed Driver   %s\n", entry->ipac_drv->name);
            else
                seq_printf(s, "    Installed Driver   NONE \n");

            seq_printf(s, "    Slot Setup         ");
            if (entry->module_id.slot_config & IPAC_INT0_EN) seq_printf(s, "INT0_EN | ");
            if (entry->module_id.slot_config & IPAC_INT1_EN) seq_printf(s, "INT1_EN | ");
            if (entry->module_id.slot_config & IPAC_EDGE_SENS) seq_printf(s, "EDGE_SENS | ");
            if (entry->module_id.slot_config & IPAC_LEVEL_SENS) seq_printf(s, "LEVEL_SENS | ");
            if (entry->module_id.slot_config & IPAC_CLK_8MHZ) seq_printf(s, "CLK_8MHZ | ");
            if (entry->module_id.slot_config & IPAC_CLK_32MHZ) seq_printf(s, "CLK_32MHZ | ");
            if (entry->module_id.slot_config & IPAC_MEM_8BIT) seq_printf(s, "MEM_8BIT | ");
            if (entry->module_id.slot_config & IPAC_MEM_16BIT) seq_printf(s, "MEM_16BIT | ");
            seq_printf(s, "\n");
            seq_printf(s, "                       Memory Size = 0x%lx\n", entry->module_id.mem_size);

            seq_printf(s, "    Interrupt Vector   System=%d, Module=%d\n",
                entry->slot.system_interrupt_vector, entry->slot.module_interrupt_vector);
            seq_printf(s, "    Interrupt Level    INT0=%d, INT1=%d\n",
                entry->slot.interrupt_level_INT0, entry->slot.interrupt_level_INT1);
            seq_printf(s, "    ID Space Addr      Physical=0x%08lx, Virtual=0x%08lx\n",
                (unsigned long)entry->slot.ID_space.physical_address, (unsigned long)entry->slot.ID_space.virtual_address);
            seq_printf(s, "    IO Space Addr      Physical=0x%08lx, Virtual=0x%08lx\n",
                (unsigned long)entry->slot.IO_space.physical_address, (unsigned long)entry->slot.IO_space.virtual_address);
            seq_printf(s, "    MEM8 Space Addr    Physical=0x%08lx, Virtual=0x%08lx\n",
                (unsigned long)entry->slot.MEM8_space.physical_address, (unsigned long)entry->slot.MEM8_space.virtual_address);
            seq_printf(s, "    MEM16 Space Addr   Physical=0x%08lx, Virtual=0x%08lx\n",
                (unsigned long)entry->slot.MEM16_space.physical_address, (unsigned long)entry->slot.MEM16_space.virtual_address);
        }
    }

    seq_printf(s, "\nRegistered Carrier Drivers:\n");

    list_for_each(ptr, &carrier_driver_root) {
        drv_entry = list_entry(ptr, struct carrier_driver, node);

        seq_printf(s, "%s V%ld.%ld.%ld\n", drv_entry->name, (drv_entry->version>>16)& 0xff, (drv_entry->version>>8)& 0xff, drv_entry->version& 0xff);
    }

    seq_printf(s, "\n");

    return 0;
}

/*
** This structure gather "function" to manage the sequence
*/
static struct seq_operations carrier_seq_ops = {
	.start = carrier_seq_start,
	.next  = carrier_seq_next,
	.stop  = carrier_seq_stop,
	.show  = carrier_seq_show
};

/*
** This function is called when the /proc file is open.
*/
static int carrier_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &carrier_seq_ops);
};

/*
** This structure gather "function" that manage the /proc file
*/
static struct file_operations carrier_file_ops = {
	.owner   = THIS_MODULE,
	.open    = carrier_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

#define proc_entry_name "tews-ip-carrier"



/*****************************************************************************
 *
 *  ipac_class_init - module initialization function
 *
 *  This module will be called during module initialization. The init function
 *  allocates necessary resources and initializes internal data structures.
 *
 *****************************************************************************/
static int ipac_class_init(void)
{
	struct proc_dir_entry *entry;


    printk(KERN_INFO "\n%s version %s (%s)\n", ipac_class_name, ipac_class_version, ipac_class_revdate);

	/*
    **  Initialize driver and IPAC module lists
    */
    INIT_LIST_HEAD(&ipac_module_root);
    INIT_LIST_HEAD(&ipac_driver_root);
    INIT_LIST_HEAD(&carrier_driver_root);

	spin_lock_init(&cc_spinlock);

    /*
    **  Try to install carrier port driver for recognized carrier boards
    */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#if (defined MODULE) && (defined CARRIER_PnP)
    if (request_module("carrier_default") == -ENOSYS) {
        printk(KERN_WARNING "%s KMOD not installed; can't load carrier port driver\n", DEBUG_NAME);
    }
    else {
        int i;

        for (i=0; carrier_PnP_list[i].vendor != 0; i++) {
            if (pci_find_subsys(carrier_PnP_list[i].vendor, carrier_PnP_list[i].device,
                carrier_PnP_list[i].ss_vendor, carrier_PnP_list[i].ss_device, NULL) != NULL) {
                request_module(carrier_PnP_list[i].driver_name);
            }
        }
    }
#endif
#endif

    /*
    **  Create /proc file system entry for the carrier class driver
    */
	entry = create_proc_entry(proc_entry_name, 0, NULL);
	if (entry) {
		entry->proc_fops = &carrier_file_ops;
	}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if (defined MODULE) && (defined CARRIER_PnP)
    wq = create_workqueue( "tews-ccwq" );

    if (wq) {
        if(!queue_work( wq, &work_obj ) ) {
            printk(KERN_WARNING "%s queue_work failed\n", DEBUG_NAME);
        }
    }
    else {
        printk(KERN_WARNING "%s create_workqueue failed\n", DEBUG_NAME);
    }
#endif
#endif

    return 0;
}

/*****************************************************************************
 *
 *  ipac_class_ - module cleanup
 *
 *  This module will be called before module removal. The cleanup function
 *  free all allocated resources.
 *
 *****************************************************************************/
static void ipac_class_cleanup(void)
{
	remove_proc_entry(proc_entry_name, NULL);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if (defined MODULE) && (defined CARRIER_PnP)
    if( wq ) {
        destroy_workqueue( wq );
    }
#endif
#endif
}



module_init(ipac_class_init);
module_exit(ipac_class_cleanup);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
EXPORT_SYMBOL(ipac_register_driver);
EXPORT_SYMBOL(ipac_unregister_driver);
EXPORT_SYMBOL(ipac_map_space);
EXPORT_SYMBOL(ipac_swap_space);
EXPORT_SYMBOL(ipac_unswap_space);
EXPORT_SYMBOL(ipac_request_irq);
EXPORT_SYMBOL(ipac_interrupt_ack);
EXPORT_SYMBOL(ipac_free_irq);
EXPORT_SYMBOL(ipac_check_access);
EXPORT_SYMBOL(ipac_enable_error);
EXPORT_SYMBOL(ipac_disable_error);
EXPORT_SYMBOL(ipac_check_error);
EXPORT_SYMBOL(ipac_enable_timeout);
EXPORT_SYMBOL(ipac_disable_timeout);
EXPORT_SYMBOL(ipac_check_timeout);
EXPORT_SYMBOL(ipac_reset);

EXPORT_SYMBOL(ipac_write_uchar);
EXPORT_SYMBOL(ipac_write_ushort);
EXPORT_SYMBOL(ipac_write_ulong);
EXPORT_SYMBOL(ipac_read_uchar);
EXPORT_SYMBOL(ipac_read_ushort);
EXPORT_SYMBOL(ipac_read_ulong);

EXPORT_SYMBOL(carrier_register_driver);
EXPORT_SYMBOL(carrier_unregister_driver);
EXPORT_SYMBOL(carrier_register_slot);
#endif
