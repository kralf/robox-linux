/* $Id: carrier_vme.c 110 2007-09-18 13:52:57Z Hesse $" */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @     CARRIER_VME     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**    File             carrier_vme.c                                         **
**                                                                           **
**    Function         Driver source for VMEbus carrier board                **
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
#define DEBUG_NAME          "VME : "

#define DRIVER_NAME         "TEWS TECHNOLOGIES - VME IPAC Carrier Driver"

/*
**  Define the following macro if the VMEbus interrupt handler cannot handler
**  odd interrupt vectors.
*/
#undef UNIV_ODD_INT_PATCH


#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <asm/system.h>
#include <asm/io.h>

#include "../include/tpmodule.c"
#include <ipac_carrier.h>
#include "resource.h"


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif


#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("VMEbus IPAC Carrier Driver");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS TECHNOLOGIES <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


struct vme_intr_entry {

    struct list_head  node;     
        /* used to manage the list of known carrier boards           */

    struct list_head  intr_sharer;
        /* linked list of port driver ISR's that share this VMEbus interrupt */

    int interrupt_level;
        /* interrupt level */

    int irq;
        /* interrupt vector number */

};


struct vme_intr_sharer {

    struct list_head  node;     
        /* used to manage the list interrupt sharers                 */

    struct carrier_slot *slot;
        /* back link to associated carrier slot                      */
    
    int_handler_t *handler;
        /* pointer to the connected port driver ISR                  */
    
    void *dev_id;
        /* optional ISR argument                                     */

};



static struct carrier_slot  slot;
static struct list_head  vme_intr_root;


/* function prototypes */
static void free_vme_resources(void);
extern int  allocate_vme_window(unsigned long addr, unsigned long size, int space, int width, unsigned long *phys);
extern void free_vme_window(int handle);
extern int  request_vmeirq(int irqlevel, void *handler, void *arg);
extern void free_vmeirq(int irqlevel);
extern void enable_vmeirq(int irqlevel);
extern void disable_vmeirq(int irqlevel);



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
    return -1;
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
	return -1;
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
 *  carrier_isr      carrier board interrupt handler
 *
 *  Because the VME interrupt handler doesn't support interrupt sharing we
 *  have to implement our own interrupt handler.
 *****************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static TP_IRQ_RETURN_T carrier_isr(int irqlevel, int irqvector, void *dev_id, struct pt_regs *regs)
#else
static TP_IRQ_RETURN_T carrier_isr(int irqlevel, int irqvector, void *dev_id)
#endif
{
    struct list_head  *ptr;
    struct vme_intr_sharer  *sharer_entry;
    struct vme_intr_entry *vme_intr = (struct vme_intr_entry*)dev_id;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    TP_IRQ_RETURN_T retval = IRQ_NONE;
#endif

    /* traverse the list of interrupt sharer and call their ISR */
    list_for_each(ptr, &vme_intr->intr_sharer) {
        sharer_entry = list_entry(ptr, struct vme_intr_sharer, node);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        (sharer_entry->handler)(irqvector, sharer_entry->dev_id, regs);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
        if ((sharer_entry->handler)(irqvector, sharer_entry->dev_id, regs) == IRQ_HANDLED) retval = IRQ_HANDLED;
#else
        if ((sharer_entry->handler)(irqvector, sharer_entry->dev_id) == IRQ_HANDLED) retval = IRQ_HANDLED;
#endif
#endif
    }

    TP_IRQ_RETURN(retval);
}



/*****************************************************************************
 *  carrier_request_irq      install an ISR for a specified interrupt vector number
 *
 *  This function returns 0 on success or -1 on error
 *****************************************************************************/
static int carrier_request_irq(struct carrier_slot *slot, unsigned int irq, int_handler_t *handler, unsigned long flags, const char *dev_name, void *dev_id)
{
    struct list_head  *ptr;
    struct vme_intr_sharer  *sharer_entry = NULL;
    struct vme_intr_entry  *vme_intr_entry = NULL;
    int  irq_level[2], max_level=0;
    int  retval;
    int  i;

#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "%s carrier_request_irq() slot=%d, vec=%d, lev=%d,%d\n", DEBUG_NAME,
        slot->slot_index, irq, slot->interrupt_level_INT0, slot->interrupt_level_INT1);
#endif

    /* IPAC slots supports up to 2 different interrupt level for INT0 and INT1 */
    irq_level[0] = slot->interrupt_level_INT0;
    irq_level[1] = slot->interrupt_level_INT1;
    if (irq_level[0] != 0) max_level++;
    if (irq_level[1] != 0) max_level++;


    for (i=0; i<max_level; i++) {

        if (irq_level[i] == 0) continue;

        /* traverse the list of allocated VME interrupts and check if this IRQ is already registered */
        list_for_each(ptr, &vme_intr_root) {
            vme_intr_entry = list_entry(ptr, struct vme_intr_entry, node);
            
            if (vme_intr_entry->interrupt_level == irq_level[i]) {
                /* this interrupt level is aleady attached */
                break;
            }
        }
        
        if (list_empty(&vme_intr_root) || vme_intr_entry->interrupt_level != irq_level[i]) {
            /*
            **  this level isn't attached so we have to create an entry in the list of
            **  attached VMEbus IRQ's
            */
            /* allocate kernel memory for this entry */
            if (!(vme_intr_entry = (struct vme_intr_entry*)kmalloc(sizeof(struct vme_intr_entry), GFP_KERNEL))) {        
                printk(KERN_DEBUG "\n%s unable to allocate memory for the VME interrupt entry\n", DEBUG_NAME);
                return -1;
            }
            
            vme_intr_entry->interrupt_level = irq_level[i];
            
            /* register ISR for the requested VME interrupt level */
            retval = request_vmeirq(vme_intr_entry->interrupt_level, (void*)carrier_isr, (void*)vme_intr_entry);
            
            if (retval == -1) {
                printk(KERN_DEBUG "\n%s request_vmeirq() failed (level=%d)\n", DEBUG_NAME, vme_intr_entry->interrupt_level);
                kfree((char*)vme_intr_entry);
                return -1;
            }
            
            vme_intr_entry->irq = irq;
            INIT_LIST_HEAD(&vme_intr_entry->intr_sharer);
            
            /* insert entry at the end of the VME interrupt list */
            list_add_tail(&vme_intr_entry->node, &vme_intr_root);
            
            enable_vmeirq(vme_intr_entry->interrupt_level);
        }
        
        
        /* traverse the list of interrupt sharer and check if this ISR is already registered */
        list_for_each(ptr, &vme_intr_entry->intr_sharer) {
            sharer_entry = list_entry(ptr, struct vme_intr_sharer, node);
            
            /* each ISR is identified by its argument */ 
            if (sharer_entry->dev_id == dev_id) {
                /* the ISR is aleady attached */
                return 0;
            }
        }
        
        
        /* allocate kernel memory for the ISR entry */
        if (!(sharer_entry = (struct vme_intr_sharer*)kmalloc((long)sizeof(struct vme_intr_sharer), GFP_KERNEL))) {        
            printk(KERN_DEBUG "\n%s unable to allocate memory for the ISR sharer entry\n", DEBUG_NAME);
            
            if (list_empty(&vme_intr_entry->intr_sharer)) {
                /* delete the unused VME interrupt entry */
                disable_vmeirq(vme_intr_entry->interrupt_level);
                free_vmeirq(vme_intr_entry->interrupt_level);
                
                list_del(&vme_intr_entry->node);
                
                kfree((char*)vme_intr_entry);
            }
            return -1;
        }
        
        sharer_entry->handler  = handler;
        sharer_entry->dev_id   = dev_id;
        sharer_entry->slot     = slot;
        
        list_add_tail(&sharer_entry->node, &vme_intr_entry->intr_sharer);
    }

    /*
    **  The Tundra Universe and maybe other VME bridges can handle only even VME interrupts.
    **  For odd VME interrupts bit 0 will be masked and the interrupt occurs at irq-1. To
    **  handle this interrupt too we register both, even and odd, interrupts.
    */
#ifdef UNIV_ODD_INT_PATCH
    if (irq & 1) {
        return carrier_register_isr(slot, irq & ~1, handler, flags, dev_name, dev_id);
    }
    else {
        return 0;
    }
#else
    return 0;
#endif

}


/*****************************************************************************
 *  carrier_free_irq         remove an ISR from the IVT
 *
 *  
 *****************************************************************************/
static void carrier_free_irq(struct carrier_slot *slot, unsigned int irq, void *dev_id)
{
    struct list_head  *ptr, *temp;
    struct vme_intr_sharer  *sharer_entry = NULL;
    struct vme_intr_entry  *vme_intr_entry = NULL;
    int  irq_level[2], max_level=0;
    int  i;


    /* IPAC slots supports up to 2 different interrupt level for INT0 and INT1 */
    irq_level[0] = slot->interrupt_level_INT0;
    irq_level[1] = slot->interrupt_level_INT1;
    if (irq_level[0] != 0) max_level++;
    if (irq_level[1] != 0) max_level++;


    for (i=0; i<max_level; i++) {
        
        if (irq_level[i] == 0) continue;
        
        /* traverse the list of allocated VME interrupts and check if this IRQ is registered */
        list_for_each(ptr, &vme_intr_root) {
            vme_intr_entry = list_entry(ptr, struct vme_intr_entry, node);
            
            if (vme_intr_entry->interrupt_level == irq_level[i]) {
                /* this interrupt level is aleady attached */
                break;
            }
        }
        
        if (list_empty(&vme_intr_root) || vme_intr_entry->interrupt_level != irq_level[i]) {
            printk(KERN_DEBUG "%s IRQ(%d) isn't registered", DEBUG_NAME, irq_level[i]);
            continue;
        }
        
        /* traverse the interrupt sharer list and delete the requested entry */
        list_for_each_safe(ptr, temp, &vme_intr_entry->intr_sharer) {
            sharer_entry = list_entry(ptr, struct vme_intr_sharer, node);
            
            if (sharer_entry->dev_id == dev_id) {
                list_del(&sharer_entry->node);
                
                kfree((char*)sharer_entry);
                
                /* there is at most one entry in the list */
                break;
            }
        }
        
        
        if (list_empty(&vme_intr_entry->intr_sharer)) {
            /* delete the unused VME interrupt entry */
            disable_vmeirq(vme_intr_entry->interrupt_level);
            free_vmeirq(vme_intr_entry->interrupt_level);
            
            list_del(&vme_intr_entry->node);
            
            kfree((char*)vme_intr_entry);
        }
    }

    /*
    **  Because the carrier_register_isr() function registers two ISR's if the
    **  requested IRQ is odd (see description above) we must unregister also the
    **  even IRQ.
    */
#ifdef UNIV_ODD_INT_PATCH
    if (irq & 1) {
        carrier_unregister_isr(slot, irq & ~1, dev_id);
    }
#endif
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
static int __init carrier_init(void)
{
    int i;
    int num_slots = 0;
    unsigned long  addr;


    printk(KERN_INFO "\n%s version %s (%s)\n", DRIVER_NAME, IPAC_CARRIER_DRIVER_VERSION, IPAC_CARRIER_DRIVER_REVDATE);

    INIT_LIST_HEAD(&vme_intr_root);


    /* Map all configured VMEbus master access windows */
    for (i=0;  image_desc[i].image_type != -1; i++) {
        
        image_desc[i].image_handle = allocate_vme_window(
            image_desc[i].vme_image_start,
            image_desc[i].vme_image_size,
            image_desc[i].vme_addr_space,
            image_desc[i].vme_data_width,
            &image_desc[i].physical_address
            );

#ifdef DEBUG_CARRIER
            printk(KERN_DEBUG "%s handle=%d, start=%lx, size=%lx,phys=%lx\n", DEBUG_NAME, image_desc[i].image_handle,
                image_desc[i].vme_image_start, image_desc[i].vme_image_size,image_desc[i].physical_address);
#endif

        if (image_desc[i].image_handle == -1) {
            free_vme_resources();
            return -ENOMEM;
        }
    }


    if (carrier_register_driver(&carrier_car_driver) == 0) {

        for (i=0;  slot_desc[i].slot_index != -1; i++) {

            slot.slot_index                     = slot_desc[i].slot_index;
            slot.private_data                   = 0;
            slot.module_interrupt_vector        = slot_desc[i].module_interrupt_vector;
            slot.system_interrupt_vector        = slot_desc[i].system_interrupt_vector;
            slot.interrupt_level_INT0           = slot_desc[i].interrupt_level_INT0;
            slot.interrupt_level_INT1           = slot_desc[i].interrupt_level_INT1;

#ifdef DEBUG_CARRIER
            printk(KERN_DEBUG "%s slot=%d, vec=%d, lev=%d,%d\n", DEBUG_NAME, 
                slot.slot_index, slot.module_interrupt_vector, slot.interrupt_level_INT0, slot.interrupt_level_INT1);
#endif
            addr = image_desc[slot_desc[i].IO_image_type].physical_address
                   + (slot_desc[i].IO_vme_address - image_desc[slot_desc[i].IO_image_type].vme_image_start);
            slot.IO_space.physical_address      = (void*)(addr);
            slot.IO_space.space_size            = slot_desc[i].IO_space_size;

            addr = image_desc[slot_desc[i].ID_image_type].physical_address
                   + (slot_desc[i].ID_vme_address - image_desc[slot_desc[i].ID_image_type].vme_image_start);
            slot.ID_space.physical_address      = (void*)(addr);
            slot.ID_space.space_size            = slot_desc[i].ID_space_size;

            addr = image_desc[slot_desc[i].MEM_image_type].physical_address
                   + (slot_desc[i].MEM_vme_address - image_desc[slot_desc[i].MEM_image_type].vme_image_start);
            slot.MEM16_space.physical_address   = (void*)(addr);
            slot.MEM16_space.space_size         = slot_desc[i].MEM_space_size;

            slot.MEM8_space.physical_address    = (void*)0;
            slot.MEM8_space.space_size          = 0;

#ifdef DEBUG_CARRIER
            printk(KERN_DEBUG "%s IO=%lx  ID=%lx  MEM=%lx\n", DEBUG_NAME, (unsigned long)slot.IO_space.physical_address,
                (unsigned long)slot.ID_space.physical_address, (unsigned long)slot.MEM16_space.physical_address);
#endif
            if (carrier_register_slot(&carrier_car_driver, &slot) == -1) {
                printk(KERN_WARNING "\n%s unable to register IPAC slot entry [%d]\n", DEBUG_NAME, i);
            }
            else {
                num_slots++;
            }
        }

        if (num_slots == 0) {
            carrier_unregister_driver(&carrier_car_driver);
            free_vme_resources();
            return -ENODEV;
        }
        else {
            return 0;
        }
    }
    else {
#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Register carrier driver failed\n", DEBUG_NAME);
#endif    
        free_vme_resources();
        return -ENODEV;
    }
}



/*****************************************************************************
 *
 *  free_vme_resources - free allocated VMEbus resources
 *
 *****************************************************************************/
static void free_vme_resources(void)
{
    int i;

    for (i=0;  image_desc[i].image_type != -1; i++) {

        if (image_desc[i].image_handle != -1) {
            free_vme_window(image_desc[i].image_handle);
        }
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
static void __exit carrier_cleanup(void)
{

#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Carrier port driver removed\n", DEBUG_NAME);
#endif    

    free_vme_resources();

    carrier_unregister_driver(&carrier_car_driver);
}



module_init(carrier_init);
module_exit(carrier_cleanup);


