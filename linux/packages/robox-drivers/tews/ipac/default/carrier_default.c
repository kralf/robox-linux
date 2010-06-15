/* $Id: carrier_default.c 110 2007-09-18 13:52:57Z Hesse $" */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   CARRIER_DEFAULT   @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**    File             carrier_default.c                                     **
**                                                                           **
**    Function         Driver source for default carrier board               **
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
**    $Date: 2007-09-18 15:52:57 +0200 (Di, 18 Sep 2007) $   $Rev: 110 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#undef DEBUG_CARRIER
#define DEBUG_NAME          "DEFAULT : "

#define DRIVER_NAME         "TEWS TECHNOLOGIES - Default Carrier"

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


#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif


#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("Default IPAC Carrier Driver");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS Technologies <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


struct carrier_slot_info {

    int  slot_index;
        /*  zero-based slot position on a carrier e.g. 0 for slot A, 3 for slot D */

    unsigned long  ID_phy_address;
            /*  Physical address of the IPAC ID space.                  */

    unsigned long  ID_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes.                                                  */

    unsigned long  IO_phy_address;
            /*  Physical address of the IPAC ID space.                  */

    unsigned long  IO_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes. Set to 0 if the space is not available           */

    unsigned long  MEM8_phy_address;
            /*  Physical address of the IPAC ID space.                  */

    unsigned long  MEM8_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes.                                                  */

    unsigned long  MEM16_phy_address;
            /*  Physical address of the IPAC ID space.                  */

    unsigned long  MEM16_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes.                                                  */

    int  system_interrupt_vector;
            /*  Interrupt vector used to register the ISR in the        */
            /*  Linux Kernel.                                           */

    int  module_interrupt_vector;
            /*  Interrupt vector used to setup the interrupt vector     */
            /*  register of the IPAC module may be different to the     */
            /*  system_interrupt_vector for instance at VMEbus          */
            /*  carrier boards.                                         */

    int  interrupt_level_INT0;
    int  interrupt_level_INT1;
            /*  Optional interrupt level. Maybe used for Motorola 68k   */
            /*  systems. Set to 0 for other systens.                    */
};




static struct carrier_slot_info  slot_info[] = {

/*   slot-      ID-Space           IO-Space           MEM8-Space           MEM16-Space       Int-Vector    */
/*   index   base      size     base      size     base       size       base       size    sys mod level  */  
/*   ----------------------------------------------------------------------------------------------------  */

/*   Please add slot entries here! */

/*  
**  This is an example how entries for a 4 slot carrier looks like!
**  
**  {  0, 0xE8001100,  0x80, 0xE8001000,  0x80, 0x00000000, 0x000000, 0xE9000000, 0x800000,  9,  9,  0, 0 },
**  {  1, 0xE8002100,  0x80, 0xE8002000,  0x80, 0x00000000, 0x000000, 0xE9800000, 0x800000,  9,  9,  0, 0 },
**  {  2, 0xE8003100,  0x80, 0xE8003000,  0x80, 0x00000000, 0x000000, 0xEA000000, 0x800000,  9,  9,  0, 0 },
**  {  3, 0xE8004100,  0x80, 0xE8004000,  0x80, 0x00000000, 0x000000, 0xEA800000, 0x800000,  9,  9,  0, 0 },
*/
    
    /* end of list entry (slot_index must be -1) */
    { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};



static struct carrier_slot  slot;


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
 *  carrier_request_irq      install an ISR for a specified interrupt vector number
 *
 *  This function returns 0 on success or -1 on error
 *****************************************************************************/
static int carrier_request_irq(struct carrier_slot *slot, unsigned int irq, int_handler_t (*handler), unsigned long flags, const char *dev_name, void *dev_id)
{
	return request_irq(slot->system_interrupt_vector, handler, flags, dev_name, dev_id);
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


    printk(KERN_INFO "\n%s version %s (%s)\n", DRIVER_NAME, IPAC_CARRIER_DRIVER_VERSION, IPAC_CARRIER_DRIVER_REVDATE);


    if (carrier_register_driver(&carrier_car_driver) == 0) {


        for (i=0;  slot_info[i].slot_index != -1; i++) {

            slot.slot_index                     = slot_info[i].slot_index;
            slot.private_data                   = 0;
            slot.module_interrupt_vector        = slot_info[i].module_interrupt_vector;
            slot.system_interrupt_vector        = slot_info[i].system_interrupt_vector;
            slot.interrupt_level_INT0           = slot_info[i].interrupt_level_INT0;
            slot.interrupt_level_INT1           = slot_info[i].interrupt_level_INT1;

            slot.IO_space.physical_address      = (void*)(slot_info[i].IO_phy_address);
            slot.IO_space.space_size            = slot_info[i].IO_space_size;
            slot.ID_space.physical_address      = (void*)(slot_info[i].ID_phy_address);
            slot.ID_space.space_size            = slot_info[i].ID_space_size;
            slot.MEM8_space.physical_address    = (void*)(slot_info[i].MEM8_phy_address);
            slot.MEM8_space.space_size          = slot_info[i].MEM8_space_size;
            slot.MEM16_space.physical_address   = (void*)(slot_info[i].MEM16_phy_address);
            slot.MEM16_space.space_size         = slot_info[i].MEM16_space_size;

            if (carrier_register_slot(&carrier_car_driver, &slot) == -1) {
                printk(KERN_WARNING "\n%s unable to register IPAC slot entry [%d]\n", DEBUG_NAME, i);
            }
            else {
                num_slots++;
            }
        }

        if (num_slots == 0) {
            carrier_unregister_driver(&carrier_car_driver);
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
static void __exit carrier_cleanup(void)
{

#ifdef DEBUG_CARRIER
    printk(KERN_DEBUG "\n%s Carrier port driver removed\n", DEBUG_NAME);
#endif    

    carrier_unregister_driver(&carrier_car_driver);
}



module_init(carrier_init);
module_exit(carrier_cleanup);


