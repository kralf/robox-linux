/* $Id: ipac_carrier.h 110 2007-09-18 13:52:57Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @    IPAC_CARRIER     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**                                                                           **
**    File             ipac_carrier.h                                        **
**                                                                           **
**                                                                           **
**    Function         Driver header file                                    **
**                                                                           **
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


#ifndef __IPAC_CARRIER_H__
#define __IPAC_CARRIER_H__

#define IPAC_CARRIER_DRIVER_VERSION      "1.3.3"
#define IPAC_CARRIER_DRIVER_BIN_VERSION  0x10303
#define IPAC_CARRIER_DRIVER_REVDATE      "2007-09-18"


#include <linux/list.h>

/*
**  Defintions for IPAC slot configuration
*/
#define IPAC_INT0_EN        (1<<0)
#define IPAC_INT1_EN        (1<<1)
#define IPAC_EDGE_SENS      (1<<2)
#define IPAC_LEVEL_SENS     (1<<3)
#define IPAC_CLK_8MHZ       (1<<4)
#define IPAC_CLK_32MHZ      (1<<5)
#define IPAC_MEM_8BIT       (1<<6)
#define IPAC_MEM_16BIT      (1<<7)

/*
**  Definitions to differentiate between IPAC spaces
*/
#define IPAC_IO_SPACE               1
#define IPAC_ID_SPACE               2
#define IPAC_MEM_SPACE              3
#define IPAC_MEM8_SPACE             4
#define IPAC_MEM16_SPACE            5

/*
**  IPAC address space attribute definitions
*/
#define IPAC_ATTR_USER_SWAP         (1<<0)
#define IPAC_ATTR_UCHAR_SWAP        (1<<1)
#define IPAC_ATTR_USHORT_SWAP       (1<<2)
#define IPAC_ATTR_ULONG_BYTE_SWAP   (1<<3)
#define IPAC_ATTR_ULONG_WORD_SWAP   (1<<4)
#define IPAC_ATTR_MEM_8BIT          (1<<8)
#define IPAC_ATTR_MEM_16BIT         (1<<9)

/*
**  ID PROM offsets
*/
#define IPAC_OFF_I                  0x01
#define IPAC_OFF_P                  0x03
#define IPAC_OFF_A                  0x05
#define IPAC_OFF_C                  0x07
#define IPAC_OFF_MANUFACTURER_ID    0x09
#define IPAC_OFF_MODEL              0x0B
#define IPAC_OFF_REVISION           0x0D
#define IPAC_OFF_RESERVED           0x0F
#define IPAC_OFF_DRIVER_ID_L        0x11
#define IPAC_OFF_DRIVER_ID_H        0x13
#define IPAC_OFF_NUM_BYTES          0x15
#define IPAC_OFF_CRC                0x17

/*
**	Define interrupt handler type
*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
typedef void (int_handler_t)(int, void *, struct pt_regs *);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
typedef irqreturn_t (int_handler_t)(int, void *, struct pt_regs *);
#else
typedef irqreturn_t (int_handler_t)(int, void *);
#endif
#endif

 /*
 ** Structures and definitions needed to check if the found 
 ** module is in the list of rejected modules.
 ** This adaptation is necessary to let other drivers co-exist
 ** with the TEWS TECHNOLOGIES IPAC Carrier Driver.
 */
typedef struct PciDeviceStruct {
    unsigned char busNo;
    unsigned char devNo;
    unsigned char funcNo;
} PciDeviceStruct;

#define MAX_REJECT_PCI_DEVICES     2
static PciDeviceStruct rejectedPciDevices[MAX_REJECT_PCI_DEVICES] __attribute__((unused)) =
{
	{0x00, 0xff, 0x00}, 
	{0x00, 0xff, 0x00}  // not possible
};


/*
**  The addr_space_desc structure contains the physical and virtual address 
**  mapping information of an individual IPAC address space together with 
**  special memory access attributes.
*/
struct addr_space_desc {

    int  space_id;
            /*  IPAC address space identification code.                 */
            /*      IPAC_IO_SPACE       This address space is IPAC I/O  */
            /*      IPAC_ID_SPACE       This address space is IPAC ID   */
            /*      IPAC_MEM_SPACE      This address space is IPAC MEM  */
    
    void  *physical_address;
            /*  Physical address of the IPAC space.                     */

    unsigned long  space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes. The size depend on the type of address space and */
            /*  the carrier board.                                      */

    void  *virtual_address;
            /*  Virtual (kernel space) address of the mapped IPAC       */
            /*  space or NULL if the space isn't mapped at the moment.  */

    unsigned long  mapped_size;
            /*  Specifies the size of the mapped IPAC space in bytes.   */
            /*  This information can be used also for access range      */
            /*  checking.                                               */

    unsigned long  attribute;
            /*  Access attributes of this IPAC space.                   */

    int  range_error;
            /*  TRUE if the last access was out of range, FALSE         */
            /*  otherwise. To minimize IPAC access overhead range error */
            /*  will be modified only in case of an error. This item    */
            /*  will be reset (FALSE) only after reading with the       */
            /*  appropriate function.                                   */

    struct ipac_module  *ipac;
            /*  Link up with the enclosing ipac_module structure.       */
};



/*
**  The carrier_slot structure describes the underlying carrier slot of a 
**  registered IPAC module and must be filled by the carrier port driver.
*/
struct carrier_slot {

    int  slot_index;
            /*  Contains the index number (0...n) of the IPAC slot      */
            /*  seen from the carrier port driver.                      */

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
            /*  Optional interrupt level for INT0 and INT1              */

    struct addr_space_desc  ID_space;
    struct addr_space_desc  IO_space;
    struct addr_space_desc  MEM8_space;
    struct addr_space_desc  MEM16_space;
            /*  Address space descriptors for all relevant IPAC spaces. */

    unsigned long private_data;
            /*  Data private to the carrier port driver. Can be used    */
            /*  to identify the carrier board containing this slot      */
            /*  or whatever.                                            */
};


/*
**  The carrier_driver structure contains information about the carrier 
**  port driver and a set of function pointer. 
*/
struct carrier_driver {

    struct list_head  node;
            /*  Used to manage the list of carrier port drivers by the  */
            /*  carrier class driver.                                   */

    char  *name;
            /*  Name of the carrier port driver.                        */

    unsigned long  version;
            /*  Current version of the driver module just like the      */
            /*  Linux kernel version convention. A module version       */
            /*  1.2.48 expands to 0x010230.                             */

    void  (*in_use)(struct carrier_slot *slot);
            /*  This function will be called by the carrier class       */
            /*  driver every time an IPAC module was allocated by an    */
            /*  IPAC port driver. This function must increment the      */
            /*  module "used count" and whatever necessary to do when   */
            /*  an IPAC module was allocated. Every carrier port driver */
            /*  must export this function.                              */

    void  (*out_of_use)(struct carrier_slot *slot);
            /*  This function will be called by the carrier class       */
            /*  driver every time an IPAC module was no longer used by  */
            /*  an IPAC port driver. This function must decrement the   */
            /*  module "used count" and whatever necessary to do when   */
            /*  an IPAC module was no longer allocated. Every carrier   */
            /*  port driver must export this function.                  */

    int  (*enable_module_intr) (struct carrier_slot *slot, int intr_line);
    int  (*disable_module_intr) (struct carrier_slot *slot, int intr_line);
            /*  These functions can be used to enable or disable the    */
            /*  interrupt generation of the specified slot. The         */
            /*  argument intr_line must set be to 0 for line INT0 or 1  */
            /*  for line INT1. These functions returns -1 if a carrier  */
            /*  board doesn't support separately controlled interrupt   */
            /*  lines, otherwise 0 is returned.                         */

    int  (*setup_intr_detection) (struct carrier_slot *slot, int intr_line, int sensitive);
            /*  The interrupt detection mode can be set either to level */
            /*  or edge sensitive mode. Use the following symbols to    */
            /*  select the clock rate: IPAC_EDGE_SENS, IPAC_LEVEL_SENS. */

    int  (*setup_clock_rate) (struct carrier_slot *slot, int clock_rate);
            /*  The IPAC clock rate can be set either to 8 MHz or       */
            /*  32 MHz. Use the following symbols to select the clock   */
            /*  rate: IPAC_CLK_8MHZ, IPAC_CLK_32MHZ.                    */

    int  (*enable_error_intr) (struct carrier_slot *slot);
    int  (*disable_error_intr) (struct carrier_slot *slot);
    int  (*check_error_status) (struct carrier_slot *slot);
    int  (*enable_timeout_intr) (struct carrier_slot *slot);
    int  (*disable_timeout_intr) (struct carrier_slot *slot);
    int  (*check_timeout_status) (struct carrier_slot *slot);
    int  (*interrupt_ack) (struct carrier_slot *slot, int intr_line);
            /*  Please refer to the description of the corresponding    */
            /*  interface functions at paragraph 4.1.2.2.3.             */

    int  (*reset_carrier) (struct carrier_slot *slot);
            /*  Perform a hardware reset of the whole carrier board.    */
            /*  The carrier board will be selected by any IPAC slot.    */
            /*  This function return -1 if a carrier board doesn't      */
            /*  support this feature, otherwise 0 is returned.          */

    int  (*reset_slot) (struct carrier_slot *slot);
            /*  Perform a hardware reset of the specified slot          */
            /*  This function return -1 if a carrier board doesn't      */
            /*  support this feature, otherwise 0 is returned.          */

    unsigned char (*read_uchar) (struct addr_space_desc *space,  unsigned long offset);
    unsigned short (*read_ushort) (struct addr_space_desc *space,  unsigned long offset);
    unsigned long (*read_ulong) (struct addr_space_desc *space,  unsigned long offset);
            /*  Read one item from the IPAC space location specified    */
            /*  by the address space descriptor and the relative offset.*/

    void  (*write_uchar) (struct addr_space_desc *space,  unsigned long offset, unsigned char value);
    void  (*write_ushort) (struct addr_space_desc *space,  unsigned long offset, unsigned short value);
    void  (*write_ulong) (struct addr_space_desc *space,  unsigned long offset, unsigned long value);
            /*  Write one item to the IPAC space location specified by  */
            /*  the address space descriptor and the relative offset.   */
    
	int   (*carrier_request_irq) (struct carrier_slot *slot, unsigned int irq, int_handler_t *handler, unsigned long flags, const char *dev_name, void *dev_id);
	void  (*carrier_free_irq) (struct carrier_slot *slot, unsigned int irq, void *dev_id);
	
};


/*
**  This structure describes an IPAC module to allocate by the driver. 
*/
struct ipac_module_id {

    unsigned long   manufacturer;
            /*  Used to specify the IPAC module.                        */

    unsigned long   model_number;
            /*  Used to specify the IPAC module.                        */

    unsigned long   slot_config;
            /*  Set of bit flags which specify properties of the IPAC   */
            /*  module. This information will be used by the carrier    */
            /*  class driver to setup the carrier slot during           */
            /*  allocation.                                             */
            /*  IPAC_INT0_EN        Enables IPAC interrupt line 0       */
            /*  IPAC_INT1_EN        Enables IPAC interrupt line 1       */
            /*  IPAC_EDGE_SENS      Interrupt detection is edge         */
            /*                      sensitive                           */
            /*  IPAC_LEVEL_SENS     Interrupt detection is level        */
            /*                      sensitive (default)                 */
            /*  IPAC_CLK_8MHZ       IPAC clock is 8 MHz (default)       */
            /*  IPAC_CLK_32MHZ      IPAC clock is 32 MHz                */
            /*  IPAC_MEM_8BIT       IPAC memory space is 8-bit wide     */
            /*                      (D0...D7 only)                      */
            /*  IPAC_MEM_16BIT      IPAC memory space is 16-bit wide    */
            /*                      (default)                           */

    unsigned long   mem_size;
            /*  Specifies the required memory size in the IPAC memory   */
            /*  space. This information is used by the carrier driver   */
            /*  for mapping purposes and necessary configuration of the */
            /*  carrier memory space. If the IPAC module has no memory  */
            /*  requirements this parameter must be set to 0.           */

    unsigned long   private_data;
            /*  Optional private storage for the IPAC port driver. This */
            /*  data will be echoed together with the ipac_module_id    */
            /*  structure as argument of the probe() function.          */
};


/*
**  This structure contains necessary information needed by the driver 
**  to access the IPAC module.  
*/
struct ipac_module {

    struct list_head  node;
            /*  Used to manage the list of IPAC modules by the carrier  */
            /*  class driver.                                           */

    struct ipac_driver  *ipac_drv;
            /*  Pointer to the IPAC driver entry which has occupied the */
            /*  IPAC module or NULL if the module isn't attached.       */

    struct carrier_driver  *carrier_drv;
            /*  Pointer to the carrier driver entry which supplies the  */
            /*  slot where the IPAC module is plugged.                  */

    struct ipac_module_id  module_id;
            /*  Contains information about the IPAC module at this slot.*/
            /*  The parameter slot_config and private_data are supplied */
            /*  by the IPAC port driver.                                */

    struct carrier_slot slot;
            /*  Contains information about the IPAC slot where the      */
            /*  module is plugged like physical addresses interrupt     */
            /*  vector and so on (see also 4.2.1.3). All this           */
            /*  information's are delivered by the carrier port driver  */
            /*  during slot registration.                               */
};


/*
**  This structure contains necessary information needed by the driver 
**  to access the IPAC module.
*/
struct ipac_driver {

    struct list_head  node;
            /*  Used to manage the list of IPAC port drivers by the     */
            /*  carrier class driver.                                   */

    char  *name;
            /*  Name of the IPAC port driver.                           */

    unsigned long  version;
            /*  Current version of the driver module just like the      */
            /*  Linux kernel version convention. A module version       */
            /*  1.2.48 expands to 0x010230.                             */

    struct ipac_module_id  *id_table;
            /*  The ID table is an array of struct ipac_module_id       */
            /*  ending with an all-zero entry. This structure describes */
            /*  the desired IPAC modules handled by this driver.        */

    int (*probe) (struct ipac_module  *ipac, const struct ipac_module_id  *module_id);
            /*  This driver supplied function gets called for existing  */
            /*  IP modules that match the module table and are not      */
            /*  handled by other drivers yet.                           */
};


/*
**  Prototypes of exported functions
*/
int carrier_register_driver(struct carrier_driver  *drv);
int carrier_unregister_driver(struct carrier_driver  *drv);
int carrier_register_slot(struct carrier_driver  *drv, struct carrier_slot  *slot);
int ipac_register_driver(struct ipac_driver  *drv);
int ipac_unregister_driver(struct ipac_driver  *drv);
struct addr_space_desc *ipac_map_space(struct ipac_module  *ipac, int  space_id);
void ipac_swap_space(struct addr_space_desc *space);
void ipac_unswap_space(struct addr_space_desc *space);
unsigned char ipac_read_uchar(struct addr_space_desc *space, unsigned long  offset);
unsigned short ipac_read_ushort(struct addr_space_desc *space, unsigned long  offset);
unsigned long ipac_read_ulong(struct addr_space_desc *space, unsigned long  offset);
void ipac_write_uchar(struct addr_space_desc *space, unsigned long offset, unsigned char value);
void ipac_write_ushort(struct addr_space_desc *space, unsigned long offset, unsigned short value);
void ipac_write_ulong(struct addr_space_desc *space, unsigned long offset, unsigned long value);
int ipac_check_access(struct addr_space_desc  *space);
int ipac_enable_error(struct ipac_module *ipac);
int ipac_disable_error(struct ipac_module *ipac);
int ipac_check_error(struct ipac_module *ipac);
int ipac_enable_timeout(struct ipac_module *ipac);
int ipac_disable_timeout(struct ipac_module *ipac);
int ipac_check_timeout(struct ipac_module *ipac);
int ipac_interrupt_ack(struct ipac_module *ipac, int intr_line);
int ipac_reset(struct ipac_module *ipac);
int ipac_request_irq(struct ipac_module *ipac, unsigned int irq, int_handler_t *handler, unsigned long flags, const char *dev_name, void *dev_id);
void ipac_free_irq(struct ipac_module *ipac, unsigned int irq, void *dev_id);


#endif      /* __IPAC_CARRIER_H__ */
