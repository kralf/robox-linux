/* $Id: resource.h 17 2006-02-13 13:38:24Z Welzel $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @   R E S O U R C E   @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux - IPAC Carrier Driver                           **
**                                                                           **
**                                                                           **
**    File             resource.h                                            **
**                                                                           **
**                                                                           **
**    Function         VMEbus and IPAC slot resource definitions             **
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
**                     Copyright (c) 2006                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2006-02-13 14:38:24 +0100 (Mo, 13 Feb 2006) $   $Rev: 17 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/


#ifndef __RESOURCE_H__
#define __RESOURCE_H__

/*
**  VMEbus image description
*/
struct vme_image_desc {

    int  image_type;
            /* image type identifier (-1 means image list)              */

    unsigned long   vme_image_start;
            /* Physical VMEbus image base address                       */

    unsigned long   vme_image_size;
            /* Image size in bytes                                      */
            /* Note. The size granularity is either 4 KB or 64 KB       */
    
    unsigned long   vme_addr_space;
            /* VMEbus address space: A16, A24 or A32                    */

    unsigned long   vme_data_width;
            /* VMEbus maximum data width: D8, D16, D32 or D64           */

    unsigned long   physical_address;
            /* Physical VMEbus base address                             */

    int   image_handle;
            /* handle of allocated master access window or -1 if no     */
            /* window assigned.                                         */
};


/*
**  IPAC slot description
*/
struct carrier_slot_desc {

    int  slot_index;
            /*  zero-based slot position on a carrier e.g. 0 for slot A, 3 for slot D */

    unsigned long  ID_vme_address;
            /*  Physical VMEbus address of the IPAC ID space.           */

    unsigned long  ID_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes.                                                  */

    unsigned long  ID_image_type;
            /*  Specifies the VMEbus image window of the ID space. The  */
            /*  value must correspond to an entry in the image_desc     */
            /*  array.                                                  */

    unsigned long  IO_vme_address;
            /*  Physical VMEbus address of the IPAC ID space.           */

    unsigned long  IO_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes. Set to 0 if the space is not available           */

    unsigned long  IO_image_type;
            /*  Specifies the VMEbus image window of the IO space. The  */
            /*  value must correspond to an entry in the image_desc     */
            /*  array.                                                  */

    unsigned long  MEM_vme_address;
            /*  Physical VMEbus address of the IPAC ID space.           */

    unsigned long  MEM_space_size;
            /*  Specifies the maximum size of the address space in      */
            /*  bytes.                                                  */

    unsigned long  MEM_image_type;
            /*  Specifies the VMEbus image window of the MEM space. The */
            /*  value must correspond to an entry in the image_desc     */
            /*  array.                                                  */

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
            /*  Interrupt level; used to enable the VMEbus interrupt    */
            /*  level.                                                  */
};

/*======================================================================*/

/* predefined image types (add image types as desired with unique       */
/* numbers                                                              */
#define A16D16      0
#define A24D16      1
#define A32D32      2

/* VMEbus address space definitions */
#define VME_A16     16
#define VME_A24     24
#define VME_A32     32

/* VMEbus access data width */
#define VME_D8      8
#define VME_D16     16
#define VME_D32     32
#define VME_D64     64



static struct vme_image_desc   image_desc[] = {

/*   The following example shows a configuration with 2 VMEbus images   */
/*   1. Image:  A16/D16 (Short I/O), size = 64KB                        */
/*   2. Image:  A24/D16, size = 1MB                                     */
/*                                                                      */
/*   The Universe II VMEbus interface controller supports 8 VMEbus      */
/*   master images at all!                                              */ 
/*                                                                      */
/*     image         ID-Space           address    data   |  don't      */
/*      type      base      size         space     width  |  change     */  
/*   -----------------------------------------------------------------  */
    {  A16D16, 0x00000000, 0x00010000,  VME_A16,  VME_D16,  0,  -1 },
    {  A24D16, 0x00D00000, 0x00100000,  VME_A24,  VME_D16,  0,  -1 },
    /*   Please add new entries here! */

    /* end of list entry (image_type must be -1) */
    { -1, 0, 0, 0, 0 },
};



static struct carrier_slot_desc  slot_desc[] = {
  
/*   An IPAC carrier slot is specified by the following resources:                                          */
/*      IPAC IO and ID address and size (usually VME A16/D16)                                               */
/*      IPAC MEM address and size (usally VME A24/D16)                                                      */
/*      VME interrupt level for INT0 and INT1 (1..7)                                                        */
/*      VME interrupt vector (any unused even vector between 64 and 254)                                    */
/*                                                                                                          */
/*   The field "slot_index" specifies the position on the carrier (SlotA=0, SlotB=1,...).                   */
/*   The field "image_type" binds the IPAC space to a VME image specified above. Usualy IPAC IO and ID      */
/*   spaces resides in the VME A16/D16 address space.                                                       */
/*   The fields "system_interrupt_vector" and "module_interrupt_vector" must be set to the same interrupt   */
/*   vector. The fields "interrupt_level_INT0" and "interrupt_level_INT1" must match to the configuration   */
/*   of the carrier board slot and will be also used to enable handling of the interrupt levels at this CPU.*/
/*                                                                                                          */
/*                                                                                                          */
/*   This is an example how entries for a 4 slot carrier (e.g. TVME200) looks like!                         */
/*      A16/D16 base address is 0x6000 (default for TVME200)                                                */
/*      A24/D16 base address is 0xD00000 (default for TVME200)                                              */
/*      A24/D16 memory size is 1 Mbyte (256 Kbyte/IP) (rotary switch S4=4 for TVME200)                      */
/*      The interrupt level configuration is default for TVME200 (S3=1)                                     */
/*                                                                                                          */
/*                                                                                               Interrupt  */
/*   slot-      ID-Space     image       IO-Space       image         MEM-Space       image   Vector    |   */
/*   index   base      size   type    base      size     type      base       size     type   sys mod level */  
/*   ------------------------------------------------------------------------------------------------------ */
    {  0, 0x00006080,  0x80, A16D16, 0x00006000,  0x80, A16D16, 0x00D00000, 0x040000, A24D16, 64, 64,  1, 2 },
    {  1, 0x00006180,  0x80, A16D16, 0x00006100,  0x80, A16D16, 0x00D40000, 0x040000, A24D16, 68, 68,  3, 4 },
    {  2, 0x00006280,  0x80, A16D16, 0x00006200,  0x80, A16D16, 0x00D80000, 0x040000, A24D16, 72, 72,  5, 6 },
    {  3, 0x00006380,  0x80, A16D16, 0x00006300,  0x80, A16D16, 0x00DC0000, 0x040000, A24D16, 76, 76,  7, 0 },
    /*   Please add slot entries here! */
    

    /* end of list entry (slot_index must be -1) */
    { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};


#endif      /* __RESOURCE_H__ */
