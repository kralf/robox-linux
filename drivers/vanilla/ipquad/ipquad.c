/*-------------------------------------------------------------
 * Linux on RoboX project - ASL ETHZ
 *-------------------------------------------------------------
 *
 * SBS Technologies IP Quadrature linux driver for kernel 2.6
 *
 *-------------------------------------------------------------
 * $Id: ipquad.c,v 1.2 2008/04/27 08:25:15 kralf Exp $
 *-------------------------------------------------------------*/

#include <linux/version.h>
#include "config.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>

#include "tpmodule.c"
#include "ipquad.h"
#include "ipac.h"

/*
==============================================================================
        Constant definitions
==============================================================================
*/

// IDs and names
#define IPQUAD_VENDORID       0xFO  // Vendor ID
#define IPQUAD_DEVICEID       0x41  // Device ID
#define IPQUAD_NAME           "SBS Technologies IP Quadrature decoder"
#define IPQUAD_DRVNAME        "ipquad"

// Character devices
#define IPQUAD_MAJOR_BASE 212
#define IPQUAD_DEVCLASS "ipquad"
#define IPQUAD_DEVNAME "ipquad"

/*
==============================================================================
        Kernel messages and debugging
==============================================================================
*/

static int debug;

#define ipquad_printk(fmt, arg...) {\
  char message[256]; \
  sprintf(message, fmt, ## arg); \
  printk("%s: %s", IPQUAD_DRVNAME, message); \
}
#define ipquad_alertk(fmt, arg...) \
  ipquad_printk(KERN_ALERT fmt, ## arg)
#define ipquad_debugk(fmt, arg...) \
  if (debug) ipquad_printk(KERN_DEBUG fmt, ## arg)

/*
==============================================================================
        Driver structures and declarations
==============================================================================
*/



/*
==============================================================================
       Kernel driver code
==============================================================================
*/

MODULE_AUTHOR("Frederic Pont and Ralf Kaestner");
MODULE_DESCRIPTION("Linux driver for SBS Technologies IP Quadrature decoder");
MODULE_LICENSE("GPL");
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

/*
==============================================================================
        Device initialization and removal
==============================================================================
*/

static int __init ipquad_init(void) {
  // create device class
  ipquad_class = class_create(THIS_MODULE, IPQUAD_DEVCLASS);

  // find devices and register driver
  ipquad_num_devs = 0;
  ipquad_default = 0;
  return pci_module_init(&ipquad_driver);
}

static void __exit ipquad_exit(void) {
  // unregister driver
  pci_unregister_driver(&ipquad_driver);

  // destroy device class
  class_destroy(ipquad_class);
}

module_init(ipquad_init);
module_exit(ipquad_exit);













/*****************************************************************************
 *
 * tip_probe() - This function initializes all 8 channel of a TIP866 module
 *               and all required data structures
 *
 *****************************************************************************/

static int tip_probe(struct ipac_module *ipac, const struct ipac_module_id *module_id)
{
    struct module_info_struct *mod;
    struct addr_space_desc  *io_space, *id_space;
    int line_base;
    int result = 0;
    int i;

    if (module_count >= TIP866_MAX_NUM_MOD) {
        printk(KERN_INFO "Maximum number of TIP866 device exceeded (number=%ld)\n", module_count);
        return -ENOMEM;
    }

    printk(KERN_INFO "%s Probe new TIP866 mounted on <%s> at slot %c\n",
	    TIP866_DBG_NAME, ipac->carrier_drv->name, 'A'+ipac->slot.slot_index);

    /*  first try to map the IPAC IO space */
    if ((io_space = ipac_map_space(ipac, IPAC_IO_SPACE)) == 0) {
        printk("%s unable to map IPAC IO space\n", TIP866_DBG_NAME);
        return -1;
    }

    if ((id_space = ipac_map_space(ipac, IPAC_ID_SPACE)) == 0) {
        printk("%s unable to map IPAC ID space\n", TIP866_DBG_NAME);
        return -1;
    }

    /*
	**	Setup Interrupt Vector in the device; necessary on VMEbus carrier
	*/
    ipac_write_uchar(io_space, TIP_VECTOR, ipac->slot.module_interrupt_vector);

    /*
     *  The first module we found contains line 0..7, the second line 8..15 and so on
     */
    line_base = module_count * 8;

    /*
     *  Allocate a module info structure for this TIP866 module and
     *  initialze it with 0.
     */

    mod = kmalloc(sizeof(struct module_info_struct), GFP_KERNEL);

    if (!mod) {
        return -ENOMEM;
    }
    memset(mod, 0, sizeof(struct module_info_struct));

    module_table[module_count++] = mod;
    mod->ipac = ipac;

    /*
     *  Initialize each channel in the module info structure with appropriate values
     */
    for (i=0; i<TIP866_CHAN_PER_MOD; i++) {
        /*
         *  port state structure
         */
        mod->state[i].baud_base = TIP866_CLOCK_RATE / 16;
        mod->state[i].port = i * TIP866_CHAN_SPAN;
        mod->state[i].space = io_space;
        mod->state[i].irq = ipac->slot.system_interrupt_vector;
        mod->state[i].flags = 0;
        mod->state[i].type = ipac_read_uchar(id_space, TIP_BOARD_OPTION);
#ifdef TIP866_DEBUG_XX2
		printk("Moduletype TIP866-%d\n", mod->state[i].type);
#endif
        mod->state[i].line = line_base + i;
        mod->state[i].xmit_fifo_size = SERIAL_FIFO_SIZE;
        mod->state[i].count = 0;
        mod->state[i].close_delay = 5*HZ/10;
        mod->state[i].closing_wait = 30*HZ;
        mod->state[i].callout_termios = callout_driver.init_termios;
        mod->state[i].normal_termios = tip866_driver.init_termios;
        mod->state[i].icount.cts = mod->state[i].icount.dsr =
        mod->state[i].icount.rng = mod->state[i].icount.dcd = 0;
        mod->state[i].icount.rx = mod->state[i].icount.tx = 0;
        mod->state[i].icount.frame = mod->state[i].icount.parity = 0;
        mod->state[i].icount.overrun = mod->state[i].icount.brk = 0;
        mod->state[i].info = &mod->info[i];

        /*
         *  info structure
         */
        init_waitqueue_head(&mod->info[i].open_wait);
        init_waitqueue_head(&mod->info[i].close_wait);
        init_waitqueue_head(&mod->info[i].delta_msr_wait);

        mod->info[i].magic = TIP866_MAGIC;
        mod->info[i].port = i * TIP866_CHAN_SPAN;
        mod->info[i].space = io_space;
        mod->info[i].flags = 0;
        mod->info[i].xmit_fifo_size = mod->state[i].xmit_fifo_size;
        mod->info[i].line = line_base + i;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        /* Kernel 2.4.x */
        INIT_TQUEUE( &mod->info[i].tqueue, do_softint, (void*)(&mod->info[i]) );
#else
        /* Kernel 2.6.x */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        INIT_WORK( &mod->info[i].work, do_softint, (void*)(&mod->info[i]) );
#else
        INIT_WORK( &mod->info[i].work, do_softint );
#endif
#endif
        spin_lock_init(&mod->info[i].lock);


		mod->info[i].state = &mod->state[i];

        /* We can only determine if the transmit FIFO is able to accept new */
        /* data if we use this hardware wired flags.                        */
        mod->info[i].fifo_status_reg = i<4 ? TIP_FRR14 : TIP_FRR58;
        mod->info[i].tx_empty_bit = 1 << (i & 3);

        /*
         * Reset this UART
         */
        tip866_out(&mod->info[i], UART_FCR, (UART_FCR_ENABLE_FIFO |
                                              UART_FCR_CLEAR_RCVR |
                                              UART_FCR_CLEAR_XMIT));
        tip866_out(&mod->info[i], UART_FCR, 0);
        (void)tip866_in(&mod->info[i], UART_RX);
        tip866_out(&mod->info[i], UART_IER, 0);

		/* Create a tty and a cua DEVFS node per channel */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
		tty_register_devfs(&tip866_driver, 0,
				   tip866_driver.minor_start + mod->state[i].line);
		tty_register_devfs(&callout_driver, 0,
				   callout_driver.minor_start + mod->state[i].line);
#else
		tty_register_device(&tip866_driver,
			tip866_driver.minor_start + mod->state[i].line,
			mod->info[i].dev);
#endif
    }


    /*
     *  Register the ISRs for this TIP866 module
     */
	mod->_1st_uart.parent = (void *)mod;
	mod->_1st_uart.controller_num = 0;
	result = ipac_request_irq(ipac, ipac->slot.system_interrupt_vector, tp_interrupt, 0, "TIP866", (void *)&mod->_1st_uart);

    if (result != 0) {
        printk("Couldn't allocate tip866 interrupt (IRQ=%d)\n", ipac->slot.system_interrupt_vector);
        /*
         *  Free all resources allocated by this device
         */
        kfree(mod);
        module_count--;
    }

	mod->_2nd_uart.parent = (void *)mod;
	mod->_2nd_uart.controller_num = 1;
    result = ipac_request_irq(ipac, ipac->slot.system_interrupt_vector+1, tp_interrupt, 0, "TIP866", (void *)&mod->_2nd_uart);

    if (result != 0) {
        printk("Couldn't allocate tip866 interrupt (IRQ=%d)\n", ipac->slot.system_interrupt_vector+1);
        /*
         *  Free all resources allocated by this device
         */
        ipac_free_irq(ipac, ipac->slot.system_interrupt_vector, (void *)&mod->_1st_uart);
        kfree(mod);
        module_count--;
    }

	return result;
}



/*****************************************************************************
 *
 * The tip866 driver initialization code!
 *
 *****************************************************************************/

/*
**  Supported TIP modules by this driver and their initialization values
*/
struct ipac_module_id tip866_id[] = {
    {
        manufacturer:   MANUFACTURER_TEWS,
        model_number:   MODULE_TIP866,
        slot_config:    IPAC_INT0_EN | IPAC_INT1_EN | IPAC_LEVEL_SENS | IPAC_CLK_8MHZ,
        mem_size:       0,
        private_data:   0,
    },
    {
        manufacturer:   0,      /* end of list */
        model_number:   0,
    }
};


/*
**  Device driver register structure
*/
struct ipac_driver tip866_drv = {

    name:       DRIVER_NAME,
    version:    DRIVER_BIN_VERSION,
    id_table:   tip866_id,
    probe:      tip_probe,
};


static int __init t866_init(void)
{
    int i;
    int result;


    show_tip866_version();

    /* Initialize the tty_driver structure */

    memset(&tip866_driver, 0, sizeof(struct tty_driver));
    tip866_driver.magic = TTY_DRIVER_MAGIC;

    tip866_driver.driver_name = "tip866";
#if defined CONFIG_DEVFS_FS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    tip866_driver.name = "ttySTIP866_%d";
#else
    tip866_driver.name = "ttySTIP866_";
    tip866_driver.devfs_name = tip866_driver.name;
#endif
#else
    tip866_driver.name = "ttySTIP866_";
#endif /* CONFIG_DEVFS_FS */
    tip866_driver.major = TIP866_TTY_MAJOR;
    tip866_driver.minor_start = 0;
    tip866_driver.num = NR_PORTS;
    tip866_driver.type = TTY_DRIVER_TYPE_SERIAL;
    tip866_driver.subtype = SERIAL_TYPE_NORMAL;
    tip866_driver.init_termios = tty_std_termios;
    tip866_driver.init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
    tip866_driver.flags = TTY_DRIVER_REAL_RAW  | TTY_DRIVER_NO_DEVFS;
#else
    tip866_driver.flags = TTY_DRIVER_REAL_RAW  | TTY_DRIVER_DYNAMIC_DEV;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    tip866_driver.refcount = &tip866_refcount;
    tip866_driver.table = tip866_table;
#else
	tip866_driver.ttys = tip866_table;
	tip866_driver.owner = THIS_MODULE;
#endif
	tip866_driver.termios = tip866_termios;
    tip866_driver.termios_locked = tip866_termios_locked;

    tip866_driver.open = tip866_open;
    tip866_driver.close = tp_close;
    tip866_driver.write = tp_write;
    tip866_driver.put_char = tp_put_char;
    tip866_driver.flush_chars = tp_flush_chars;
    tip866_driver.write_room = tp_write_room;
    tip866_driver.chars_in_buffer = tp_chars_in_buffer;
    tip866_driver.flush_buffer = tp_flush_buffer;
    tip866_driver.ioctl = tp_ioctl;
    tip866_driver.throttle = tp_throttle;
    tip866_driver.unthrottle = tp_unthrottle;
    tip866_driver.set_termios = tp_set_termios;
    tip866_driver.stop = tp_stop;
    tip866_driver.start = tp_start;
    tip866_driver.hangup = tp_hangup;
    tip866_driver.break_ctl = tp_break;
    tip866_driver.send_xchar = tp_send_xchar;
    tip866_driver.wait_until_sent = tp_wait_until_sent;
    tip866_driver.read_proc = tp_read_proc;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    /*
     * The callout device is just like normal device except for
     * major number and the subtype code.
     */
    callout_driver = tip866_driver;
#if defined CONFIG_DEVFS_FS
    callout_driver.name = "cuaTIP866_%d";
#else
    callout_driver.name = "cuaTIP866_";
#endif /* CONFIG_DEVFS_FS */

    callout_driver.major = TIP866_CUA_MAJOR;
    callout_driver.minor_start = 0;
    callout_driver.subtype = SERIAL_TYPE_CALLOUT;
    callout_driver.read_proc = 0;
    callout_driver.proc_entry = 0;

    if ((result = tty_register_driver(&callout_driver)) < 0)
        printk("%s(%d):Couldn't register TIP866 callout driver (%d)\n",__FILE__,__LINE__,result);
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

	if ((result = tty_register_driver(&tip866_driver)) < 0)
        printk("%s(%d):Couldn't register TIP866 serial driver (%d)\n",__FILE__,__LINE__,result);



    /*
     *  Init the module table
     *  This driver supports up to #TIP866_MAX_NUM_MOD TIP866 modules, each with 8 channels
     */
    for (i=0; i<TIP866_MAX_NUM_MOD; i++) module_table[i] = NULL;

    module_count = 0;


    if (ipac_register_driver(&tip866_drv) == -1) {
        /*
        **  Unregister the driver if we found no TIP866 modules or if an error
        **  occured during minor device initialization
        */
        if (tty_unregister_driver(&tip866_driver) < 0)
            printk("%s(%d): failed to unregister TIP866 serial driver\n",__FILE__,__LINE__);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        if (tty_unregister_driver(&callout_driver) < 0)
            printk("%s(%d): failed to unregister TIP866 callout driver\n",__FILE__,__LINE__);
#endif
        return -ENODEV;
    }

    return 0;
}


/*****************************************************************************
 *
 * Called to remove the driver
 *
 *****************************************************************************/
static void t866_cleanup(void)
{
    int result;
    int i, j;

	if ((result = tty_unregister_driver(&tip866_driver)) < 0)
        printk("%s(%d): failed to unregister TIP866 serial driver (%d)\n",__FILE__,__LINE__,result);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    if ((result = tty_unregister_driver(&callout_driver)) < 0)
        printk("%s(%d): failed to unregister TIP866 callout driver (%d)\n",__FILE__,__LINE__,result);
#endif

    /*
     *  Free all allocated resources
     *  Note. At this time all interrupt sources on the modules are disabled
     *        and all IRQ's are freed
     */
    for (i=0; i<module_count; i++) {

        ipac_free_irq(module_table[i]->ipac, module_table[i]->state[0].irq, (void *)&module_table[i]->_1st_uart);
        ipac_free_irq(module_table[i]->ipac, module_table[i]->state[0].irq+1, (void *)&module_table[i]->_2nd_uart);

		/* Remove DEVFS nodes */
	    for (j = 0; j < TIP866_CHAN_PER_MOD; j++)
		{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
			tty_unregister_devfs(&tip866_driver,
					   tip866_driver.minor_start + module_table[i]->state[j].line);
			tty_unregister_devfs(&callout_driver,
					   callout_driver.minor_start + module_table[i]->state[j].line);
#else
			tty_unregister_device(&tip866_driver,
					   tip866_driver.minor_start + module_table[i]->state[j].line);
#endif
		}
        kfree(module_table[i]);
    }

    ipac_unregister_driver(&tip866_drv);
}


module_init(t866_init);
module_exit(t866_cleanup);
