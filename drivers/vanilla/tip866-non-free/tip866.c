/* $Id: tip866.c 101 2007-09-18 14:20:33Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @     T I P 8 6 6     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TIP866 - Device Driver                                **
**                                                                           **
**    File             tip866.c                                              **
**                                                                           **
**    Description      Device driver source file                             **
**                                                                           **
**    Owner            TEWS TECHNOLOGIES GmbH                                **
**                     Am Bahnhof 7                                          **
**                     D-25469 Halstenbek                                    **
**                     Germany                                               **
**                                                                           **
**                     Tel.: +49 / (0)4101 / 4058-0                          **
**                     Fax.: +49 / (0)4101 / 4058-19                         **
**                     EMail: Support@tews.com                               **
**                     Web: http://www.tews.com                              **
**                                                                           **
**                                                                           **
**                     Copyright (c) 2006-2007                               **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**                                                                           **
*******************************************************************************
*******************************************************************************/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#define DRIVER_NAME         "TIP866 - 8 Channel Serial IP"
#define DRIVER_VERSION      "1.3.2"
#define DRIVER_BIN_VERSION  0x10302
#define DRIVER_REVDATE      "2006-09-18"



/*
 *  Defines the major device number for the TIP866 TTY and CUA driver
 *
 *  A value 0 means dynamic major number allocation
 */
#define TIP866_TTY_MAJOR       0
#define TIP866_CUA_MAJOR       0


/*
 *  The following symbols defines the default trigger level
 *
 *  This values are used for all found TIP866 devices!
 *
 *  The symbol TIP866_ENA_FIFO enables or disables the FIFO in all found devices
 */
#define TIP866_ENA_FIFO        1       //  0 = disabled, 1 = enabled

#define TIP866_RX_TRG_DEF   UART_FCR_R_TRIGGER_56
#define TIP866_TX_TRG_DEF   UART_FCR_T_TRIGGER_8


/*
 * End of tip866 driver configuration section.
 */

#include <linux/version.h>
#include "config.h"

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pci.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/bitops.h>

#include <linux/serial.h>

#include "tpmodule.c"
#include "tip866def.h"
#include "ipac.h"



#undef SERIAL_PARANOIA_CHECK
#define CONFIG_SERIAL_NOPAUSE_IO
#define SERIAL_DO_RESTART


/* Set of debugging defines */

#undef TIP866_DEBUG_INTR
#undef TIP866_DEBUG_OPEN
#undef TIP866_DEBUG_FLOW
#undef TIP866_DEBUG_RS_WAIT_UNTIL_SENT
#undef TIP866_DEBUG_PCI
#undef TIP866_DEBUG_RW
#undef TIP866_DEBUG_XX
#undef TIP866_DEBUG_XX1
#undef TIP866_DEBUG_XX2
#undef TIP866_DEBUG_XX3
#undef TIP866_DEBUG_FIFO
#define TIP866_DBG_NAME     "tip866:"

#define RS_STROBE_TIME (10*HZ)
#define RS_ISR_PASS_LIMIT 256


#ifdef TIP866_DEBUG_OPEN
#define OPNDBG(fmt, arg...) printk(KERN_DEBUG fmt , ## arg)
#else
#define OPNDBG(fmt, arg...) do { } while (0)
#endif

#define DBG_CNT(s)

#if defined(MODULE_DESCRIPTION)
MODULE_DESCRIPTION("TIP866 - 8 Channel Serial IP");
#endif
#if defined(MODULE_AUTHOR)
MODULE_AUTHOR("TEWS TECHNOLOGIES <support@tews.com>");
#endif
#if defined(MODULE_LICENSE)
MODULE_LICENSE("GPL");
#endif


static char *tip866_name = "TIP866 serial driver";

static struct tty_driver tip866_driver, callout_driver;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
static int tip866_refcount;
#endif

/* tip866 subtype definitions */
#ifndef SERIAL_TYPE_NORMAL
#define SERIAL_TYPE_NORMAL  1
#endif
#ifndef SERIAL_TYPE_CALLOUT
#define SERIAL_TYPE_CALLOUT 2
#endif

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

TPMODULE_DECLARE_ISR(tp_interrupt);


static void change_speed(struct info_struct *info, struct TP_TERMIOS *old);
static void tp_wait_until_sent(struct tty_struct *tty, int timeout);


#define NR_PORTS    (TIP866_MAX_NUM_MOD * TIP866_CHAN_PER_MOD)

static struct module_info_struct *module_table[TIP866_MAX_NUM_MOD];
static unsigned long module_count;

static struct tty_struct *tip866_table[NR_PORTS];
static struct TP_TERMIOS *tip866_termios[NR_PORTS];
static struct TP_TERMIOS *tip866_termios_locked[NR_PORTS];

#ifndef MIN
#define MIN(a,b)    ((a) < (b) ? (a) : (b))
#endif

/*
 * tmp_buf is used as a temporary buffer by tip866_write.  We need to
 * lock it in case the copy_from_user blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *tmp_buf;
#ifdef DECLARE_MUTEX
static DECLARE_MUTEX(tmp_buf_sem);
#else
static struct semaphore tmp_buf_sem = MUTEX;
#endif

#define lock_save(info) \
	spin_lock_irqsave(&info->lock, info->lock_flags)

#define unlock_restore(info) \
	spin_unlock_irqrestore(&info->lock, info->lock_flags)


/****************************************************************************
 * tip866_paranoia_check  - Check the magic number for the async_structure
 ****************************************************************************/
static int tip866_paranoia_check(struct info_struct *info,
                    int line, const char *routine)
{
#ifdef SERIAL_PARANOIA_CHECK
    static const char *badmagic =
        "Warning: bad magic number for tip866 struct (line %d) in %s\n";
    static const char *badinfo =
        "Warning: null info_struct for (line %d) in %s\n";

    if (!info) {
        printk(badinfo, line, routine);
        return 1;
    }
    if (info->magic != TIP866_MAGIC) {
        printk(badmagic, line, routine);
        return 1;
    }
#endif
    return 0;
}


/****************************************************************************
 * tip866_in   - Read one byte from the specified register
 ****************************************************************************/
static unsigned int tip866_in(struct info_struct *info, int offset)
{
    return ipac_read_uchar(info->space, info->port+offset);
}


/****************************************************************************
 * tip866_out   - Write one byte to the specified register
 ****************************************************************************/
static void tip866_out(struct info_struct *info, int offset, int value)
{
    ipac_write_uchar(info->space, info->port+offset, value);
#if defined __powerpc__
    eieio();
#endif

}


/*
 * ------------------------------------------------------------
 * tp_stop() and tp_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void tp_stop(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;


    if (tip866_paranoia_check(info, info->line, "tp_stop"))
        return;

    lock_save(info);
    if (info->IER & UART_IER_THRI) {
        info->IER &= ~UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
    }
    unlock_restore(info);
}


static void tp_start(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;


    if (tip866_paranoia_check(info, info->line, "tp_start"))
        return;

    lock_save(info);
    if (info->xmit_cnt && info->xmit_buf && !(info->IER & UART_IER_THRI)) {
        info->IER |= UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
    }
	unlock_restore(info);
}


/*
 * ----------------------------------------------------------------------
 *
 * Here starts the interrupt handling routines.  All of the following
 * subroutines are declared as inline and are folded into
 * tp_interrupt().  They were separated out for readability's sake.
 *
 * Note: tp_interrupt() is a "fast" interrupt, which means that it
 * runs with interrupts turned off.  People who may want to modify
 * tp_interrupt() should try to keep the interrupt handler as fast as
 * possible.  After you are done making modifications, it is not a bad
 * idea to do:
 * -----------------------------------------------------------------------
 */

/*****************************************************************************
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 *****************************************************************************/
static void tp_sched_event(struct info_struct *info,
                  int event)
{
    info->event |= 1 << event;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    /* Kernel 2.4.x */
	queue_task(&info->tqueue, &tq_immediate);
    mark_bh(IMMEDIATE_BH);
#else
    /* Kernel 2.6.x */
    schedule_work(&info->work);
#endif
}


/*****************************************************************************
 * receive_chars()
 *****************************************************************************/
static void receive_chars(struct info_struct *info, int *status)
{
    struct tty_struct *tty = info->tty;
    unsigned char ch, flag;
    int ignored = 0;
    struct  info_icount *icount;
#ifdef TIP866_DEBUG_XX
    int fifo_cnt=0;
#endif


    icount = &info->state->icount;
    do {
        ch = tip866_in(info, UART_RX);
        icount->rx++;
        /* character is inserted into the FLIP buffer later, let's check the
        ** status flag first. */
#ifdef TIP866_DEBUG_XX
        fifo_cnt++;
#endif

#ifdef TIP866_DEBUG_INTR
        printk("DR%02x:%02x...", ch, *status);
#endif
        flag = 0;
        if (*status & (UART_LSR_BI | UART_LSR_PE |
                   UART_LSR_FE | UART_LSR_OE)) {
            /*
             * For statistics only
             */
            if (*status & UART_LSR_BI) {
                *status &= ~(UART_LSR_FE | UART_LSR_PE);
                icount->brk++;
            } else if (*status & UART_LSR_PE)
                icount->parity++;
            else if (*status & UART_LSR_FE)
                icount->frame++;
            if (*status & UART_LSR_OE)
                icount->overrun++;

            /*
             * Now check to see if character should be
             * ignored, and mask off conditions which
             * should be ignored.
             */
            if (*status & info->ignore_status_mask) {
                if (++ignored > 100)
                    break;
                goto ignore_char;
            }
            *status &= info->read_status_mask;

            if (*status & (UART_LSR_BI)) {
#ifdef TIP866_DEBUG_INTR
                printk("handling break....");
#endif

                flag = TTY_BREAK;

                if (info->flags & ASYNC_SAK)
                    do_SAK(tty);
            } else if (*status & UART_LSR_PE)
                flag = TTY_PARITY;
            else if (*status & UART_LSR_FE)
                flag = TTY_FRAME;
            if (*status & UART_LSR_OE) {
                /*
                 * Overrun is special, since it's
                 * reported immediately, and doesn't
                 * affect the current character
                 */
                flag = TTY_OVERRUN;
            }
        }

        tty_insert_flip_char(tty, ch, flag);
    ignore_char:
        *status = tip866_in(info, UART_LSR);
    } while (*status & UART_LSR_DR);
#ifdef TIP866_DEBUG_XX
printk("#%d\n", fifo_cnt);
#endif
    tty_flip_buffer_push(tty);
}


/*****************************************************************************
 * transmit_chars()
 *****************************************************************************/
static void transmit_chars(struct info_struct *info, int *intr_done)
{
    struct info_struct  info_istat;
	int room = 0;

    if (info->x_char) {
        tip866_out(info, UART_TX, info->x_char);
        info->state->icount.tx++;
        info->x_char = 0;
        if (intr_done)
            *intr_done = 0;
        return;
    }
    if ((info->xmit_cnt <= 0) || info->tty->stopped ||
        info->tty->hw_stopped) {
        info->IER &= ~UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
        return;
    }


    /*
     *  Write chars into the FIFO while there are chars to send and
     *  there is space in the FIFO
     */
    info_istat.port = info->fifo_status_reg;
    info_istat.space = info->space;

	room = info->xmit_fifo_size - info->xmit_fifo_trigger; /* calculate write room */

    do {
        tip866_out(info, UART_TX, info->xmit_buf[info->xmit_tail++]);
        info->xmit_tail = info->xmit_tail & (SERIAL_XMIT_SIZE-1);
        info->state->icount.tx++;
		room--;
        if (--info->xmit_cnt <= 0)
            break;
    } while (room > 0);
/*    } while ((tip866_in(&info_istat, 0) & info->tx_empty_bit) == 0); if tx empty bit should be used */


    if (info->xmit_cnt < WAKEUP_CHARS)
        tp_sched_event(info, RS_EVENT_WRITE_WAKEUP);

#ifdef TIP866_DEBUG_INTR
    printk("THRE2:%d...", info->state->icount.tx);
#endif
    if (intr_done)
        *intr_done = 0;

    if (info->xmit_cnt <= 0) {
        info->IER &= ~UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
    }
}


/*****************************************************************************
 * check_modem_status()
 *****************************************************************************/
static void check_modem_status(struct info_struct *info)
{
    int status;
    struct  info_icount *icount;


    if (info->state->type != TIP866_20) {
        status = tip866_in(info, UART_MSR);

        if (status & UART_MSR_ANY_DELTA) {
            icount = &info->state->icount;
            /* update input line counters */
            if (status & UART_MSR_DCTS)
                icount->cts++;
            if (status & UART_MSR_TERI)
                icount->rng++;
            if (status & UART_MSR_DDSR)
                icount->dsr++;
            if (status & UART_MSR_DDCD)
                icount->dcd++;
            if (status & UART_MSR_DCTS)
                icount->cts++;
            wake_up_interruptible(&info->delta_msr_wait);
        }

        if ((info->flags & ASYNC_CHECK_CD) && (status & UART_MSR_DDCD)) {
#if (defined(SERIAL_DEBUG_OPEN) || defined(SERIAL_DEBUG_INTR))
            printk("ttys%d CD now %s...", info->line,
                (status & UART_MSR_DCD) ? "on" : "off");
#endif
            if (status & UART_MSR_DCD)
                wake_up_interruptible(&info->open_wait);
            else if (!((info->flags & TIP866_CALLOUT_ACTIVE) &&
                (info->flags & ASYNC_CALLOUT_NOHUP))) {
#ifdef SERIAL_DEBUG_OPEN
                printk("doing serial hangup...");
#endif
                if (info->tty)
                    tty_hangup(info->tty);
            }
        }


        if (info->flags & ASYNC_CTS_FLOW) {
            if (info->tty->hw_stopped) {
                if (status & UART_MSR_CTS) {
#if (defined(TIP866_DEBUG_INTR) || defined(TIP866_DEBUG_FLOW))
                    printk("CTS tx start...");
#endif
                    info->tty->hw_stopped = 0;
                    info->IER |= UART_IER_THRI;
                    tip866_out(info, UART_IER, info->IER);
                    tp_sched_event(info, RS_EVENT_WRITE_WAKEUP);
                    return;
                }
            } else {
                if (!(status & UART_MSR_CTS)) {
#if (defined(TIP866_DEBUG_INTR) || defined(TIP866_DEBUG_FLOW))
                    printk("CTS tx stop...");
#endif
                    info->tty->hw_stopped = 1;
                    info->IER &= ~UART_IER_THRI;
                    tip866_out(info, UART_IER, info->IER);
                }
            }
        }
    }
}


/*
 * This is the tip866 driver's generic interrupt routine
 */
TPMODULE_DEFINE_ISR(tp_interrupt)
{
	struct module_info_struct *mod = (struct module_info_struct *)((struct uart_interrupt_info *)dev_id)->parent;
	unsigned long int_number = ((struct uart_interrupt_info *)dev_id)->controller_num;
	unsigned char ch_min, ch_max;
    struct info_struct *info;
    unsigned int status, int_status;
    int i, retval = IRQ_NONE;
#if defined (TIP866_DEBUG_FIFO)
	static long txICounter = 0, rxICounter = 0;
#endif

    status = ipac_interrupt_ack(mod->ipac, int_number);  /* clear pending interrupt */

	if (int_number)
	{ /* handling 2nd uart controller */
		ch_min = TIP866_CHAN_PER_MOD >> 1;
		ch_max = TIP866_CHAN_PER_MOD;
	}
	else
	{ /* handling 1st uart controller */
		ch_min = 0;
		ch_max = TIP866_CHAN_PER_MOD >> 1;
	}

    for (i = ch_min; i < ch_max; i++)
	{
        while (((int_status = tip866_in(&mod->info[i], UART_ISR)) & UART_ISR_NO_INT) != UART_ISR_NO_INT)
		{
			retval = IRQ_HANDLED;
#if defined (TIP866_DEBUG_XX3)
			printk("[IIR(%d)=%02X] \n", i, int_status);
#endif

#if defined (TIP866_DEBUG_FIFO)
			switch(int_status & UART_ISR_EVMASK)
			{
			case UART_ISR_THRI:	/* Tx Interrupt */
				txICounter++;
				break;
			case UART_ISR_RDI:	/* Rx Interrupt */
				rxICounter++;
				break;
			}
			printk("FIFO_DEBUG: Interrupt Counter: Rx: %ld -- Tx: %ld\n", rxICounter, txICounter);
#endif

			/* service this channel interrupt */
            info = &mod->info[i];

			spin_lock(&info->lock);

            /*
            *  Read the modem status register to see if there is something to do
            */
            status = tip866_in(info, UART_LSR);

            if (status & UART_LSR_DR)
                receive_chars(info, &status);

            check_modem_status(info);

            if ((int_status & UART_ISR_EVMASK) == UART_ISR_THRI)
                transmit_chars(info, 0);

            info->last_active = jiffies;
			spin_unlock(&info->lock);
        }
    }
	TP_IRQ_RETURN(retval);
}


/*
 * -------------------------------------------------------------------
 * Here ends the tip866 interrupt routines.
 * -------------------------------------------------------------------
 */

/*
 * This routine is used to handle the "bottom half" processing for the
 * tip866 driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * tp_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using tp_sched_event(), and they get done here.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void do_softint(void *private_)
{
    struct info_struct *info = (struct info_struct *) private_;
#else
static void do_softint(struct work_struct *work)
{
    struct info_struct  *info = container_of(work, struct info_struct, work);
#endif
    struct tty_struct   *tty;

    tty = info->tty;
    if (!tty)
        return;

    if (test_and_clear_bit(RS_EVENT_WRITE_WAKEUP, &info->event)) {
        if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
            tty->ldisc.write_wakeup)
            (tty->ldisc.write_wakeup)(tty);
        wake_up_interruptible(&tty->write_wait);
    }
}


/*****************************************************************************
 * Low level utility subroutines for the tip866 driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 *****************************************************************************/

static int startup(struct info_struct * info)
{

    int retval=0;
    struct tip866_state *state= info->state;
    unsigned long page;


    page = get_zeroed_page(GFP_KERNEL);
    if (!page)
        return -ENOMEM;

    lock_save(info);

    if (info->flags & TIP866_INITIALIZED) {
        free_page(page);
        goto errout;
    }

    if (!CONFIGURED_SERIAL_PORT(state) || !state->type) {
        if (info->tty)
            set_bit(TTY_IO_ERROR, &info->tty->flags);
        free_page(page);
        goto errout;
    }
    if (info->xmit_buf)
        free_page(page);
    else
        info->xmit_buf = (unsigned char *) page;

#ifdef TIP866_DEBUG_OPEN
    printk("starting up ttyTP%d (irq %d)...", info->line, state->irq);
#endif


    /* Wake up UART */
    tip866_out(info, UART_LCR, 0xBF);
    tip866_out(info, UART_EFR, UART_EFR_ECB);
    /*
     * Turn off LCR == 0xBF so we actually set the IER
     * register on the XR16C850
     */
    tip866_out(info, UART_LCR, 0);
    tip866_out(info, UART_IER, 0);

    tip866_out(info, UART_LCR, 0xBF);

    tip866_out(info, UART_LCR, 0);


    /*
     * Clear the FIFO buffers and disable them
     * (they will be reenabled in change_speed())
     */
    tip866_out(info, UART_FCR, UART_FCR_ENABLE_FIFO);
    tip866_out(info, UART_FCR, (UART_FCR_ENABLE_FIFO |
                      UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT));
    tip866_out(info, UART_FCR, 0);

    /*
     * Clear the interrupt registers.
     */
    (void) tip866_in(info, UART_LSR);
    (void) tip866_in(info, UART_RX);
    (void) tip866_in(info, UART_ISR);
    (void) tip866_in(info, UART_MSR);

    /*
     * At this point there's no way the LSR could still be 0xFF;
     * if it is, then bail out, because there's likely no UART
     * here.
     */
    if (tip866_in(info, UART_LSR) == 0xff) {
        printk("LSR safety check engaged!\n");
        if (capable(CAP_SYS_ADMIN)) {
            if (info->tty)
                set_bit(TTY_IO_ERROR, &info->tty->flags);
        } else
            retval = -ENODEV;
        goto errout;
    }

    /*
     * Now, initialize the UART
     */
    tip866_out(info, UART_LCR, UART_LCR_WLEN8);    /* reset DLAB */

    info->MCR = 0;
    if (info->tty->termios->c_cflag & CBAUD)
        info->MCR = UART_MCR_DTR | UART_MCR_RTS;

    info->MCR |= UART_MCR_OUT2;
    tip866_out(info, UART_MCR, info->MCR);

    /*
     * Finally, enable interrupts
     */
    info->IER = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;
    tip866_out(info, UART_IER, info->IER); /* enable interrupts */

    /*
     * And clear the interrupt registers again for luck.
     */
    (void)tip866_in(info, UART_LSR);
    (void)tip866_in(info, UART_RX);
    (void)tip866_in(info, UART_ISR);
    (void)tip866_in(info, UART_MSR);

    if (info->tty)
        clear_bit(TTY_IO_ERROR, &info->tty->flags);
    info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

    /*
     * Set up the tty->alt_speed kludge
     */
    if (info->tty) {
        if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
            info->tty->alt_speed = 57600;
        if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
            info->tty->alt_speed = 115200;
        if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_SHI)
            info->tty->alt_speed = 230400;
        if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_WARP)
            info->tty->alt_speed = 460800;
    }

    /*
     * and set the speed of the tip866 port
     */
    change_speed(info, 0);

    info->flags |= TIP866_INITIALIZED;
    unlock_restore(info);
    return 0;

errout:
    unlock_restore(info);
    return retval;
} /* end of startup */



/*****************************************************************************
 * This routine will shutdown a tip866 port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 *****************************************************************************/
static void shutdown(struct info_struct * info)
{
    struct tip866_state *state;

    if (!(info->flags & TIP866_INITIALIZED))
        return;

    state = info->state;

#ifdef TIP866_DEBUG_OPEN
    printk("Shutting down tip866 port %d (irq %d)....", info->line,
           state->irq);
#endif

    /*
     * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
     * here so the queue might never be waken up
     */
    wake_up_interruptible(&info->delta_msr_wait);

    if (info->xmit_buf) {
        free_page((unsigned long) info->xmit_buf);
        info->xmit_buf = 0;
    }

    info->IER = 0;
    tip866_out(info, UART_IER, 0x00);  /* disable all intrs */

    info->MCR &= ~UART_MCR_OUT2;

    /* disable break condition */
    tip866_out(info, UART_LCR, tip866_in(info, UART_LCR) & ~UART_LCR_SBC);

    if (!info->tty || (info->tty->termios->c_cflag & HUPCL))
        info->MCR &= ~(UART_MCR_DTR|UART_MCR_RTS);
    tip866_out(info, UART_MCR, info->MCR);

    /* disable FIFO's */
    tip866_out(info, UART_FCR, (UART_FCR_ENABLE_FIFO |
                     UART_FCR_CLEAR_RCVR |
                     UART_FCR_CLEAR_XMIT));
    tip866_out(info, UART_FCR, 0);

    (void)tip866_in(info, UART_RX);    /* read data port to reset things */

    if (info->tty)
        set_bit(TTY_IO_ERROR, &info->tty->flags);

    info->flags &= ~TIP866_INITIALIZED;
}



/*****************************************************************************
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a tip866 port.
 *****************************************************************************/
static void change_speed(struct info_struct *info, struct TP_TERMIOS *old_termios)
{
    int quot = 0, baud_base, baud;
    unsigned cflag, iflag, cval, efr;
    int bits;

    if (!info->tty || !info->tty->termios)
        return;

    cflag = info->tty->termios->c_cflag;
    iflag = info->tty->termios->c_iflag;

    if (!CONFIGURED_SERIAL_PORT(info))
        return;

    /* byte size and parity */
    switch (cflag & CSIZE) {
        case CS5: cval = 0x00; bits = 7; break;
        case CS6: cval = 0x01; bits = 8; break;
        case CS7: cval = 0x02; bits = 9; break;
        case CS8: cval = 0x03; bits = 10; break;
        /* Never happens, but GCC is too dumb to figure it out */
        default:  cval = 0x00; bits = 7; break;
    }

    if (cflag & CSTOPB) {
        cval |= 0x04;
        bits++;
    }
    if (cflag & PARENB) {
        cval |= UART_LCR_PARITY;
        bits++;
    }

    if (!(cflag & PARODD))
        cval |= UART_LCR_EPAR;
#ifdef CMSPAR
    if (cflag & CMSPAR)
        cval |= UART_LCR_SPAR;
#endif

    /* Determine divisor based on baud rate */
    baud = tty_get_baud_rate(info->tty);

    if (!baud) baud = 9600;    /* B0 transition handled in tp_set_termios */
    baud_base = info->state->baud_base;

    if (baud == 134)
        /* Special case since 134 is really 134.5 */
        quot = (2*baud_base / 269);
    else if (baud)
        quot = baud_base / baud;

    /* If the quotient is zero refuse the change */
    if (!quot && old_termios) {
        info->tty->termios->c_cflag &= ~CBAUD;
        info->tty->termios->c_cflag |= (old_termios->c_cflag & CBAUD);
        baud = tty_get_baud_rate(info->tty);

        if (!baud) baud = 9600;

        if (baud == 134)
            /* Special case since 134 is really 134.5 */
            quot = (2*baud_base / 269);
        else if (baud)
            quot = baud_base / baud;
    }


    /* As a last resort, if the quotient is zero, default to 9600 bps */
    if (!quot) quot = baud_base / 9600;

    info->quot = quot;
    info->timeout = ((info->xmit_fifo_size*HZ*bits*quot) / baud_base);
    info->timeout += HZ/50;     /* Add .02 seconds of slop */


    /* CTS flow control flag and modem status interrupts */
    info->IER &= ~UART_IER_MSI;
    if (info->flags & ASYNC_HARDPPS_CD)
        info->IER |= UART_IER_MSI;
    if (cflag & CRTSCTS) {
        info->flags |= ASYNC_CTS_FLOW;
        info->IER |= UART_IER_MSI;
    } else
        info->flags &= ~ASYNC_CTS_FLOW;
    if (cflag & CLOCAL)
        info->flags &= ~ASYNC_CHECK_CD;
    else {
        info->flags |= ASYNC_CHECK_CD;
        info->IER |= UART_IER_MSI;
    }
    tip866_out(info, UART_IER, info->IER);

    /*
     * Set up parity check flag
     */
#define RELEVANT_IFLAG(iflag) (iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

    info->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
    if (I_INPCK(info->tty))
        info->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
    if (I_BRKINT(info->tty) || I_PARMRK(info->tty))
        info->read_status_mask |= UART_LSR_BI;

    /*
     * Characters to ignore
     */
    info->ignore_status_mask = 0;
    if (I_IGNPAR(info->tty))
        info->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
    if (I_IGNBRK(info->tty)) {
        info->ignore_status_mask |= UART_LSR_BI;
        /*
         * If we're ignore parity and break indicators, ignore
         * overruns too.  (For real raw support).
         */
        if (I_IGNPAR(info->tty))
            info->ignore_status_mask |= UART_LSR_OE;
    }
    /*
     * !!! ignore all characters if CREAD is not set
     */
    if ((cflag & CREAD) == 0)
        info->ignore_status_mask |= UART_LSR_DR;

    tip866_out(info, UART_LCR, 0xBF);

    /*
     *  Setup automatic hardware and software flow control. As well as saving
     *  a few CPU cycles it will also greatly improve flow control reliability.
     */
    efr = UART_EFR_ECB;

    efr |= (cflag & CRTSCTS) ? (UART_EFR_CTS | UART_EFR_RTS) : 0;


    /*
     *  Setup Xon/Xoff characters
     */
    tip866_out(info, UART_XON1, info->tty->termios->c_cc[VSTART]);
    tip866_out(info, UART_XOFF1, info->tty->termios->c_cc[VSTOP]);
    tip866_out(info, UART_XON2, 0);
    tip866_out(info, UART_XOFF2, 0);

    if (iflag & IXON) {
        /* enable XON/XOFF flow control on output */
        efr |= UART_EFR_CONT1;  /* receiver compares XON/XOFF */
    }

    if (iflag & IXOFF) {
        /* enable XON/XOFF flow control on input */
        efr |= UART_EFR_CONT3;  /* transmit XON/XOFF */
    }

    tip866_out(info, UART_EFR, efr);


    tip866_out(info, UART_LCR, cval);  /* deselect EFR */


    if (((iflag & IXON) || (iflag & IXOFF)) && (iflag & IXANY)) {
        /* enable any character to restart output */
        info->MCR |= UART_MCR_XON_ANY;
    }
    else {
        info->MCR &= ~UART_MCR_XON_ANY;
    }

    tip866_out(info, UART_MCR, info->MCR);

#ifdef TIP866_DEBUG_XX1
    printk("efr=%x, mcr=%x, iflag=%ooct, xon=%x, xoff=%x\n", efr, info->MCR, iflag,
        info->tty->termios->c_cc[VSTART], info->tty->termios->c_cc[VSTOP]);
#endif



    tip866_out(info, UART_LCR, cval | UART_LCR_DLAB);  /* set DLAB */
    tip866_out(info, UART_DLL, quot & 0xff);   /* LS of divisor */
    tip866_out(info, UART_DLM, quot >> 8);     /* MS of divisor */
    tip866_out(info, UART_LCR, cval);          /* reset DLAB */
    info->LCR = cval;                           /* Save LCR */

	/* Set up FIFO's */
    if (TIP866_ENA_FIFO)
	{
		tip866_out(info, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR_DMA_SELECT |
                     TIP866_RX_TRG_DEF | TIP866_TX_TRG_DEF);

		switch (TIP866_TX_TRG_DEF)
		{
		case UART_FCR_T_TRIGGER_8:
			info->xmit_fifo_trigger = 8;
			break;
		case UART_FCR_T_TRIGGER_16:
			info->xmit_fifo_trigger = 16;
			break;
		case UART_FCR_T_TRIGGER_32:
			info->xmit_fifo_trigger = 32;
			break;
		case UART_FCR_T_TRIGGER_56:
			info->xmit_fifo_trigger = 56;
			break;
		}
    }
    else {
        /* disable FIFO's */
        tip866_out(info, UART_FCR, 0);
		info->xmit_fifo_trigger = info->xmit_fifo_size - 1;
    }
}



static void tp_put_char(struct tty_struct *tty, unsigned char ch)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;


    if (tip866_paranoia_check(info, info->line, "tp_put_char"))
        return;

    if (!tty || !info->xmit_buf)
        return;

    lock_save(info);
    if (info->xmit_cnt >= SERIAL_XMIT_SIZE - 1) {
        unlock_restore(info);
        return;
    }

    info->xmit_buf[info->xmit_head++] = ch;
    info->xmit_head &= SERIAL_XMIT_SIZE-1;
    info->xmit_cnt++;
    unlock_restore(info);
}

static void tp_flush_chars(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;


    if (tip866_paranoia_check(info, info->line, "tp_flush_chars"))
        return;

    if (info->xmit_cnt <= 0 || tty->stopped || tty->hw_stopped ||
        !info->xmit_buf)
        return;

    lock_save(info);
    info->IER |= UART_IER_THRI;
    tip866_out(info, UART_IER, info->IER);
    unlock_restore(info);
}


/* http://www.kernel.org/pub/linux/kernel/v2.6/ChangeLog-2.6.10
<torvalds@evo.osdl.org>
	Update tty layer to not mix kernel and user pointers.

	Instead, tty_io.c will always copy user space data to
	kernel space, leaving the drivers to worry only about
	normal kernel buffers.

	No more "from_user" flag, and having the user copy in
	each driver.

	This cleans up the code and also fixes a number of
	locking bugs.
*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static int tp_write(struct tty_struct * tty, int from_user,
            const unsigned char *buf, int count)
{
#else
static int tp_write(struct tty_struct * tty,
            const unsigned char *buf, int count)
{
	int from_user = 0;
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) */

    int c, ret = 0;
    struct info_struct *info = (struct info_struct *)tty->driver_data;



    if (tip866_paranoia_check(info, info->line, "tp_write"))
        return 0;

    if (!tty || !info->xmit_buf || !tmp_buf)
        return 0;

    if (from_user) {
        down(&tmp_buf_sem);
        while (1) {
            c = MIN(count,
                MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
                    SERIAL_XMIT_SIZE - info->xmit_head));
            if (c <= 0)
                break;

            c -= copy_from_user(tmp_buf, buf, c);
            if (!c) {
                if (!ret)
                    ret = -EFAULT;
                break;
            }
            lock_save(info);
            c = MIN(c, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
                       SERIAL_XMIT_SIZE - info->xmit_head));
            memcpy(info->xmit_buf + info->xmit_head, tmp_buf, c);
            info->xmit_head = ((info->xmit_head + c) &
                       (SERIAL_XMIT_SIZE-1));
            info->xmit_cnt += c;
            unlock_restore(info);
            buf += c;
            count -= c;
            ret += c;
        }
        up(&tmp_buf_sem);
    } else {
        while (1) {
            lock_save(info);
            c = MIN(count,
                MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
                    SERIAL_XMIT_SIZE - info->xmit_head));
            if (c <= 0) {
                unlock_restore(info);
                break;
            }
            memcpy(info->xmit_buf + info->xmit_head, buf, c);
            info->xmit_head = ((info->xmit_head + c) &
                       (SERIAL_XMIT_SIZE-1));
            info->xmit_cnt += c;
            unlock_restore(info);
            buf += c;
            count -= c;
            ret += c;
        }
    }
    if (info->xmit_cnt && !tty->stopped && !tty->hw_stopped &&
        !(info->IER & UART_IER_THRI)) {
        info->IER |= UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
    }
    return ret;
}

static int tp_write_room(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;
    int ret;

    if (tip866_paranoia_check(info, info->line, "tp_write_room"))
        return 0;
    ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
    if (ret < 0)
        ret = 0;
    return ret;
}

static int tp_chars_in_buffer(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;


    if (tip866_paranoia_check(info, info->line, "tp_chars_in_buffer"))
        return 0;
    return info->xmit_cnt;
}

static void tp_flush_buffer(struct tty_struct *tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;

    if (tip866_paranoia_check(info, info->line, "tp_flush_buffer"))
        return;
    info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
    wake_up_interruptible(&tty->write_wait);
    if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
        tty->ldisc.write_wakeup)
        (tty->ldisc.write_wakeup)(tty);
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void tp_send_xchar(struct tty_struct *tty, char ch)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;

    if (tip866_paranoia_check(info, info->line, "tp_send_char"))
        return;

    info->x_char = ch;
    if (ch) {
        /* Make sure transmit interrupts are on */
        info->IER |= UART_IER_THRI;
        tip866_out(info, UART_IER, info->IER);
    }
}

/*
 * ------------------------------------------------------------
 * tp_throttle()
 *
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void tp_throttle(struct tty_struct * tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;

#ifdef TIP866_DEBUG_THROTTLE
    char    buf[64];

    printk("throttle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (tip866_paranoia_check(info, info->line, "tp_throttle"))
        return;

    if (I_IXOFF(tty))
        tp_send_xchar(tty, STOP_CHAR(tty));

    lock_save(info);
    if (tty->termios->c_cflag & CRTSCTS)
        info->MCR &= ~UART_MCR_RTS;

	tip866_out(info, UART_MCR, info->MCR);
    unlock_restore(info);
}

static void tp_unthrottle(struct tty_struct * tty)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;

#ifdef TIP866_DEBUG_THROTTLE
    char    buf[64];

    printk("unthrottle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (tip866_paranoia_check(info, info->line, "tp_unthrottle"))
        return;

    if (I_IXOFF(tty)) {
        if (info->x_char)
            info->x_char = 0;
        else
            tp_send_xchar(tty, START_CHAR(tty));
    }
    if (tty->termios->c_cflag & CRTSCTS)
        info->MCR |= UART_MCR_RTS;
    /*lock_save(info);*/
    tip866_out(info, UART_MCR, info->MCR);
    /*unlock_restore(info);*/
}



/*
 * ------------------------------------------------------------
 * tp_ioctl() and friends
 * ------------------------------------------------------------
 */

/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 *      is emptied.  On bus types like RS485, the transmitter must
 *      release the bus after transmitting. This must be done when
 *      the transmit shift register is empty, not be done when the
 *      transmit holding register is empty.  This functionality
 *      allows an RS485 driver to be written in user space.
 */
static int get_lsr_info(struct info_struct * info, unsigned int *value)
{
    unsigned char status;
    unsigned int result;


    status = tip866_in(info, UART_LSR);
    result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);

    /*
     * If we're about to load something into the transmit
     * register, we'll pretend the transmitter isn't empty to
     * avoid a race condition (depending on when the transmit
     * interrupt happens).
     */
    if (info->x_char ||
        ((info->xmit_cnt > 0) && !info->tty->stopped &&
         !info->tty->hw_stopped))
        result &= TIOCSER_TEMT;

    if (copy_to_user(value, &result, sizeof(int)))
        return -EFAULT;
    return 0;
}


static int get_modem_info(struct info_struct * info, unsigned int *value)
{
    unsigned char control, status;
    unsigned int result;



    control = info->MCR;
    status = tip866_in(info, UART_MSR);

    if (info->state->type != TIP866_20) {
        result =  ((control & UART_MCR_RTS) ? TIOCM_RTS : 0)
		| ((control & UART_MCR_DTR) ? TIOCM_DTR : 0)
		| ((status  & UART_MSR_DCD) ? TIOCM_CAR : 0)
		| ((status  & UART_MSR_RI)  ? TIOCM_RNG : 0)
		| ((status  & UART_MSR_DSR) ? TIOCM_DSR : 0)
		| ((status  & UART_MSR_CTS) ? TIOCM_CTS : 0);
    }
    else {
        result = 0;
    }

    if (copy_to_user(value, &result, sizeof(int)))
        return -EFAULT;
    return 0;
}

static int set_modem_info(struct info_struct * info, unsigned int cmd,
              unsigned int *value)
{
    unsigned int arg;


    /* for TIP866-20 modem settings will be ignored */
    if (info->state->type != TIP866_20) {
        if (copy_from_user(&arg, value, sizeof(int)))
            return -EFAULT;

        switch (cmd) {
        case TIOCMBIS:
            if (arg & TIOCM_RTS)
                info->MCR |= UART_MCR_RTS;
            if (arg & TIOCM_DTR)
                info->MCR |= UART_MCR_DTR;
            if (arg & TIOCM_LOOP)
                info->MCR |= UART_MCR_LOOP;
            break;
        case TIOCMBIC:
            if (arg & TIOCM_RTS)
                info->MCR &= ~UART_MCR_RTS;
            if (arg & TIOCM_DTR)
                info->MCR &= ~UART_MCR_DTR;
            if (arg & TIOCM_LOOP)
                info->MCR &= ~UART_MCR_LOOP;
            break;
        case TIOCMSET:
            info->MCR = ((info->MCR & ~(UART_MCR_RTS |
                UART_MCR_LOOP |
                UART_MCR_DTR))
                | ((arg & TIOCM_RTS) ? UART_MCR_RTS : 0)
                | ((arg & TIOCM_LOOP) ? UART_MCR_LOOP : 0)
                | ((arg & TIOCM_DTR) ? UART_MCR_DTR : 0));
        default:
            return -EINVAL;
        }
        tip866_out(info, UART_MCR, info->MCR);
    }

    return 0;
}


/*
 * tp_break() --- routine which turns the break handling on or off
 */
static void tp_break(struct tty_struct *tty, int break_state)
{
    struct info_struct * info = (struct info_struct *)tty->driver_data;



    if (tip866_paranoia_check(info, info->line, "tp_break"))
        return;

    if (!CONFIGURED_SERIAL_PORT(info))
        return;
    lock_save(info);
    if (break_state == -1)
        info->LCR |= UART_LCR_SBC;
    else
        info->LCR &= ~UART_LCR_SBC;
    tip866_out(info, UART_LCR, info->LCR);
    unlock_restore(info);
}



/*****************************************************************************
 * tp_ioctl()
 *
 *****************************************************************************/

static int tp_ioctl(struct tty_struct *tty, struct file * file,
            unsigned int cmd, unsigned long arg)
{
    struct info_struct * info = (struct info_struct *)tty->driver_data;
    struct info_icount cprev, cnow;    /* kernel counter temps */
    struct serial_icounter_struct icount;
	DECLARE_WAITQUEUE(wait, current);
	int result = 0;


    if (tip866_paranoia_check(info, info->line, "tp_ioctl"))
        return -ENODEV;

    if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
        (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) &&
        (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
        if (tty->flags & (1 << TTY_IO_ERROR))
            return -EIO;
    }

    switch (cmd) {
        case TIOCMGET:
            return get_modem_info(info, (unsigned int *) arg);
        case TIOCMBIS:
        case TIOCMBIC:
        case TIOCMSET:
            return set_modem_info(info, cmd, (unsigned int *) arg);

        case TIOCSERGETLSR: /* Get line status register */
            return get_lsr_info(info, (unsigned int *) arg);

        case TIOCSERGSTRUCT:
            if (copy_to_user((struct info_struct *) arg,
                     info, sizeof(struct info_struct)))
                return -EFAULT;
            return 0;

        /*
         * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
         * - mask passed in arg for lines of interest
         *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
         * Caller should use TIOCGICOUNT to see which one it was
         */
        case TIOCMIWAIT:
            if (info->state->type != TIP866_20) {
                lock_save(info);
                /* note the counters on entry */
                cprev = cnow = info->state->icount;
                unlock_restore(info);
                /* Force modem status interrupts on */
                info->IER |= UART_IER_MSI;
                tip866_out(info, UART_IER, info->IER);
				add_wait_queue(&info->delta_msr_wait, &wait);
                while (1) {
					set_current_state(TASK_INTERRUPTIBLE);

                    if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
                        ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
                        ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
                        ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) )
					{
                        break;
                    }
                    cprev = cnow;

					schedule();

					if (signal_pending(current))
					{
                        result = -ERESTARTSYS;
						break;
					}

					lock_save(info);
                    cnow = info->state->icount; /* atomic copy */
                    unlock_restore(info);
                    if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
                        cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
					{
                        result = -EIO; /* no change => error */
						break;
					}
                }
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&info->delta_msr_wait, &wait);
				return result;
            }
            else {
                return -ENOIOCTLCMD;
            }
            /* NOTREACHED */

        /*
         * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
         * Return: write counters to the user passed counter struct
         * NB: both 1->0 and 0->1 transitions are counted except for
         *     RI where only 0->1 is counted.
         */
        case TIOCGICOUNT:
            lock_save(info);
            cnow = info->state->icount;
            unlock_restore(info);
            icount.cts = cnow.cts;
            icount.dsr = cnow.dsr;
            icount.rng = cnow.rng;
            icount.dcd = cnow.dcd;
            icount.rx = cnow.rx;
            icount.tx = cnow.tx;
            icount.frame = cnow.frame;
            icount.overrun = cnow.overrun;
            icount.parity = cnow.parity;
            icount.brk = cnow.brk;
            icount.buf_overrun = cnow.buf_overrun;

            if (copy_to_user((void *)arg, &icount, sizeof(icount)))
                return -EFAULT;
            return 0;

        default:
            return -ENOIOCTLCMD;
        }
    return 0;
}

static void tp_set_termios(struct tty_struct *tty, struct TP_TERMIOS *old_termios)
{
    struct info_struct *info = (struct info_struct *)tty->driver_data;

    unsigned int cflag = tty->termios->c_cflag;


    if (   (cflag == old_termios->c_cflag)
        && (   RELEVANT_IFLAG(tty->termios->c_iflag)
        == RELEVANT_IFLAG(old_termios->c_iflag)))
      return;

    change_speed(info, old_termios);

    /* Handle transition to B0 status */
    if ((old_termios->c_cflag & CBAUD) && !(cflag & CBAUD))
	{
        lock_save(info);
        info->MCR &= ~(UART_MCR_DTR|UART_MCR_RTS);
        tip866_out(info, UART_MCR, info->MCR);
        unlock_restore(info);
    }

    /* Handle transition away from B0 status */
    if (!(old_termios->c_cflag & CBAUD) && (cflag & CBAUD))
	{
        lock_save(info);
        info->MCR |= UART_MCR_DTR;
        if (!(tty->termios->c_cflag & CRTSCTS) ||
            !test_bit(TTY_THROTTLED, &tty->flags)) {
            info->MCR |= UART_MCR_RTS;
        }
        tip866_out(info, UART_MCR, info->MCR);
        unlock_restore(info);
    }

    /* Handle turning off CRTSCTS */
    if ((old_termios->c_cflag & CRTSCTS) &&
        !(tty->termios->c_cflag & CRTSCTS)) {
        tty->hw_stopped = 0;
        tp_start(tty);
    }
}

/*
 * ------------------------------------------------------------
 * tp_close()
 *
 * This routine is called when the tip866 port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 * ------------------------------------------------------------
 */
static void tp_close(struct tty_struct *tty, struct file * filp)
{
    struct info_struct * info = (struct info_struct *)tty->driver_data;
    struct tip866_state *state;


    if (!info || tip866_paranoia_check(info, info->line, "tp_close"))
        return;

    state = info->state;

    lock_save(info);

    if (tty_hung_up_p(filp)) {
        DBG_CNT("before DEC-hung");
        TP_MOD_DEC_USE_COUNT;
        unlock_restore(info);
        return;
    }

#ifdef TIP866_DEBUG_OPEN
    printk("tp_close ttyTP%d, count = %d\n", info->line, state->count);
#endif
    if ((tty->count == 1) && (state->count != 1)) {
        /*
         * Uh, oh.  tty->count is 1, which means that the tty
         * structure will be freed.  state->count should always
         * be one in these conditions.  If it's greater than
         * one, we've got real problems, since it means the
         * tip866 port won't be shutdown.
         */
        printk("tp_close: bad tip866 port count; tty->count is 1, "
               "state->count is %d\n", state->count);
        state->count = 1;
    }
    if (--state->count < 0) {
        printk("tp_close: bad tip866 port count for ttyTP%d: %d\n",
               info->line, state->count);
        state->count = 0;
    }
    if (state->count) {
        DBG_CNT("before DEC-2");
        TP_MOD_DEC_USE_COUNT;
        unlock_restore(info);
        return;
    }
    info->flags |= TIP866_CLOSING;
    /*
     * Save the termios structure, since this port may have
     * separate termios for callout and dialin.
     */
    if (info->flags & TIP866_NORMAL_ACTIVE)
        info->state->normal_termios = *tty->termios;
    if (info->flags & TIP866_CALLOUT_ACTIVE)
        info->state->callout_termios = *tty->termios;
    /*
     * Now we wait for the transmit buffer to clear; and we notify
     * the line discipline to only process XON/XOFF characters.
     */
    tty->closing = 1;
    if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
	{
		unlock_restore(info);
        tty_wait_until_sent(tty, info->closing_wait);
		lock_save(info);
	}
    /*
     * At this point we stop accepting input.  To do this, we
     * disable the receive line status interrupts, and tell the
     * interrupt driver to stop checking the data ready bit in the
     * line status register.
     */
    info->IER &= ~UART_IER_RLSI;
    info->read_status_mask &= ~UART_LSR_DR;
    if (info->flags & TIP866_INITIALIZED) {
        tip866_out(info, UART_IER, info->IER);
        /*
         * Before we drop DTR, make sure the UART transmitter
         * has completely drained; this is especially
         * important if there is a transmit FIFO!
         */
		unlock_restore(info);
        tp_wait_until_sent(tty, info->timeout);
		lock_save(info);
    }
    shutdown(info);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    if (tty->driver.flush_buffer)
        tty->driver.flush_buffer(tty);
#else
    if (tty->driver->flush_buffer)
        tty->driver->flush_buffer(tty);
#endif

	if (tty->ldisc.flush_buffer)
        tty->ldisc.flush_buffer(tty);

	tty->closing = 0;
    info->event = 0;
    info->tty = 0;
    if (info->blocked_open) {
        if (info->close_delay) {
			unlock_restore(info);
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(info->close_delay);
			lock_save(info);
        }
        wake_up_interruptible(&info->open_wait);
    }
    info->flags &= ~(TIP866_NORMAL_ACTIVE|TIP866_CALLOUT_ACTIVE|
             TIP866_CLOSING);
    wake_up_interruptible(&info->close_wait);
    TP_MOD_DEC_USE_COUNT;
    unlock_restore(info);
} /* end of tp_close() */

/*
 * tp_wait_until_sent() --- wait until the transmitter is empty
 */
static void tp_wait_until_sent(struct tty_struct *tty, int timeout)
{
    struct info_struct * info = (struct info_struct *)tty->driver_data;
    unsigned long orig_jiffies, char_time;
    int lsr;


    if (tip866_paranoia_check(info, info->line, "tp_wait_until_sent"))
        return;

    if (info->state->type == PORT_UNKNOWN)
        return;

    if (info->xmit_fifo_size == 0)
        return; /* Just in case.... */

    orig_jiffies = jiffies;
    /*
     * Set the check interval to be 1/5 of the estimated time to
     * send a single character, and make it at least 1.  The check
     * interval should also be less than the timeout.
     *
     * Note: we have to use pretty tight timings here to satisfy
     * the NIST-PCTS.
     */
    char_time = (info->timeout - HZ/50) / info->xmit_fifo_size;
    char_time = char_time / 5;
    if (char_time == 0)
        char_time = 1;
    if (timeout)
      char_time = MIN(char_time, timeout);
    /*
     * If the transmitter hasn't cleared in twice the approximate
     * amount of time to send the entire FIFO, it probably won't
     * ever clear.  This assumes the UART isn't doing flow
     * control, which is currently the case.  Hence, if it ever
     * takes longer than info->timeout, this is probably due to a
     * UART bug of some kind.  So, we clamp the timeout parameter at
     * 2*info->timeout.
     */
    if (!timeout || timeout > 2*info->timeout)
        timeout = 2*info->timeout;
#ifdef TIP866_DEBUG_RS_WAIT_UNTIL_SENT
    printk("In tp_wait_until_sent(%d) check=%lu...", timeout, char_time);
    printk("jiff=%lu...", jiffies);
#endif
    while (!((lsr = tip866_in(info, UART_LSR)) & UART_LSR_TEMT)) {
#ifdef TIP866_DEBUG_RS_WAIT_UNTIL_SENT
        printk("lsr = %d (jiff=%lu)...", lsr, jiffies);
#endif
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(char_time);
        if (signal_pending(current))
            break;
        if (timeout && time_after(jiffies, orig_jiffies + timeout))
            break;
    }
    set_current_state(TASK_RUNNING);
#ifdef TIP866_DEBUG_RS_WAIT_UNTIL_SENT
    printk("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
}

/*
 * tp_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void tp_hangup(struct tty_struct *tty)
{
    struct info_struct * info = (struct info_struct *)tty->driver_data;
    struct tip866_state *state = info->state;

    if (tip866_paranoia_check(info, info->line, "tp_hangup"))
        return;

    state = info->state;

    tp_flush_buffer(tty);
    shutdown(info);
    info->event = 0;
    state->count = 0;
    info->flags &= ~(TIP866_NORMAL_ACTIVE|TIP866_CALLOUT_ACTIVE);
    info->tty = 0;
    wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * tip866_open() and friends
 * ------------------------------------------------------------
 */
static int block_til_ready(struct tty_struct *tty, struct file * filp,
               struct info_struct *info)
{
    DECLARE_WAITQUEUE(wait, current);
    struct tip866_state *state = info->state;
    int     retval;
    int     do_clocal = 0, extra_count = 0;

    /*
     * If the device is in the middle of being closed, then block
     * until it's done, and then try again.
     */
    if (tty_hung_up_p(filp) || (info->flags & TIP866_CLOSING))
	{
		add_wait_queue(&info->close_wait, &wait);
        while (1)
		{
			set_current_state(TASK_INTERRUPTIBLE);
			if (!(info->flags & TIP866_CLOSING)) break;
			schedule();
			if (signal_pending(current)) break;
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&info->close_wait, &wait);

#ifdef SERIAL_DO_RESTART
        return ((info->flags & ASYNC_HUP_NOTIFY) ?
            -EAGAIN : -ERESTARTSYS);
#else
        return -EAGAIN;
#endif
    }

    /*
     * If this is a callout device, then just make sure the normal
     * device isn't being used.
     */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    if (tty->driver.subtype == SERIAL_TYPE_CALLOUT) {
        if (info->flags & TIP866_NORMAL_ACTIVE)
            return -EBUSY;
        if ((info->flags & TIP866_CALLOUT_ACTIVE) &&
            (info->flags & ASYNC_SESSION_LOCKOUT) &&
            (info->session != current->session))
            return -EBUSY;
        if ((info->flags & TIP866_CALLOUT_ACTIVE) &&
            (info->flags & ASYNC_PGRP_LOCKOUT) &&
            (info->pgrp != current->pgrp))
            return -EBUSY;
        info->flags |= TIP866_CALLOUT_ACTIVE;
        return 0;
    }
#endif

    /*
     * If non-blocking mode is set, or the port is not enabled,
     * then make the check up front and then exit.
     */
    if ((filp->f_flags & O_NONBLOCK) ||
        (tty->flags & (1 << TTY_IO_ERROR))) {
        if (info->flags & TIP866_CALLOUT_ACTIVE)
            return -EBUSY;
        info->flags |= TIP866_NORMAL_ACTIVE;
        return 0;
    }

    if (info->flags & TIP866_CALLOUT_ACTIVE) {
        if (state->normal_termios.c_cflag & CLOCAL)
            do_clocal = 1;
    } else {
        if (tty->termios->c_cflag & CLOCAL)
            do_clocal = 1;
    }


    /*
     * Block waiting for the carrier detect and the line to become
     * free (i.e., not in use by the callout).  While we are in
     * this loop, state->count is dropped by one, so that
     * tp_close() knows when to free things.  We restore it upon
     * exit, either normal or abnormal.
     */
    retval = 0;
    add_wait_queue(&info->open_wait, &wait);
#ifdef TIP866_DEBUG_OPEN
    printk("block_til_ready before block: ttyTP%d, count = %d\n",
           state->line, state->count);
#endif
    lock_save(info);
    if (!tty_hung_up_p(filp)) {
        extra_count = 1;
        state->count--;
    }
    info->blocked_open++;
    unlock_restore(info);
    while (1) {
        lock_save(info);
        if (!(info->flags & TIP866_CALLOUT_ACTIVE) &&
            (tty->termios->c_cflag & CBAUD))
            tip866_out(info, UART_MCR,
                   tip866_in(info, UART_MCR) |
                   (UART_MCR_DTR | UART_MCR_RTS));
        unlock_restore(info);
        set_current_state(TASK_INTERRUPTIBLE);
        if (tty_hung_up_p(filp) ||
            !(info->flags & TIP866_INITIALIZED)) {
#ifdef SERIAL_DO_RESTART
            if (info->flags & ASYNC_HUP_NOTIFY)
                retval = -EAGAIN;
            else
                retval = -ERESTARTSYS;
#else
            retval = -EAGAIN;
#endif
            break;
        }
        if (info->state->type != TIP866_20) {
            if (!(info->flags & TIP866_CALLOUT_ACTIVE) &&
                !(info->flags & TIP866_CLOSING) &&
                (do_clocal || (tip866_in(info, UART_MSR) &
                UART_MSR_DCD)))
                break;
        }
        else {
            if (!(info->flags & TIP866_CALLOUT_ACTIVE) &&
                !(info->flags & TIP866_CLOSING) && do_clocal)
                break;
        }
        if (signal_pending(current)) {
            retval = -ERESTARTSYS;
            break;
        }
#ifdef TIP866_DEBUG_OPEN
        printk("block_til_ready blocking: ttyTP%d, count = %d\n",
               info->line, state->count);
#endif
        schedule();
    }
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&info->open_wait, &wait);
    if (extra_count)
        state->count++;
    info->blocked_open--;
#ifdef TIP866_DEBUG_OPEN
    printk("block_til_ready after blocking: ttyTP%d, count = %d\n",
           info->line, state->count);
#endif
    if (retval)
        return retval;
    info->flags |= TIP866_NORMAL_ACTIVE;
    return 0;
} /* end of block_til_ready */


/*
 * This routine is called whenever a tip866 port is opened.  It
 * enables interrupts for a tip866 port, linking in its async structure into
 * the IRQ chain.   It also performs the tip866-specific
 * initialization for the tty structure.
 */
static int tip866_open(struct tty_struct *tty, struct file * filp)
{
    int  retval, line;
    struct info_struct *info;
    unsigned long page;
	DECLARE_WAITQUEUE(wait, current);


	TP_MOD_INC_USE_COUNT;
    /*
     *  First get the minor device number and check if this a valid device
     */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    line = MINOR(tty->device) - tty->driver.minor_start;
#else
    line = MINOR(tty->index) - tty->driver->minor_start;
#endif

    if ((line / TIP866_CHAN_PER_MOD) >= module_count) {
        TP_MOD_DEC_USE_COUNT;
        return -ENODEV;
    }

    /*
     *  Get a pointer to the info structure from the module table for this minor device
     *
     *  Organization:
     *
     *           module 0   module 1    module 2     ...
     *  ----------------------------------------------------------
     *  minor       0          8           16
     *  minor       1          9           17
     *  minor       2          10          18
     *             ...        ...         ...
     *
     *  minor       7          15
     *
     */
    info = &module_table[line / TIP866_CHAN_PER_MOD]->info[line % TIP866_CHAN_PER_MOD];

    info->state->count++;

    tty->driver_data = info;
    info->tty = tty;
    if (tip866_paranoia_check(info, info->line, "tip866_open"))
        return -ENODEV;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	OPNDBG("tip866_open %s%d, count = %d, tty=%p\n", tty->driver.name, info->line, info->state->count, tty);
#else
	OPNDBG("tip866_open %s%d, count = %d, tty=%p\n", tty->driver->name, info->line, info->state->count, tty);
#endif

    info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    if (!tmp_buf) {
        page = get_zeroed_page(GFP_KERNEL);
        if (!page) {
            return -ENOMEM;
        }
        if (tmp_buf)
            free_page(page);
        else
            tmp_buf = (unsigned char *) page;
    }
    /*
     * If the port is the middle of closing, bail out now
     */
    if (tty_hung_up_p(filp) || (info->flags & TIP866_CLOSING))
	{
		add_wait_queue(&info->close_wait, &wait);
        while (1)
		{
			set_current_state(TASK_INTERRUPTIBLE);
			if (!(info->flags & TIP866_CLOSING)) break;
			schedule();
			if (signal_pending(current)) break;
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&info->close_wait, &wait);

#ifdef SERIAL_DO_RESTART
        return ((info->flags & ASYNC_HUP_NOTIFY) ?
            -EAGAIN : -ERESTARTSYS);
#else
        return -EAGAIN;
#endif
    }

    /*
     * Start up tip866 port
     */
    retval = startup(info);
    if (retval) {
        return retval;
    }

    retval = block_til_ready(tty, filp, info);
    if (retval) {
#ifdef TIP866_DEBUG_OPEN
        printk("tip866_open returning after block_til_ready with %d\n",
               retval);
#endif
        return retval;
    }

    if ((info->state->count == 1) &&
        (info->flags & ASYNC_SPLIT_TERMIOS)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
#else
        if (tty->driver->subtype == SERIAL_TYPE_NORMAL)
#endif
            *tty->termios = info->state->normal_termios;
        else
            *tty->termios = info->state->callout_termios;
        change_speed(info, 0);
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    info->session = current->session;
    info->pgrp = current->pgrp;
#endif

    OPNDBG("tip866_open ttyST866_%d successful...\n", info->line);

	return 0;
}

/*
 * /proc fs routines....
 */

static inline int line_info(char *buf, struct tip866_state *state)
{
    struct info_struct *info = state->info, scr_info;
    char    control, status;
    int ret;


    ret = sprintf(buf, "%d: uart:%s port:0x%X irq:%d",
        state->line, "ST16C654",
		(int)(info->space->physical_address + state->port), state->irq);

    if (state->type == PORT_UNKNOWN) {
        ret += sprintf(buf+ret, "\n");
        return ret;
    }

    /*
     * Figure out the current RS-232 lines
     */
    if (!info) {
        info = &scr_info;   /* This is just for serial_{in,out} */

        info->magic = TIP866_MAGIC;
        info->port = state->port;
        info->flags = state->flags;
        info->quot = 0;
        info->tty = 0;
    }
    lock_save(info);
    status = tip866_in(info, UART_MSR);
    control = info ? info->MCR : tip866_in(info, UART_MCR);
    unlock_restore(info);

    if (info->quot) {
        ret += sprintf(buf+ret, " baud:%d",
                   state->baud_base / info->quot);
    }

    ret += sprintf(buf+ret, " tx:%d rx:%d",
              state->icount.tx, state->icount.rx);

    if (state->icount.frame)
        ret += sprintf(buf+ret, " fe:%d", state->icount.frame);

    if (state->icount.parity)
        ret += sprintf(buf+ret, " pe:%d", state->icount.parity);

    if (state->icount.brk)
        ret += sprintf(buf+ret, " brk:%d", state->icount.brk);

    if (state->icount.overrun)
        ret += sprintf(buf+ret, " oe:%d", state->icount.overrun);

    /*
     * Last thing is the RS-232 status lines
     */
    if (info->state->type != TIP866_20)
	{
        if (control & UART_MCR_RTS)
            ret += sprintf(buf+ret, "|RTS");
        if (status & UART_MSR_CTS)
            ret += sprintf(buf+ret, "|CTS");
        if (control & UART_MCR_DTR)
            ret += sprintf(buf+ret, "|DTR");
        if (status & UART_MSR_DSR)
            ret += sprintf(buf+ret, "|DSR");
        if (status & UART_MSR_DCD)
            ret += sprintf(buf+ret, "|CD");
        if (status & UART_MSR_RI)
            ret += sprintf(buf+ret, "|RI");
    }

	ret += sprintf(buf+ret, "\n");

	return ret;
}

int tp_read_proc(char *page, char **start, off_t off, int count,
         int *eof, void *data)
{
    int i, len = 0, l;
    off_t   begin = 0;
    struct tip866_state  *state;

    len += sprintf(page, "TEWS TECHNOLOGIES - TIP866 serial driver - V%s, date %s\n",
               DRIVER_VERSION, DRIVER_REVDATE);

    for (i = 0; i < NR_PORTS && len < 4000; i++)
	{
        state = &module_table[i / TIP866_CHAN_PER_MOD]->state[i % TIP866_CHAN_PER_MOD];
        if (state == NULL) break;

        l = line_info(page + len, state);
        len += l;
        if (len+begin > off+count)
            goto done;
        if (len+begin < off) {
            begin += len;
            len = 0;
        }
    }
    *eof = 1;
done:
    if (off >= len+begin)
        return 0;
    *start = page + (begin-off);
    return ((count < begin+len-off) ? count : begin+len-off);
}

/*
 * ---------------------------------------------------------------------
 * tp_init() and friends
 *
 * tp_init() is called at boot-time to initialize the tip866 driver.
 * ---------------------------------------------------------------------
 */

/*
 * This routine prints out the appropriate tip866 driver version
 * number, and identifies which options were configured into this
 * driver.
 */
static void show_tip866_version(void)
{
    printk(KERN_INFO "%s version %s (%s)\n", tip866_name,
           DRIVER_VERSION, DRIVER_REVDATE);
}




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
        mod->state[i].xmit_fifo_size = 64;
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
