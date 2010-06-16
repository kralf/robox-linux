/* $Id: tip866def.h 87 2007-07-26 11:51:45Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @  T I P 8 6 6 D E F  @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          TIP866 - Device Driver                                **
**                                                                           **
**    File             tip866def.h                                           **
**                                                                           **
**    Description      Local driver definitions and prototypes               **
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
**                     Copyright (c) 2006                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**                                                                           **
**    System           Linux                                                 **
**                                                                           **
**    $Date: 2007-07-26 13:51:45 +0200 (Do, 26 Jul 2007) $   $Rev: 87 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/


#ifndef __TIP866DEF_H
#define __TIP866DEF_H

#define TIP866_CHAN_PER_MOD    2
#define TIP866_MAX_NUM_MOD     32
#define TIP866_CLOCK_RATE      7987200
#define TIP866_CHAN_SPAN       0x10


#define MANUFACTURER_TEWS      0xB3    /* TEWS TECHNOLOGIES */
#define MODULE_TIP866          0x1D

#define TIP866_10              10
#define TIP866_11              11
#define TIP866_20              20

#define TIP866_INTVEC          0x0f

/*
 * Counters of the input lines (CTS, DSR, RI, CD) interrupts
 */
struct info_icount {
	__u32	cts, dsr, rng, dcd, tx, rx;
	__u32	frame, parity, overrun, brk;
	__u32	buf_overrun;
};

/* Internal flags  */
#define TIP866_INITIALIZED       0x80000000 /* Serial port was initialized */
#define TIP866_CALLOUT_ACTIVE    0x40000000 /* Call out device is active */
#define TIP866_NORMAL_ACTIVE     0x20000000 /* Normal device is active */
#define TIP866_CLOSING           0x08000000 /* Serial port is closing */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
#define TP_TERMIOS      termios
#else
#define TP_TERMIOS      ktermios
#endif


struct tip866_state {
	int	baud_base;
    struct addr_space_desc  *space;
	int	port;
	int	irq;
	int	flags;
	int	type;
	int	line;
	int	xmit_fifo_size;
	int	count;
	unsigned short	    close_delay;
	unsigned short	    closing_wait; /* time to wait before closing */
	struct info_icount	icount;
	struct TP_TERMIOS		normal_termios;
	struct TP_TERMIOS		callout_termios;
	struct info_struct  *info;
};

struct info_struct {
	int			        magic;
    struct addr_space_desc  *space;
	int			        port;
	int			        flags;
	int			        xmit_fifo_size;
	struct tip866_state	*state;
	struct tty_struct 	*tty;
	int			        read_status_mask;
	int			        ignore_status_mask;
	int			        timeout;
	int			        quot;
	int			        x_char;	/* xon/xoff character */
	int			        close_delay;
	unsigned short		closing_wait;
	unsigned short		closing_wait2;
	int			        IER;    /* Interrupt Enable Register */
	int			        MCR;    /* Modem control register */
	int			        LCR;    /* Line control register */
	unsigned long		event;
	unsigned long		last_active;
	int			        line;
	int			        blocked_open; /* # of blocked opens */
	long			    session; /* Session of opening process */
	long			    pgrp; /* pgrp of opening process */
	unsigned char 		*xmit_buf;
	int			        xmit_head;
	int			        xmit_tail;
	int			        xmit_cnt;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    /* interrupt bottom-half for Kernel 2.4.x */
    struct tq_struct    tqueue;
#else
    /* interrupt bottom-half for Kernel 2.6.x */
    struct work_struct  work;
#endif
    wait_queue_head_t   open_wait;
    wait_queue_head_t   close_wait;
    wait_queue_head_t   delta_msr_wait;

	int                 tx_empty_bit;       /* bit in the FIFO status register for this port */
	int                 fifo_status_reg;    /* this two parameters describes the corresponding */
	struct device		*dev;
    spinlock_t			lock;
	unsigned long		lock_flags;

	int					xmit_fifo_trigger;	/*	*/
        int                                     rcpt_fifo_trigger;      /*      */
};


struct uart_interrupt_info
{
	void *parent;			/* pointer to parent module_info_struct */
	int controller_num;		/* 0 - 1st UART (INT0), 1 - 2nd UART (INT1) */
};

struct module_info_struct
{
    struct tip866_state     state[TIP866_CHAN_PER_MOD];
    struct info_struct      info[TIP866_CHAN_PER_MOD];
    struct ipac_module      *ipac;
	struct uart_interrupt_info _1st_uart;
	struct uart_interrupt_info _2nd_uart;
};



#define CONFIGURED_SERIAL_PORT(info) ((info)->space)

#define TIP866_MAGIC 0x4058

/*
 * The size of the serial xmit buffer is 1 page, or 4096 bytes
 */
#ifndef SERIAL_XMIT_SIZE
#define SERIAL_XMIT_SIZE 4096
#endif
/*
 * The size of the serial FIFO
 */
#ifndef SERIAL_FIFO_SIZE
#define SERIAL_FIFO_SIZE 64
#endif

/*
 * Events are used to schedule things to happen at timer-interrupt
 * time, instead of at rs interrupt time.
 */
#define RS_EVENT_WRITE_WAKEUP	0


/*
 *  16C654 and XR16C864 Hardware Register definitions
 */

#define UART_RX     0x1 /* In:  Receive buffer (DLAB=0) */
#define UART_TX     0x1 /* Out: Transmit buffer (DLAB=0) */
#define UART_DLL    0x1 /* Out: Divisor Latch Low (DLAB=1) */
#define UART_TRG    0x1 /* (LCR=BF) FCTR bit 7 selects Rx or Tx
                 * In: Fifo count
                 * Out: Fifo custom trigger levels
                 * XR16C85x only */

#define UART_DLM    0x3 /* Out: Divisor Latch High (DLAB=1) */
#define UART_IER    0x3 /* Out: Interrupt Enable Register */
#define UART_FCTR   0x3 /* (LCR=BF) Feature Control Register
                 * XR16C85x only */

#define UART_ISR    0x5 /* In:  Interrupt ID Register */
#define UART_FCR    0x5 /* Out: FIFO Control Register */
#define UART_EFR    0x5 /* I/O: Extended Features Register */
                /* (DLAB=1, 16C660 only) */

#define UART_LCR    0x7 /* Out: Line Control Register */
#define UART_MCR    0x9 /* Out: Modem Control Register */
#define UART_LSR    0xb /* In:  Line Status Register */
#define UART_MSR    0xd /* In:  Modem Status Register */
#define UART_SCR    0xf /* I/O: Scratch Register */
#define UART_EMSR   0xf /* (LCR=BF) Extended Mode Select Register */
                        /* FCTR bit 6 selects SCR or EMSR XR16c85x only */

#define UART_XON1   0x9 /* 1st Xon character for automatic software handshake */
#define UART_XON2   0xb /* 2nd Xon character for automatic software handshake */
#define UART_XOFF1  0xd /* 1st Xoff character for automatic software handshake */
#define UART_XOFF2  0xf /* 2nd Xoff character for automatic software handshake */

#define TIP_VECTOR  0x0F    /* Interrupt Vector Register */
#define TIP_FRR14   0x1F    /* TIP866 FIFO ready register for CH1..CH4 */
#define TIP_FRR58   0x5F    /* TIP866 FIFO ready register for CH5..CH8 */
#define TIP_BOARD_OPTION  0x19    /* ID space offset */

/*
 * These are the definitions for the FIFO Control Register
 */
#define UART_FCR_ENABLE_FIFO    0x01 /* Enable the FIFO */
#define UART_FCR_CLEAR_RCVR     0x02 /* Clear the RCVR FIFO */
#define UART_FCR_CLEAR_XMIT     0x04 /* Clear the XMIT FIFO */
#define UART_FCR_DMA_SELECT     0x08 /* For DMA applications */
#define UART_FCR_R_TRIGGER_MASK 0xC0 /* Mask for receive trigger range */
#define UART_FCR_R_TRIGGER_8    0x00 /* Mask for receive trigger set at 8 */
#define UART_FCR_R_TRIGGER_16   0x40 /* Mask for receive trigger set at 16 */
#define UART_FCR_R_TRIGGER_56   0x80 /* Mask for receive trigger set at 56 */
#define UART_FCR_R_TRIGGER_60   0xC0 /* Mask for receive trigger set at 60 */
#define UART_FCR_T_TRIGGER_MASK 0x30 /* Mask for transmit trigger range */
#define UART_FCR_T_TRIGGER_8    0x00 /* Mask for transmit trigger set at 8 */
#define UART_FCR_T_TRIGGER_16   0x10 /* Mask for transmit trigger set at 16 */
#define UART_FCR_T_TRIGGER_32   0x20 /* Mask for transmit trigger set at 32 */
#define UART_FCR_T_TRIGGER_56   0x30 /* Mask for transmit trigger set at 56 */

/*
 * These are the definitions for the Line Control Register
 *
 * Note: if the word length is 5 bits (UART_LCR_WLEN5), then setting
 * UART_LCR_STOP will select 1.5 stop bits, not 2 stop bits.
 */
#define UART_LCR_DLAB   0x80    /* Divisor latch access bit */
#define UART_LCR_SBC    0x40    /* Set break control */
#define UART_LCR_SPAR   0x20    /* Stick parity (?) */
#define UART_LCR_EPAR   0x10    /* Even parity select */
#define UART_LCR_PARITY 0x08    /* Parity Enable */
#define UART_LCR_STOP   0x04    /* Stop bits: 0=1 stop bit, 1= 2 stop bits */
#define UART_LCR_WLEN5  0x00    /* Wordlength: 5 bits */
#define UART_LCR_WLEN6  0x01    /* Wordlength: 6 bits */
#define UART_LCR_WLEN7  0x02    /* Wordlength: 7 bits */
#define UART_LCR_WLEN8  0x03    /* Wordlength: 8 bits */

/*
 * These are the definitions for the Line Status Register
 */
#define UART_LSR_TEMT   0x40    /* Transmitter empty */
#define UART_LSR_THRE   0x20    /* Transmit-hold-register empty */
#define UART_LSR_BI     0x10    /* Break interrupt indicator */
#define UART_LSR_FE     0x08    /* Frame error indicator */
#define UART_LSR_PE     0x04    /* Parity error indicator */
#define UART_LSR_OE     0x02    /* Overrun error indicator */
#define UART_LSR_DR     0x01    /* Receiver data ready */

/*
 * These are the definitions for the Interrupt Identification Register
 */
#define UART_ISR_NO_INT 0x01    /* No interrupts pending */
#define UART_ISR_ID     0x06    /* Mask for the interrupt ID */

#define UART_ISR_MSI    0x00    /* Modem status interrupt */
#define UART_ISR_THRI   0x02    /* Transmitter holding register empty */
#define UART_ISR_RDI    0x04    /* Receiver data interrupt */
#define UART_ISR_RLSI   0x06    /* Receiver line status interrupt */
#define UART_ISR_EVMASK	0x3E	/* Mask bits indicating interrupt event */

/*
 * These are the definitions for the Interrupt Enable Register
 */
#define UART_IER_SLEEP  0x10    /* Enable sleep mode */
#define UART_IER_MSI    0x08    /* Enable Modem status interrupt */
#define UART_IER_RLSI   0x04    /* Enable receiver line status interrupt */
#define UART_IER_THRI   0x02    /* Enable Transmitter holding register int. */
#define UART_IER_RDI    0x01    /* Enable receiver data interrupt */

/*
 * These are the definitions for the Modem Control Register
 */
#define UART_MCR_XON_ANY 0x20   /* Enable any character to restart output */
#define UART_MCR_LOOP   0x10    /* Enable loopback test mode */
#define UART_MCR_OUT2   0x08    /* Out2 complement */
#define UART_MCR_OUT1   0x04    /* Out1 complement */
#define UART_MCR_RTS    0x02    /* RTS complement */
#define UART_MCR_DTR    0x01    /* DTR complement */

/*
 * These are the definitions for the Modem Status Register
 */
#define UART_MSR_DCD    0x80    /* Data Carrier Detect */
#define UART_MSR_RI     0x40    /* Ring Indicator */
#define UART_MSR_DSR    0x20    /* Data Set Ready */
#define UART_MSR_CTS    0x10    /* Clear to Send */
#define UART_MSR_DDCD   0x08    /* Delta DCD */
#define UART_MSR_TERI   0x04    /* Trailing edge ring indicator */
#define UART_MSR_DDSR   0x02    /* Delta DSR */
#define UART_MSR_DCTS   0x01    /* Delta CTS */
#define UART_MSR_ANY_DELTA 0x0F /* Any of the delta bits! */

/*
 * These are the definitions for the Extended Features Register
 * (StarTech 16C654 and XR16C864, when DLAB=1)
 */
#define UART_EFR_CTS    0x80    /* CTS flow control */
#define UART_EFR_RTS    0x40    /* RTS flow control */
#define UART_EFR_SCD    0x20    /* Special character detect */
#define UART_EFR_ECB    0x10    /* Enhanced control bit */
#define UART_EFR_CONT3  0x08    /* Tx,Rx software flow control */
#define UART_EFR_CONT2  0x04    /* Tx,Rx software flow control */
#define UART_EFR_CONT1  0x02    /* Tx,Rx software flow control */
#define UART_EFR_CONT0  0x01    /* Tx,Rx software flow control */

#define UART_EFR_CMASK  (UART_EFR_CONT0 | UART_EFR_CONT1 | UART_EFR_CONT2 | UART_EFR_CONT3)

/*
 * These are the definitions for the Feature Control Register
 * (XR16C85x only, when LCR=bf; doubles with the Interrupt Enable
 * Register, UART register #1)
 */
#define UART_FCTR_RTS_NODELAY   0x00  /* RTS flow control delay */
#define UART_FCTR_RTS_4DELAY    0x01
#define UART_FCTR_RTS_6DELAY    0x02
#define UART_FCTR_RTS_8DELAY    0x03
#define UART_FCTR_IRDA          0x04  /* IrDa data encode select */
#define UART_FCTR_RS485         0x08  /* RS485 auto control */
#define UART_FCTR_TRGA          0x00  /* Tx/Rx 550 trigger table select */
#define UART_FCTR_TRGB          0x10  /* Tx/Rx 650 trigger table select */
#define UART_FCTR_TRGC          0x20  /* Tx/Rx 654 trigger table select */
#define UART_FCTR_TRGD          0x30  /* Tx/Rx 864 programmable trigger select */
#define UART_FCTR_SCR_SWAP      0x40  /* Scratch pad register swap */
#define UART_FCTR_RX            0x00  /* Programmable trigger mode select */
#define UART_FCTR_TX            0x80  /* Programmable trigger mode select */

/*
 * These are the definitions for the Enhanced Mode Select Register
 * (XR16C85x only, when LCR=bf and FCTR bit 6=1; doubles with the
 * Scratch register, UART register #7)
 */
#define UART_EMSR_FIFO_COUNT    0x01  /* Rx/Tx select */
#define UART_EMSR_ALT_COUNT     0x02  /* Alternating count select */

/*
 * These are the definitions for the Programmable Trigger
 * Register (XR16C85x only, when LCR=bf; doubles with the UART RX/TX
 * register, UART register #0)
 */
#define UART_TRG_1      0x01
#define UART_TRG_4      0x04
#define UART_TRG_8      0x08
#define UART_TRG_16     0x10
#define UART_TRG_32     0x20
#define UART_TRG_64     0x40
#define UART_TRG_96     0x60
#define UART_TRG_120    0x78
#define UART_TRG_128    0x80

#endif /* __TIP866DEF_H */

