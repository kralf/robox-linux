/* $Id: tpmodule.h 44 2007-09-11 09:13:07Z Hesse $ */
/******************************************************************************
*******************************************************************************
**                                                                           **
**                                                                           **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                          @                     @                          **
**                          @ t p m o d u l e     @                          **
**                          @                     @                          **
**                          @@@@@@@@@@@@@@@@@@@@@@@                          **
**                                                                           **
**                                                                           **
**    Project          Linux Device Driver                                   **
**                                                                           **
**    File             tpmodule.h                                            **
**                                                                           **
**    Function         Module independent include header                     **
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
**                     Copyright (c) 2007                                    **
**                     TEWS TECHNOLOGIES GmbH                                **
**                                                                           **
**                                                                           **
**    $Date: 2007-09-11 11:13:07 +0200 (Di, 11 Sep 2007) $   $Rev: 44 $      **
**                                                                           **
*******************************************************************************
*******************************************************************************/


#ifndef __TPMODULE_H__
#define __TPMODULE_H__

#include <linux/version.h>

#define TPMODULE_STORAGE_CLASS static
#define TPMODULE_FUNC_ATTR __attribute__((unused))

/*
** to do:
*/
/*
int tpmodule_init(void);
void tpmodule_cleanup(void);
*/

/******************************************************************************
** tpmodule interrupt handling and misc - ...
**
******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

#define TP_IRQ_RETURN_T				void
#undef  IRQ_NONE
#define IRQ_NONE (0)
#undef  IRQ_HANDLED
#define IRQ_HANDLED (1)
#define TP_IRQ_RETURN(x)			return

#define TP_MOD_INC_USE_COUNT		MOD_INC_USE_COUNT
#define TP_MOD_DEC_USE_COUNT		MOD_DEC_USE_COUNT

#else

#define TP_IRQ_RETURN_T				irqreturn_t
#define TP_IRQ_RETURN(x)			return x;

#define TP_MOD_INC_USE_COUNT
#define TP_MOD_DEC_USE_COUNT

#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define TPMODULE_DEFINE_ISR(_isr_name) \
	static TP_IRQ_RETURN_T _isr_name(int irq, void *dev_id, struct pt_regs * regs)
#else
#define TPMODULE_DEFINE_ISR(_isr_name) \
	static TP_IRQ_RETURN_T _isr_name(int irq, void *dev_id)
#endif

#define TPMODULE_DECLARE_ISR(_isr_name) \
	TPMODULE_DEFINE_ISR(_isr_name)

#define TPMODULE_ISR_PRIVATE_DATA dev_id

/*
** interrupt request abstraction prototype
*/
int tpmodule_request_irq(unsigned int irq, void* handler, const char *devname, void *dev_id);

/*
** PCI driver register abstraction prototype
*/
int tpmodule_pci_register_driver( struct pci_driver *driver );
void tpmodule_pci_unregister_driver( struct pci_driver *driver );

/*
** Bottom Half abstraction
*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    #define tpmodule_bhtask_t   struct tq_struct
#else
    #define tpmodule_bhtask_t   struct work_struct
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    #define tpmodule_bharg_t      void
    #define tpmodule_bh_getdcb( _arg, _dcb_type, _bh_task )      (_arg)
#else
    #define tpmodule_bharg_t      struct work_struct
    #define tpmodule_bh_getdcb( _arg, _dcb_type, _bh_task )      container_of( _arg, _dcb_type, _bh_task )
#endif
void tpmodule_init_bottomhalf( tpmodule_bhtask_t* bh_task, void* handler, void* arg );
void tpmodule_schedule_bottomhalf( tpmodule_bhtask_t* bh_task );

/******************************************************************************
** tpmodule Filesystem API - ...
**
******************************************************************************/
#define TP_MAX_DEV_NAME_LEN			32

#if defined CONFIG_DEVFS_FS
#include <linux/devfs_fs_kernel.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
typedef void* devfs_handle_t; /* avoid #if defined... inside structure declarations */
#endif
#else
typedef void* devfs_handle_t; /* avoid #if defined... inside structure declarations */
#endif /* CONFIG_DEVFS_FS */


typedef struct
{
	devfs_handle_t devfs_handle;	/* handle for DEV file system */
	dev_t device;					/* major and minor number */
} TPMODULE_DEVICE_T;


#define TP_DEV_HANDLE_T				TPMODULE_DEVICE_T*
#define TP_INVALID_HANDLE			NULL



/******************************************************************************
** tpmodule Wait API - ...
**
******************************************************************************/
#define TP_WAIT_QUEUE_HEAD_T		wait_queue_head_t
#define TP_TIMEOUT_T				long
#define TP_FLAG_T					unsigned long

#define TP_MAX_CONDS_PER_WAIT		10

/* wait_flags - values */
#define TP_LOCK_COND				(1<<0)		/* Enable SMP save condition testing */
#define TP_RET_LOCKED				(1<<1)		/* Permission to return from wait without an unlock if a condition becomes TRUE */

typedef int (*tp_wait_cond_func_t)(void *data);
typedef void (*tp_lock_func_t)(void *data);


/*
** This structure encapsulates a wait condition. The get_state() function
** is defined by the driver developer inside the driver. 
**
** This function should return the following:
** Return				Meaning
** 0					The condition is false
** != 0					The condition is true (breaks the current wait job)
*/
typedef struct
{
	tp_wait_cond_func_t get_state;
	int result;
	void *data;

} tp_wait_cond_t;



/*
** This structure describes a synchronous wait object.
** So the development of wait jobs inside the driver is as simple as possible.
*/
typedef struct
{
	/* Common wait elements */
	TP_WAIT_QUEUE_HEAD_T		*queue;
	tp_wait_cond_t				cond[TP_MAX_CONDS_PER_WAIT];
	int							nconds;
	TP_FLAG_T					wait_flags;		/* {TP_LOCK_COND, TP_RET_LOCKED} */

	/* Support for concurrency */
	tp_lock_func_t				tp_lock;		/* pointer of the drivers lock function */
	tp_lock_func_t				tp_unlock;		/* pointer of the drivers unlock function */
	void *data;									/* data which is passed to the lock functions */

} tp_wait_sync_t;

#define tpmodule_wait_event_timeout( wq, condition, timeout )   \
    do {                                                \
        wait_queue_t __wait;                            \
        int         __timeout=timeout;                  \
        init_waitqueue_entry(&__wait, current);         \
        add_wait_queue(&wq, &__wait);                   \
        while (1)   \
	    {           \
            set_current_state(TASK_INTERRUPTIBLE);      \
            if (condition) break;                       \
            if (timeout)                                \
		    {                                           \
                __timeout = schedule_timeout(__timeout);    \
                if (__timeout == 0) break;              \
            } else {                                    \
                schedule();                             \
            }                                           \
            /* check for received signals */            \
            if (signal_pending(current)) break;         \
        }                                               \
        set_current_state(TASK_RUNNING);                \
        remove_wait_queue(&wq, &__wait);                \
    } while (0)


#endif /* __TPMODULE_H__ */
