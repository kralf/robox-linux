/* $Id: tpmodule.c 44 2007-09-11 09:13:07Z Hesse $ */
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
**    File             tpmodule.c                                            **
**                                                                           **
**    Function         Module independent functions                          **
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

#include <linux/version.h>

#define KERNEL_2_6 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
#define KERNEL_2_4 (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))

#include <linux/interrupt.h>
#if KERNEL_2_6
#include <linux/device.h>
#endif
#include <linux/tty.h>
#include "tpmodule.h"

#undef DEBUG_TPMODULE
#define TPMODULE_DEBUG_NAME "tpmodule: " 


TPMODULE_STORAGE_CLASS TP_DEV_HANDLE_T tpmodule_dev_create(const char *name, struct file_operations *fops, void *dcb, void *info) TPMODULE_FUNC_ATTR;
TPMODULE_STORAGE_CLASS void tpmodule_dev_destroy(TP_DEV_HANDLE_T dev, const char *name, void *info) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS void tpmodule_tty_dev_create(struct tty_driver *driver, unsigned minor, void* info) TPMODULE_FUNC_ATTR;
TPMODULE_STORAGE_CLASS void tpmodule_tty_dev_destroy(struct tty_driver *driver, unsigned minor,  void* info) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS int tpmodule_register_chrdev(int major, const char *drv_name, struct file_operations *fops, void *info) TPMODULE_FUNC_ATTR;
TPMODULE_STORAGE_CLASS void tpmodule_unregister_chrdev(int major, const char *drv_name, void *info) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS void tpmodule_init_wait(tp_wait_sync_t *wait_sync,
						 TP_WAIT_QUEUE_HEAD_T *queue,
						 TP_FLAG_T wait_flags,

						 void (*tp_lock)(void *data),
						 void (*tp_unlock)(void *data),
						 void *data
						 ) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS int tpmodule_get_wait_capacity(tp_wait_sync_t *wait_sync) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS void tpmodule_add_break_cond(tp_wait_sync_t *wait_sync,
							 tp_wait_cond_func_t get_state,
							 void *data
							 ) TPMODULE_FUNC_ATTR;

TPMODULE_STORAGE_CLASS int tpmodule_wait_sync_timeout(tp_wait_sync_t *wait_sync,
								struct file *filp,
								TP_TIMEOUT_T *timeout) TPMODULE_FUNC_ATTR;


TPMODULE_STORAGE_CLASS void tpmodule_init_waitqueue_head(TP_WAIT_QUEUE_HEAD_T *queue) TPMODULE_FUNC_ATTR;
TPMODULE_STORAGE_CLASS void tpmodule_wake_up_interruptible(TP_WAIT_QUEUE_HEAD_T *queue) TPMODULE_FUNC_ATTR;
/******************************************************************************
** tpmodule Filesystem API - ...
**
******************************************************************************/

#if KERNEL_2_6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
static struct class_simple* device_class;
#else
static struct class* device_class;
#endif
#endif

/*
** tpmodule_dev_create() - ...
**
*/
TPMODULE_STORAGE_CLASS TP_DEV_HANDLE_T tpmodule_dev_create(const char *name, struct file_operations *fops, void *dcb, void *info)
{
	TP_DEV_HANDLE_T	dev_handle;
	
	/* allocate kernel memory for TPMODULE_DEVICE_T */
	if (!(dev_handle = kmalloc(sizeof(TPMODULE_DEVICE_T), GFP_KERNEL)))
	{        
		printk(KERN_WARNING "\n%s unable to allocate memory for %s device structure\n", TPMODULE_DEBUG_NAME, name);
		return NULL;
	}
	memset(dev_handle, 0x00, sizeof(TPMODULE_DEVICE_T));

/* BEGIN Common Kernel 2.6.x SYSFS(UDEV), DEVFS support */
#if KERNEL_2_6
	if (info == NULL)
	{
		printk(KERN_WARNING "\n%s %s info NULL pointer!\n", TPMODULE_DEBUG_NAME, name);
		kfree(dev_handle);
		return NULL;
	}
	dev_handle->device = *((dev_t *)(info));
#endif
/* END Common Kernel 2.6.x SYSFS(UDEV), DEVFS support */


/* BEGIN SYSFS(UDEV) support */
#if KERNEL_2_6
	if (device_class != NULL)
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
		/* use class_simple API */
		class_simple_device_add(device_class, dev_handle->device, NULL, name);
#else
		/* use new class API */
        #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
        /* Kernel 2.6.13 + 2.6.14 */
        class_device_create(device_class, dev_handle->device, NULL, (char*)name);
        #else
            #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
            /* Kernel 2.6.15 - 2.6.18 */
		    class_device_create(device_class, NULL, dev_handle->device, NULL, (char*)name);
            #else
            /* Kernel >= 2.6.19 */
            class_device_create(device_class, NULL, dev_handle->device, NULL, name); 
            #endif
        #endif
#endif
	}
#endif
/* END SYSFS(UDEV) support */


/* BEGIN DEVFS support */
#if defined CONFIG_DEVFS_FS

#if KERNEL_2_6
	devfs_mk_cdev(dev_handle->device, S_IFCHR | S_IRUGO | S_IWUGO, name);
#else
	/*
	** Create /dev/.. minor node
	*/
	dev_handle->devfs_handle = devfs_register(NULL, name, DEVFS_FL_AUTO_DEVNUM, 0, 0,
		S_IFCHR | S_IRUGO | S_IWUGO, fops, (void *)dcb);
#endif /* KERNEL_2_6 */

#if defined DEBUG_TPMODULE
	printk("%sCreate minor node /dev/%s (devfs).\n", TPMODULE_DEBUG_NAME, name);
#endif

#endif /* CONFIG_DEVFS_FS */
/* END DEVFS support */

	return dev_handle;
}


/*
** tpmodule_dev_destroy() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_dev_destroy(TP_DEV_HANDLE_T dev, const char *name, void *info)
{
	if (dev == NULL) return;

/* BEGIN SYSFS(UDEV) support */
#if KERNEL_2_6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
	/* use class_simple API */
	class_simple_device_remove(dev->device);
#else
	if (device_class != NULL)
	{
		/* use new class API */
		class_device_destroy(device_class, dev->device); 
	}
#endif
#endif
/* END SYSFS(UDEV) support */


/* BEGIN DEVFS support */
#if defined CONFIG_DEVFS_FS
#if KERNEL_2_6
	devfs_remove(name);
#if defined DEBUG_TPMODULE
	printk("%sRemove /dev/%s node (devfs).\n", TPMODULE_DEBUG_NAME, name);
#endif
#else /* KERNEL_2_4 */
	if (dev->devfs_handle)
	{
		/*
		** Remove /dev/.. minor node
		*/
		devfs_unregister(dev->devfs_handle);
#if defined DEBUG_TPMODULE
		printk("%sRemove /dev/%s node (devfs).\n", TPMODULE_DEBUG_NAME, name);
#endif
	}
#endif /* KERNEL_2_6 */
#endif /* CONFIG_DEVFS_FS */
/* END DEVFS support */

	kfree(dev);	/* free kernel memory for device structure */
}


/*
** tpmodule_tty_dev_create() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_tty_dev_create(struct tty_driver *driver, unsigned minor, void* info)
{
#if defined CONFIG_DEVFS_FS
	/*
	** Create /dev/.. minor node
	*/
#if KERNEL_2_4
	tty_register_devfs(driver, 0, minor);
#endif
#endif /* CONFIG_DEVFS_FS */
}


/*
** tpmodule_tty_dev_destroy() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_tty_dev_destroy(struct tty_driver *driver, unsigned minor,  void* info)
{
#if defined CONFIG_DEVFS_FS
	/*
	** Remove /dev/.. minor node
	*/
#if KERNEL_2_4
    tty_unregister_devfs(driver, minor);        
#endif
#endif /* CONFIG_DEVFS_FS */
}




/*
** tpmodule_register_chrdev() - ...
**
*/
TPMODULE_STORAGE_CLASS int tpmodule_register_chrdev(int major, const char *drv_name, struct file_operations *fops, void *info)
{
/* BEGIN SYSFS(UDEV) support */
#if KERNEL_2_6
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
	/* use class_simple API */
	device_class = class_simple_create(THIS_MODULE, (char *)drv_name);
#else
	/* use new class API */
	device_class = class_create(THIS_MODULE, (char *)drv_name);
#endif
	if (IS_ERR(device_class)) device_class = NULL;
#endif
/* END SYSFS(UDEV) support */

	return register_chrdev(major, drv_name, fops);
}


/*
** tpmodule_unregister_chrdev() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_unregister_chrdev(int major, const char *drv_name, void *info)
{
/* BEGIN SYSFS(UDEV) support */
#if KERNEL_2_6
	if (device_class != NULL)
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
		/* use class_simple API */
		class_simple_destroy(device_class);
#else
		/* use new class API */
		class_destroy(device_class);
#endif
	}
#endif
/* END SYSFS(UDEV) support */
	unregister_chrdev(major, drv_name);
}


/******************************************************************************
** tpmodule Wait API -
**
******************************************************************************/

/*
** tpmodule_wait_init() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_init_wait(tp_wait_sync_t *wait_sync,
						 TP_WAIT_QUEUE_HEAD_T *queue,
						 TP_FLAG_T wait_flags,

						 tp_lock_func_t tp_lock,
						 tp_lock_func_t tp_unlock,
						 void *data
						 )
{
	wait_sync->queue = queue;
	wait_sync->nconds = 0;
	wait_sync->wait_flags = wait_flags;
	wait_sync->tp_lock = tp_lock;
	wait_sync->tp_unlock = tp_unlock;
	wait_sync->data = data;
}


/*
** tpmodule_get_wait_capacity() - Retrieve how much break conditions could be added to
**                                the certain wait object
**
*/
TPMODULE_STORAGE_CLASS int tpmodule_get_wait_capacity(tp_wait_sync_t *wait_sync)
{
	return TP_MAX_CONDS_PER_WAIT - wait_sync->nconds;
}


/*
** tpmodule_wait_add_cond() - Add a break condition to given synchronous wait object
** 
** ATTENTION:
** User has to prove "break condition capacity" by calling tpmodule_get_wait_capacity(...)
** before adding new break conditions. If the maximum count of break conditions per
** wait object is reached, no error is returned, but the concerning break condition
** is ignored.
*/
TPMODULE_STORAGE_CLASS void tpmodule_add_break_cond(tp_wait_sync_t *wait_sync,
							 tp_wait_cond_func_t get_state,
							 void *data
							 )
{
	tp_wait_cond_t *wait_cond;
	
	if (wait_sync->nconds < TP_MAX_CONDS_PER_WAIT)
	{
		wait_cond = &wait_sync->cond[wait_sync->nconds++];
		wait_cond->get_state = get_state;
		wait_cond->result = wait_sync->nconds;
		wait_cond->data = data;
	}
}

/*
** tp_wait_sync_timeout() - ...
**
*/
TPMODULE_STORAGE_CLASS int tpmodule_wait_sync_timeout(tp_wait_sync_t *wait_sync,
								struct file *filp,
								TP_TIMEOUT_T *timeout
								)
{
	wait_queue_t	wait;
	void			*dcb = wait_sync->data;
	TP_FLAG_T		wait_flags = wait_sync->wait_flags;
	tp_wait_cond_t	*wait_cond;
	int				i, result = 0;

    /* initialize and add a wait object to the drivers wait queue */
    init_waitqueue_entry(&wait, current);
    add_wait_queue(wait_sync->queue, &wait);
    
    while (1)
	{
        /*
        **  To avoid race conditions we first set the task state to TASK_INTERRUPTIBLE,
        **  which means if the scheduler will be called the task is going to sleep.
        **  Now we check if the requested transition(s) have already occurred. If so we
        **  leave the loop immediately, otherwise we call the scheduler to switch to
        **  the current process state. 
        **  If the transition has occurred between our check and the call of schedule()
        **  the task state was already set to TASK_RUNNING by the botton half of the ISR
        **  and schedule will return immediately without waiting.
        **
        */
        set_current_state(TASK_INTERRUPTIBLE);
        
		if (wait_flags & TP_LOCK_COND)
		{
			wait_sync->tp_lock(dcb);
		}

		/* we test all conditions the calling task is waiting for */
		for (i = 0; i < wait_sync->nconds; i++)
		{
			wait_cond = &wait_sync->cond[i];
			if (wait_cond->get_state(wait_cond->data))
			{
				result = wait_cond->result;
				break;
			}
		}

		/* we test whether one of the conditions is TRUE */
		if (result > 0)
		{
			/* return to calling task */
			if ((wait_flags & TP_LOCK_COND) && !(wait_flags & TP_RET_LOCKED))
			{
				wait_sync->tp_unlock(dcb);
			}
			break;
		}

		if (wait_flags & TP_LOCK_COND)
		{
			wait_sync->tp_unlock(dcb);
		}

        if (filp->f_flags & O_NONBLOCK)
		{
            /* this task should never be blocked */
            result = -EAGAIN;
			break;
        }        

        if (*timeout)
		{
            *timeout = schedule_timeout(*timeout);
            if (*timeout == 0)
			{
                result = -ETIME;
				break;
            }
        }
        else
		{
            schedule();
        }
        
        /* check for received signals */
        if (signal_pending(current))
		{
            result = -ERESTARTSYS;
			break;
        }
    }

    set_current_state(TASK_RUNNING);
    remove_wait_queue(wait_sync->queue, &wait);

	return result;
}


/*
** tpmodule_init_waitqueue_head() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_init_waitqueue_head(TP_WAIT_QUEUE_HEAD_T *queue)
{
	init_waitqueue_head(queue);
}


/*
** tpmodule_wake_up_interruptible() - ...
**
*/
TPMODULE_STORAGE_CLASS void tpmodule_wake_up_interruptible(TP_WAIT_QUEUE_HEAD_T *queue)
{
	wake_up_interruptible(queue);
}


/*
** tpmodule_request_irq()
*/
int tpmodule_request_irq(unsigned int irq, void* handler, const char *devname, void *dev_id)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    return request_irq(irq, handler, SA_SHIRQ, devname, dev_id);
#else
    return request_irq(irq, handler, IRQF_SHARED, devname, dev_id);
#endif
}

/*
** tpmodule_register_driver()
*/
int tpmodule_pci_register_driver( struct pci_driver *driver )
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    return pci_module_init(driver);
#else
    return pci_register_driver(driver);
#endif
}

/*
** tpmodule_unregister_driver()
*/
void tpmodule_pci_unregister_driver( struct pci_driver *driver )
{
    pci_unregister_driver(driver);
}

/*
** tpmodule_init_bottomhalf()
*/
void tpmodule_init_bottomhalf( tpmodule_bhtask_t* bh_task, void* handler, void* arg )
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        /* Kernel 2.4.x */
        INIT_TQUEUE( bh_task, handler, arg );
#else
        /* Kernel 2.6.x */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
        INIT_WORK( bh_task, handler, arg );
#else
        INIT_WORK( bh_task, handler );
#endif

#endif
}

/*
** tpmodule_schedule_bottomhalf()
*/
void tpmodule_schedule_bottomhalf( tpmodule_bhtask_t* bh_task )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
		schedule_work(bh_task);
#else
		queue_task(bh_task, &tq_immediate);
		mark_bh(IMMEDIATE_BH);
#endif
}
