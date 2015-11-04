 /*
 * USB Morepork driver - 0.1 
 *
 * Copyright (C) 2015 MakerBot Industries
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 3.12 kernel version of drivers/usb/usb-skeleton.c
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include "usb_morepork_foley.h"

static struct usb_driver foley_driver;
static void foley_draw_down(struct usbfoley *dev);

static void foley_delete(struct kref *kref)
{
	struct usbfoley *dev = usbfoley_from_kref(kref);

	usb_kill_anchored_urbs(&dev->bulk_out_anchor);
	usb_free_urb(dev->bulk_in_urb);
	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int foley_open(struct inode *inode, struct file *file)
{
	struct usbfoley *dev;
	struct usb_interface *interface;
	int subminor;
	int rv = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&foley_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
		       __func__, subminor);
		rv = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		rv = -ENODEV;
		goto exit;
	}

	rv = usb_autopm_get_interface(interface);
	if (rv) {
		goto exit;
	}
	// increment our usage count for the device
	kref_get(&dev->kref);

	// save our object in the file's private structure
	file->private_data = dev;

	//This call marks the device file as nonseekable
	nonseekable_open(inode, file);
 exit:
	return rv;
}

static int foley_release(struct inode *inode, struct file *file)
{
	struct usbfoley *dev;

	dev = file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (dev->intf) {
		usb_autopm_put_interface(dev->intf);
	}
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, foley_delete);
	return 0;
}

static int foley_flush(struct file *file, fl_owner_t id)
{
	struct usbfoley *dev;
	int res;

	dev = file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	foley_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

static inline int foley_set_interface(struct usbfoley *dev, int interface, int altsetting)
{
	int rv = 0;
	if (interface != dev->current_interface ||
	altsetting != dev->current_altsetting) {
		rv = usb_set_interface(dev->udev, interface, altsetting);
		if (rv == 0) {
			dev->current_interface = interface;
			dev->current_altsetting = altsetting;
		} else {
			dev_err(&dev->intf->dev,
				"Failed to set interface intf: %d, alt: %d, err: %d\n",
				interface, altsetting, rv);
		}
	}
	return rv;
}

static void foley_read_bulk_callback(struct urb *urb)
{
	struct usbfoley *dev;

	dev = urb->context;

	dev_dbg(&dev->intf->dev, "FOLEY_READ_BULK_CALLBACK\n");

	spin_lock(&dev->err_lock);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&dev->intf->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
		}

		dev->errors = urb->status;
	} else {
		dev->bulk_in_filled = urb->actual_length;
	}
	dev->ongoing_bulk_read = 0;
	spin_unlock(&dev->err_lock);

	wake_up_interruptible(&dev->wait);
}

static int foley_do_read_bulk_io(struct usbfoley *dev, size_t count)
{
	int rv;

	/* prepare a read */
	usb_fill_bulk_urb(dev->bulk_in_urb,
			  dev->udev,
			  usb_rcvbulkpipe(dev->udev,
					  dev->bulk_in_epAddr),
			  dev->bulk_in_buffer,
			  min(dev->bulk_in_ep_size, count),
			  foley_read_bulk_callback, dev);
	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->err_lock);
	dev->ongoing_bulk_read = 1;
	spin_unlock_irq(&dev->err_lock);

	/* submit bulk in urb, which means no data to deliver */
	dev->bulk_in_filled = 0;
	dev->bulk_in_copied = 0;

	/* do it */
	rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
	if (rv < 0) {
		dev_err(&dev->intf->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->err_lock);
		dev->ongoing_bulk_read = 0;
		spin_unlock_irq(&dev->err_lock);
	}

	return rv;
}

static ssize_t foley_bulk_read(struct file *file, char *buffer, size_t count)
{
	struct usbfoley *dev;
	int rv;
	bool ongoing_io;

	dev = file->private_data;

	/* if we cannot read at all, return EOF */
	if (!dev->bulk_in_urb || !count) {
		return 0;
	}

	/* no concurrent readers */
	rv = mutex_lock_interruptible(&dev->io_mutex);
	if (rv < 0) {
		return rv;
	}

	if (!dev->intf) {	/* disconnect() was called */
		rv = -ENODEV;
		goto exit;
	}

	/* if IO is under way, we must not touch things */
 retry:
	spin_lock_irq(&dev->err_lock);
	ongoing_io = dev->ongoing_bulk_read;
	spin_unlock_irq(&dev->err_lock);

	if (ongoing_io) {
		/* nonblocking IO shall not wait */
		if (file->f_flags & O_NONBLOCK) {
			rv = -EAGAIN;
			goto exit;
		}
		/*
		 * IO may take forever
		 * hence wait in an interruptible state
		 */
		rv = wait_event_interruptible(dev->wait, (!dev->ongoing_bulk_read));
		if (rv < 0) {
			goto exit;
		}
	}

	/* errors must be reported */
	rv = dev->errors;
	if (rv < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		rv = (rv == -EPIPE) ? rv : -EIO;
		/* report it */
		goto exit;
	}

	/*
	 * if the buffer is filled we may satisfy the read
	 * else we need to start IO
	 */

	if (dev->bulk_in_filled) {
		/* we had read data */
		size_t available = dev->bulk_in_filled - dev->bulk_in_copied;
		size_t chunk = min(available, count);

		if (!available) {
			/*
			 * all data has been used
			 * actual IO needs to be done
			 */
			rv = foley_do_read_bulk_io(dev, count);
			if (rv < 0) {
				goto exit;
			} else {
				goto retry;
			}
		}
		/*
		 * data is available
		 * chunk tells us how much shall be copied
		 */

		if (copy_to_user(buffer,
				 dev->bulk_in_buffer + dev->bulk_in_copied,
				 chunk)) {
			rv = -EFAULT;
		} else {
			rv = chunk;
		}

		dev->bulk_in_copied += chunk;

		/*
		 * if we are asked for more than we have,
		 * we start IO but don't wait
		 */
		if (available < count) {
			foley_do_read_bulk_io(dev, count - chunk);
		}
	} else {
		/* no data in the buffer */
		rv = foley_do_read_bulk_io(dev, count);
		if (rv < 0) {
			goto exit;
		} else {
			goto retry;
		}
	}
 exit:
	mutex_unlock(&dev->io_mutex);
	return rv;
}

//Debug call to dump out useful urp info
static void foley_dump_urb(struct urb *urb)
{
	struct usbfoley *dev;
	dev = urb->context;

	dev_info(&dev->intf->dev, "urb                   :%p", urb);
	//dev_info(&dev->intf->dev, "next                  :%p", urb->next);
	dev_info(&dev->intf->dev, "dev                   :%p", urb->dev);
	dev_info(&dev->intf->dev, "pipe                  :%08X",
		 urb->pipe);
	dev_info(&dev->intf->dev, "status                :%d",
		 urb->status);
	dev_info(&dev->intf->dev, "transfer_flags        :%08X",
		 urb->transfer_flags);
	dev_info(&dev->intf->dev, "transfer_buffer       :%p",
		 urb->transfer_buffer);
	dev_info(&dev->intf->dev, "transfer_buffer_length:%d",
		 urb->transfer_buffer_length);
	dev_info(&dev->intf->dev, "actual_length         :%d",
		 urb->actual_length);
	dev_info(&dev->intf->dev, "setup_packet          :%p",
		 urb->setup_packet);
	dev_info(&dev->intf->dev, "start_frame           :%d",
		 urb->start_frame);
	dev_info(&dev->intf->dev, "number_of_packets     :%d",
		 urb->number_of_packets);
	dev_info(&dev->intf->dev, "interval              :%d",
		 urb->interval);
	dev_info(&dev->intf->dev, "error_count           :%d",
		 urb->error_count);
	dev_info(&dev->intf->dev, "context               :%p",
		 urb->context);
	dev_info(&dev->intf->dev, "complete              :%p",
		 urb->complete);
}

static ssize_t foley_read(struct file *file, char *buffer, size_t count,
			  loff_t * ppos)
{
	struct usbfoley *dev;

	if (*ppos) {
		return -ESPIPE;
	}

	dev = file->private_data;

	foley_set_interface(dev, FOLEY_INTERFACE, FOLEY_ALTSETTING);

	return foley_bulk_read(file, buffer, count);
}

static void foley_write_bulk_callback(struct urb *urb)
{
	struct usbfoley *dev;

	dev = urb->context;

	dev_dbg(&dev->intf->dev, "FOLEY_WRITE_BULK_CALLBACK\n");

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&dev->intf->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
		}

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;
		spin_unlock(&dev->err_lock);
	}

	/*
	usb_free_coherent doesn't like to run while atomic (how callbacks are run).
	So for now we are using a normal trans buf, instead of a coherent dma buf
	http://linux-usb.vger.kernel.narkive.com/i56rTPwA/usb-skeleton-c-generates-warning
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	*/

	// free up our allocated buffer
	kfree(urb->transfer_buffer);

	up(&dev->limit_sem);
}

static ssize_t foley_bulk_write(struct file *file, const char *user_buffer,
				size_t count)
{
	struct usbfoley *dev;
	int rv = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t) MAX_TRANSFER);

	dev = file->private_data;

	dev_dbg(&dev->intf->dev, "BULK_WRITE_COUNT: %d\nBULK_WRITESIZE: %d\n",
		(int)count, (int)writesize);

	/* verify that we actually have some data to write */
	if (count == 0) {
		goto exit;
	}

	/*
	 * limit the number of URBs in flight to stop a user from using up all
	 * RAM
	 */
	if (!(file->f_flags & O_NONBLOCK)) {
		if (down_interruptible(&dev->limit_sem)) {
			rv = -ERESTARTSYS;
			goto exit;
		}
	} else {
		if (down_trylock(&dev->limit_sem)) {
			rv = -EAGAIN;
			goto exit;
		}
	}

	spin_lock_irq(&dev->err_lock);
	rv = dev->errors;
	if (rv < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		rv = (rv == -EPIPE) ? rv : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (rv < 0) {
		goto error;
	}

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		rv = -ENOMEM;
		goto error;
	}

	/* TODO should revisit this if non-dma memory turns out to be an issue
	usb_free_coherent doesn't like to run while atomic (how callbacks are run).
	So for now we are using a normal trans buf, instead of a coherent dma buf
	http://linux-usb.vger.kernel.narkive.com/i56rTPwA/usb-skeleton-c-generates-warning
	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	*/
	buf = kmalloc(writesize, GFP_KERNEL);
	if (!buf) {
		rv = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		rv = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (!dev->intf) {	/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		rv = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_epAddr),
			  buf, writesize, foley_write_bulk_callback, dev);
	//See todo comment above urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->bulk_out_anchor);

	/* send the data out the bulk port */
	rv = usb_submit_urb(urb, GFP_KERNEL);
	dev_dbg(&dev->intf->dev, "SUBMIT_BULK_WRITE_URB_RV: %d\n", rv);
	mutex_unlock(&dev->io_mutex);
	if (rv) {
		dev_err(&dev->intf->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, rv);
		goto error_unanchor;
	}

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);

	return writesize;

 error_unanchor:
	usb_unanchor_urb(urb);
 error:
	if (urb) {
		if (buf) {
			kfree(buf);
		}
		//See todo comment above usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);

 exit:
	return rv;
}

static ssize_t foley_write(struct file *file, const char *buffer, size_t count,
			   loff_t * ppos)
{
	struct usbfoley *dev;

	if (*ppos) {
		return -ESPIPE;
	}

	dev = file->private_data;

	foley_set_interface(dev, FOLEY_INTERFACE, FOLEY_ALTSETTING);

	return foley_bulk_write(file, buffer, count);
}

static const struct file_operations foley_fops = {
	.owner = THIS_MODULE,
	.read = foley_read,
	.write = foley_write,
	.open = foley_open,
	.release = foley_release,
	.flush = foley_flush,
	.llseek = noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver foley_class = {
	.name = "foley%d",
	.fops = &foley_fops,
	.minor_base = USB_FOLEY_MINOR_BASE,
};

// TODO cleanup foley_probe once the rest of the driver is stable
static int foley_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct usbfoley *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *ep;
	int i;
	int rv = -ENOMEM;

	// Check that we have the altsetting and num-o-endpoints we expect
	if (interface->num_altsetting != 2) {
		dev_err(&interface->dev,
			"Device doesn't have 2 altsettings (has %d)",
			interface->num_altsetting);
		return -ENODEV;
	} else if (interface->altsetting[FOLEY_ALTSETTING].desc.bNumEndpoints != 2) {
		dev_err(&interface->dev,
			"Device's altsetting 1 doesn't have 2 endpoints (has %d)",
			interface->altsetting[FOLEY_ALTSETTING].desc.bNumEndpoints);
		return -ENODEV;
	}

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->bulk_out_anchor);
	init_waitqueue_head(&dev->wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->intf = interface;

	iface_desc = &interface->altsetting[FOLEY_ALTSETTING];

	/* set up the endpoints information */
	/* use only the first bulk-in and bulk-out endpoints */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		ep = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_epAddr &&
		    usb_endpoint_is_bulk_in(ep)) {
			// we found a bulk in endpoint
			dev->bulk_in_ep_size = usb_endpoint_maxp(ep);
			dev->bulk_in_epAddr = ep->bEndpointAddress;
			dev->bulk_in_buffer =
			    kmalloc(dev->bulk_in_ep_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				dev_err(&interface->dev,
					"Could not allocate bulk_in_buffer\n");
				goto error;
			}
			dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->bulk_in_urb) {
				dev_err(&interface->dev,
					"Could not allocate bulk_in_urb\n");
				goto error;
			}
		} else if (!dev->bulk_out_epAddr &&
			   usb_endpoint_is_bulk_out(ep)) {
			// we found a bulk out endpoint
			dev->bulk_out_epAddr = ep->bEndpointAddress;
		}
	}
	if (!(dev->bulk_in_epAddr && dev->bulk_out_epAddr)) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	rv = usb_register_dev(interface, &foley_class);
	if (rv) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	//Set interface state vars to defaults
	dev->current_interface = 0;
	dev->current_altsetting = 0;

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Foley device now attached to USBFoley-%d",
		 interface->minor);
	dev_info(&interface->dev,
		 "BULK_IN: 0x%02X, BULK_OUT: 0x%02X",
		 dev->bulk_in_epAddr, dev->bulk_out_epAddr);

	return 0;

 error:
	if (dev) {
		/* this frees allocated memory */
		kref_put(&dev->kref, foley_delete);
	}
	return rv;
}

static void foley_disconnect(struct usb_interface *interface)
{
	struct usbfoley *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &foley_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->intf = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->bulk_out_anchor);

	/* decrement our usage count */
	kref_put(&dev->kref, foley_delete);

	dev_info(&interface->dev, "USB Foley #%d now disconnected", minor);
}

static void foley_draw_down(struct usbfoley *dev)
{
	int time;

	//Not sure if having three 1000 milliseconds (possibly) pauses is an issue
	//Assuming you won't draw down during any high priority reads/writes
	time = usb_wait_anchor_empty_timeout(&dev->bulk_out_anchor, 1000);
	if (!time) {
		usb_kill_anchored_urbs(&dev->bulk_out_anchor);
	}

	usb_kill_urb(dev->bulk_in_urb);
}

static int foley_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usbfoley *dev = usb_get_intfdata(intf);

	if (!dev) {
		return 0;
	}
	foley_draw_down(dev);
	return 0;
}

static int foley_resume(struct usb_interface *intf)
{
	return 0;
}

static int foley_pre_reset(struct usb_interface *intf)
{
	struct usbfoley *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	foley_draw_down(dev);

	return 0;
}

static int foley_post_reset(struct usb_interface *intf)
{
	struct usbfoley *dev = usb_get_intfdata(intf);

	// we are sure no URBs are active - no locking needed
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver foley_driver = {
	.name = "foley",
	.probe = foley_probe,
	.disconnect = foley_disconnect,
	.suspend = foley_suspend,
	.resume = foley_resume,
	.pre_reset = foley_pre_reset,
	.post_reset = foley_post_reset,
	.id_table = foley_table,
	.supports_autosuspend = true,
};

	/* table of devices that work with this driver */
MODULE_DEVICE_TABLE(usb, foley_table);

module_usb_driver(foley_driver);

MODULE_AUTHOR("MakerBot Industries");
MODULE_DESCRIPTION("USB Driver for Morepork-Foley Toolhead");
MODULE_LICENSE("GPL");
