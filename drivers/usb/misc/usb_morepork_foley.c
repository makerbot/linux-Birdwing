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

static ssize_t foley_isoc_selected_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct usbfoley *fdev;

	fdev = (struct usbfoley *)dev_get_drvdata(dev);	//Grab pointer from devices private data
	return scnprintf(buf, PAGE_SIZE, "%d\n", fdev->isoc_selected);
}

static ssize_t foley_isoc_selected_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct usbfoley *fdev;
	int selected;
	if (count) {
		kstrtoint(&buf[0], 0, &selected);
		if (selected > 1 || selected < 0) {
			dev_err(dev,
			       "Invalid isoc_selected value, must be 1 or 0\n");
			return -EINVAL;
		}

		fdev = (struct usbfoley *)dev_get_drvdata(dev);

		dev_dbg(dev, "isoc_selected_store: %d\n", selected);
		fdev->isoc_selected = selected;
	}
	return count;
}

//Create DEVICE_ATTR isoc_selected which is 1 when isoc is selected and 0 otherwise
static DEVICE_ATTR(isoc_selected, S_IRUGO | S_IWUSR, foley_isoc_selected_show,
		   foley_isoc_selected_store);

static struct usb_driver foley_driver;
static void foley_draw_down(struct usbfoley *dev);

static void foley_delete(struct kref *kref)
{
	struct usbfoley *dev = usbfoley_from_kref(kref);

	device_remove_file(&dev->udev->dev, &dev_attr_isoc_selected);

	usb_kill_anchored_urbs(&dev->isoc_in_anchor);
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
	if (dev->interface) {
		usb_autopm_put_interface(dev->interface);
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
			dev_err(&dev->interface->dev,
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

	spin_lock(&dev->err_lock);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
		}

		dev->errors = urb->status;
	} else {
		dev->bulk_in_filled = urb->actual_length;
	}
	dev->ongoing_read = 0;
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
					  dev->bulk_in_endpointAddr),
			  dev->bulk_in_buffer,
			  min(dev->bulk_in_size, count),
			  foley_read_bulk_callback, dev);
	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->err_lock);
	dev->ongoing_read = 1;
	spin_unlock_irq(&dev->err_lock);

	/* submit bulk in urb, which means no data to deliver */
	dev->bulk_in_filled = 0;
	dev->bulk_in_copied = 0;

	/* do it */
	rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
	if (rv < 0) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting read urb, error %d\n",
			__func__, rv);
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->err_lock);
		dev->ongoing_read = 0;
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

	if (!dev->interface) {	/* disconnect() was called */
		rv = -ENODEV;
		goto exit;
	}

	/* if IO is under way, we must not touch things */
 retry:
	spin_lock_irq(&dev->err_lock);
	ongoing_io = dev->ongoing_read;
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
		rv = wait_event_interruptible(dev->wait, (!dev->ongoing_read));
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

static void foley_isoc_in_callback(struct urb *urb)
{
	struct usbfoley *dev = urb->context;
	int i;
	int len;
	int dst = 0;
	void *buf = urb->transfer_buffer;

	dev_dbg(&dev->interface->dev, "READ_ISOC_CALLBACK\n");

	// process if URB was not killed
	if (urb->status != -ENOENT) {
		unsigned int pipe =
		    usb_rcvisocpipe(urb->dev, dev->isoc_in_endpointAddr);
		int pipesize = usb_maxpacket(urb->dev, pipe, usb_pipeout(pipe));

		dev_dbg(&dev->interface->dev, "PIPESIZE: %d\n", pipesize);
		dev_dbg(&dev->interface->dev, "URB_ACT_LEN: %d\n",
			urb->actual_length);

		for (i = 0; i < urb->number_of_packets; i++) {
			if (!urb->iso_frame_desc[i].status) {
				len = urb->iso_frame_desc[i].actual_length;
				dev_dbg(&dev->interface->dev,
					"ISOC_FRAME_LEN: %d\n", len);
				if (len <= pipesize) {
					dev_dbg(&dev->interface->dev,
						"LEN <= PIPESIZE");
					memcpy(buf + dst,
					       buf + urb->iso_frame_desc[i].offset,
					       len);
					dst += len;
				} else {
					dev_err(&dev->interface->dev,
						"READ_ISOC_CALLBACK: invalid len %d\n",
						len);
				}
			} else {
				dev_err(&dev->interface->dev,
					"READ_ISOC_CALLBACK: corrupted packet status: %d\n",
					urb->iso_frame_desc[i].status);
			}
		}
		if (dst != urb->actual_length) {
			dev_err(&dev->interface->dev,
				"READ_ISOC_CALLBACK: dst!=urb->actual_length:%d!=%d\n",
				dst, urb->actual_length);
		}
	}
	//Usually one would resubmit a urb here, though since Isoc urbs occur on
	//an interval we don't really have to.

	wake_up_interruptible(&dev->wait);
}

//Debug call to dump out useful urp info
static void foley_dump_urb(struct urb *urb)
{
	struct usbfoley *dev;
	dev = urb->context;

	dev_info(&dev->interface->dev, "urb                   :%p", urb);
	//dev_info(&dev->interface->dev, "next                  :%p", urb->next);
	dev_info(&dev->interface->dev, "dev                   :%p", urb->dev);
	dev_info(&dev->interface->dev, "pipe                  :%08X",
		 urb->pipe);
	dev_info(&dev->interface->dev, "status                :%d",
		 urb->status);
	dev_info(&dev->interface->dev, "transfer_flags        :%08X",
		 urb->transfer_flags);
	dev_info(&dev->interface->dev, "transfer_buffer       :%p",
		 urb->transfer_buffer);
	dev_info(&dev->interface->dev, "transfer_buffer_length:%d",
		 urb->transfer_buffer_length);
	dev_info(&dev->interface->dev, "actual_length         :%d",
		 urb->actual_length);
	dev_info(&dev->interface->dev, "setup_packet          :%p",
		 urb->setup_packet);
	dev_info(&dev->interface->dev, "start_frame           :%d",
		 urb->start_frame);
	dev_info(&dev->interface->dev, "number_of_packets     :%d",
		 urb->number_of_packets);
	dev_info(&dev->interface->dev, "interval              :%d",
		 urb->interval);
	dev_info(&dev->interface->dev, "error_count           :%d",
		 urb->error_count);
	dev_info(&dev->interface->dev, "context               :%p",
		 urb->context);
	dev_info(&dev->interface->dev, "complete              :%p",
		 urb->complete);
}

static inline void foley_fill_isoc_urb(struct urb *urb, int len, int mtu, int num_frames)
{
	struct usbfoley *dev = urb->context;

	int i;
	int offset = 0;

	dev_dbg(&dev->interface->dev, "ISOC_WRITE LEN: %d MTU: %d\n", len, mtu);

	for (i = 0; i < num_frames && len >= mtu; i++) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;

		//Increment offset and decrement total len left
		offset += mtu;
		len -= mtu;
	}

	if (len && i < num_frames) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int foley_usb_submit_isoc_in_urb(struct usbfoley *dev, size_t count,
					gfp_t mem_flags)
{
	struct urb *urb;
	int rv;
	unsigned int pipe;
	unsigned char *buf;

	urb = usb_alloc_urb(dev->isoc_in_max_frames, mem_flags);
	if (!urb) {
		dev_err(&dev->interface->dev, "Isoc IN URB allocation failed\n");
		return -ENOMEM;
	}

	buf = kmalloc(dev->isoc_in_buf_size, GFP_ATOMIC);
	memset(buf, 0, dev->isoc_in_buf_size);
	if (!buf) {
		dev_err(&dev->interface->dev, "Isoc IN buffer allocation failed\n");
		return -ENOMEM;
	}

	mutex_lock(&dev->io_mutex);

	pipe = usb_rcvisocpipe(dev->udev, dev->isoc_in_endpointAddr);

	usb_fill_int_urb(urb, dev->udev, pipe, buf,
			 dev->isoc_in_buf_size, foley_isoc_in_callback, dev,
			 dev->isoc_in_interval);

	urb->transfer_flags = URB_ISO_ASAP | URB_FREE_BUFFER;

	foley_fill_isoc_urb(urb, dev->isoc_in_buf_size, dev->isoc_in_ep_size, dev->isoc_in_max_frames);

	usb_anchor_urb(urb, &dev->isoc_in_anchor);

	rv = usb_submit_urb(urb, mem_flags);

	if (rv < 0) {
		if (rv != -EPERM && rv != -ENODEV) {
			dev_err(&dev->interface->dev,
				"urb %p submission failed (%d)\n", urb, rv);
		}
		usb_unanchor_urb(urb);
	}

	mutex_unlock(&dev->io_mutex);

	usb_free_urb(urb);

	return rv;
}

static ssize_t foley_isoc_read(struct file *file, char *buf, size_t count)
{
	struct usbfoley *dev = file->private_data;
	unsigned ret = 0;
	int rem, rv;
	int cnt;
	struct urb *urb = NULL;

	int readptr = 0;

	dev_info(&dev->interface->dev, "foley_isoc_read\n");

	if (!dev->interface || !dev->udev) {
		return -EIO;
	}

	while (count > 0) {
		//Submitted URB here gets anchored to dev->isoc_in_anchor
		if (!usb_get_from_anchor(&dev->isoc_in_anchor)) {
			foley_usb_submit_isoc_in_urb(dev, count, GFP_ATOMIC);
		}

		urb = usb_get_from_anchor(&dev->isoc_in_anchor);

		if (!urb) {
			ret = -ENOMEM;
			goto exit;
		}

		//If the urb is still in progress return and try again later.
		if (urb->status == -EINPROGRESS) {
			//Return Non-blocking
			if (file->f_flags & O_NONBLOCK) {
				if (!ret) {
					ret = -EAGAIN;
				}
				goto exit;
			}

			rv = wait_event_interruptible(
				dev->wait,
				(urb->status != -EINPROGRESS));
			if (rv < 0) {
				ret = rv;
				goto exit;
			}
		}

		rem = urb->actual_length - readptr;	// set remaining bytes to copy

		//If the requested count is more that we have, return only what we have
		if (count >= rem) {
			cnt = rem;
		} else {
			cnt = count;
		}

		dev_dbg(&dev->interface->dev, "copy_to_user:%p %p %d\n",
			buf, urb->transfer_buffer + readptr, cnt);

		mutex_lock(&dev->io_mutex);

		if (copy_to_user(buf, urb->transfer_buffer + readptr, cnt)) {
			dev_err(&dev->interface->dev,
				"read: copy_to_user failed\n");
			if (!ret) {
				ret = -EFAULT;
			}
			mutex_unlock(&dev->io_mutex);
			goto exit;
		}
		mutex_unlock(&dev->io_mutex);


		readptr += cnt;
		count -= cnt;
		buf += cnt;
		ret += cnt;

		dev_dbg(&dev->interface->dev,
			"READPTR: %d, COUNT: %d, BUF: %p, RET: %d\n",
			(int)readptr, (int)count, buf, (int)ret);

		if (readptr == urb->actual_length) {
			readptr = 0;
		}
	}
 exit:
	return ret;
}

static ssize_t foley_read(struct file *file, char *buffer, size_t count,
			  loff_t * ppos)
{
	int rv;
	struct usbfoley *dev;

	if (*ppos) {
		return -ESPIPE;
	}

	dev = file->private_data;

	foley_set_interface(dev, FOLEY_INTERFACE, FOLEY_ALTSETTING);

	if (dev->isoc_selected) {
		dev_dbg(&dev->interface->dev, "ISOC_READ\n");
		rv = foley_isoc_read(file, buffer, count);
	} else {
		usb_poison_anchored_urbs(&dev->isoc_in_anchor); //Poison all possible anchored isoc urbs
		dev_dbg(&dev->interface->dev, "BULK_READ\n");
		rv = foley_bulk_read(file, buffer, count);
	}

	return rv;
}

static void foley_write_bulk_callback(struct urb *urb)
{
	struct usbfoley *dev;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&dev->interface->dev,
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

	dev_dbg(&dev->interface->dev, "BULK_WRITE_COUNT: %d\nBULK_WRITESIZE: %d\n",
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
	if (!dev->interface) {	/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		rv = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, foley_write_bulk_callback, dev);
	//See todo comment above urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->bulk_out_anchor);

	/* send the data out the bulk port */
	rv = usb_submit_urb(urb, GFP_KERNEL);
	dev_dbg(&dev->interface->dev, "SUBMIT_BULK_WRITE_URB_RV: %d\n", rv);
	mutex_unlock(&dev->io_mutex);
	if (rv) {
		dev_err(&dev->interface->dev,
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

static void foley_isoc_out_callback(struct urb *urb)
{
	struct usbfoley *dev = urb->context;

	dev_dbg(&dev->interface->dev, "%s urb %p status %d count %d", __func__,
		 urb, urb->status, urb->actual_length);

	//Ripped urb status processing from bulk complete
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
		}

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;
		spin_unlock(&dev->err_lock);
	}

	kfree(urb->transfer_buffer);

	up(&dev->limit_sem);
}

static int foley_submit_isoc_out_urb(struct usbfoley *dev, const char *buf,
				     size_t buflen)
{
	struct urb *urb;
	unsigned int pipe;
	int rv;

	urb = usb_alloc_urb(dev->isoc_out_max_frames, GFP_ATOMIC);
	if (!urb) {
		return -ENOMEM;
	}

	pipe = usb_sndisocpipe(dev->udev, dev->isoc_out_endpointAddr);

	mutex_lock(&dev->io_mutex);

	usb_fill_int_urb(urb, dev->udev, pipe,
			 (void *)buf, buflen, foley_isoc_out_callback,
			 dev, dev->isoc_out_interval);

	urb->transfer_flags = URB_ISO_ASAP;

	foley_fill_isoc_urb(urb, buflen, dev->isoc_out_ep_size, dev->isoc_out_max_frames);

	usb_anchor_urb(urb, &dev->isoc_out_anchor);
	rv = usb_submit_urb(urb, GFP_ATOMIC);
	if (rv < 0) {
		if (rv != -EPERM && rv != -ENODEV) {
			dev_err(&dev->interface->dev,
				"urb %p submission failed (%d)\n", urb, rv);
		}
		usb_unanchor_urb(urb);
	}

	mutex_unlock(&dev->io_mutex);

	usb_free_urb(urb);

	return rv;
}

static ssize_t foley_isoc_write(struct file *file, const char *user_buffer,
				size_t count)
{
	struct usbfoley *dev;
	char *buf = NULL;
	int rv = 0;
	size_t writesize = min(count, (size_t) MAX_TRANSFER);

	dev = file->private_data;
	dev_dbg(&dev->interface->dev, "FOLEY_ISOC_WRITE\n");

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

	buf = kmalloc(writesize, GFP_KERNEL);	//usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		rv = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		rv = -EFAULT;
		goto error;
	}

	foley_submit_isoc_out_urb(dev, buf, writesize);

	return writesize;

 error:
	if (buf) {
		kfree(buf);
	}
	up(&dev->limit_sem);

 exit:
	return rv;
}

static ssize_t foley_write(struct file *file, const char *buffer, size_t count,
			   loff_t * ppos)
{
	int rv;
	struct usbfoley *dev;

	if (*ppos) {
		return -ESPIPE;
	}

	dev = file->private_data;

	foley_set_interface(dev, FOLEY_INTERFACE, FOLEY_ALTSETTING);

	if (dev->isoc_selected) {
		dev_dbg(&dev->interface->dev, "ISOC_WRITE");
		rv = foley_isoc_write(file, buffer, count);
	} else {
		dev_dbg(&dev->interface->dev, "BULK_WRITE");
		rv = foley_bulk_write(file, buffer, count);
	}

	return rv;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int foley_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		       unsigned long arg)
#else
static long foley_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	struct usbfoley *dev;
	int rv = 0;

	dev = file->private_data;

	switch (cmd) {
	case IOCTL_FOLEY_BULK:
		dev->isoc_selected = false;
		break;
	case IOCTL_FOLEY_ISOC:
		dev->isoc_selected = true;
		break;
	default:
		rv = -ENOIOCTLCMD;
		break;
	}

	return rv;
}

static const struct file_operations foley_fops = {
	.owner = THIS_MODULE,
	.read = foley_read,
	.write = foley_write,
	.open = foley_open,
	.release = foley_release,
	.flush = foley_flush,
	.llseek = noop_llseek,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = foley_ioctl,
#else
	.unlocked_ioctl = foley_ioctl,
#endif
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

static int foley_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct usbfoley *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int rv = -ENOMEM;

	// Check that we have the altsetting and num-o-endpoints we expect
	if (interface->num_altsetting != 2) {
		dev_err(&interface->dev,
			"Device doesn't have 2 altsettings (has %d)",
			interface->num_altsetting);
		return -ENODEV;
	} else if (interface->altsetting[FOLEY_ALTSETTING].desc.bNumEndpoints != 4) {
		dev_err(&interface->dev,
			"Device's altsetting 1 doesn't have 4 endpoints (has %d)",
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
	init_usb_anchor(&dev->isoc_in_anchor);
	init_usb_anchor(&dev->isoc_out_anchor);
	init_waitqueue_head(&dev->wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	iface_desc = &interface->altsetting[FOLEY_ALTSETTING];

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint)) {
			// we found a bulk in endpoint
			dev->bulk_in_size = usb_endpoint_maxp(endpoint);
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer =
			    kmalloc(dev->bulk_in_size, GFP_KERNEL);
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
		} else if (!dev->bulk_out_endpointAddr &&
			   usb_endpoint_is_bulk_out(endpoint)) {
			// we found a bulk out endpoint
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}
	// Find Isoc endpoints in interfaces altsetting 1
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->isoc_in_endpointAddr &&
		    usb_endpoint_is_isoc_in(endpoint)) {
			dev->isoc_in_ep_size = usb_endpoint_maxp(endpoint);

			dev->isoc_in_buf_size =
			    dev->isoc_in_ep_size * FOLEY_MAX_ISOC_IN_FRAMES;
			dev->isoc_in_max_frames = FOLEY_MAX_ISOC_IN_FRAMES;
			dev->isoc_in_endpointAddr = endpoint->bEndpointAddress;
			dev->isoc_in_interval = endpoint->bInterval;
		} else if (!dev->isoc_out_endpointAddr &&
			   usb_endpoint_is_isoc_out(endpoint)) {
			dev->isoc_out_ep_size = usb_endpoint_maxp(endpoint);
			if (dev->isoc_out_ep_size == 0) {
				dev_err(&interface->dev,
					"Isoc OUT Ep Size is zero, can't determine max frames\n");
				goto error;
			}
			dev->isoc_out_max_frames = MAX_TRANSFER/dev->isoc_out_ep_size;
			dev->isoc_out_endpointAddr = endpoint->bEndpointAddress;
			dev->isoc_out_interval = endpoint->bInterval;
		}
	}

	if (!(dev->isoc_in_endpointAddr)) {	// && dev->isoc_out_endpointAddr)) {
		dev_err(&interface->dev,
			"Could not find both isoc-in and isoc-out endpoints\n");
		goto error;
	}

	if (device_create_file(&dev->udev->dev, &dev_attr_isoc_selected) != 0) {
		dev_err(&interface->dev,
		       "Sysfs Attribute Creation failed for isoc_selected\n");
	}
	dev_set_drvdata(&dev->udev->dev, dev);	//Set private data of device to usbfoley pointer

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
	dev->isoc_selected = false;
	dev->current_interface = 0;
	dev->current_altsetting = 0;

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Foley device now attached to USBFoley-%d",
		 interface->minor);
	dev_info(&interface->dev,
		 "BULK_IN: 0x%02X, BULK_OUT: 0x%02X, ISOC_IN: 0x%02X, ISOC_OUT: 0x%02X",
		 dev->bulk_in_endpointAddr, dev->bulk_out_endpointAddr,
		 dev->isoc_in_endpointAddr, dev->isoc_out_endpointAddr);

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
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->bulk_out_anchor);
	usb_kill_anchored_urbs(&dev->isoc_in_anchor);
	usb_kill_anchored_urbs(&dev->isoc_out_anchor);

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

	time = usb_wait_anchor_empty_timeout(&dev->isoc_in_anchor, 1000);
	if (!time) {
		usb_kill_anchored_urbs(&dev->isoc_in_anchor);
	}

	time = usb_wait_anchor_empty_timeout(&dev->isoc_in_anchor, 1000);
	if (!time) {
		usb_kill_anchored_urbs(&dev->isoc_out_anchor);
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
