#ifndef USB_OYSTER_H
#define USB_OYSTER_H

#define USB_OYSTER_VENDOR_ID	0x23C1
#define USB_OYSTER_PRODUCT_ID	0x0101

// Table of VID PIDs that this driver supports
static const struct usb_device_id oyster_table[] = {
	{USB_DEVICE(USB_OYSTER_VENDOR_ID, USB_OYSTER_PRODUCT_ID)},
	{}			/* Terminating entry */
};

/* Get a minor range for your devices from the usb maintainer */
#define USB_OYSTER_MINOR_BASE    244

#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */

#define WRITES_IN_FLIGHT	32	//Arbitrary num, limited to not exhaust the RAM

/* Structure to hold all of our device specific stuff */
struct usb_oyster {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	bool			ongoing_read;		/* a read is going on */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	unsigned long		disconnected:1;
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
};
#define to_oyster_dev(d) container_of(d, struct usb_oyster, kref)

#endif // End USB_OYSTER_H
