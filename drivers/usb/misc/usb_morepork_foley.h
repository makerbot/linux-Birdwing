#ifndef USB_MOREPORK_FOLEY_H
#define USB_MOREPORK_FOLEY_H

#define USB_FOLEY_VENDOR_ID	0x23C1
#define USB_FOLEY_PRODUCT_ID	0x0100

// Table of VID PIDs that this driver supports
static const struct usb_device_id foley_table[] = {
	{USB_DEVICE(USB_FOLEY_VENDOR_ID, USB_FOLEY_PRODUCT_ID)},
	{}			/* Terminating entry */
};

/* Get a minor range for your devices from the usb maintainer */
#define USB_FOLEY_MINOR_BASE    244

#define IOCTL_FOLEY_BULK _IO('f', 1)
#define IOCTL_FOLEY_ISOC _IO('f', 2)

#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */

#define WRITES_IN_FLIGHT	8	//Arbitrary num

#define FOLEY_INTERFACE 0
#define FOLEY_ALTSETTING 1

//Max frames allowed per Isoc IN transaction, Isoc OUT max frames are determined at probe
#define FOLEY_MAX_ISOC_IN_FRAMES 10

/* Structure to hold all of our device specific stuff */
struct usbfoley {
	struct usb_device *udev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct semaphore limit_sem;	/* limiting the number of writes in progress */
	struct usb_anchor submitted;	/* in case we need to retract our submissions */

	//BULK
	struct urb *bulk_in_urb;	/* the urb to read bulk data with */
	unsigned char *bulk_in_buffer;	/* the buffer to receive data */
	size_t bulk_in_size;	/* the size of the receive buffer */
	size_t bulk_in_filled;	/* number of bytes in the buffer */
	size_t bulk_in_copied;	/* already copied to user space */
	__u8 bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8 bulk_out_endpointAddr;	/* the address of the bulk out endpoint */

	//ISOC
	size_t isoc_in_ep_size;	/* the size of the endpoint buffer */
	size_t isoc_in_buf_size;	//Size of the read buffer (ep_size * ISOC_MAX_FRAMES)
	size_t isoc_in_filled;	/* number of bytes in the buffer */
	size_t isoc_in_copied;	/* already copied to user space */
	__u8 isoc_in_endpointAddr;	/* the address of the isoc in endpoint */
	int isoc_in_interval;
	int isoc_in_max_frames;
	struct usb_anchor isoc_in_anchor;

	size_t isoc_out_ep_size;
	__u8 isoc_out_endpointAddr;	/* the address of the isoc out endpoint */
	int isoc_out_interval;
	int isoc_out_max_frames;
	struct usb_anchor isoc_out_anchor;
	bool isoc_selected;	/* should we use isoc or bulk? */

	int current_interface;
	int current_altsetting;

	int errors;		/* the last request tanked */
	bool ongoing_read;	/* a read is going on */
	spinlock_t err_lock;	/* lock for errors */
	struct kref kref;
	struct mutex io_mutex;	/* synchronize I/O with disconnect */
	wait_queue_head_t wait;	/* to wait for an ongoing read */
};
#define usbfoley_from_kref(d) container_of(d, struct usbfoley, kref)

struct foley_buff {
	struct list_head buff_list;
	struct urb *urb;
	struct usbfoley *dev;
};

#endif				// End USB_MOREPORK_FOLEY_H
