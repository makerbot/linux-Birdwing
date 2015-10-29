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

#define WRITES_IN_FLIGHT	32	//Arbitrary num, limited to not exhaust the RAM

#define FOLEY_INTERFACE 0
#define FOLEY_ALTSETTING 1

//Max frames allowed per Isoc IN transaction, Isoc OUT max frames are determined at probe
#define FOLEY_MAX_ISOC_IN_FRAMES 10

/* Structure to hold all of our device specific stuff */
struct usbfoley {
	struct usb_device *udev;	// the usb device for this device
	struct usb_interface *intf;	// the interface for this device
	struct semaphore limit_sem;	// limiting the number of writes in progress

	/*** BULK ***/
	struct usb_anchor bulk_out_anchor;	// in case we need to retract our submissions
	struct urb *bulk_in_urb;		// the urb to read bulk data with

	__u8 bulk_out_epAddr;		// the address of the bulk out ep
	__u8 bulk_in_epAddr;		// the address of the bulk in ep

	unsigned char *bulk_in_buffer;	// the buffer to receive data
	size_t bulk_in_ep_size;		// max packet size of bulk IN ep(bulk_in_buffer size)
	size_t bulk_in_filled;		// number of bytes in the buffer
	size_t bulk_in_copied;		// already copied to user space
	bool ongoing_bulk_read;		// a bulk read is going on

	/*** ISOC ***/
	struct usb_anchor isoc_out_anchor;	// anchor for all isoc OUT urbs
	struct usb_anchor isoc_in_anchor;	// anchor for all isoc IN urbs

	__u8 isoc_out_epAddr;		// the address of the isoc out ep
	__u8 isoc_in_epAddr;		// the address of the isoc IN ep

	size_t isoc_in_ep_size;		// max packet size of isoc IN ep
	size_t isoc_in_buf_size;	// size of the read buffer (ep_size * ISOC_MAX_FRAMES)
	int isoc_in_interval;		// interval read from the isoc IN ep desc
	int isoc_in_max_frames;		// max frames for isoc in packets (ISOC_MAX_FRAMES)

	size_t isoc_out_ep_size;	// max packet size for isoc OUT ep
	int isoc_out_interval;		// interval read from the isoc OUT ep desc
	int isoc_out_max_frames;	// max frames calc'd at probe (MAX_TRANSFER/isoc_out_ep_size)

	bool isoc_selected;		// should we use isoc or bulk?

	int current_interface;		//current set interface num
	int current_altsetting;		//current set interface altsetting

	int errors;			// the last request tanked
	spinlock_t err_lock;		// lock for errors
	struct kref kref;
	wait_queue_head_t wait;		// to wait for an ongoing read
	struct mutex io_mutex;		// synchronize I/O with disconnect
};
#define usbfoley_from_kref(d) container_of(d, struct usbfoley, kref)

#endif // End USB_MOREPORK_FOLEY_H
