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
#define USB_FOLEY_STATUS_MINOR_BASE 250

#define IOCTL_FOLEY_BULK _IO('f', 1)
#define IOCTL_FOLEY_STATUS _IOR('f', 2, char*)

#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */

#define WRITES_IN_FLIGHT	32	//Arbitrary num, limited to not exhaust the RAM

#define FOLEY_NUM_ALTSETTINGS 1
#define FOLEY_NUM_ENDPOINTS 3

#define FOLEY_INTERFACE 0
#define FOLEY_ALTSETTING 0 

/* Structure to hold all of our device specific stuff */
struct usbfoley {
	struct usb_device *udev;	// the usb device for this device
	struct usb_interface *intf;	// the interface for this device
	struct semaphore limit_sem;	// limiting the number of writes in progress

	/*** BULK ***/
	struct usb_anchor bulk_out_anchor;	// in case we need to retract our submissions
	struct urb *bulk_in_urb;		// the urb to read bulk data with
	struct urb *status_in_urb;

	__u8 bulk_out_epAddr;		// the address of the bulk out ep
	__u8 bulk_in_epAddr;		// the address of the bulk in ep
	__u8 status_in_epAddr;

	unsigned char *bulk_in_buffer;	// the buffer to receive data
	unsigned char *status_in_buffer;
	size_t bulk_in_ep_size;		// max packet size of bulk IN ep(bulk_in_buffer size)
	size_t status_in_ep_size;
	size_t bulk_in_filled;		// number of bytes in the buffer
	size_t status_in_filled;
	size_t bulk_in_copied;		// already copied to user space
	size_t status_in_copied;
	bool ongoing_bulk_read;		// a bulk read is going on
	bool ongoing_status_read;

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
