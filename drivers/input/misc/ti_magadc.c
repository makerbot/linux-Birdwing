/*
 * TI Magnetic Stripe Reader driver
 *
 * This is a driver for TI's MAGADC IP.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/dmaengine.h>
#include <linux/edma.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/ti_magadc.h>

struct timag {
	struct  input_dev	*input;
	struct  ti_tscadc_dev	*mfd_magadc;
	struct platform_device	*pdev;
	struct  timag_track	mag_track;
	struct work_struct	decode_work;
	spinlock_t		lock;
	atomic_t                shutting_down;
	int			dma_rx_chnum;
	struct dma_chan		*dma_rx;
	long int		phy_base;
	dma_addr_t		dma_address;
	int			len;
	int			irq;
	unsigned int		tracks;
};
/*
 * Miscellaneous device structre
 */
static struct miscdevice magadc_cdev;

/*
 * variable for storing pid of Application
 */
int u_pid = 0;
/* Variable used for validating number of applications opening the node file */
static int node_open;
/*
 * Local variables used for interaction with application
 */
unsigned char *var;
struct timag_track *trackData = NULL;
struct timag_track *tData = NULL;
unsigned int roaddr = 0;
unsigned int rwaddr = 0;
unsigned int phys_addr;
unsigned int readCnt = 1;
unsigned int addrStored = 0;
unsigned char rwAddrStr[9] = { '\0'};
unsigned char roAddrStr[9] = { '\0'};
static int timag_dma_config(struct timag *mag_dev,
		struct platform_device *pdev);
/*
 * Read magadc register
 */
static inline unsigned int timag_readl(struct timag *mag_dev, unsigned int reg)
{
	return readl(mag_dev->mfd_magadc->tscadc_base + reg);
}

/*
 * Write the data to magadc register
 */
static inline void timag_writel(struct timag *mag_dev, unsigned int reg,
			unsigned int val)
{
	writel(val, mag_dev->mfd_magadc->tscadc_base + reg);
}

/*
 * Enable the step
 */
void timag_step_enable(struct timag *mag_dev, int stepid, bool enable)
{
	int val;

	val =  timag_readl(mag_dev, REG_SE);
	if (enable)
		val |= (0x1 << (stepid+1));
	else
		val &= ~(0x1 << (stepid+1));
	timag_writel(mag_dev, REG_SE, val);
}

/*
 * Configure the magadc steps
 */
static void timag_step_config(struct timag *mag_dev)
{
	unsigned int analog_input;
	unsigned int total_tracks;
	unsigned int config;
	unsigned int inm_val;
	unsigned int inp_val;
	int i;

	total_tracks = mag_dev->tracks;

	for (i = 0; i < total_tracks; i++) {
		analog_input = i;
		if (i > 0)
			analog_input = (analog_input * 2);

		inm_val = analog_input + 1;
		inp_val = analog_input;

		/* STEP delay config : sample and open delay */
		config = 0;
		config = MAGADC_SAMPLEDELAY | MAGADC_OPENDELAY;
		timag_writel(mag_dev, REG_STEPDELAY(i), config);

		/* Setting up other configs for each STEP */
		config = 0;
		config = (STEPCONFIG_DIFFCTRL_SELC |
			STEPCONFIG_AVG_4 |
			STEPCONFIG_RFM_ADCREFM |
			STEPCONFIG_INM(inm_val) |
			STEPCONFIG_INP(inp_val));

		/* GAIN is set to 12 , i.e 00 */
		config &= ~STEPCONFIG_GAIN_CTRL4_MASK;
		config &= ~STEPCONFIG_GAIN_CTRL3_MASK;
		config &= ~STEPCONFIG_GAIN_CTRL2_MASK;
		config &= ~STEPCONFIG_GAIN_CTRL1_MASK;

		/*
		 * The first step is SW sync continuous and all
		 * other steps are hardware sync
		 */
		config &= ~STEPCONFIG_MODE_MASK;

		/* only the track1 is software sync continuous
		 * all other tracks are hardware sync continuous.
		 */
		if (i == MAGADC_TRACK1) {
			config |= (STEPCONFIG_MODE_SWSYNCCONT |
				STEPCONFIG_THRES_CMP_EN);
		} else {
			config = config | STEPCONFIG_MODE_HWSYNCCONT;
			config &= ~STEPCONFIG_THRES_CMP_EN;
		}
		/* Select FIFO0 for each step */
		config &= ~STEPCONFIG_FIFO1;

		/* Trigger using swipe threshold reg1 for all steps */
		config &= ~STEPCONFIG_THRES_PTR_MASK;
		timag_writel(mag_dev, REG_STEPCONFIG(i), config);
	}
}

/*
 * Set the swipe threshold value
 */
static int timag_swipethreshold_set(struct timag *mag_dev,
				enum timag_thresdata type,
				unsigned int val)
{
	unsigned int thres;

	switch (type) {
	case MAGADC_SWIPE_THRESDATA1:
		thres = timag_readl(mag_dev, REG_SWIPECOMPARE12);
		thres &= ~SWIPETHRES_DATA1_VAL_MASK;
		thres |= SWIPETHRES_DATA1_VAL(val);
		timag_writel(mag_dev, REG_SWIPECOMPARE12, thres);
		break;
	case MAGADC_SWIPE_THRESDATA2:
		thres = timag_readl(mag_dev, REG_SWIPECOMPARE12);
		thres &= ~SWIPETHRES_DATA2_VAL_MASK;
		thres |= SWIPETHRES_DATA2_VAL(val);
		timag_writel(mag_dev, REG_SWIPECOMPARE12, thres);
		break;
	case MAGADC_SWIPE_THRESDATA3:
		thres = timag_readl(mag_dev, REG_SWIPECOMPARE34);
		thres &= ~SWIPETHRES_DATA3_VAL_MASK;
		thres |= SWIPETHRES_DATA3_VAL(val);
		timag_writel(mag_dev, REG_SWIPECOMPARE34, thres);
		break;
	case MAGADC_SWIPE_THRESDATA4:
		thres = timag_readl(mag_dev, REG_SWIPECOMPARE34);
		thres &= ~SWIPETHRES_DATA4_VAL_MASK;
		thres |= SWIPETHRES_DATA4_VAL(val);
		timag_writel(mag_dev, REG_SWIPECOMPARE34, thres);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/*
 * Track the input report
 */
static void timag_track_inputreport(struct timag *mag_dev,  u8 stepid)
{
	input_event(mag_dev->input, EV_MSC, MSC_RAW, stepid);
}
/*
 * power down the magadc
 */
static void timag_powerdown(struct timag *mag_dev, bool enable)
{
	unsigned int ctrl;

	ctrl = timag_readl(mag_dev, REG_CTRL);
	/*
	 * Write '0' to power on the AFE, '1' to power down
	 * The MAGADC should be disabled before this.
	 */
	if (enable)
		ctrl &= ~CNTRLREG_POWERDOWN;
	else
		ctrl |= CNTRLREG_POWERDOWN;

	timag_writel(mag_dev, REG_CTRL, ctrl);
}

/*
 * power down the pre - amplifier
 */
static void timag_preamp_powerdown(struct timag *mag_dev, bool enable)
{
	unsigned int ctrl;

	ctrl = timag_readl(mag_dev, REG_CTRL);
	/*
	 * Write '0' to power on the Preamp, '1' to power down
	 * The MAGADC should be disabled before this.
	 */
	if (enable)
		ctrl &= ~CNTRLREG_PREAMP_PWRDOWN;
	else
		ctrl |= CNTRLREG_PREAMP_PWRDOWN;

	timag_writel(mag_dev, REG_CTRL, ctrl);
}

/*
 * Set/reset the pre amplifier bypass
 */
static void timag_preamp_bypass(struct timag *mag_dev, bool enable)
{
	unsigned int ctrl;

	ctrl = timag_readl(mag_dev, REG_CTRL);
	/*
	 * 0 - Preamp is enabled by default
	 * 1 - Preamp is bypassed.
	 */
	if (enable)
		ctrl |= CNTRLREG_PREAMP_BYPASS;
	else
		ctrl &= ~CNTRLREG_PREAMP_BYPASS;

	timag_writel(mag_dev, REG_CTRL, ctrl);
}

/*
 * Enable the magadc
 */
static void timag_enable(struct timag *mag_dev, bool enable)
{
	unsigned int ctrl;

	ctrl = timag_readl(mag_dev, REG_CTRL);

	/* Write '1' to enable the MAGADC and '0' to disable */
	if (enable)
		ctrl |= CNTRLREG_MAGADCENB;
	else
		ctrl &= ~CNTRLREG_MAGADCENB;

	timag_writel(mag_dev, REG_CTRL, ctrl);
}

/*
 * Configure the step id
 */
static void timag_stepid_config(struct timag *mag_dev, bool enable)
{
	unsigned int ctrl;

	ctrl = timag_readl(mag_dev, REG_CTRL);

	/* Write '1' to enable the MAGADC and '0' to disable */
	if (enable)
		ctrl |= CNTRLREG_STEPID;
	else
		ctrl &= ~CNTRLREG_STEPID;

	timag_writel(mag_dev, REG_CTRL, ctrl);
}

/*
 * Enable DMA fifo
 */
void timag_dmafifo_enable(struct timag *mag_dev, bool fifosel, bool enable)
{
	unsigned int val;

	if (enable) {
		val = timag_readl(mag_dev, REG_DMAENABLESET);
		val |= (0x1 << fifosel);
		timag_writel(mag_dev, REG_DMAENABLESET, val);
	} else {
		val = timag_readl(mag_dev, REG_DMAENABLECLR);
		val |= (0x1 << fifosel);
		timag_writel(mag_dev, REG_DMAENABLECLR, val);
	}
}

/*
 * Release the dma function
 */
static void timag_release_dma(struct device *dev, struct timag *mag_dev)
{
	/* Release RX resources */
	dmaengine_terminate_all(mag_dev->dma_rx);

	dma_unmap_single(dev, mag_dev->dma_address,
			mag_dev->len, DMA_FROM_DEVICE);

	dma_release_channel(mag_dev->dma_rx);
	mag_dev->dma_rx = NULL;
}
/*
 * Api to request dma
 */
static int timag_request_dma(struct timag *mag_dev,
			struct platform_device *pdev)
{
	dma_cap_mask_t mask;
	struct device *sdev = &pdev->dev;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	mag_dev->dma_rx = dma_request_slave_channel_compat(mask,
				edma_filter_fn,
				&mag_dev->dma_rx_chnum,
				mag_dev->mfd_magadc->dev, "rx");
	if (!mag_dev->dma_rx) {
		dev_err(sdev, "request RX DMA channel failed\n");
		return -ENODEV;
	}

	mag_dev->dma_address = dma_map_single(sdev,
			&mag_dev->mag_track.adc_data[0],
			mag_dev->len, DMA_FROM_DEVICE);
	if (dma_mapping_error(sdev, mag_dev->dma_address)) {
		dev_err(sdev, "dma %u bytes error\n", mag_dev->len);
		return -EINVAL;
	}
	return 0;
}

/*
 * Callback function after completion of dma operation
 */
static void timag_dma_rx_callback(void *dev)
{
	struct timag *mag_dev;

	mag_dev = (struct timag *)dev;

	if (atomic_read(&mag_dev->shutting_down))
		return;

	/* Disable DMA mode of MAG ADC*/
	timag_dmafifo_enable(mag_dev, 0, false);
	timag_enable(mag_dev, false);

	spin_lock(&mag_dev->lock);
	mag_dev->mag_track.is_swiped = true;
	mag_dev->mag_track.total_samples = MAX_ADC_DATA_RAW;
	spin_unlock(&mag_dev->lock);

	schedule_work(&mag_dev->decode_work);
}

/*
 * Configure dma
 */
static int timag_dma_config(struct timag *mag_dev,
			struct platform_device *pdev)
{
	struct dma_slave_config dma_rx_conf = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = (unsigned long)(mag_dev->phy_base + REG_FIFO0),
		.src_addr_width = (32 * sizeof(unsigned int)),
		.src_maxburst = 1,
	};

	int dmathreshold = MAGADC_DMA0_THRESHOLD;
	struct dma_async_tx_descriptor *rxdesc;

	mag_dev->len = (MAX_ADC_DATA_RAW * sizeof(unsigned int));

	dmaengine_slave_config(mag_dev->dma_rx, &dma_rx_conf);

	rxdesc = dmaengine_prep_slave_single(mag_dev->dma_rx,
					mag_dev->dma_address,
						mag_dev->len,
					      DMA_DEV_TO_MEM,
			   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		return -EINVAL;

	rxdesc->callback = timag_dma_rx_callback;
	rxdesc->callback_param = (void *)mag_dev;

	dmaengine_submit(rxdesc);
	dma_async_issue_pending(mag_dev->dma_rx);

	timag_writel(mag_dev, REG_DMA0REQ, dmathreshold);
	/* Enable DMA for MAGADC */
	timag_dmafifo_enable(mag_dev, 0, true);

	return 0;
}

/*
 * initialize the magadc configuration
 */
static void timag_init_config(struct timag *mag_dev)
{
	timag_powerdown(mag_dev, true);

	timag_preamp_powerdown(mag_dev, true);

	timag_preamp_bypass(mag_dev, false);

	timag_swipethreshold_set(mag_dev,
				MAGADC_SWIPE_THRESDATA1,
				MAGADC_SWIPETHRES_VAL);

	/* STEPID is enabled in control register */
	timag_stepid_config(mag_dev, true);
	timag_step_config(mag_dev);
}

/*
 * magadc interrupt handler
 */
static irqreturn_t timag_irq(int irq, void *dev)
{
	struct timag *mag_dev = dev;
	bool	over_or_under_flow = false;
	unsigned int status, irqclr = 0;

	status = timag_readl(mag_dev, REG_IRQSTATUS);

	if ((status & IRQENB_FIFO0_OVERRUN) ||
	(status & IRQENB_FIFO0_UNDERFLOW) ||
	(status & IRQENB_FIFO1_OVERRUN) ||
	(status & IRQENB_FIFO1_UNDERFLOW)) {
		timag_enable(mag_dev, false);
		over_or_under_flow = true;
	}

	if (status & IRQENB_END_OF_SEQ)
		irqclr |= IRQENB_END_OF_SEQ;

	if (status & IRQENB_FIFO0_OVERRUN)
		irqclr |= IRQENB_FIFO0_OVERRUN;

	if (status & IRQENB_FIFO0_UNDERFLOW)
		irqclr |= IRQENB_FIFO0_UNDERFLOW;

	if (status & IRQENB_FIFO0THRES)
		irqclr |= IRQENB_FIFO0THRES;

	if (status & IRQENB_FIFO1THRES)
		irqclr |= IRQENB_FIFO1THRES;

	if (status & IRQENB_FIFO1_OVERRUN)
		irqclr |= IRQENB_FIFO1_OVERRUN;

	if (status & IRQENB_FIFO1_OVERRUN)
		irqclr |= IRQENB_FIFO1_OVERRUN;

	if (status & IRQENB_FIFO1_UNDERFLOW)
		irqclr |= IRQENB_FIFO1_UNDERFLOW;

	if (status & IRQENB_OUT_OF_RANGE)
		irqclr |= IRQENB_OUT_OF_RANGE;

	if (status & IRQENB_START_OF_SWIPE)
		irqclr |= IRQENB_START_OF_SWIPE;

	if (irqclr) {
		timag_writel(mag_dev, REG_IRQSTATUS, irqclr);
		return IRQ_HANDLED;
	}

	/* Re-enable the magadc module , in case of overrun/underflow */
	if (over_or_under_flow)
		timag_enable(mag_dev, true);

	return IRQ_NONE;
}

/*
 * Parse dt file
 */
static int timag_parse_dt(struct timag *mag_dev)
{
	struct device_node *node = mag_dev->pdev->dev.of_node;
	int err;

	if (!node)
		return -EINVAL;

	err = of_property_read_u32(node, "ti,tracks", &mag_dev->tracks);
	if (err < 0) {
		dev_err(&mag_dev->pdev->dev, "failed to find tracks in DT.\n");
		return err;
	}
	return 0;
}

/*
 * Bottom half of work queue
 */
static void timag_decode_work(struct work_struct *work)
{
	int i, ret = 0;
	struct siginfo info;
	struct task_struct *tid;


	struct timag *mag_dev = container_of(work, struct timag,
						 decode_work);
	struct platform_device  *pdev = mag_dev->pdev;
	const struct device *dev = &pdev->dev;

	/* If any public application is running then send the signal to app */
	if (u_pid != 0) {
		/* find task structure associated with this pid */
		tid = pid_task(find_vpid(u_pid), PIDTYPE_PID);
		info.si_signo = MAGADC_SIGID;
		info.si_code = SI_QUEUE;
		info.si_int = MAGADC_SIGINT;
		mag_dev->mag_track.num_tracks =  mag_dev->tracks;
		ret = send_sig_info(MAGADC_SIGID, &info, tid);
		if (ret < 0) {
			dev_err(dev, "error sending signal to public app:%d\n",
					ret);
			return;
		}
	}
	for (i = 0; i < mag_dev->tracks; i++)
		timag_track_inputreport(mag_dev, i);

	/* The end of swipe report is informed to the input subsystem  */
	input_sync(mag_dev->input);
	/*
	 * After sending signal to user app, Re-configure the DMA to
	 * process another card swipe. Hence this is a
	 * support for multiple swipes.
	 */
	timag_dma_config(mag_dev, mag_dev->pdev);
	for (i = 0; i < mag_dev->tracks; i++)
		timag_step_enable(mag_dev, i, true);

	/* Enable the Magadc Module */
	timag_enable(mag_dev, true);
	return;
}
static int magadc_open(struct inode *inode, struct file *file)
{
	if (node_open != 0)
		return -EBUSY;
	node_open++;
	try_module_get(THIS_MODULE);
	return 0;
}
static int magadc_release(struct inode *inode, struct file *file)
{
	if (node_open <= 0){
		return -1;
	}
	node_open--;
	module_put(THIS_MODULE);
	return 0;
}

static long magadc_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret;
	void *ptr = NULL;
	unsigned int phyAddr = 0;

	ret = copy_from_user(&phyAddr, (unsigned int *)arg, sizeof(int));
	if (ret != 0)
		return -EFAULT;

	if (cmd == MAGADC_STORE_RW_ADDR) {
		rwaddr	= phyAddr;
		sprintf(rwAddrStr, "%x", rwaddr);
		ptr = ioremap_cache(rwaddr, sizeof(struct timag_track));
		tData = (struct timag_track *)ptr;

	} else if (cmd == MAGADC_STORE_RO_ADDR) {
		roaddr	= phyAddr;
		sprintf(roAddrStr, "%x", roaddr);

	} else if (cmd == MAGADC_READ_RW_ADDR) {
		if (rwaddr == 0)
			return -1;
		if (copy_to_user((unsigned int *)arg, &rwaddr,
				sizeof(unsigned int)))
			return -EFAULT;
	} else if (cmd == MAGADC_READ_RO_ADDR) {
		if (roaddr == 0)
			return -1;
		if (copy_to_user((unsigned int *)arg,
				&roaddr, sizeof(unsigned int)))
			return -EFAULT;
	} else if (cmd == MAGADC_COPY_RAW_DATA) {
		memcpy((unsigned char *)tData,
			(unsigned char *)trackData,
			sizeof(struct timag_track));
	}
	return 0;
}
static ssize_t magadc_write(struct file *file,
	     const char __user *buffer, size_t length, loff_t *offset)
{
	int ret;

	ret = copy_from_user(&u_pid, (int *)buffer, length);
	if (ret != 0)
		return -EFAULT;
	return 0;

}
static ssize_t magadc_read(struct file *file, char __user *buf,
			size_t count, loff_t * ppos)
{
	if (copy_to_user(buf, trackData, count)) {
		return -EFAULT;
	}
	return 0;
}

static const struct file_operations magadc_fops = {
	.owner		= THIS_MODULE,
	.open		= magadc_open,
	.write		= magadc_write,
	.read           = magadc_read,
	.unlocked_ioctl	= magadc_ioctl,
	.release	= magadc_release,
};


/*
 * The functions for inserting/removing driver as a module.
 */
static int timag_probe(struct platform_device *pdev)
{
	struct timag *mag_dev;
	struct input_dev *input_dev;
	struct ti_tscadc_dev *magadc_dev = ti_tscadc_dev_get(pdev);
	int err;
	int irq_mask;
	int i;
	int ret;

	/* Allocate memory for device */
	mag_dev = devm_kzalloc(&pdev->dev, sizeof(struct timag), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!mag_dev || !input_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	trackData = &(mag_dev->mag_track);
	magadc_dev->mag = mag_dev;
	mag_dev->mfd_magadc = magadc_dev;
	mag_dev->input = input_dev;
	mag_dev->irq = magadc_dev->magirq;
	mag_dev->dma_rx_chnum = magadc_dev->dma_rx_chnum;
	mag_dev->phy_base = magadc_dev->phy_base;
	mag_dev->len = MAX_ADC_DATA_RAW;
	mag_dev->pdev = pdev;
	var = (unsigned char *)mag_dev;
	input_set_capability(input_dev, EV_MSC, MSC_RAW);
	err = timag_parse_dt(mag_dev);
	if (err) {
		dev_err(&pdev->dev, "Could not find valid DT data.\n");
		goto err_free_mem;
	}

	if (mag_dev->tracks > MAX_NO_OF_TRACKS) {
		dev_err(&pdev->dev, "Invalid number of magnetic track .\n");
		goto err_free_mem;
	}

	err = devm_request_irq(&pdev->dev, mag_dev->irq, timag_irq,
				IRQF_TRIGGER_HIGH, pdev->dev.driver->name,
				mag_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_free_mem;
	}

	spin_lock_init(&mag_dev->lock);

	/* Enable clock */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);

	magadc_cdev.minor  = MISC_DYNAMIC_MINOR;
	magadc_cdev.name   = "magadc";
	magadc_cdev.fops   = &magadc_fops;
	magadc_cdev.parent = &(pdev->dev);

	ret = misc_register(&magadc_cdev);
	if (ret)
		goto err_free_mem;

	INIT_WORK(&mag_dev->decode_work, timag_decode_work);
	atomic_set(&mag_dev->shutting_down, false);

	/* Perform initial configuration and enable required interrupts */
	timag_init_config(mag_dev);

	/* Enable the interrupt */
	irq_mask = (IRQENB_FIFO0_OVERRUN | IRQENB_FIFO0THRES |
			IRQENB_FIFO1THRES |
			IRQENB_FIFO0_UNDERFLOW |
			IRQENB_FIFO1_OVERRUN |
			IRQENB_FIFO1_UNDERFLOW);

	timag_writel(mag_dev, REG_IRQENABLE, irq_mask);

	input_dev->name = "ti-mag";
	input_dev->dev.parent = &pdev->dev;

	/* register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	platform_set_drvdata(pdev, mag_dev);

	timag_request_dma(mag_dev, pdev);

	timag_dma_config(mag_dev, pdev);

	for (i = 0; i < mag_dev->tracks; i++)
		timag_step_enable(mag_dev, i, true);

	/* Enable the MAGModule */
	timag_enable(mag_dev, true);

	return 0;

err_free_irq:
	devm_free_irq(&pdev->dev, mag_dev->irq, mag_dev);
	/* Disable clock */
	pm_runtime_disable(&pdev->dev);
	misc_deregister(&magadc_cdev);
err_free_mem:
	if (input_dev)
			input_free_device(input_dev);
	if (mag_dev)
			devm_kfree(&pdev->dev, mag_dev);
	return err;
}

/*
 * Remove driver function
 */
static int timag_remove(struct platform_device *pdev)
{
	struct timag *mag_dev = platform_get_drvdata(pdev);

	atomic_set(&mag_dev->shutting_down, true);
	cancel_work_sync(&mag_dev->decode_work);

	/* Disable the MAGADC Module */
	timag_enable(mag_dev, false);

	timag_release_dma(&pdev->dev, mag_dev);

	devm_free_irq(&pdev->dev, mag_dev->irq, mag_dev);

	timag_preamp_powerdown(mag_dev, false);

	timag_powerdown(mag_dev, false);

	input_unregister_device(mag_dev->input);

	input_free_device(mag_dev->input);

	/* deallocate mag_dev */
	devm_kfree(&pdev->dev, mag_dev);

	/* Disable clock */
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	misc_deregister(&magadc_cdev);
	return 0;
}
static const struct of_device_id ti_mag_dt_ids[] = {
	{ .compatible = "ti,am4372-mag", },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_mag_dt_ids);

#ifdef CONFIG_PM
/*
 * Suspend function to suspend the magadc
 */
static int timag_suspend(struct device *dev)
{
	struct timag *mag_dev = dev_get_drvdata(dev);

	atomic_set(&mag_dev->shutting_down, true);

	/* Clear the user application pid */
	u_pid = 0;

	/* Disable the MAGADC Module */
	timag_enable(mag_dev, false);

	/* To decrement the device's usage count */
	pm_runtime_put_sync_autosuspend(dev);

	timag_release_dma(dev, mag_dev);

	timag_preamp_powerdown(mag_dev, false);

	timag_powerdown(mag_dev, false);

	/* Disable clock */
	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);
	return 0;

}

/*
 * Function to resume from sleep
 */
static int timag_resume(struct device *dev)
{
	struct timag *mag_dev = dev_get_drvdata(dev);
	struct platform_device *pdev;
	struct ti_tscadc_dev *magadc_dev;
	int i;

	pdev = to_platform_device(dev);
	magadc_dev = ti_tscadc_dev_get(pdev);

	/* Enable clock */
	/* To increment the device's usage count */
	pm_runtime_get_sync(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_active(dev);

	atomic_set(&mag_dev->shutting_down, false);

	/* Perform initial configuration and enable required interrupts */
	timag_init_config(mag_dev);

	timag_request_dma(mag_dev, pdev);

	timag_dma_config(mag_dev, pdev);

	for (i = 0; i < mag_dev->tracks; i++)
		timag_step_enable(mag_dev, i, true);

	/* Enable the MAGModule */
	timag_enable(mag_dev, true);

	return 0;
}

static const struct dev_pm_ops timag_pm_ops = {
		.suspend = timag_suspend,
		.resume = timag_resume,
};
#define TIMAG_PM_OPS (&timag_pm_ops)
#else
#define TIMAG_PM_OPS NULL
#endif

static struct platform_driver ti_mag_driver = {
	.probe  = timag_probe,
	.remove = timag_remove,
	.driver = {
		.name   = "TI-am43xx-mag",
		.owner  = THIS_MODULE,
                .pm	= TIMAG_PM_OPS,
		.of_match_table = of_match_ptr(ti_mag_dt_ids),
	},
};

module_platform_driver(ti_mag_driver);

MODULE_DESCRIPTION("TI Magnetic Card Reader driver");
MODULE_AUTHOR("Priyaranjan Das <priyaranjandas@ti.com>");
MODULE_LICENSE("GPL v2");
