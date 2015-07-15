/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft6x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/ft6x06_ts.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>

#define FTS_CTL_IIC
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "ft6x06_ex_fun.h"
#endif
#warning Compiling FTS controller
enum ts_event_type {
    TS_EV_NONE =0,
    TS_EV_SFSC =1,
    TS_EV_SFDC =2,
    TS_EV_TFSC =3,
    TS_EV_TFDC =4,
    TS_EV_FE   =5,
    TS_EV_FL   =6,
    TS_EV_FM   =7,
    TS_EV_MAX  =8, //Logic only - this event isn't valid
    TS_EV_INVALID =15,
};

struct ft6x06_ts_event {
	u16 x;	/*x coordinate */
	u16 y;	/*y coordinate */
	enum ts_event_type ev_type;	/*touch event */
	u8 finger_id;	/*touch ID */
};

struct ft6x06_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ft6x06_platform_data *pdata;
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02


/*
*ft6x06_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = I2C_M_IGNORE_NAK,//0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/*release the point*/
static void ft6x06_ts_release(struct ft6x06_ts_data *data)
{
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}

/*Read touch point information when the interrupt  is asserted.*/
static int ft6x06_read_Touchdata(struct ft6x06_ts_data *data,
                                 struct ft6x06_ts_event *events)
{
    // 4 bytes in the event structure, POINT_READ_BUF points in the device FIFO
	u8 buf[POINT_READ_BUF*4] = { 0 };
	int ret=-1, i=0;
	u8 regaddr = FT6x06_REG_EVENTSTACK, status = FT6x06_REG_EVENTSTATUS;
    // Read the event stack until it's empty
    ft6x06_i2c_Read(data->client, &status, 1, &status, 1);
    dev_dbg(&data->client->dev, "%s read touchdata status: %x\n",
             __func__, status);
    do {
        ret = ft6x06_i2c_Read(data->client,
                              &regaddr, 1,
                              buf, 4);
        if (ret < 0) {
            dev_err(&data->client->dev, "%s read touchdata failed: %d.\n",
                    __func__, ret);
            return ret;
        }
        events[i].ev_type = buf[0]&0x0f;
        events[i].finger_id = (buf[0]&0xf0) >> 4;
        events[i].x = buf[1] + (((u16)(buf[3]&0x30))<<4);
        events[i].y = buf[2] + (((u16)(buf[3]&0x03))<<8);
        if(events[i].ev_type >= TS_EV_MAX
           && events[i].ev_type != TS_EV_INVALID) {
            dev_err(&data->client->dev, "%s read touchdata got bad event %d.\n",
                    __func__, events[i].ev_type);
        }
        i++;
    } while (events[i-1].ev_type != TS_EV_NONE
             && i<POINT_READ_BUF);
	return 0;
}

/*
*report the point information
*/
static void ft6x06_report_value(struct ft6x06_ts_data *data,
                                struct ft6x06_ts_event *events)
{
	int i = 0;
    if(events[0].ev_type != TS_EV_NONE) {
        // We have at least one touch, emit a BTN_TOUCH per
        // Documentation/input/event-codes.txt
        dev_dbg(&data->client->dev, "Reporting touch!");
        input_report_key(data->input_dev, BTN_TOUCH, 1);
    }
	for (i = 0; (i < POINT_READ_BUF)
             && (events[i].ev_type != TS_EV_NONE); i++) {
		/* LCD view area */
        struct ft6x06_ts_event event=events[i];
		if (event.x < data->x_max
		    && event.y < data->y_max
            && event.ev_type < TS_EV_MAX) {
            dev_dbg(&data->client->dev, "Reporting event!");
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
                             event.x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
                             event.y);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
                             event.finger_id);
            input_report_abs(data->input_dev,
                             ABS_MT_TOUCH_MAJOR, 0);
            input_mt_sync(data->input_dev);
        } else {
            dev_dbg(&data->client->dev, "Touch out of bounds (x: %d, y: %d: max %d, %d) Corrupt data?",
                     event.x, event.y, data->x_max, data->y_max);
        }
    }
    if (i > 0) {
        dev_dbg(&data->client->dev, "Sending input packet");
		input_sync(data->input_dev);
    } else {
        dev_err(&data->client->dev, "How did we get no touch events here");
        ft6x06_ts_release(data);
    }
}

/*The ft6x06 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *ft6x06_ts = dev_id;
    struct ft6x06_ts_event events[POINT_READ_BUF];
	int ret = 0;
	disable_irq_nosync(ft6x06_ts->irq);
    memset(events, 0, POINT_READ_BUF*sizeof(struct ft6x06_ts_event));
	ret = ft6x06_read_Touchdata(ft6x06_ts, events);
    enable_irq(ft6x06_ts->irq);
	if (ret == 0)
		ft6x06_report_value(ft6x06_ts, events);


	return IRQ_HANDLED;
}

static int ft6x06_ts_init_input(struct i2c_client *client,
                                struct ft6x06_ts_data *ft6x06_ts) {
    int err=0;
    struct input_dev *input_dev;
	input_dev = input_allocate_device();
	if (!input_dev) {
        dev_err(&client->dev, "failed to allocate input device\n");
		err = -ENOMEM;
        goto input_allocate_failed;
	}
    dev_dbg(&client->dev, "FT6x06_ts: Initializing input interface!");
	ft6x06_ts->input_dev = input_dev;
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, ft6x06_ts->x_max, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, ft6x06_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	input_dev->name = FT6X06_NAME;
    input_dev->phys = "mbot_ts/input0";
	err = input_register_device(input_dev);
    if(err < 0) {
        dev_err(&client->dev, "failed to register input device: %d\n", err);
        goto input_register_failed;
    }
    dev_dbg(&client->dev, "FT6x06_ts: Input interface initialized!");
    return err;

 input_register_failed:
    input_free_device(input_dev);
 input_allocate_failed:
    return err;
}

static int ft6x06_i2c_init(struct i2c_client *client) {
    int err=0;
    u8 uc_reg_arr[4] = {0};
    int pinctr=0, pinid=0;
    dev_dbg(&client->dev, "FT6x06_ts: Initializing panel via i2c!");
	/*make sure CTP already finish startup process */
	msleep(150);
    /* wake up the touchscreen controller */
    uc_reg_arr[0] = FT6x06_REG_ENABLE;
    uc_reg_arr[1] = 0x00;
    if((err = ft6x06_i2c_Write(client, uc_reg_arr, 2) < 0)) {
        return err;
    }
    msleep(10);
	/* configure sub-frames per scan */
    uc_reg_arr[0] = FT6x06_REG_CLKDOMAIN;
    uc_reg_arr[1] = 0x3;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    /* configure drive electrode count */
    uc_reg_arr[0] = FT6x06_REG_DRIVE_CNT;
    uc_reg_arr[1] = 0x0D;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    /* configure sense electrode count */
    uc_reg_arr[0] = FT6x06_REG_SENSE_CNT;
    uc_reg_arr[1] = 0x0D;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    /* configure pin assignment and slew rates  */
    for(pinctr=FT6x06_REG_PINCFG0, pinid=0;
        pinctr<FT6x06_REG_PINCFG20+1;
        pinctr++, pinid++) {
        uc_reg_arr[0] = pinctr;
        uc_reg_arr[1] = pinid;
        if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
            return err;
        }
    }

    uc_reg_arr[0] = FT6x06_REG_SUBFRAMES;
    uc_reg_arr[1] = 0x3;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_SAMPLEDELAY;
    uc_reg_arr[1] = 0x1;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_WRITE_MODE;
    uc_reg_arr[1] = 0xd;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_BOOSTERCTL;
    uc_reg_arr[1] = 0x02;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_DRIVELVL;
    uc_reg_arr[1] = 0x0C;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_DELTARANGE;
    uc_reg_arr[1] = 0x0;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_MINFINGERAREA;
    uc_reg_arr[1] = 0x01;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_MINFINGERLVL;
    uc_reg_arr[1] = 0x3A;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_MINFINGERWT;
    uc_reg_arr[1] = 0x00;
    uc_reg_arr[2] = 0x40;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 3))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_MAXFINGERAREA;
    uc_reg_arr[1] = 0x1e;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0){
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_MAXSEGDEPTH;
    uc_reg_arr[1] = 0x03;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_CGMETHOD;
    uc_reg_arr[1] = 0x01;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_AVGFILTEN;
    uc_reg_arr[1] = 0x01;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_1CLICKTIME;
    uc_reg_arr[1] = 0x00;
    uc_reg_arr[2] = 0xFF;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 3))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_2CLICKTIME;
    uc_reg_arr[1] = 0x00;
    uc_reg_arr[2] = 0xFF;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 3))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_CGTOLERANCE;
    uc_reg_arr[1] = 0x20;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_XTOLERANCE;
    uc_reg_arr[1] = 0x40;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0){
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_YTOLERANCE;
    uc_reg_arr[1] = 0x40;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_SENSEFILT;
    uc_reg_arr[1] = 0x01;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_SAMPLEDELAY;
    uc_reg_arr[1] = 0x03;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_DELTAFILTER;
    uc_reg_arr[1] = 0x01;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_RESETINIT;
    uc_reg_arr[1] = 0x00;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_XCOORDSCALE;
    uc_reg_arr[1] = 0x35;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_YCOORDSCALE;
    uc_reg_arr[1] = 0x36;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_EVENTMASK;
    // Turn on finger entry, leave, move
    // are these masks one to enable and 0 to disable? who knows
    uc_reg_arr[1] = (u8)~((1<<7) | (1<<6) | (1<<5));
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }

    uc_reg_arr[0] = FT6x06_REG_IRQMASK;
    uc_reg_arr[1] = ~(1<<4);
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }
    
    uc_reg_arr[0] = FT6x06_REG_EVENTSTATUS;
    uc_reg_arr[1] = -1;
    if((err=ft6x06_i2c_Read(client, uc_reg_arr, 1, uc_reg_arr, 1))<0) {
        return err;
    }
    
    uc_reg_arr[0] = FT6x06_REG_EVSTACKCLR;
    uc_reg_arr[1] = 0;
    if((err=ft6x06_i2c_Write(client, uc_reg_arr, 2))<0) {
        return err;
    }
    dev_dbg(&client->dev, "FT6x06_ts: Finished initializing panel!");
    return 0;
}

static int ft6x06_ts_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
	struct ft6x06_platform_data *pdata =
	    (struct ft6x06_platform_data *)client->dev.platform_data;
	struct ft6x06_ts_data *ft6x06_ts;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft6x06_ts = kzalloc(sizeof(struct ft6x06_ts_data), GFP_KERNEL);

	if (!ft6x06_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	i2c_set_clientdata(client, ft6x06_ts);
	ft6x06_ts->irq = client->irq;
	ft6x06_ts->client = client;
	ft6x06_ts->pdata = pdata;
	ft6x06_ts->x_max = pdata->x_max - 1;
	ft6x06_ts->y_max = pdata->y_max - 1;

#ifdef CONFIG_PM
	err = gpio_request(pdata->reset, "ft6x06 reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
#endif

	err = request_threaded_irq(client->irq, NULL, //ft6x06_ts_primary_irq_handler,
                               ft6x06_ts_interrupt,
                               IRQF_TRIGGER_NONE | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                               client->dev.driver->name,
                               ft6x06_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft6x06_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);
    err = ft6x06_ts_init_input(client, ft6x06_ts);
    if(err < 0) {
        dev_err(&client->dev,
                "%s: failed to initialize input device: %d\n",
                __func__, err);
        goto exit_input_init_failed;
    }

    err = ft6x06_i2c_init(client);
    if(err < 0) {
        dev_err(&client->dev,
                "%s: failed to finish i2c setup: %d\n",
                __func__, err);
        goto exit_i2c_init_failed;
    }

#ifdef SYSFS_DEBUG
	ft6x06_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif

    enable_irq(client->irq);
    dev_dbg(&client->dev, "Ft6x06_ts: Finished initializing touch device!");
    return 0;
exit_i2c_init_failed:
exit_input_init_failed: // everything was handled in the function
    free_irq(client->irq, ft6x06_ts);
exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft6x06_ts);
#ifdef CONFIG_PM
exit_request_reset:
	gpio_free(ft6x06_ts->pdata->reset);
#endif
exit_alloc_data_failed:
exit_check_functionality_failed:
    dev_err(&client->dev, "FT6x06_ts: Error in initialization!");
	return err;
}

static int ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	input_unregister_device(ft6x06_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(ft6x06_ts->pdata->reset);
	#endif

	#ifdef SYSFS_DEBUG
	ft6x06_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	free_irq(client->irq, ft6x06_ts);
	kfree(ft6x06_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft6x06_ts_id[] = {
	{FT6X06_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);

static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.id_table = ft6x06_ts_id,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
        .name = FT6X06_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init ft6x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft6x06_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft6x06 driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft6x06_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}

module_init(ft6x06_ts_init);
module_exit(ft6x06_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL");
