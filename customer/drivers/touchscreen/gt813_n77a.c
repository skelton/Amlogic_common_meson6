/* drivers/input/touchscreen/gt813_827_828.c
 *
 * 2010 - 2012 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version:1.2
 * Author:scott@goodix.com
 * Release Date:2012/06/08
 * Revision record:
 *	  V1.0:2012/05/01,create file.
 *	  V1.0:2012/06/08,add slot report mode.
 */

#include <linux/irq.h>
#include <linux/gt813_n77a.h>
#include <linux/input/mt.h>

static const char	*goodix_ts_name = "gt813_n77a";
static struct		workqueue_struct *goodix_wq;
struct i2c_client	*i2c_connect_client = NULL;
static u8 		config[GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH] \
			 = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

static unsigned GTP_RST_PORT;    //PAD_GPIOA_6
static unsigned GTP_INT_PORT;    //PAD_GPIOA_16

static int gt813_printk_enable_flag=0;
#define print_gt(fmt, args...)  { if(gt813_printk_enable_flag) \
					printk("==GT813==[%d]"fmt"\n", __LINE__, ##args); }

static s8 gtp_i2c_test(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif


#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

/*******************************************************
Function:
	Read data from the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.

Output:
	numbers of i2c_msgs to transfer
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret=-1;
	s32 retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = GTP_I2C_ADDR;//client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = GTP_I2C_ADDR;//client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)break;
		retries++;
	}
	return ret;
}

/*******************************************************
Function:
	write data to the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.

Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
	struct i2c_msg msg;
	s32 ret=-1;
	s32 retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = GTP_I2C_ADDR;//client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)break;
		retries++;
	}
	return ret;
}

/*******************************************************
Function:
	write i2c end cmd.

Input:
	client:	i2c device.

Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
s32 gtp_i2c_end_cmd(struct i2c_client *client)
{
	s32 ret = -1;
	u8 end_cmd_data[2]={0x80, 0x00};

	ret = gtp_i2c_write(client, end_cmd_data, 2);

	return ret;
}

/*******************************************************
Function:
	Send config Function.

Input:
	client:	i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 0;
#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, config , GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
		gtp_i2c_end_cmd(client);
		if (ret > 0){
			break;
		}
	}
#endif
	return ret;
}

/*******************************************************
Function:
	Enable IRQ Function.

Input:
	ts:	i2c client private struct.

Output:
	None.
*******************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable)
	{
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Disable IRQ Function.

Input:
	ts:	i2c client private struct.

Output:
	None.
*******************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable)
	{
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Touch down report function.

Input:
	ts:private data.
	id:tracking id.
	x:input x.
	y:input y.
	w:input weight.

Output:
	None.
*******************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
	int xmax,ymax;
	int offset=18;
	xmax = ts->pdata->xmax;
	ymax = ts->pdata->ymax;
	if(ts->pdata->swap_xy){
		GTP_SWAP(x, y);
	}
	if((x > xmax)||(y > ymax)) return;
	if (x>=xmax-offset && x<=xmax){
		x -= offset;
	}else if (x>=0 && x<=offset){
		x += offset;
	}
	if (y>=ymax-offset && y<=ymax){
		y -= offset;
	}else if (y>=0 && y<=offset){
		y += offset;
	}
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
#else
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif

	print_gt("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
	Touch up report function.

Input:
	ts:private data.

Output:
	None.
*******************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	print_gt("Touch id[%2d] release!", id);
#else
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
#endif
}

/*******************************************************
Function:
	Goodix touchscreen work function.

Input:
	work:	work_struct of goodix_wq.

Output:
	None.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8  point_data[2 + 2 + 5 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8  check_sum = 0;
	u8  touch_num = 0;
	u8  finger = 0;
	static u8 pre_touch = 0;
	u8  key_value = 0;
	u8* coor_data = NULL;
	s32 x = 0;
	s32 y = 0;
	s32 input_w = 0;
	s32 idx = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;

	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update)
	{
		goto exit_work_func;
	}

	ret = gtp_i2c_read(ts->client, point_data, 10);
	if (ret < 0)
	{
		goto exit_work_func;
	}

	finger = point_data[GTP_ADDR_LENGTH];
	touch_num = (finger & 0x01) + !!(finger & 0x02) + !!(finger & 0x04) + !!(finger & 0x08) + !!(finger & 0x10);
	if (touch_num > 1)
	{
		u8 buf[25] = {(GTP_READ_COOR_ADDR + 8) >> 8, (GTP_READ_COOR_ADDR + 8) & 0xff};
		ret = gtp_i2c_read(ts->client, buf, 2 + 5 * (touch_num - 1));
		memcpy(&point_data[10], &buf[2], 5 * (touch_num - 1));
	}
	gtp_i2c_end_cmd(ts->client);

	if((finger & 0xC0) != 0x80)
	{
		print_gt("Data not ready!");
		goto exit_work_func;
	}

	key_value = point_data[3]&0x0f;
	if ((key_value & 0x0f) == 0x0f)
	{
		ret = gtp_send_cfg(ts->client);
		if (ret < 0)
		{
			print_gt("Reload config failed!\n");
		}
		goto exit_work_func;
	}

	coor_data = &point_data[4];
	check_sum = 0;
	for ( idx = 0; idx < 5 * touch_num; idx++)
	{
		check_sum += coor_data[idx];
	}
	if (check_sum != coor_data[5 * touch_num])
	{
		print_gt("Check sum error!");
		goto exit_work_func;
	}

	//print_gt("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT
	if (pre_touch || touch_num)
	{
		s32 pos = 0;
		for (idx = 0; idx < GTP_MAX_TOUCH; idx++)
		{
			if (finger & (0x01 << idx))
			{
				x = (coor_data[pos] << 8) | coor_data[pos + 1];
				y = (coor_data[pos + 2] << 8) | coor_data[pos + 3];
				y = ts->pdata->ymax - y;
				input_w = coor_data[pos + 4];
				pos += 5;
				gtp_touch_down(ts, idx, x, y, input_w);
			}
			else if (pre_touch & (0x01 << idx))
			{
				gtp_touch_up(ts, idx);
			}
		}
	}
#else
	if (touch_num)
	{
		s32 pos = 0;
		for (idx = 0; idx < GTP_MAX_TOUCH; idx++)
		{
			if (!(finger & (0x01 << idx)))
			{
				continue;
			}
			x = (coor_data[pos] << 8) | coor_data[pos + 1];
			y = (coor_data[pos + 2] << 8) | coor_data[pos + 3];
			y = ts->pdata->ymax - y;
			input_w = coor_data[pos + 4];
			pos += 5;
			gtp_touch_down(ts, idx, x, y, input_w);
		}
	}
	else if (pre_touch)
	{
		//print_gt("Touch Release!");
		gtp_touch_up(ts, 0);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
#endif

	input_sync(ts->input_dev);
	pre_touch = finger & 0x1f;
exit_work_func:
	if (ts->use_irq){
		gtp_irq_enable(ts);
	}
}

/*******************************************************
Function:
	Timer interrupt service routine.

Input:
	timer:	timer struct pointer.

Output:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);
	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine.

Input:
	irq:	interrupt number.
	dev_id: private data pointer.

Output:
	irq execute status.
*******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;
	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);
	return IRQ_HANDLED;
}

/*******************************************************
Function:
	Reset chip Function.

Input:
	ms:reset time.

Output:
	None.
*******************************************************/
void gtp_reset_guitar(s32 ms)
{
	GTP_GPIO_AS_OUTPUT(GTP_RST_PORT);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	msleep(30);
	GTP_GPIO_OUTPUT(GTP_RST_PORT,0);
	msleep(20);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);
	//GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	msleep(20);

	return;
}

/*******************************************************
Function:
	Eter sleep function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {GTP_REG_SLEEP >> 8, GTP_REG_SLEEP & 0xff, 0xc0};

	while(retry++ < 5)
	{
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		gtp_i2c_end_cmd(ts->client);
		if (ret > 0)
		{
			print_gt("GTP enter sleep!");
			return ret;
		}
		msleep(10);
	}
	print_gt("GTP send sleep cmd failed.");
	return ret;
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	ts:	private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
	u8 retry = 0;
	s8 ret = -1;

#if GTP_POWER_CTRL_SLEEP
	while(retry++ < 5)
	{
		gtp_reset_guitar(20);
		ret = gtp_send_cfg(ts->client);
		if (ret > 0)
		{
			print_gt("Wakeup sleep send config success.");
			return ret;
		}
	}
#else
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(2);
	GTP_GPIO_AS_INPUT(GTP_INT_PORT);
	msleep(2);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(2);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	msleep(50);
	while(retry++ < 10)
	{
		ret = gtp_i2c_test(ts->client);
		if (ret > 0)
		{
			print_gt("GTP wakeup sleep.");
			return ret;
		}
		gtp_reset_guitar(20);
	}
#endif

	print_gt("GTP wakeup sleep failed.");
	return ret;
}

/*******************************************************
Function:
	GTP initialize function.

Input:
	ts:	i2c client private struct.

Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;

	print_gt("%s: ts->pdata->xmax == %d ; ts->pdata->ymax == %d\n", __FUNCTION__,ts->pdata->xmax,ts->pdata->ymax);

#if GTP_DRIVER_SEND_CFG
	u8 rd_cfg_buf[16];

	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 *send_cfg_buf[3] = {cfg_info_group1, cfg_info_group2, cfg_info_group3};
	u8 cfg_info_len[3] = { sizeof(cfg_info_group1)/sizeof(cfg_info_group1[0]),
				sizeof(cfg_info_group2)/sizeof(cfg_info_group2[0]),
				sizeof(cfg_info_group3)/sizeof(cfg_info_group3[0])
				};
	print_gt("len1=%d,len2=%d,len3=%d",cfg_info_len[0],cfg_info_len[1],cfg_info_len[2]);
	if ((!cfg_info_len[1]) && (!cfg_info_len[2]))
	{
		rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
	}
	else
	{
		rd_cfg_buf[0] = GTP_REG_SENSOR_ID >> 8;
		rd_cfg_buf[1] = GTP_REG_SENSOR_ID & 0xff;
		ret = gtp_i2c_read(ts->client, rd_cfg_buf, 3);
		gtp_i2c_end_cmd(ts->client);
		if (ret < 0)
		{
			GTP_ERROR("Read SENSOR ID failed,default use group1 config!");
			rd_cfg_buf[GTP_ADDR_LENGTH] = 0;
		}
		rd_cfg_buf[GTP_ADDR_LENGTH] &= 0x03;
	}
	print_gt("SENSOR ID:%d", rd_cfg_buf[GTP_ADDR_LENGTH]);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[rd_cfg_buf[GTP_ADDR_LENGTH]], GTP_CONFIG_LENGTH);


	config[RESOLUTION_LOC]	= (u8)((ts->pdata->xmax)>>8);//(u8)(GTP_MAX_WIDTH>>8);
	config[RESOLUTION_LOC + 1]	= (u8)(ts->pdata->xmax);//(u8)GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 2]	= (u8)((ts->pdata->ymax)>>8);//(u8)(GTP_MAX_HEIGHT>>8);
	config[RESOLUTION_LOC + 3]	= (u8)(ts->pdata->ymax);//(u8)GTP_MAX_HEIGHT;

	if (ts->pdata->irq_edge == 0){  //FALLING
		config[TRIGGER_LOC] &= 0xf7;
	}
	else{//RISING
		config[TRIGGER_LOC] |= 0x08;
	}
#else //else DRIVER NEED NOT SEND CONFIG
	ret = gtp_i2c_read(ts->client, config, GTP_CONFIG_LENGTH + GTP_ADDR_LENGTH);
	gtp_i2c_end_cmd(ts->client);
	if (ret < 0)
	{
		print_gt("GTP read resolution & max_touch_num failed, use default value!");
		ts->abs_x_max = ts->pdata->xmax;//GTP_MAX_WIDTH;
		ts->abs_y_max = ts->pdata->ymax;//GTP_MAX_HEIGHT;
		ts->int_trigger_type = ts->pdata->irq_edge;
	}
#endif //endif GTP_DRIVER_SEND_CFG

	ts->abs_x_max = ts->pdata->xmax;
	ts->abs_y_max = ts->pdata->ymax;
	ts->int_trigger_type = ts->pdata->irq_edge;
	ret = gtp_send_cfg(ts->client);
	if (ret < 0)
	{
		print_gt("Send config error.");
	}

	print_gt("ts->abs_x_max = %d,ts->abs_y_max = %d,ts->int_trigger_type = 0x%02x",
		ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

	return 0;
}

/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client:	i2c client struct.
	version:address to store version info

Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	ret = gtp_i2c_read(client, buf, 6);
	gtp_i2c_end_cmd(client);
	if (ret < 0)
	{
		print_gt("GTP read version failed");
		return ret;
	}

	*version = (buf[3] << 8) | buf[4];

	print_gt("IC VERSION:%02x_%02x%02x", buf[2], buf[3], buf[4]);

	return ret;
}

/*******************************************************
Function:
	I2c test Function.

Input:
	client:i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 retry = 0;
	s8 ret = -1;
	while(retry++ < 5)
	{
		ret = gtp_i2c_end_cmd(client);
		if (ret > 0)
		{
			return ret;
		}
		print_gt("GTP i2c test failed time %d.",retry);
		msleep(10);
	}
	return ret;
}

/*******************************************************
Function:
	Request gpio Function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	gpio_set_status(GTP_INT_PORT, gpio_status_in);
	gpio_irq_set(GTP_INT_PORT, GPIO_IRQ( (GTP_INT_IRQ -INT_GPIO_0), GPIO_IRQ_FALLING));
	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	gtp_reset_guitar(20);

	return ret;
}

/*******************************************************
Function:
	Request irq Function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;

	ret  = request_irq(GTP_INT_IRQ, goodix_ts_irq_handler,
		IRQF_DISABLED,
		"GTP_INT_IRQ", ts);
	if (ret)
	{
		print_gt("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	}
	else
	{
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
	Request input device Function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
	u16 report_max_x = 0;
	u16 report_max_y = 0;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
	{
		print_gt("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
#if GTP_ICS_SLOT_REPORT
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	input_mt_init_slots(ts->input_dev, 255);
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

	report_max_x = ts->abs_x_max;
	report_max_y = ts->abs_y_max;
	if(ts->pdata->swap_xy){
		GTP_SWAP(report_max_x, report_max_y);
	}

	input_set_abs_params(ts->input_dev, ABS_X, 0, report_max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, report_max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, report_max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, report_max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	sprintf(phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		print_gt("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;
}

/*******************************************************
Function:
	Goodix touchscreen probe function.

Input:
	client:	i2c device struct.
	id:device id.

Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;
	client->addr = GTP_I2C_ADDR;
	printk("I2C addr:%x", client->addr);

	//do NOT remove these output log
	printk("GTP Driver Version:%s",GTP_DRIVER_VERSION);
	printk("GTP Driver build@%s,%s", __TIME__,__DATE__);

	i2c_connect_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("I2C check functionality failed.");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
	{
		printk("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}
	memset(ts, 0, sizeof(*ts));
	ts->pdata = client->dev.platform_data;
	GTP_RST_PORT = ts->pdata->gpio_rst;
	GTP_INT_PORT = ts->pdata->gpio_irq;

	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	spin_lock_init(&ts->irq_lock);

	ret = gtp_request_io_port(ts);
//	if (ret < 0)
//	{
//		GTP_ERROR("GTP request IO port failed.");
//		kfree(ts);
//		return ret;
//	}

	ret = gtp_i2c_test(client);
	if (ret < 0)
	{
		printk("I2C communication ERROR!");
	}

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0)
	{
		printk("Create update thread error.");
	}
#endif

	ret = gtp_init_panel(ts);
	if (ret < 0)
	{
		printk("GTP init panel failed.");
	}

	ret = gtp_request_input_dev(ts);
	if (ret < 0)
	{
		printk("GTP request input dev failed");
	}

	ret = gtp_request_irq(ts);
	if (ret < 0)
	{
		printk("GTP works in polling mode.");
	}
	else
	{
		printk("GTP works in interrupt mode.");
	}

	ret = gtp_read_version(client, &version_info);
	if (ret < 0)
	{
		printk("Read version failed.");
	}

	gtp_irq_enable(ts);

	return 0;
}


/*******************************************************
Function:
	Goodix touchscreen driver release function.

Input:
	client:	i2c device struct.

Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts)	{
		if (ts->use_irq){
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			GTP_GPIO_FREE(GTP_INT_PORT);
			free_irq(client->irq, ts);
		}
		else{
			hrtimer_cancel(&ts->timer);
		}
	}

	//print_gt("GTP driver is removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

/*******************************************************
Function:
	Early suspend function.

Input:
	h:early_suspend struct.

Output:
	None.
*******************************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	if (ts->use_irq){
		gtp_irq_disable(ts);
	}
	else{
		hrtimer_cancel(&ts->timer);
	}
	ret = gtp_enter_sleep(ts);
	if (ret < 0){
		printk("GTP early suspend failed.");
	}
}

/*******************************************************
Function:
	Late resume function.

Input:
	h:early_suspend struct.

Output:
	None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	ret = gtp_wakeup_sleep(ts);
	if (ret < 0)
	{
		printk("GTP later resume failed.");
	}

	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
	else
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver goodix_ts_driver = {
	.probe	  = goodix_ts_probe,
	.remove	 = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= goodix_ts_early_suspend,
	.resume	 = goodix_ts_later_resume,
#endif
	.id_table   = goodix_ts_id,
	.driver = {
		.name	 = GTP_I2C_NAME,
		.owner	= THIS_MODULE,
	},
};

/*******************************************************
Function:
	Driver Install function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit goodix_ts_init(void)
{
	s32 ret;

	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq)
	{
		printk("Creat workqueue failed.");
		return -ENOMEM;
	}
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************
Function:
	Driver uninstall function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
	{
		destroy_workqueue(goodix_wq);
	}
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
