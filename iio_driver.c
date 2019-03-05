/*
 * Support for ST VL53L0X FlightSense ToF Ranging Sensor on a i2c bus.
 *
 * Copyright (C) 2016 STMicroelectronics Imaging Division.
 * 
 * Default 7-bit i2c slave address 0x29.
 *
 * https://www.st.com/resource/en/datasheet/vl53l0x.pdf
 * https://documentation.help/VL53L0X-API/documentation.pdf
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api_core.h"

#define VL53L0X_NUMBER_I2C_BUS 2
#define IRQ_NUM                25
#define VL53L0X_DRV_NAME       "vl53l0x"

#define VL53L0X_INTERRUPT_POLARITY_LOW 0

#define CHECK_ENABLE_SIGMA_FINAL_RANGE       0
#define CHECK_ENABLE_SIGNAL_RATE_FINAL_RANGE 1

#define VL53L0X_VCSEL_PERIOD_PRERANGE   0
#define VL53L0X_VCSEL_PERIOD_FINALRANGE 1

#define VL53L0X_SYSTEM_INTERRUPT_CLEAR  0x000B
#define VL53L0X_RESULT_INTERRUPT_STATUS 0x0013

#define VL53L0X_ON                     "on"
#define VL53L0X_OFF                    "off"
#define VL53L0X_SET_LONG_MODE          "long mode"
#define VL53L0X_SET_HIGH_ACCURACY_MODE "high accurancy"

#define vl53l0x_msg_log(fmt, ...)           \
    printk(KERN_INFO VL53L0X_DRV_NAME ": "  \
           fmt "\n",                        \
           ##__VA_ARGS__)

#define vl53l0x_msg_err(fmt, ...)           \
    printk(KERN_ERR VL53L0X_DRV_NAME ": "   \
           "[%s] " fmt "\n",                \
           __func__, ##__VA_ARGS__)

static int __must_check
vl53l0x_start(struct stmvl53l0x_data *data);

static int __must_check 
vl53l0x_stop(struct stmvl53l0x_data *data);

static int __must_check
vl53l0x_set_vcsel_pulse_period(struct stmvl53l0x_data *data,
                               u8 type, u8 pulse_period)
{
	return VL53L0X_SetVcselPulsePeriod(data, type, pulse_period);
}

static int __must_check
vl53l0x_get_ranging_measurement_data(struct stmvl53l0x_data *data,
                                     VL53L0X_RangingMeasurementData_t *rang)
{
	return VL53L0X_GetRangingMeasurementData(data, rang);
}

static int __must_check
vl53l0x_set_limit_check(struct stmvl53l0x_data *data, u8 cmd, uint point)
{
	return VL53L0X_SetLimitCheckValue(data, cmd, point);
}

static void  __maybe_unused
vl53l0x_device_info_print(VL53L0X_DeviceInfo_t *dev_info)
{
	if (likely(dev_info)) {
		vl53l0x_msg_log("device name: %s", dev_info->Name);
		vl53l0x_msg_log("device type: %s", dev_info->Type);
		vl53l0x_msg_log("device id: %s", dev_info->ProductId);
		vl53l0x_msg_log("product type: %d\n", dev_info->ProductType);
		vl53l0x_msg_log("product revision major : %d",
		                dev_info->ProductRevisionMajor);
		vl53l0x_msg_log("product revision minor : %d\n",
		                dev_info->ProductRevisionMinor);
	}
}

static int __must_check
vl53l0x_perform_ref_spad_management(struct stmvl53l0x_data *data,
                                    u32 *ref_spad_count, u8 *is_aperture_spads)
{
	return VL53L0X_PerformRefSpadManagement(data,
	                                        ref_spad_count, is_aperture_spads);
}

static int __must_check
vl53l0x_set_device_mode(struct stmvl53l0x_data *data, u8 device_mode)
{
	return VL53L0X_SetDeviceMode(data, device_mode);
}

static int __must_check
vl53l0x_perform_ref_calibration(struct stmvl53l0x_data *data,
                                u8 *vhv_settings, u8 *phase_cal)
{
	return VL53L0X_PerformRefCalibration(data, vhv_settings, phase_cal);
}

static int __must_check
vl53l0x_static_init(struct stmvl53l0x_data *data)
{
	return VL53L0X_StaticInit(data);
}

static int __must_check
vl53l0x_data_init(struct stmvl53l0x_data *data)
{
	return VL53L0X_DataInit(data);
}

static int __must_check
vl53l0x_start_measurement(struct stmvl53l0x_data *data)
{
	return VL53L0X_StartMeasurement(data);
}

static int __must_check
vl53l0x_stop_measurement(struct stmvl53l0x_data *data)
{
	return VL53L0X_StopMeasurement(data);
}

static int __must_check
vl53l0x_clear_interrupt_mask(struct stmvl53l0x_data *data)
{
	int err;
	u8 count = 0, byte;
	struct i2c_data *i2c_object =
			(__force struct i2c_data *)data->client_object;
	do {
		err = i2c_smbus_write_byte_data(i2c_object->client,
		                                VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (unlikely(err < 0)) {
			vl53l0x_msg_err("failed write byte 0x01, err=%d", err);
			return err;
		}
		err = i2c_smbus_write_byte_data(i2c_object->client,
		                                VL53L0X_SYSTEM_INTERRUPT_CLEAR, 0x00);
		if (unlikely(err < 0)) {
			vl53l0x_msg_err("failed write byte 0x0, err=%d", err);
			return err;
		}
		byte = i2c_smbus_read_byte_data(i2c_object->client,
		                                VL53L0X_RESULT_INTERRUPT_STATUS);
		++count;
	} while (((byte & 0x07) != 0x00) && (count < 3) && (!err));

	if (unlikely(count >= 3))
		err = -VL53L0X_ERROR_INTERRUPT_NOT_CLEARED;

	return err;
}

static irqreturn_t vl53l0x_interrupt_handler(int irq, void *irq_data)
{
	struct stmvl53l0x_data *data =
		(__force struct stmvl53l0x_data *)irq_data;
	if (likely(data->irq == irq))
		schedule_delayed_work(&data->dwork, 0);

	return IRQ_HANDLED;
}

static const struct iio_chan_spec vl53l0x_channels[] = {
	{
		.type = IIO_DISTANCE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int vl53l0x_read_distance(struct stmvl53l0x_data *data, int *val)
{
	*val = data->rangeData.RangeMilliMeter;

	return 0;
}

static int vl53l0x_read_raw(struct iio_dev *indio_dev,
                            const struct iio_chan_spec *chan,
                            int *val, int *val2, long mask)
{
	int err = -EINVAL;
	struct stmvl53l0x_data *data =
			(__force struct stmvl53l0x_data *)iio_priv(indio_dev);
	if (unlikely(chan->type != IIO_DISTANCE)) {
		vl53l0x_msg_err("invalid argument");
		return err;
	}
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->work_mutex);
		err = vl53l0x_read_distance(data, val);
		mutex_unlock(&data->work_mutex);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed read distance, err=%d", err);
			return err;
		}
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int __must_check
vl53l0x_set_working_state(struct stmvl53l0x_data *data, int state)
{
	int err = 0;

	mutex_lock(&data->work_mutex);
	if (!data->enable_ps_sensor && state) {
		err = vl53l0x_start(data);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed start for sensor, err=%d", err);
			goto exit;
		}
	} else if (data->enable_ps_sensor && !state) {
		err = vl53l0x_stop(data);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed stop for sensor, err=%d", err);
			goto exit;
		}
		data->rangeData.RangeMilliMeter = 0;
	}

exit:
	mutex_unlock(&data->work_mutex);
	return err;
}

static int __must_check
vl53l0x_set_ranging_mode(struct stmvl53l0x_data *data, int state)
{
	int err = 0;

	mutex_lock(&data->work_mutex);
	if (state)
		data->useLongRange = 1;
	else
		data->useLongRange = 0;
	if (likely(data->enable_ps_sensor)) {
		err = vl53l0x_stop(data);
		if (likely(!err))
			err = vl53l0x_start(data);
	}
	mutex_unlock(&data->work_mutex);

	return err;
}

static ssize_t vl53l0x_write_control(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t len)
{
	int err;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct stmvl53l0x_data *data =
			(__force struct stmvl53l0x_data *)iio_priv(indio_dev);
	if (unlikely(!data))
		return -ENOMEM;
	if (!strncmp(buf, VL53L0X_ON, strlen(VL53L0X_ON)))
		err = vl53l0x_set_working_state(data, 1);
	else if (!strncmp(buf, VL53L0X_OFF, strlen(VL53L0X_OFF)))
		err = vl53l0x_set_working_state(data, 0);
	else if (!strncmp(buf, VL53L0X_SET_LONG_MODE,
	         strlen(VL53L0X_SET_LONG_MODE)))
		err = vl53l0x_set_ranging_mode(data, 1);
	else if (!strncmp(buf, VL53L0X_SET_HIGH_ACCURACY_MODE,
	         strlen(VL53L0X_SET_HIGH_ACCURACY_MODE)))
		err = vl53l0x_set_ranging_mode(data, 0);
	else
		return -EINVAL;

	return err ? err : len;
}

static ssize_t vl53l0x_read_control(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	ssize_t len;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct stmvl53l0x_data *data = iio_priv(indio_dev);

	if (unlikely(!data))
		return -ENOMEM;
	len = sprintf(buf, "commands for control of vl53l0x proximity sensor:\n"
	                   "run: " "{" VL53L0X_ON ", " VL53L0X_OFF "}\n"
	                   "mode: " "{"VL53L0X_SET_LONG_MODE ", "
	                   VL53L0X_SET_HIGH_ACCURACY_MODE "}\n"
	                   "Current settings:\n");
	mutex_lock(&data->work_mutex);
	if (data->enable_ps_sensor)
		len += sprintf(buf + len, VL53L0X_DRV_NAME " " VL53L0X_ON "\n");
	else
		len += sprintf(buf + len, VL53L0X_DRV_NAME " " VL53L0X_OFF "\n");
	if (data->useLongRange)
		len += sprintf(buf + len, VL53L0X_DRV_NAME " "
		               "mode = " VL53L0X_SET_LONG_MODE "\n");
	else
		len += sprintf(buf + len, VL53L0X_DRV_NAME " "
		               "mode = " VL53L0X_SET_HIGH_ACCURACY_MODE "\n");
	mutex_unlock(&data->work_mutex);

	return len;
}

static IIO_DEVICE_ATTR(control, S_IRUGO | S_IWUSR,
                       vl53l0x_read_control, vl53l0x_write_control, 0);

static struct attribute *vl53l0x_attributes[] = {
	&iio_dev_attr_control.dev_attr.attr,
	NULL,
};

static const struct attribute_group vl53l0x_group = {
	.attrs = vl53l0x_attributes,
};

static const struct iio_info vl53l0x_info = {
	.attrs = &vl53l0x_group,
	.read_raw = vl53l0x_read_raw,
};

static void vl53l0x_ps_read_measurement(struct stmvl53l0x_data *data)
{
	data->ps_data = data->rangeData.RangeMilliMeter;
	input_report_abs(data->input_dev_ps, ABS_DISTANCE,
	                 (__force int)(data->ps_data + 5)/10);
	input_sync(data->input_dev_ps);
}

static void vl53l0x_work_handler(struct work_struct *work)
{
	int err;
	struct stmvl53l0x_data *data =
			container_of(work, struct stmvl53l0x_data, dwork.work);

	mutex_lock(&data->work_mutex);
	if (likely(data->enable_ps_sensor)) {
		err = vl53l0x_clear_interrupt_mask(data);
		if (unlikely(err))
			vl53l0x_msg_err("failed clear interrupt mask, err=%d", err);
		err = vl53l0x_get_ranging_measurement_data(data, &(data->rangeData));
		if (unlikely(err))
			vl53l0x_msg_err("failed get measurement data, err=%d", err);
		vl53l0x_ps_read_measurement(data);
	}
	mutex_unlock(&data->work_mutex);
}

static int __must_check
vl53l0x_setup(struct stmvl53l0x_data *data)
{
	int err;

	if (unlikely(!data))
		return ENOMEM;

	mutex_init(&data->work_mutex);
	INIT_DELAYED_WORK(&data->dwork, vl53l0x_work_handler);

	data->input_dev_ps = input_allocate_device();
	if (unlikely(!data->input_dev_ps)) {
		err = -ENOMEM;
		vl53l0x_msg_err("failed allocate memory, err=%d", err);
		goto err_exit;
	}
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	data->input_dev_ps->name = "vl53l0x proximity sensor";
	err = input_register_device(data->input_dev_ps);
	if (unlikely(err)) {
		err = -ENOMEM;
		vl53l0x_msg_err("failed input register device, err=%d", err);
		goto err_exit_free_dev_ps;
	}
	input_set_drvdata(data->input_dev_ps, data);

	data->reset = 1;
	data->delay_ms = 30;
	data->comms_type = 1;
	data->enableDebug = 0;
	data->useLongRange = 1;
	data->I2cDevAddr = 0x52;
	data->interMeasurems = 30;
	data->low_threshold = 200;
	data->enable_ps_sensor = 0;
	data->high_threshold = 400;
	data->comms_speed_khz = 400;
	data->timingBudget = 26000;
	data->gpio_polarity = VL53L0X_INTERRUPT_POLARITY_LOW;
	data->deviceMode = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
	data->gpio_function = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;

	return 0;

err_exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
err_exit:
	return err;
}

static int __must_check
vl53l0x_init_client(struct stmvl53l0x_data *data)
{
	int err;
	u32 ref_spad_count;
	u8 is_aperture_spads, vhv_settings, phase_cal;

	if (data->reset) {
		err = vl53l0x_data_init(data);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed data init, err=%d", err);
			return err;
		}
	}
	err = vl53l0x_static_init(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed data init, err=%d", err);
		return err;
	}
	if (data->reset) {
		err = vl53l0x_perform_ref_calibration(data, &vhv_settings, &phase_cal);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed perform ref calibration, err=%d", err);
			return err;
		}
		err = vl53l0x_perform_ref_spad_management(data, &ref_spad_count,
		                                          &is_aperture_spads);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed perform spad management, err=%d", err);
			return err;
		}
		data->reset = 0;
	}
	err = vl53l0x_set_device_mode(data, data->deviceMode);
	if (unlikely(err))
		vl53l0x_msg_err("failed set device mode, err=%d", err);

	return err;
}

static int __must_check
vl53l0x_configure_range(struct stmvl53l0x_data *data)
{
	int err;

	if (data->useLongRange) {
		vl53l0x_msg_log("configure long ranging");
		err = vl53l0x_set_limit_check(data,
		                              CHECK_ENABLE_SIGNAL_RATE_FINAL_RANGE,
		                              (__force uint)(65536/10));
		if (unlikely(err)) {
			vl53l0x_msg_err("failed signal rate final range, err=%d", err);
			return err;
		}
		err = vl53l0x_set_limit_check(data,
		                              CHECK_ENABLE_SIGMA_FINAL_RANGE,
		                              (__force uint)(60*65536));
		if (unlikely(err)) {
			vl53l0x_msg_err("failed sigma final range, err=%d", err);
			return err;
		}
		err = vl53l0x_set_vcsel_pulse_period(data,
		                                     VL53L0X_VCSEL_PERIOD_PRERANGE,
		                                     18);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed set pulse period 18, err=%d", err);
			return err;
		}
		err = vl53l0x_set_vcsel_pulse_period(data,
		                                     VL53L0X_VCSEL_PERIOD_FINALRANGE,
		                                     14);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed set pulse period 14, err=%d", err);
			return err;
		}
	} else {
		vl53l0x_msg_log("configure high accuracy");
		err = vl53l0x_set_limit_check(data,
		                              CHECK_ENABLE_SIGNAL_RATE_FINAL_RANGE,
		                              (__force uint)(25*65536/100));
		if (unlikely(err)) {
			vl53l0x_msg_err("failed signal rate final range, err=%d", err);
			return err;
		}
		err = vl53l0x_set_limit_check(data,
		                              CHECK_ENABLE_SIGMA_FINAL_RANGE,
		                              (__force uint)(18*65536));
		if (unlikely(err)) {
			vl53l0x_msg_err("failed sigma final range, err=%d", err);
			return err;
		}
		err = vl53l0x_set_vcsel_pulse_period(data,
		                                     VL53L0X_VCSEL_PERIOD_PRERANGE,
		                                     14);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed set pulse period 10, err=%d", err);
			return err;
		}
		err = vl53l0x_set_vcsel_pulse_period(data,
		                                     VL53L0X_VCSEL_PERIOD_FINALRANGE,
		                                     10);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed set pulse period 14, err=%d", err);
			return err;
		}
	}

	return err;
}

static int __must_check
vl53l0x_set_gpio(struct stmvl53l0x_data *data)
{
	int err, irq;

	err = gpio_request(IRQ_NUM, "vl53l0x_gpio_int");
	if (unlikely(err)) {
		vl53l0x_msg_err("gpio request false, err=%d", err);
		return err;
	}
	err = gpio_direction_input(IRQ_NUM);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed gpio direction input, err=%d", err);
		return err;
	}
	irq = gpio_to_irq(IRQ_NUM);
	if (unlikely(irq < 0)) {
		vl53l0x_errmsg("failed to map GPIO: %d to interrupt:%d\n",
			IRQ_NUM, irq);
		return err;
	}
	err = request_threaded_irq(irq, NULL, vl53l0x_interrupt_handler,
	                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	                           "vl53l0x_interrupt", (void *)data);
	if (unlikely(err)) {
		vl53l0x_msg_err("could not allocate irq, err=%d", err);
		free_irq(irq, data);
		return err;
	}
	data->irq = irq;

	return err;
}

static int __must_check
vl53l0x_start(struct stmvl53l0x_data *data)
{
	int err;

	err = vl53l0x_init_client(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed init client, err=%d", err);
		return err;
	}
	err = vl53l0x_configure_range(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed configure range, err=%d", err);
		return err;
	}
	err = vl53l0x_clear_interrupt_mask(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed clear interrupt mask, err=%d", err);
		return err;
	}
	err = vl53l0x_start_measurement(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed start measurement, err=%d", err);
		return err;
	}

	data->enable_ps_sensor = 1;

	return err;
}

static int __must_check
vl53l0x_stop(struct stmvl53l0x_data *data)
{
	int err;

	if (data->deviceMode == VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ||
	    data->deviceMode == VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING) {
		err = vl53l0x_stop_measurement(data);
		if (unlikely(err)) {
			vl53l0x_msg_err("failed stop measurement, err=%d", err);
			return err;
		}
	}
	err = vl53l0x_clear_interrupt_mask(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed clear interrupt mask, err=%d", err);
		return err;
	}
	data->enable_ps_sensor = 0;

	vl53l0x_msg_log("proximity sensor stopped");

	return err;
}

static int __cold vl53l0x_probe(struct i2c_client *client,
                                const struct i2c_device_id *id)
{
	int err;
	struct iio_dev *indio_dev;
	struct stmvl53l0x_data *data;
	struct i2c_data *i2c_object;

	if (unlikely(!i2c_check_functionality(client->adapter,
	                                      I2C_FUNC_SMBUS_BYTE_DATA))) {
		vl53l0x_msg_err("smbus not support");
		err = -EOPNOTSUPP;
		goto err_exit;
	}
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (unlikely(!indio_dev)) {
		vl53l0x_msg_err("indio_dev not allocated");
		err = -ENOMEM;
		goto err_exit;
	}
	data = (__force struct stmvl53l0x_data *)iio_priv(indio_dev);
	data->client_object = kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
	if (unlikely(!data->client_object)) {
		vl53l0x_msg_err("client_object not allocated");
		err = -ENOMEM;
		goto err_exit_flush_iio;
	}
	i2c_object = (__force struct i2c_data *)data->client_object;
	i2c_object->client = client;
	data->bus_type = I2C_BUS;
	i2c_set_clientdata(client, indio_dev);

	indio_dev->info = &vl53l0x_info;
	indio_dev->name = VL53L0X_DRV_NAME;
	indio_dev->channels = vl53l0x_channels;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = ARRAY_SIZE(vl53l0x_channels);

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (unlikely(err)) {
		vl53l0x_msg_err("indio_dev not registed, err=%d", err);
		goto err_exit_flush_client_object;
	}
	err = vl53l0x_setup(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed setup for sensor, err=%d", err);
		goto err_exit_iio_device_unregister;
	}
	err = vl53l0x_set_gpio(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed set gpio config, err=%d", err);
		goto err_exit_iio_device_unregister;
	}
	err = vl53l0x_start(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed start for sensor, err=%d", err);
		goto err_exit_free_irq;
	}

	vl53l0x_msg_log("probe");

	return err;

err_exit_free_irq:
	free_irq(data->irq, data);
err_exit_iio_device_unregister:
	devm_iio_device_unregister(&client->dev, indio_dev);
err_exit_flush_client_object:
	kfree(data->client_object);
err_exit_flush_iio:
	devm_iio_device_free(&client->dev, indio_dev);
err_exit:
	return err;
}

static int __cold
vl53l0x_remove(struct i2c_client *client)
{
	int err;
	struct iio_dev *indio_dev;
	struct stmvl53l0x_data *data;

	indio_dev = (__force struct iio_dev *)i2c_get_clientdata(client);
	if (unlikely(!indio_dev))
		return -ENOMEM;
	data = (__force struct stmvl53l0x_data *)iio_priv(indio_dev);
	if (unlikely(!data))
		return -ENOMEM;
	err = vl53l0x_stop(data);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed stop for sensor, err=%d", err);
		return err;
	}
	free_irq(data->irq, data);
	gpio_free(IRQ_NUM);
	devm_iio_device_unregister(&client->dev, indio_dev);
	kfree(data->client_object);
	devm_iio_device_free(&client->dev, indio_dev);
	input_unregister_device(data->input_dev_ps);
	input_free_device(data->input_dev_ps);
	kfree(data);

	return err;
}

static const struct i2c_device_id vl53l0x_id[] = {
	{ VL53L0X_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, vl53l0x_id);

static const struct of_device_id vl53l0x_devtree_match[] = {
	{ .compatible = "st,vl53l0x-i2c", },
	{ }
};
MODULE_DEVICE_TABLE(of, vl53l0x_devtree_match);

static struct i2c_driver vl53l0x_driver = {
	.driver = {
		.name = VL53L0X_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vl53l0x_devtree_match,
	},
	.probe = vl53l0x_probe,
	.remove = vl53l0x_remove,
	.id_table = vl53l0x_id,
};

static int vl53l0x_init_i2c(void)
{
	int err;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {
		.type = VL53L0X_DRV_NAME,
		.addr = 0x29,
	};
	err = i2c_add_driver(&vl53l0x_driver);
	if (unlikely(err)) {
		vl53l0x_msg_err("failed add i2c driver, err=%d", err);
		goto err_exit;
	}
	adapter = i2c_get_adapter(VL53L0X_NUMBER_I2C_BUS);
	if (unlikely(!adapter)) {
		vl53l0x_msg_err("not find adapter, err=%d", err);
		goto err_exit_del_driver;
	}
	client = i2c_new_device(adapter, &info);
	if (unlikely(!client)) {
		vl53l0x_msg_err("didn't added client, err=%d", err);
		goto err_exit_put_adapter;
	}

	return err;

err_exit_put_adapter:
	i2c_put_adapter(adapter);
err_exit_del_driver:
	i2c_del_driver(&vl53l0x_driver);
err_exit:
	return err;
}

static void vl53l0x_exit_i2c(void)
{
	i2c_del_driver(&vl53l0x_driver);
}

static int __init vl53l0x_init(void)
{
	return vl53l0x_init_i2c();
}

static void __exit vl53l0x_exit(void)
{
	vl53l0x_exit_i2c();
}

module_init(vl53l0x_init);
module_exit(vl53l0x_exit);

MODULE_AUTHOR("Anton Mikaiev CPI Kiev");
MODULE_DESCRIPTION(VL53L0X_DRV_NAME "sensor driver");
MODULE_LICENSE("GPL");
