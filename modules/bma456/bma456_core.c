// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456_core.c - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 */

#include "bma456.h"


static void bma456_default(struct bma456_data *data)
{
	/* features:
		BMA456_SINGLE_TAP_INT  BMA456_STEP_CNTR_INT  BMA456_ACTIVITY_INT
		BMA456_WRIST_WEAR_INT  BMA456_DOUBLE_TAP_INT BMA456_ANY_MOT_INT
		BMA456_NO_MOT_INT      BMA456_ERROR_INT
	*/
	data->int_map = BMA456_ACTIVITY_INT | BMA456_STEP_CNTR_INT 
			/*| BMA456_ANY_MOT_INT | BMA456_NO_MOT_INT*/
			| BMA456_SINGLE_TAP_INT | BMA456_DOUBLE_TAP_INT
			| BMA4_DATA_RDY_INT;

    data->single_tap_sens = 0;
    data->double_tap_sens = 0;

    /* Accelerometer configuration settings */
	/* Output data Rate */
	data->accel.odr = BMA4_OUTPUT_DATA_RATE_200HZ;

	/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
	data->accel.range = BMA4_ACCEL_RANGE_2G;

	/* The bandwidth parameter is used to configure the number of sensor samples that are averaged
		if it is set to 2, then 2^(bandwidth parameter) samples
		are averaged, resulting in 4 averaged samples
		Note1 : For more information, refer the datasheet.
		Note2 : A higher number of averaged samples will result in a less noisier signal, but
		this has an adverse effect on the power consumed.
	*/
	data->accel.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

	/* Enable the filter performance mode where averaging of samples
		will be done based on above set bandwidth and ODR.
		There are two modes
		0 -> Averaging samples (Default)
		1 -> No averaging
		For more info on No Averaging mode refer datasheet.
	*/
	data->accel.perf_mode = BMA4_CIC_AVG_MODE;


    /*
        Set the slope threshold:
        Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
    */
    data->any_mot.threshold = 100;

    /*
        Set the duration for any-motion interrupt:
        Duration defines the number of consecutive data points for which threshold condition must be true(1
        bit =
        20ms)
    */
    data->any_mot.duration = 4;

    /* Enabling X, Y, and Z axis for Any-motion feature */
    data->any_mot.axes_en = BMA456_EN_ALL_AXIS;


    /*
        Set the slope threshold:
        Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
    */
    data->no_mot.threshold = 10;

    /*
        Set the duration for no-motion interrupt:
        Duration defines the number of consecutive data points for which threshold condition must be
        true(1 bit = 20ms)
    */
    data->no_mot.duration = 4;

    /* Enabling X, Y, and Z axis for no-motion feature */
    data->no_mot.axes_en = BMA456_EN_ALL_AXIS;
}


/* probe */
int8_t bma456_probe(struct device *dev, struct bma456_data *data)
{
//	struct iio_dev *indio_dev;
	struct bma4_dev *bma = &data->bma;
	int8_t rslt = 0;

	DBG_FUNC("");

    bma456_default(data);

	rslt = bma4_soft_reset(bma);
	bma4_error_codes_print_result("bma4_soft_reset status", rslt);
	if (rslt != BMA4_OK)
		return -ENODEV;
	msleep(150);

	/* Sensor initialization */
	rslt = bma456_init(bma);
	bma4_error_codes_print_result("bma456_init status", rslt);
	if (rslt != BMA4_OK) {
		return -ENODEV;
	}
	dev_info(dev, "chip_id:0x%x\n", bma->chip_id);

	/* Upload the configuration file to enable the features of the sensor. */
	rslt = bma456_write_config_file(bma);
	bma4_error_codes_print_result("bma456_write_config status", rslt);

	/* Enable the accelerometer */
	rslt = bma4_set_accel_enable(BMA4_ENABLE, bma);
	bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

	//set_accel_config(&data->bma);
    /* Set the accel configurations */
	rslt = bma4_set_accel_config(&data->accel, bma);
	bma4_error_codes_print_result("bma4_set_accel_config status", rslt);

	/* Map the interrupt 1 for any/no-motion step activity and step counter, single double tap*/
#if 1
	rslt = bma456_map_interrupt(BMA4_INTR1_MAP, data->int_map, BMA4_ENABLE, bma);
	bma4_error_codes_print_result("BMA4_INTR1_MAP status", rslt);
#else
	rslt = bma456_map_interrupt(BMA4_INTR1_MAP, 
		(BMA456_ACTIVITY_INT | BMA456_STEP_CNTR_INT | BMA456_ANY_MOT_INT | \
			BMA456_NO_MOT_INT | BMA456_SINGLE_TAP_INT | BMA456_DOUBLE_TAP_INT), 
		BMA4_ENABLE, bma);
	bma4_error_codes_print_result("bma456_map1_interrupt status", rslt);

	/* Mapping data ready interrupt with interrupt pin 2 to get interrupt status once getting new accel data */
	rslt = bma456_map_interrupt(BMA4_INTR2_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, bma);
	bma4_error_codes_print_result("bma456_map2_interrupt status", rslt);
#endif

	/* Setting watermark level 1, the output step resolution is 20 steps.
	Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
	*/
	rslt = bma456_step_counter_set_watermark(1, bma);
	bma4_error_codes_print_result("bma456_step_counter_set_watermark status", rslt);
	if (rslt != BMA4_OK)
		return rslt;
	
	/* Enabling step detector, tap features */
	rslt = bma456_feature_enable(BMA456_STEP_ACT | BMA456_STEP_CNTR | \
				BMA456_SINGLE_TAP | BMA456_DOUBLE_TAP, BMA4_ENABLE, bma);
	bma4_error_codes_print_result("bma456_feature_enable status", rslt);

    /* Set the threshold and duration configuration */
    rslt = bma456_set_any_mot_config(&data->any_mot, bma);
    bma4_error_codes_print_result("bma456_set_any_mot_config status", rslt);

    /* Set the threshold and duration configuration */
    rslt = bma456_set_no_mot_config(&data->any_mot, bma);
    bma4_error_codes_print_result("bma456_set_no_mot_config status", rslt);

	bma456_single_tap_set_sensitivity(data->single_tap_sens, bma);
	bma456_double_tap_set_sensitivity(data->double_tap_sens, bma);

	return rslt;
}

int bma456_remove(struct bma456_data *data)
{
	return 0;
}
