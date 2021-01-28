// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456_irq.c - Linux kernel driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 */

#include "bma456.h"


static irqreturn_t bma456_irq(int irq, void *handle)
{
	uint8_t rslt;
	uint16_t int_status;
	struct bma456_data *data = (struct bma456_data *)handle;

	/* Read interrupt status */
	rslt = bma456_read_int_status(&int_status, &data->bma);
	bma4_error_codes_print_result("bma456_read_int_status", rslt);
	if (rslt != BMA4_OK)
		return IRQ_HANDLED;
	data->int_status = int_status;
	
	if (int_status & BMA456_ANY_MOT_INT) {
		//DBG_FUNC("BMA456_ANY_MOT_INT");
	}
	if (int_status & BMA456_NO_MOT_INT) {
		//DBG_FUNC("BMA456_NO_MOT_INT");
	}
	if (int_status & BMA456_SINGLE_TAP_INT) {
		DBG_FUNC("BMA456_SINGLE_TAP_INT");
		bma456_do_tap(data);
	}
	if (int_status & BMA456_DOUBLE_TAP_INT) {
		DBG_FUNC("BMA456_DOUBLE_TAP_INT");
		bma456_do_tap(data);
		bma456_do_tap(data);
	}
	if (int_status & BMA456_ACTIVITY_INT) {
		uint8_t activity_output = 0;

		rslt = bma456_activity_output(&activity_output, &data->bma);
		bma4_error_codes_print_result("bma456_activity_output status", rslt);
		if (rslt != BMA4_OK)
			return IRQ_HANDLED;
		switch (activity_output)
        {
		case BMA456_USER_STATIONARY:
			DBG_FUNC("BMA456_USER_STATIONARY");
		break;
		
		case BMA456_USER_WALKING:
			DBG_FUNC("BMA456_USER_WALKING");
		break;
		
		case BMA456_USER_RUNNING:
			DBG_FUNC("BMA456_USER_RUNNING");
		break;

		case BMA456_STATE_INVALID:
			DBG_FUNC("BMA456_STATE_INVALID");
		break;

		default:
		break;
		}
	}
	if (int_status & BMA456_STEP_CNTR_INT) {
		uint32_t step_out = 0;
		rslt = bma456_step_counter_output(&step_out, &data->bma);
		bma4_error_codes_print_result("bma456_step_counter_output status", rslt);
		DBG_FUNC("step_out=%d", step_out);
	}
	if (int_status & BMA4_ACCEL_DATA_RDY_INT) {
		struct bma4_accel sens_data = { 0 };
		rslt = bma4_read_accel_xyz(&sens_data, &data->bma);
      	if (rslt == BMA4_OK) {
			//DBG_FUNC("x=%d,y=%d,z=%d", sens_data.x, sens_data.y, sens_data.z);
		}
	}

	return IRQ_HANDLED;
}

/* work_cb_irq: scan bma456's interrupt. */
static void work_cb_irq(struct work_struct *work)
{
	struct bma456_data *data = container_of((struct delayed_work*)work, 
		struct bma456_data, work_irq);
	int msec = bw_table[data->accel.odr - BMA4_OUTPUT_DATA_RATE_0_78HZ] / 1000;

    if (msec <= 0) {
        msec = 1;
    }

	bma456_irq(data->client->irq, data);
	schedule_delayed_work(&data->work_irq, msecs_to_jiffies(msec));

	return;
}


int bma456_irq_request(struct device *dev, int irq, const char *devname, struct bma456_data *data)
{
	int ret = 0;

	if (irq) {
		ret = devm_request_threaded_irq(dev, irq, 
				NULL,
				bma456_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				dev_name(dev),
				(void *)data);
		if (ret) {
			dev_err(dev, "irq %d busy?\n", irq);
			return -EBUSY;
		}
	}
	else {
		INIT_DELAYED_WORK(&data->work_irq, work_cb_irq);
		schedule_delayed_work(&data->work_irq, 0);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(bma456_irq_request);


int bma456_irq_free(struct bma456_data *data, int irq)
{
	if (!irq) {
		cancel_delayed_work_sync(&data->work_irq);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bma456_irq_free);
