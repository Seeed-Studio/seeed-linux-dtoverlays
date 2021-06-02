/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

//#include <stdio.h>

#include "bma4_common.h"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of I2C bus.
 */
int8_t user_i2c_init(void)
{

    /* Implement I2C bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through SPI bus.
 */
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI read routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the I2C read routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through SPI bus.
 */
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI write routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the I2C write routine according to the target machine. */
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay(uint32_t period_us, void *intf_ptr)
{
    /* Implement the delay routine according to the target machine. */
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bma4_interface_selection(struct bma4_dev *bma)
{
    int8_t rslt = BMA4_OK;

    if (bma != NULL)
    {
        /* Select the interface for execution
         * For I2C : BMA4_I2C_INTF
         * For SPI : BMA4_SPI_INTF
         */
        bma->intf = BMA4_I2C_INTF;

        /* Bus configuration : I2C */
        if (bma->intf == BMA4_I2C_INTF)
        {
            printk("I2C Interface \n");

            /* To initialize the user I2C function */
            user_i2c_init();
            dev_addr = BMA4_I2C_ADDR_PRIMARY;
            bma->bus_read = user_i2c_read;
            bma->bus_write = user_i2c_write;
        }

        /* Bus configuration : SPI */
        else if (bma->intf == BMA4_SPI_INTF)
        {
            printk("SPI Interface \n");

            /* To initialize the user SPI function */
            user_spi_init();
            dev_addr = 0;
            bma->bus_read = user_spi_read;
            bma->bus_write = user_spi_write;
        }

        /* Assign device address to interface pointer */
        bma->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bma->delay_us = user_delay;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bma->read_write_len = 8;
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma4_error_codes_print_result(const char api_name[], /*uint16_t*/int8_t rslt)
{
    if (rslt != BMA4_OK)
    {
        printk("%s: ", api_name);
        if (rslt == BMA4_E_NULL_PTR)
        {
            printk("Error [%d] : Null pointer\n", rslt);
        }
        else if (rslt == BMA4_E_CONFIG_STREAM_ERROR)
        {
            printk("Error [%d] : Invalid configuration stream\n", rslt);
        }
        else if (rslt == BMA4_E_SELF_TEST_FAIL)
        {
            printk("Error [%d] : Self test failed\n", rslt);
        }
        else if (rslt == BMA4_E_INVALID_SENSOR)
        {
            printk("Error [%d] : Device not found\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printk("Error [%d] : Unknown error code\n", rslt);
        }
    }
}
