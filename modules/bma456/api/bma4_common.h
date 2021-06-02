/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef _BMA4_COMMON_H
#define _BMA4_COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

//#include <stdio.h>
#include "bma4.h"

/*!
 *  @brief Function for initialization of I2C bus
 *
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t user_i2c_init(void);

/*!
 *  @brief Function for initialization of SPI bus
 *
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t user_spi_init(void);

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMA4_INTF_RET_SUCCESS -> Success
 *  @retval != BMA4_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMA4_INTF_RET_SUCCESS -> Success
 *  @retval != BMA4_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMA4_INTF_RET_SUCCESS -> Success
 *  @retval != BMA4_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BMA4_INTF_RET_SUCCESS -> Success
 *  @retval != BMA4_INTF_RET_SUCCESS  -> Failure Info
 *
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us    : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void user_delay(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bma : Structure instance of bma4_dev
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bma4_interface_selection(struct bma4_dev *bma);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bma4_error_codes_print_result(const char api_name[], /*uint16_t*/int8_t rslt);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMA4_COMMON_H */
