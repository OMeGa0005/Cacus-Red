/***************************************************************************/ /**
* @file Si1133.c
*******************************************************************************
* @section License
* <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
*******************************************************************************
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************/

/*
 * Si1133 sensor driver for light and UV measurements.
 * Provides initialization, configuration, and data retrieval functions.
 */

// Reference: Silicon Labs documentation [https://os.mbed.com/teams/SiliconLabs/code/Si1133//file/f780ca9105bb/Si1133.cpp/]
// Took inspiration but heavily modified to run in zephyr
// I mainly needed all of the obscure values, masks, and addresses

#include "Si1133.h"
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/i2c.h>

#define SI1133_I2C_ADDRESS (0x55) /** Hardcoded address for Si1133 sensor */

// We will register logging to the "si1133" module
LOG_MODULE_REGISTER(si1133, LOG_LEVEL_DBG);

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

#define X_ORDER_MASK 0x0070
#define Y_ORDER_MASK 0x0007
#define SIGN_MASK 0x0080
#define GET_X_ORDER(m) (((m) & X_ORDER_MASK) >> 4)
#define GET_Y_ORDER(m) (((m) & Y_ORDER_MASK))
#define GET_SIGN(m) (((m) & SIGN_MASK) >> 7)

#define UV_INPUT_FRACTION 15
#define UV_OUTPUT_FRACTION 12
#define UV_NUMCOEFF 2

#define ADC_THRESHOLD 16000
#define INPUT_FRACTION_HIGH 7
#define INPUT_FRACTION_LOW 15
#define LUX_OUTPUT_FRACTION 12
#define NUMCOEFF_LOW 9
#define NUMCOEFF_HIGH 4

/** @endcond */

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
/***************************************************************************/ /**
* @brief
*    Coefficients for lux calculation
******************************************************************************/
const LuxCoeff_t lk = {
    {{0, 209},       /**< coeff_high[0]   */
     {1665, 93},     /**< coeff_high[1]   */
     {2064, 65},     /**< coeff_high[2]   */
     {-2671, 234}},  /**< coeff_high[3]   */
    {{0, 0},         /**< coeff_low[0]    */
     {1921, 29053},  /**< coeff_low[1]    */
     {-1022, 36363}, /**< coeff_low[2]    */
     {2320, 20789},  /**< coeff_low[3]    */
     {-367, 57909},  /**< coeff_low[4]    */
     {-1774, 38240}, /**< coeff_low[5]    */
     {-608, 46775},  /**< coeff_low[6]    */
     {-1503, 51831}, /**< coeff_low[7]    */
     {-1886, 58928}} /**< coeff_low[8]    */
};

/***************************************************************************/ /**
* @brief
*    Coefficients for UV index calculation
******************************************************************************/
const Coeff_t uk[2] = {
    {1281, 30902}, /**< coeff[0]        */
    {-638, 46301}  /**< coeff[1]        */
};

/**************************************************************************/ /**
* @name Error Codes
* @{
******************************************************************************/
#define SI1133_OK 0x0000                                                     /**< No errors                  */
#define SI1133_ERROR_I2C_TRANSACTION_FAILED 0x0001                           /**< I2C transaction failed     */
#define SI1133_ERROR_SLEEP_FAILED 0x0002                                     /**< Entering sleep mode failed */
/**@}*/

/** @endcond */

/* Helper macro: choose the Zephyr I2C speed setting based on the requested frequency.
 * Zephyr typically supports I2C_SPEED_STANDARD (100kHz) and I2C_SPEED_FAST (400kHz).
 */
static inline uint32_t get_i2c_config(int hz)
{
    if (hz == 400000)
    {
        return I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST);
    }
    /* Fallback to standard speed if not 400kHz */
    return I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD);
}

/*
 * @brief Initializes the Si1133 sensor with the specified I2C device and frequency.
 *
 * @param sensor Pointer to the sensor state structure.
 * @param i2c_dev Pointer to the I2C device.
 * @param hz I2C frequency in Hz.
 * @return 0 on success, negative error code on failure.
 */
int si1133_init(si1133_t *sensor, const struct device *i2c_dev, int hz)
{
    int ret;
    if (!sensor || !i2c_dev)
    {
        printk("si1133_init: invalid argument(s)\n");
        return -EINVAL;
    }

    sensor->i2c_dev = i2c_dev;
    sensor->i2c_hz = hz;

    ret = i2c_configure(sensor->i2c_dev, get_i2c_config(hz));
    if (ret < 0)
    {
        printk("I2C configuration error: %d\n", ret);
        return ret;
    }
    // printk("Si1133 sensor initialised on I2C bus %s at %d Hz\n",
    //        sensor->i2c_dev->name, hz);

    return 0;
}

/*
 * @brief Opens the Si1133 sensor and verifies its presence.
 *
 * @param sensor Pointer to the sensor state structure.
 * @return true if the sensor is successfully opened, false otherwise.
 */
bool si1133_open(si1133_t *sensor)
{
    if (!sensor || !sensor->i2c_dev)
    {
        LOG_ERR("Invalid sensor or I2C device\n");
        return false;
    }

    // Perform a Zero Length Transfer to probe the Si1133.
    uint8_t dummy; // Dummy variable to use as a non-NULL pointer.
    int ret = i2c_write(sensor->i2c_dev, &dummy, 0, SI1133_I2C_ADDRESS);
    if (ret != 0)
    {
        LOG_ERR("I2C probe failed: %d\n", ret);
        return false;
    }

    // Initialise the sensor (assume `si1133_init` handles necessary setup).
    if (si1133_setup(sensor, sensor->i2c_dev, sensor->i2c_hz) == SI1133_OK)
    {
        // LOG_DBG("Si1133 sensor initialised successfully\n");
        return true;
    }

    LOG_ERR("Si1133 sensor initialisation failed\n");
    return false;
}

/*
 * @brief Measures the current light level in lux.
 *
 * @return Light level in lux.
 */
float get_light_level()
{
    float lux, uvi;
    measure_lux_uv(&lux, &uvi);
    return lux;
}

/*
 * @brief Measures the current UV index.
 *
 * @return UV index.
 */
float get_uv_index()
{
    float lux, uvi;
    measure_lux_uv(&lux, &uvi);
    return uvi;
}

/*
 * @brief Retrieves both light level and UV index from the sensor.
 *
 * @param light_level Pointer to store the light level in lux.
 * @param uv_index Pointer to store the UV index.
 * @return true on success, false on failure.
 */
bool get_light_and_uv(float *light_level, float *uv_index)
{
    if (measure_lux_uv(light_level, uv_index))
    {
        return false;
    }
    return true;
}

/***************************************************************************/ /**
* @brief
*    Reads register from the Si1133 sensor
*
* @param[in] reg
*    The register address to read from in the sensor.
*
* @param[out] data
*    The data read from the sensor
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_read_register(si1133_t *sensor, uint8_t reg, uint8_t *data)
{
    if (!sensor || !sensor->i2c_dev || !data)
    {
        return -EINVAL; // Invalid argument
    }

    /* Write the register address to the sensor */
    int ret = i2c_write(sensor->i2c_dev, &reg, 1, SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        printk("I2C write failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    /* Read the data from the sensor */
    ret = i2c_read(sensor->i2c_dev, data, 1, SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        printk("I2C read failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Writes register in the Si1133 sensor
*
* @param[in] reg
*    The register address to write to in the sensor
*
* @param[in] data
*    The data to write to the sensor
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_write_register(si1133_t *sensor, uint8_t reg, uint8_t data)
{
    if (!sensor || !sensor->i2c_dev)
    {
        printk("Invalid sensor or I2C device\n");
        return -EINVAL; // Invalid argument
    }

    /* Buffer to hold the register address and data */
    uint8_t buf[2] = {reg, data};

    /* Write the register address and data to the sensor */
    int ret = i2c_write(sensor->i2c_dev, buf, sizeof(buf), SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        printk("I2C write failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Writes a block of data to the Si1133 sensor.
*
* @param[in] reg
*    The first register to begin writing to
*
* @param[in] length
*    The number of bytes to write to the sensor
*
* @param[in] data
*    The data to write to the sensor
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_write_register_block(si1133_t *sensor, uint8_t reg, uint8_t length, const uint8_t *data)
{
    if (!sensor || !sensor->i2c_dev || !data)
    {
        LOG_ERR("Invalid argument(s) provided\n");
        return -EINVAL; // Invalid argument
    }

    if (length > SI1133_MAX_WRITE_LENGTH)
    {
        LOG_ERR("Write length exceeds maximum allowed: %d\n", length);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    /* Allocate a buffer to hold the register address and data */
    uint8_t buf[1 + SI1133_MAX_WRITE_LENGTH];
    buf[0] = reg; /* Set the first byte as the register address */

    /* Copy the data into the buffer starting from the second byte */
    memcpy(&buf[1], data, length);

    /* Perform the I2C write operation */
    int ret = i2c_write(sensor->i2c_dev, buf, length + 1, SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        LOG_ERR("I2C write failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Reads a block of data from the Si1133 sensor.
*
* @param[in] reg
*    The first register to begin reading from
*
* @param[in] length
*    The number of bytes to write to the sensor
*
* @param[out] data
*    The data read from the sensor
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_read_register_block(si1133_t *sensor, uint8_t reg, uint8_t length, uint8_t *data)
{
    if (!sensor || !sensor->i2c_dev || !data)
    {
        LOG_ERR("Invalid argument(s) provided\n");
        return -EINVAL; // Invalid argument
    }

    /* Write the register address to the sensor (to set up the read pointer) */
    int ret = i2c_write(sensor->i2c_dev, &reg, 1, SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        LOG_ERR("I2C write failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    /* Read the block of data from the sensor */
    ret = i2c_read(sensor->i2c_dev, data, length, SI1133_I2C_ADDRESS);
    if (ret < 0)
    {
        LOG_ERR("I2C read failed: %d\n", ret);
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Waits until the Si1133 is sleeping before proceeding
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t wait_until_sleep(void)
{
    k_sleep(K_MSEC(10));
    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Resets the Si1133
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_reset(si1133_t *sensor)
{
    int ret;

    if (!sensor || !sensor->i2c_dev)
    {
        printk("Invalid sensor state\n");
        return -EINVAL;
    }

    /* Wait at least 25 ms after power-up; use 30 ms to be safe */
    k_sleep(K_MSEC(30));

    /* Send the reset command via I2C.
     * Assume si1133_write_register() is the function we defined earlier that
     * performs a single-register write.
     */
    ret = si1133_write_register(sensor, REG_COMMAND, CMD_RESET);
    if (ret < 0)
    {
        printk("Reset command failed: %d\n", ret);
        return ret;
    }

    /* Delay for 10 ms for the sensor to complete its internal reset */
    k_sleep(K_MSEC(10));

    return SI1133_OK;
}

/**
 * @brief Sends a command to the Si1133 sensor.
 *
 * This function first reads the response register, waits for the sensor to
 * be ready by calling wait_until_sleep(), and then sends the command. It waits
 * for the response register to change (unless the command is CMD_RESET_CMD_CTR)
 * as an indication that the command has been accepted.
 *
 * @param sensor Pointer to the sensor state structure.
 * @param command The command to send.
 *
 * @return SI1133_OK on success, or an error code otherwise.
 */
int si1133_send_cmd(si1133_t *sensor, uint8_t command)
{
    int ret;
    uint8_t response;
    uint8_t response_stored;
    uint8_t count = 0;

    /* Get the response register contents */
    ret = si1133_read_register(sensor, REG_RESPONSE0, &response_stored);
    if (ret != SI1133_OK)
    {
        return ret;
    }

    response_stored &= RSP0_COUNTER_MASK;

    /* Double-check that the response register is consistent */
    while (count < 5)
    {
        ret = wait_until_sleep();
        if (ret != SI1133_OK)
        {
            return ret;
        }

        /* Skip further consistency checks if the command is RESET COMMAND COUNTER */
        if (command == CMD_RESET_CMD_CTR)
        {
            break;
        }

        ret = si1133_read_register(sensor, REG_RESPONSE0, &response);
        if ((response & RSP0_COUNTER_MASK) == response_stored)
        {
            break;
        }
        else
        {
            if (ret != SI1133_OK)
            {
                return ret;
            }
            else
            {
                response_stored = response & RSP0_COUNTER_MASK;
            }
        }

        count++;
    }

    /* Send the command */
    ret = si1133_write_register(sensor, REG_COMMAND, command);
    if (ret != SI1133_OK)
    {
        return ret;
    }

    count = 0;
    /* Wait for a change in the response register (if not a reset command) */
    while (count < 5)
    {
        if (command == CMD_RESET_CMD_CTR)
        {
            break;
        }

        ret = si1133_read_register(sensor, REG_RESPONSE0, &response);
        if ((response & RSP0_COUNTER_MASK) != response_stored)
        {
            break;
        }
        else
        {
            if (ret != SI1133_OK)
            {
                return ret;
            }
        }

        count++;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Sends a RESET COMMAND COUNTER command to the Si1133
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t reset_cmd_counter(void)
{
    return send_cmd(CMD_RESET_CMD_CTR);
}

/***************************************************************************/ /**
* @brief
*    Sends a FORCE command to the Si1133
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t force_measurement(void)
{
    return send_cmd(CMD_FORCE_CH);
}

/***************************************************************************/ /**
* @brief
*    Sends a START command to the Si1133
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t start_measurement(void)
{
    return send_cmd(CMD_START);
}

/***************************************************************************/ /**
* @brief
*    Sends a PAUSE command to the Si1133
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t pause_measurement(void)
{
    return send_cmd(CMD_PAUSE_CH);
}


/**
 * @brief Writes a byte to an Si1133 parameter.
 *
 * This function ensures the sensor is idle by waiting until it is asleep, reads
 * the current response register, sends the parameter write via a block write,
 * and then waits (up to 5 attempts) to see that the response register has changed.
 *
 * @param[in] sensor   Pointer to the sensor state (si1133_t structure).
 * @param[in] address  The parameter address (of enum si1133_parameter type).
 * @param[in] value    The byte value to set.
 *
 * @return 0 on success (SI1133_OK), or a nonzero error code otherwise.
 */
int si1133_set_parameter(si1133_t *sensor, enum Parameter address, uint8_t value)
{
    int ret;
    uint8_t buffer[2];
    uint8_t response_stored;
    uint8_t response;
    int count;

    /* Ensure the sensor is idle (sleep state) before sending a parameter */
    ret = wait_until_sleep();
    if (ret != SI1133_OK)
    {
        return ret;
    }

    /* Read the current response register and mask the counter bits */
    ret = si1133_read_register(sensor, REG_RESPONSE0, &response_stored);
    if (ret != SI1133_OK)
    {
        return ret;
    }
    response_stored &= RSP0_COUNTER_MASK;

    /* Prepare the command block: first byte is the desired value and second
     * is the command to write the parameter. The parameter command is built as:
     *  0x80 + (address & 0x3F)
     */
    buffer[0] = value;
    buffer[1] = 0x80 + (((uint8_t)address) & 0x3F);

    ret = si1133_write_register_block(sensor, REG_HOSTIN0, 2, buffer);
    if (ret != SI1133_OK)
    {
        return ret;
    }

    /* Now wait for the command to complete: poll the response register and
     * expect its counter bits to change from the previously stored value.
     */
    count = 0;
    while (count < 5)
    {
        ret = si1133_read_register(sensor, REG_RESPONSE0, &response);
        if (ret != SI1133_OK)
        {
            return ret;
        }
        if ((response & RSP0_COUNTER_MASK) != response_stored)
        {
            break;
        }
        count++;
    }

    if (count >= 5)
    {
        return SI1133_ERROR_I2C_TRANSACTION_FAILED;
    }

    return SI1133_OK;
}

/***************************************************************************/ /**
* @brief
*    Reads a parameter from the Si1133
*
* @param[in] address
*    The address of the parameter.
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t read_parameter(enum Parameter address)
{
    uint8_t retval;
    uint8_t cmd;
    uint8_t response;

    cmd = 0x40 + ((uint8_t)address & 0x3F);

    retval = send_cmd((enum Command)cmd);
    if (retval != SI1133_OK)
    {
        return retval;
    }

    si1133_read_register(REG_RESPONSE1, &response, &retval);

    return retval;
}

/**
 * @brief Performs the complete initialisation of the Si1133 sensor.
 *
 * This function first initialises the sensor object and configures the I2C bus,
 * then performs the chip-specific reset and parameter configuration.
 *
 * @param sensor  Pointer to the sensor state structure.
 * @param i2c_dev Pointer to the I2C bus device.
 * @param hz      I2C frequency (e.g. 400000 for 400kHz).
 *
 * @return SI1133_OK on success, or an error code otherwise.
 */
int si1133_setup(si1133_t *sensor, const struct device *i2c_dev, int hz)
{
    int ret = 0;

    /* 1. Initialise the sensor structure and configure the I2C bus */
    ret = si1133_init(sensor, i2c_dev, hz);
    if (ret != 0)
    {
        LOG_ERR("si1133_setup: driver-level initialisation failed: %d\n", ret);
        return ret;
    }

    /* 2. Allow time for the sensor to power up */
    k_sleep(K_MSEC(5));

    /* 3. Reset the sensor */
    ret = si1133_reset(sensor);
    if (ret != SI1133_OK)
    {
        LOG_ERR("si1133_setup: reset failed: %d\n", ret);
        return ret;
    }

    /* 4. Wait for a short period after reset */
    k_sleep(K_MSEC(50));

    /* 5. Set up sensor parameters.
     *   Each call here returns SI1133_OK on success or an error code.
     *   We check every return code and return immediately on failure.
     */
    ret = si1133_set_parameter(sensor, PARAM_CH_LIST, 0x0f);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCCONFIG0, 0x78);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCSENS0, 0x71);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCPOST0, 0x40);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCCONFIG1, 0x4d);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCSENS1, 0xe1);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCPOST1, 0x40);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCCONFIG2, 0x41);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCSENS2, 0xe1);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCPOST2, 0x50);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCCONFIG3, 0x4d);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCSENS3, 0x87);
    if (ret != SI1133_OK)
        return ret;
    ret = si1133_set_parameter(sensor, PARAM_ADCPOST3, 0x40);
    if (ret != SI1133_OK)
        return ret;

    /* 6. Finally, write to the IRQ enable register */
    ret = si1133_write_register(sensor, REG_IRQ_ENABLE, 0x0f);
    if (ret != SI1133_OK)
    {
        LOG_ERR("si1133_setup: write_register(REG_IRQ_ENABLE) failed: %d\n", ret);
        return ret;
    }

    return SI1133_OK;
}

/**
 * @brief Creates a new Si1133 sensor object.
 *
 * This function acts as a "constructor" that initialises a si1133_t
 * structure with the provided I2C device and bus frequency.
 *
 * @param i2c_bus Pointer to the I2C bus device.
 * @param hz      I2C bus frequency (Hz).
 *
 * @return A si1133_t structure with the associated I2C device and frequency.
 */
si1133_t si1133_create(const struct device *i2c_bus, int hz)
{
    si1133_t sensor;

    sensor.i2c_dev = i2c_bus;
    sensor.i2c_hz = hz;

    return sensor;
}

/***************************************************************************/ /**
* @brief
*    Read samples from the Si1133 chip
*
* @param[out] samples
*    Retrieves interrupt status and measurement data for channel 0..3 and
*    converts the data to int32_t format
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
int si1133_measure(si1133_t *sensor, Samples_t *samples)
{
    uint8_t buffer[13];
    int ret;

    /* Read 13 bytes starting at REG_IRQ_STATUS */
    ret = si1133_read_register_block(sensor, REG_IRQ_STATUS, 13, buffer);
    if (ret != SI1133_OK)
    {
        return ret;
    }

    /* The first byte is the IRQ status */
    samples->irq_status = buffer[0];

    /* Build 24-bit samples for channels 0..3 from the retrieved bytes.
       Then perform sign extension for negative values. */

    samples->ch0 = ((uint32_t)buffer[1] << 16) |
                   ((uint32_t)buffer[2] << 8) |
                   ((uint32_t)buffer[3]);
    if (samples->ch0 & 0x800000U)
    {
        samples->ch0 |= 0xFF000000U;
    }

    samples->ch1 = ((uint32_t)buffer[4] << 16) |
                   ((uint32_t)buffer[5] << 8) |
                   ((uint32_t)buffer[6]);
    if (samples->ch1 & 0x800000U)
    {
        samples->ch1 |= 0xFF000000U;
    }

    samples->ch2 = ((uint32_t)buffer[7] << 16) |
                   ((uint32_t)buffer[8] << 8) |
                   ((uint32_t)buffer[9]);
    if (samples->ch2 & 0x800000U)
    {
        samples->ch2 |= 0xFF000000U;
    }

    samples->ch3 = ((uint32_t)buffer[10] << 16) |
                   ((uint32_t)buffer[11] << 8) |
                   ((uint32_t)buffer[12]);
    if (samples->ch3 & 0x800000U)
    {
        samples->ch3 |= 0xFF000000U;
    }

    return SI1133_OK;
}

int32_t calculate_polynomial_helper(int32_t input, int8_t fraction, uint16_t mag, int8_t shift)
{
    int32_t value;

    if (shift < 0)
    {
        value = ((input << fraction) / mag) >> -shift;
    }
    else
    {
        value = ((input << fraction) / mag) << shift;
    }

    return value;
}

int32_t calculate_polynomial(int32_t x, int32_t y, uint8_t input_fraction, uint8_t output_fraction, uint8_t num_coeff, const Coeff_t *kp)
{
    uint8_t info, x_order, y_order, counter;
    int8_t sign, shift;
    uint16_t mag;
    int32_t output = 0, x1, x2, y1, y2;

    for (counter = 0; counter < num_coeff; counter++)
    {
        info = kp->info;
        x_order = GET_X_ORDER(info);
        y_order = GET_Y_ORDER(info);

        shift = ((uint16_t)kp->info & 0xff00) >> 8;
        shift ^= 0x00ff;
        shift += 1;
        shift = -shift;

        mag = kp->mag;

        if (GET_SIGN(info))
        {
            sign = -1;
        }
        else
        {
            sign = 1;
        }

        if ((x_order == 0) && (y_order == 0))
        {
            output += sign * mag << output_fraction;
        }
        else
        {
            if (x_order > 0)
            {
                x1 = calculate_polynomial_helper(x, input_fraction, mag, shift);
                if (x_order > 1)
                {
                    x2 = calculate_polynomial_helper(x, input_fraction, mag, shift);
                }
                else
                {
                    x2 = 1;
                }
            }
            else
            {
                x1 = 1;
                x2 = 1;
            }

            if (y_order > 0)
            {
                y1 = calculate_polynomial_helper(y, input_fraction, mag, shift);
                if (y_order > 1)
                {
                    y2 = calculate_polynomial_helper(y, input_fraction, mag, shift);
                }
                else
                {
                    y2 = 1;
                }
            }
            else
            {
                y1 = 1;
                y2 = 1;
            }

            output += sign * x1 * x2 * y1 * y2;
        }

        kp++;
    }

    if (output < 0)
    {
        output = -output;
    }

    return output;
}

/***************************************************************************/ /**
* @brief
*    Compute UV index
*
* @param[in] uv
*    UV sensor raw data
*
* @param[in] uk
*    UV calculation coefficients
*
* @return
*    UV index scaled by UV_OUPTUT_FRACTION
******************************************************************************/
int32_t get_uv(int32_t uv)
{
    int32_t uvi;

    uvi = calculate_polynomial(0, uv, UV_INPUT_FRACTION, UV_OUTPUT_FRACTION, UV_NUMCOEFF, uk);

    return uvi;
}

/***************************************************************************/ /**
* @brief
*    Compute lux value
*
* @param[in] vis_high
*    Visible light sensor raw data
*
* @param[in] vis_low
*    Visible light sensor raw data
*
* @param[in] ir
*    Infrared sensor raw data
*
* @param[in] lk
*    Lux calculation coefficients
*
* @return
*    Lux value scaled by LUX_OUPTUT_FRACTION
******************************************************************************/
int32_t get_lux(int32_t vis_high, int32_t vis_low, int32_t ir)
{
    int32_t lux;

    if ((vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD))
    {
        lux = calculate_polynomial(vis_high,
                                    ir,
                                    INPUT_FRACTION_HIGH,
                                    LUX_OUTPUT_FRACTION,
                                    NUMCOEFF_HIGH,
                                    &(lk.coeff_high[0]));
    }
    else
    {
        lux = calculate_polynomial(vis_low,
                                    ir,
                                    INPUT_FRACTION_LOW,
                                    LUX_OUTPUT_FRACTION,
                                    NUMCOEFF_LOW,
                                    &(lk.coeff_low[0]));
    }

    return lux;
}

/***************************************************************************/ /**
* @brief
*    Measure lux and UV index using the Si1133 sensor
*
* @param[out] lux
*    The measured ambient light illuminace in lux
*
* @param[out] uvi
*    UV index
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t measure_lux_uv(float *lux, float *uvi)
{
    Samples_t samples;
    uint32_t retval;
    uint8_t response;

    /* Force measurement */
    retval = force_measurement();

    /* Go to sleep while the sensor does the conversion */
    wait_ms(200);

    uint32_t temp;

    /* Check if the measurement finished, if not then wait */
    int ret = si1133_read_register(REG_IRQ_STATUS, &response, &temp);
    retval += temp;
    while (response != 0x0F)
    {
        wait_ms(5);

        temp = si1133_read_register(REG_IRQ_STATUS, &response, &temp);
        retval += temp;
    }

    /* Get the results */
    measure(&samples);

    /* Convert the readings to lux */
    *lux = (float)get_lux(samples.ch1, samples.ch3, samples.ch2);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the readings to UV index */
    *uvi = (float)get_uv(samples.ch0);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return retval;
}

/***************************************************************************/ /**
* @brief
*    Reads Hardware ID from the SI1133 sensor
*
* @param[out] hardwareID
*    The Hardware ID of the chip (should be 0x33)
*
* @return
*    Returns zero on OK, non-zero otherwise
******************************************************************************/
uint32_t get_hardware_id(uint8_t *hardware_id)
{
    uint32_t retval;

    int ret = si1133_read_register(REG_PART_ID, hardware_id, &retval);

    return retval;
}

/**
 * @brief Retrieve the measurement from the Si1133 sensor and convert the results
 *        to lux and UV index values.
 *
 * @param[in]  sensor Pointer to the sensor state structure.
 * @param[out] lux    Pointer to a float where the ambient light (in lux) is stored.
 * @param[out] uvi    Pointer to a float where the UV index is stored.
 *
 * @return SI1133_OK (0) on success, or a nonzero error code otherwise.
 */
int si1133_get_measurement(si1133_t *sensor, float *lux, float *uvi)
{
    Samples_t samples;
    int ret;

    /* Retrieve the raw samples from the sensor */
    ret = si1133_measure(sensor, &samples);
    if (ret != SI1133_OK)
    {
        return ret;
    }

    /* Convert the raw readings to lux.
       get_lux() takes channels ch1, ch3 and ch2 as inputs.
       Then we scale the result using the defined fraction.
    */
    *lux = (float)get_lux(samples.ch1, samples.ch3, samples.ch2);
    *lux = *lux / (1 << LUX_OUTPUT_FRACTION);

    /* Convert the UV raw reading to a UV index.
       get_uv() takes ch0 as input.
       Then we scale it with the UV output fraction.
    */
    *uvi = (float)get_uv(samples.ch0);
    *uvi = *uvi / (1 << UV_OUTPUT_FRACTION);

    return SI1133_OK;
}
