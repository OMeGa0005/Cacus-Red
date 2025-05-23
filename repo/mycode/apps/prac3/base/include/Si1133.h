/***************************************************************************/ /**
* @file Si1133.h
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

// Reference: Silicon Labs documentation [https://os.mbed.com/teams/SiliconLabs/code/Si1133//file/f780ca9105bb/Si1133.h/]
// Took inspiration but heavily modified to run in zephyr

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

#ifndef SI1133_H
#define SI1133_H

/* Define constants for return values */
#define SI1133_OK 0
#define SI1133_ERROR_I2C_TRANSACTION_FAILED -1
#define SI1133_MAX_WRITE_LENGTH 2 /* Update this based on device limitations */

/* Define a structure to hold sensor state */
typedef struct
{
    const struct device *i2c_dev; /* Pointer to the I2C bus device */
    int i2c_hz;                   /* I2C bus frequency (in Hz) */
    /* Add other sensor-specific members here */
} si1133_t;

/* Prototype for your sensor "constructor" function */
si1133_t si1133_create(const struct device *i2c_bus, int hz);

/**
 * @brief Initialise the Si1133 sensor.
 *
 * This function configures the I2C bus for the sensor and saves the
 * sensor configuration in the provided structure.
 *
 * @param sensor Pointer to a si1133_t instance.
 * @param i2c_dev Pointer to the I2C bus device.
 * @param hz The I2C bus frequency in Hz.
 *
 * @return 0 on success, negative error code otherwise.
 */
int si1133_setup(si1133_t *sensor, const struct device *i2c_dev, int hz);

bool si1133_open(si1133_t *sensor);

/** Probe for the Si1133 and try to initialise the sensor
 *
 * @returns
 *   'true' if the device exists on the bus,
 *   'false' if the device doesn't exist on the bus.
 */
bool open();

/** Measure the current light level (in lux) on the Si1133
 *
 * @returns The current temperature measurement in Lux.
 */
float get_light_level();

/** Measure the current UV Index on the Si1133
 *
 * @returns The current UV index measurement.
 */
float get_uv_index();

/** Do a combined measurement and return both the light level and UV index
 *
 * @param[out] light_level Measured light level in Lux
 * @param[out] uv_index Measured UV index
 *
 * @returns true if measurement was successful
 */
bool get_light_and_uv(float *light_level, float *uv_index);

#ifdef MBED_OPERATORS
    /** A shorthand for get_light_level()
     *
     * @returns The current temperature measurement in Lux.
     */
    operator float();
#endif

/**
 * @name I2C Registers
 */
enum Register
{
    REG_PART_ID = 0x00,    /* Part ID                                                            */
    REG_HW_ID = 0x01,      /* Hardware ID                                                        */
    REG_REV_ID = 0x02,     /* Hardware revision                                                  */
    REG_HOSTIN0 = 0x0A,    /* Data for parameter table on PARAM_SET write to COMMAND register    */
    REG_COMMAND = 0x0B,    /* Initiated action in Sensor when specific codes written here        */
    REG_IRQ_ENABLE = 0x0F, /* Interrupt enable                                                   */
    REG_RESPONSE1 = 0x10,  /* Contains the readback value from a query or a set command          */
    REG_RESPONSE0 = 0x11,  /* Chip state and error status                                        */
    REG_IRQ_STATUS = 0x12, /* Interrupt status                                                   */
    REG_HOSTOUT0 = 0x13,   /* Sensor Data                                                        */
    REG_HOSTOUT1 = 0x14,   /* Sensor Data                                                        */
    REG_HOSTOUT2 = 0x15,   /* Sensor Data                                                        */
    REG_HOSTOUT3 = 0x16,   /* Sensor Data                                                        */
    REG_HOSTOUT4 = 0x17,   /* Sensor Data                                                        */
    REG_HOSTOUT5 = 0x18,   /* Sensor Data                                                        */
    REG_HOSTOUT6 = 0x19,   /* Sensor Data                                                        */
    REG_HOSTOUT7 = 0x1A,   /* Sensor Data                                                        */
    REG_HOSTOUT8 = 0x1B,   /* Sensor Data                                                        */
    REG_HOSTOUT9 = 0x1C,   /* Sensor Data                                                        */
    REG_HOSTOUT10 = 0x1D,  /* Sensor Data                                                        */
    REG_HOSTOUT11 = 0x1E,  /* Sensor Data                                                        */
    REG_HOSTOUT12 = 0x1F,  /* Sensor Data                                                        */
    REG_HOSTOUT13 = 0x20,  /* Sensor Data                                                        */
    REG_HOSTOUT14 = 0x21,  /* Sensor Data                                                        */
    REG_HOSTOUT15 = 0x22,  /* Sensor Data                                                        */
    REG_HOSTOUT16 = 0x23,  /* Sensor Data                                                        */
    REG_HOSTOUT17 = 0x24,  /* Sensor Data                                                        */
    REG_HOSTOUT18 = 0x25,  /* Sensor Data                                                        */
    REG_HOSTOUT19 = 0x26,  /* Sensor Data                                                        */
    REG_HOSTOUT20 = 0x27,  /* Sensor Data                                                        */
    REG_HOSTOUT21 = 0x28,  /* Sensor Data                                                        */
    REG_HOSTOUT22 = 0x29,  /* Sensor Data                                                        */
    REG_HOSTOUT23 = 0x2A,  /* Sensor Data                                                        */
    REG_HOSTOUT24 = 0x2B,  /* Sensor Data                                                        */
    REG_HOSTOUT25 = 0x2C,  /* Sensor Data                                                        */
};

/**
 * @name Parameters
 */
enum Parameter
{
    PARAM_I2C_ADDR = 0x00,     /* I2C address                                                        */
    PARAM_CH_LIST = 0x01,      /* Channel list                                                       */
    PARAM_ADCCONFIG0 = 0x02,   /* ADC config for Channel 0                                           */
    PARAM_ADCSENS0 = 0x03,     /* ADC sensitivity setting for Channel 0                              */
    PARAM_ADCPOST0 = 0x04,     /* ADC resolution, shift and threshold settings for Channel 0         */
    PARAM_MEASCONFIG0 = 0x05,  /* ADC measurement counter selection for Channel 0                    */
    PARAM_ADCCONFIG1 = 0x06,   /* ADC config for Channel 1                                           */
    PARAM_ADCSENS1 = 0x07,     /* ADC sensitivity setting for Channel 1                              */
    PARAM_ADCPOST1 = 0x08,     /* ADC resolution, shift and threshold settings for Channel 1         */
    PARAM_MEASCONFIG1 = 0x09,  /* ADC measurement counter selection for Channel 1                    */
    PARAM_ADCCONFIG2 = 0x0A,   /* ADC config for Channel 2                                           */
    PARAM_ADCSENS2 = 0x0B,     /* ADC sensitivity setting for Channel 2                              */
    PARAM_ADCPOST2 = 0x0C,     /* ADC resolution, shift and threshold settings for Channel 2         */
    PARAM_MEASCONFIG2 = 0x0D,  /* ADC measurement counter selection for Channel 2                    */
    PARAM_ADCCONFIG3 = 0x0E,   /* ADC config for Channel 3                                           */
    PARAM_ADCSENS3 = 0x0F,     /* ADC sensitivity setting for Channel 3                              */
    PARAM_ADCPOST3 = 0x10,     /* ADC resolution, shift and threshold settings for Channel 3         */
    PARAM_MEASCONFIG3 = 0x11,  /* ADC measurement counter selection for Channel 3                    */
    PARAM_ADCCONFIG4 = 0x12,   /* ADC config for Channel 4                                           */
    PARAM_ADCSENS4 = 0x13,     /* ADC sensitivity setting for Channel 4                              */
    PARAM_ADCPOST4 = 0x14,     /* ADC resolution, shift and threshold settings for Channel 4         */
    PARAM_MEASCONFIG4 = 0x15,  /* ADC measurement counter selection for Channel 4                    */
    PARAM_ADCCONFIG5 = 0x16,   /* ADC config for Channel 5                                           */
    PARAM_ADCSENS5 = 0x17,     /* ADC sensitivity setting for Channel 5                              */
    PARAM_ADCPOST5 = 0x18,     /* ADC resolution, shift and threshold settings for Channel 5         */
    PARAM_MEASCONFIG5 = 0x19,  /* ADC measurement counter selection for Channel 5                    */
    PARAM_MEASRATE_H = 0x1A,   /* Main measurement rate counter MSB                                  */
    PARAM_MEASRATE_L = 0x1B,   /* Main measurement rate counter LSB                                  */
    PARAM_MEASCOUNT0 = 0x1C,   /* Measurement rate extension counter 0                               */
    PARAM_MEASCOUNT1 = 0x1D,   /* Measurement rate extension counter 1                               */
    PARAM_MEASCOUNT2 = 0x1E,   /* Measurement rate extension counter 2                               */
    PARAM_THRESHOLD0_H = 0x25, /* Threshold level 0 MSB                                              */
    PARAM_THRESHOLD0_L = 0x26, /* Threshold level 0 LSB                                              */
    PARAM_THRESHOLD1_H = 0x27, /* Threshold level 1 MSB                                              */
    PARAM_THRESHOLD1_L = 0x28, /* Threshold level 1 LSB                                              */
    PARAM_THRESHOLD2_H = 0x29, /* Threshold level 2 MSB                                              */
    PARAM_THRESHOLD2_L = 0x2A, /* Threshold level 2 LSB                                              */
    PARAM_BURST = 0x2B,        /* Burst enable and burst count                                       */
};

/**
 * @name Commands
 */
enum Command
{
    CMD_RESET_CMD_CTR = 0x00, /**< Resets the command counter                                         */
    CMD_RESET = 0x01,         /**< Forces a Reset                                                     */
    CMD_NEW_ADDR = 0x02,      /**< Stores the new I2C address                                         */
    CMD_FORCE_CH = 0x11,      /**< Initiates a set of measurements specified in CHAN_LIST parameter   */
    CMD_PAUSE_CH = 0x12,      /**< Pauses autonomous measurements                                     */
    CMD_START = 0x13,         /**< Starts autonomous measurements                                     */
    CMD_PARAM_SET = 0x80,     /**< Sets a parameter                                                   */
    CMD_PARAM_QUERY = 0x40,   /**< Reads a parameter                                                  */
};

/**
 * @name Responses
 */
enum Response
{
    RSP0_CHIPSTAT_MASK = 0xE0, /**< Chip state mask in Response0 register                              */
    RSP0_COUNTER_MASK = 0x1F,  /**< Command counter and error indicator mask in Response0 register     */
    RSP0_SLEEP = 0x20,         /**< Sleep state indicator bit mask in Response0 register               */
};

/**
 * @brief
 *    Structure to store the data measured by the Si1133
 */
typedef struct
{
    uint8_t irq_status; /**< Interrupt status of the device    */
    int32_t ch0;        /**< Channel 0 measurement data        */
    int32_t ch1;        /**< Channel 1 measurement data        */
    int32_t ch2;        /**< Channel 2 measurement data        */
    int32_t ch3;        /**< Channel 3 measurement data        */
} Samples_t;

/**
 * @brief
 *    Structure to store the calculation coefficients
 */
typedef struct
{
    int16_t info; /**< Info                              */
    uint16_t mag; /**< Magnitude                         */
} Coeff_t;

/**
 * @brief
 *    Structure to store the coefficients used for Lux calculation
 */
typedef struct
{
    Coeff_t coeff_high[4]; /**< High amplitude coeffs */
    Coeff_t coeff_low[9];  /**< Low amplitude coeffs  */
} LuxCoeff_t;

/* Private functions */
uint32_t read_register(enum Register reg, uint8_t *data);
uint32_t write_register(enum Register reg, uint8_t data);
uint32_t write_register_block(enum Register reg, uint8_t length, uint8_t *data);
uint32_t read_register_block(enum Register reg, uint8_t length, uint8_t *data);
uint32_t get_irq_status(uint8_t *irq_status);
uint32_t wait_until_sleep(void);
uint32_t reset(void);
uint32_t reset_cmd_counter(void);
uint32_t send_cmd(enum Command command);
uint32_t force_measurement(void);
uint32_t pause_measurement(void);
uint32_t start_measurement(void);
uint32_t set_parameter(enum Parameter address, uint8_t value);
uint32_t read_parameter(enum Parameter address);
uint32_t init(void);
uint32_t deinit(void);
uint32_t measure(Samples_t *samples);
int32_t get_uv(int32_t uv);
int32_t get_lux(int32_t vis_high, int32_t vis_low, int32_t ir);
uint32_t measure_lux_uv(float *lux, float *uvi);
uint32_t get_measurement(float *lux, float *uvi);
uint32_t get_hardware_id(uint8_t *hardware_id);

int32_t calculate_polynomial_helper(int32_t input, int8_t fraction, uint16_t mag, int8_t shift);
int32_t calculate_polynomial(int32_t x, int32_t y, uint8_t input_fraction, uint8_t output_fraction, uint8_t num_coeff, const Coeff_t *kp);

#endif
