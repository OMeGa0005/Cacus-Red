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
#include <zephyr/data/json.h>

#include "Si1133.h" // That took me WAYYYYYYYYYYY too long
#include "rtc.h"
#include <sensors.h>

#define RING_BUFFER_SIZE 512
RING_BUF_DECLARE(ringTemp, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringPress, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringHum, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringLight, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringUV, RING_BUFFER_SIZE);

K_MUTEX_DEFINE(ring_mutex);

struct sensor_json_struct
{
    int DID;
    char *time;
    char *value;
};

struct sensor_json_struct_light
{
    int DID;
    char *time;
    int value[2];
    int len;
};

// JSON descriptor
static const struct json_obj_descr sensor_json_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct sensor_json_struct, DID, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json_struct, time, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct sensor_json_struct, value, JSON_TOK_STRING),
};

static const struct json_obj_descr sensor_json_descr_light[] = {
    JSON_OBJ_DESCR_PRIM(struct sensor_json_struct_light, DID, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct sensor_json_struct_light, time, JSON_TOK_STRING),
    JSON_OBJ_DESCR_ARRAY(struct sensor_json_struct_light, value, 2, len, JSON_TOK_NUMBER),
};

/* Definitions for continuous sensor thread configuration */
#define SENSOR_STACK_SIZE 1024
#define SENSOR_THREAD_PRIORITY 5
#define MAX_SENSOR_THREADS 4

/* Global sampling rate in milliseconds. Default = 1000 ms */
static volatile int global_sampling_rate_ms = 1000;

static volatile bool global_sampling_enabled = true;

/* Thread identifiers*/
static k_tid_t sensor_tid_by_type[10] = {0};

/* Allocate thread control blocks and stacks for up to MAX_SENSOR_THREADS concurrent sensor threads */
static struct k_thread sensor_threads[MAX_SENSOR_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(sensor_stacks, MAX_SENSOR_THREADS, SENSOR_STACK_SIZE);

/* A counter to pick the next available thread slot (cyclic) */
static atomic_t sensor_thread_slot = ATOMIC_INIT(0);

/* A parameter structure passed to each continuous sensor thread */
struct continuous_sensor_params
{
    int sensor_type; /* Valid values: 0 = temperature, 1 = humidity, 2 = pressure, 5 = light/UV */
    int rate;        /* Delay in milliseconds between sensor reads */
};

const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *const light_dev = DEVICE_DT_GET_ANY(siliconlabs_si11330);

#define LIGHT_DATA_REG 0x00
#define LIGHT_DATA_LEN 8

// SENSOR_DT_READ_IODEV(iodev_light,
//                      DT_COMPAT_GET_ANY_STATUS_OKAY(siliconlabs_si11330),
//                      {SENSOR_CHAN_LIGHT, 0});

SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bme280),
                     {SENSOR_CHAN_AMBIENT_TEMP, 0},
                     {SENSOR_CHAN_HUMIDITY, 0},
                     {SENSOR_CHAN_PRESS, 0});   

RTIO_DEFINE(ctx, 1, 1);

// We will register logging to the "sensors" module
LOG_MODULE_REGISTER(sensors, LOG_LEVEL_DBG);

// I2C Below ========================================================================

static char last_byte;

/*
 * @brief Callback which is called when a write request is received from the master.
 * @param config Pointer to the target configuration.
 */
int sample_target_write_requested_cb(struct i2c_target_config *config)
{
    printk("sample target write requested\n");
    return 0;
}

/*
 * @brief Callback which is called when a write is received from the master.
 * @param config Pointer to the target configuration.
 * @param val The byte received from the master.
 */
int sample_target_write_received_cb(struct i2c_target_config *config, uint8_t val)
{
    printk("sample target write received: 0x%02x\n", val);
    last_byte = val;
    return 0;
}

/*
 * @brief Callback which is called when a read request is received from the master.
 * @param config Pointer to the target configuration.
 * @param val Pointer to the byte to be sent to the master.
 */
int sample_target_read_requested_cb(struct i2c_target_config *config, uint8_t *val)
{
    printk("sample target read request: 0x%02x\n", *val);
    *val = 0x42;
    return 0;
}

/*
 * @brief Callback which is called when a read is processed from the master.
 * @param config Pointer to the target configuration.
 * @param val Pointer to the next byte to be sent to the master.
 */
int sample_target_read_processed_cb(struct i2c_target_config *config, uint8_t *val)
{
    printk("sample target read processed: 0x%02x\n", *val);
    *val = 0x43;
    return 0;
}

/*
 * @brief Callback which is called when the master sends a stop condition.
 * @param config Pointer to the target configuration.
 */
int sample_target_stop_cb(struct i2c_target_config *config)
{
    printk("sample target stop callback\n");
    return 0;
}

static struct i2c_target_callbacks sample_target_callbacks = {
    .write_requested = sample_target_write_requested_cb,
    .write_received = sample_target_write_received_cb,
    .read_requested = sample_target_read_requested_cb,
    .read_processed = sample_target_read_processed_cb,
    .stop = sample_target_stop_cb,
};

// I2C Above ========================================================================

static const struct device *check_bme280_device(void)
{
    // printk("checking device");

    if (dev == NULL)
    {
        /* No such node, or the node does not have status "okay". */
        printk("\nError: no device found.\n");
        return NULL;
    }

    if (!device_is_ready(dev))
    {
        printk("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    // printk("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}

/*
 * @brief Reads temperature data from the BME280 sensor.
 *
 * @param silent If true, returns the data without logging it.
 * @return Sensor data as a sensor_q31_data structure.
 */
struct sensor_q31_data temperature_sensor(bool silent)
{
    struct sensor_q31_data output = {0};
    const struct device *dev = check_bme280_device();

    if (dev == NULL)
    {
        return output;
    }

    uint8_t buf[128];

    // printk("About to read");
    int rc = sensor_read(&iodev, &ctx, buf, 128);
    // printk("Finished read");

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_read() failed: %d\n", dev->name, rc);
        return output;
    }

    const struct sensor_decoder_api *decoder;

    rc = sensor_get_decoder(dev, &decoder);

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_get_decode() failed: %d\n", dev->name, rc);
        return output;
    }

    uint32_t temp_fit = 0;
    struct sensor_q31_data temp_data = {0};

    decoder->decode(buf,
                    (struct sensor_chan_spec){SENSOR_CHAN_AMBIENT_TEMP, 0},
                    &temp_fit, 1, &temp_data);

    if (!silent) {
        if (k_mutex_lock(&ring_mutex, K_MSEC(500)) == 0)
        {
            ring_buf_put(&ringTemp, &temp_data, sizeof(temp_data));
            k_mutex_unlock(&ring_mutex);
        }
        else
        {
            LOG_ERR("Cannot lock ring mutex\n");
        }
    }
    else {
        return temp_data;
    }
}

/*
 * @brief Reads pressure data from the BME280 sensor.
 *
 * @param silent If true, returns the data without logging it.
 * @return Sensor data as a sensor_q31_data structure.
 */
struct sensor_q31_data pressure_sensor(bool silent)
{
    struct sensor_q31_data output = {0};
    const struct device *dev = check_bme280_device();

    if (dev == NULL)
    {
        return output;
    }

    uint8_t buf[128];

    int rc = sensor_read(&iodev, &ctx, buf, 128);

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_read() failed: %d\n", dev->name, rc);
        return output;
    }

    const struct sensor_decoder_api *decoder;

    rc = sensor_get_decoder(dev, &decoder);

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_get_decode() failed: %d\n", dev->name, rc);
        return output;
    }

    uint32_t press_fit = 0;
    struct sensor_q31_data press_data = {0};

    decoder->decode(buf,
                    (struct sensor_chan_spec){SENSOR_CHAN_PRESS, 0},
                    &press_fit, 1, &press_data);

    if (!silent) {
        if (k_mutex_lock(&ring_mutex, K_MSEC(500)) == 0)
        {
            ring_buf_put(&ringPress, &press_data, sizeof(press_data));
            k_mutex_unlock(&ring_mutex);
        }
        else
        {
            LOG_ERR("Cannot lock ring mutex\n");
        }
    }
    else {
        return press_data;
    }
}

/*
 * @brief Reads humidity data from the BME280 sensor.
 *
 * @param silent If true, returns the data without logging it.
 * @return Sensor data as a sensor_q31_data structure.
 */
struct sensor_q31_data humidity_sensor(bool silent)
{
    struct sensor_q31_data output = {0};
    const struct device *dev = check_bme280_device();

    if (dev == NULL)
    {
        return output;
    }

    uint8_t buf[128];

    int rc = sensor_read(&iodev, &ctx, buf, 128);

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_read() failed: %d\n", dev->name, rc);
        return output;
    }

    const struct sensor_decoder_api *decoder;

    rc = sensor_get_decoder(dev, &decoder);

    if (rc != 0)
    {
        LOG_ERR("%s: sensor_get_decode() failed: %d\n", dev->name, rc);
        return output;
    }

    uint32_t hum_fit = 0;
    struct sensor_q31_data hum_data = {0};

    decoder->decode(buf,
                    (struct sensor_chan_spec){SENSOR_CHAN_HUMIDITY, 0},
                    &hum_fit, 1, &hum_data);

    if (!silent) {
        if (k_mutex_lock(&ring_mutex, K_MSEC(500)) == 0)
        {
            ring_buf_put(&ringHum, &hum_data, sizeof(hum_data));
            k_mutex_unlock(&ring_mutex);
        }
        else
        {
            LOG_ERR("Cannot lock ring mutex\n");
        }
    }
    else {
        return hum_data;
    }
}

/*
 * @brief Reads light and UV data from the Si1133 sensor.
 *
 * @param silent If true, returns the data without logging it.
 * @return Sensor data as a light_sensor_output structure.
 */
light_sensor_output lightUV_sensor(bool silent)
{
    light_sensor_output output = {};
    const struct device *dev = check_bme280_device();

    if (dev == NULL)
    {
        output.lux = 0;
        output.uv = 0;
        return output;
    }

    int ret;
    const struct device *i2c_dev;

    /* Get the I2C device from the devicetree.
     */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev))
    {
        LOG_WRN("I2C device not ready!\n");
        output.lux = 0;
        output.uv = 0;
        return output;
    }

    si1133_t sensor = si1133_create(i2c_dev, 400000);

    /* si1133_open() returns a boolean (true on success).
     * If the sensor is successfully opened (detected and configured), we proceed.
     */
    if (si1133_open(&sensor))
    {
        float lux, uvi = 0;
        ret = si1133_get_measurement(&sensor, &lux, &uvi);
        if (ret != SI1133_OK)
        {
            LOG_ERR("Measurement error: %d\n", ret);
        }
        else
        {
            uint8_t luxint = (uint8_t)lux;
            uint8_t uviint = (uint8_t)uvi;

            if (!silent) {
                if (k_mutex_lock(&ring_mutex, K_MSEC(500)) == 0)
                {
                    ring_buf_put(&ringLight, &luxint, sizeof(luxint));
                    ring_buf_put(&ringTemp, &uviint, sizeof(uviint));
                    k_mutex_unlock(&ring_mutex);
                }
                else
                {
                    LOG_ERR("Cannot lock ring mutex\n");
                }
            }
            else {
                output.lux = luxint;
                output.uv = uviint;
                return output;
            }

            // LOG_INF("Lux = %.3f\n", lux);
            // LOG_INF("UV index = %.3f\n", uvi);
        }
    }
    else
    {
        LOG_ERR("Device not detected!\n");
    }
}

/*
 * @brief Shell command to retrieve one-shot sensor data.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success, -EINVAL on invalid arguments.
 */
int cmd_sensors(const struct shell *shell, size_t argc, char *argv[])
{
    if (argc == 2)
    {
        // Possible options: 0 (temp), 1 (humidity), 2 (pressure), 5 (light/UV), 15 (All)

        // Get the temp
        if (strcmp(argv[1], "0") == 0)
        {
            temperature_sensor(false);
        }

        // Get the hum
        else if (strcmp(argv[1], "1") == 0)
        {
            humidity_sensor(false);
        }

        // Get the pressure
        else if (strcmp(argv[1], "2") == 0)
        {
            pressure_sensor(false);
        }

        // Get the light
        else if (strcmp(argv[1], "5") == 0)
        {
            lightUV_sensor(false);
        }

        // Get the all
        else if (strcmp(argv[1], "15") == 0)
        {
            temperature_sensor(false);
            humidity_sensor(false);
            pressure_sensor(false);
            lightUV_sensor(false);
        }

        else
        {
            LOG_ERR("Invalid command");
            return -EINVAL;
        }
    }
    else
    {
        LOG_ERR("Invalid command");
        return -EINVAL;
    }
}

/*
 * @brief Toggles the global sampling mode (enabled/disabled).
 */
void sensor_switch_sampling_mode(void) {
    global_sampling_enabled = !global_sampling_enabled;
}

/*
 * @brief Thread function for continuous sensor sampling.
 *
 * @param p1 Pointer to a struct continuous_sensor_params.
 * @param p2 Not used.
 * @param p3 Not used.
 */
static void sensor_continuous_thread(void *p1, void *p2, void *p3)
{
    struct continuous_sensor_params *params = p1;
    /* Switch on sensor type and call the corresponding continuous function.
     */
    int ret = 0;
    char json_output[BUFFER_SIZE];
    char tmp_str[32];

    while (1)
    {
        if (!global_sampling_enabled) {
            k_msleep(100);
            continue;
        }

        switch (params->sensor_type)
        {
            case 0:
                struct sensor_q31_data temp = temperature_sensor(true);

                snprintf(tmp_str, sizeof(tmp_str), "%s%d.%d", PRIq_arg(temp.readings[0].temperature, 3, temp.shift));

                // sets up json object
                struct sensor_json_struct temp_data = {
                    .DID = params->sensor_type,       // Device ID
                    .time = rtc_get_time_formatted(), // Timestamp
                    .value = tmp_str
                };
                ret = json_obj_encode_buf(sensor_json_descr, ARRAY_SIZE(sensor_json_descr), &temp_data, json_output, sizeof(json_output));
                printf("%s\n", json_output);
                break;
            case 1:
                struct sensor_q31_data hum = humidity_sensor(true);

                snprintf(tmp_str, sizeof(tmp_str), "%s%d.%d", PRIq_arg(hum.readings[0].humidity / 10000, 3, hum.shift));

                // sets up json object
                struct sensor_json_struct hum_data = {
                    .DID = params->sensor_type,       // Device ID
                    .time = rtc_get_time_formatted(), // Timestamp
                    .value = tmp_str
                };
                ret = json_obj_encode_buf(sensor_json_descr, ARRAY_SIZE(sensor_json_descr), &hum_data, json_output, sizeof(json_output));
                printf("%s\n", json_output);
                break;
            case 2:
                struct sensor_q31_data press = pressure_sensor(true);

                snprintf(tmp_str, sizeof(tmp_str), "%s%d.%d", PRIq_arg(press.readings[0].pressure * 10, 3, press.shift));

                // sets up json object
                struct sensor_json_struct press_data = {
                    .DID = params->sensor_type,       // Device ID
                    .time = rtc_get_time_formatted(), // Timestamp
                    .value = tmp_str
                };
                ret = json_obj_encode_buf(sensor_json_descr, ARRAY_SIZE(sensor_json_descr), &press_data, json_output, sizeof(json_output));
                printf("%s\n", json_output);
                break;
            case 5:
                light_sensor_output light_val = lightUV_sensor(true);

                // sets up json object
                struct sensor_json_struct_light light_data = {
                    .DID = params->sensor_type,       // Device ID
                    .time = rtc_get_time_formatted(), // Timestamp
                    .value[0] = light_val.lux,
                    .value[0] = light_val.uv,
                    .len = 2};
                ret = json_obj_encode_buf(sensor_json_descr_light, ARRAY_SIZE(sensor_json_descr_light), &light_data, json_output, sizeof(json_output));
                printf("%s\n", json_output);
                break;
            default:
                LOG_ERR("Invalid sensor type %d", params->sensor_type);
                break;
        }

        k_msleep(global_sampling_rate_ms);
    }

    /* Free the parameters once the thread function exits.
     * In continuous mode these threads are not expected to exit.
     */
    k_free(params);
    /* Optionally abort the thread if it ever returns. */
    k_thread_abort(k_current_get());
}

/*
 * @brief Shell command to start or stop continuous sensor sampling.
 *
 * @param shell Pointer to the shell instance.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success, -EINVAL on invalid arguments.
 */
int cmd_sensors_sample(const struct shell *shell, size_t argc, char *argv[])
{
    int sensor_type;
    struct continuous_sensor_params *params;
    int slot;

    if (argc != 3)
    {
        LOG_ERR("Invalid command. Usage:\n\r"
                "  sample s <DID>   # start continuous sampling for sensor <DID>\n\r"
                "  sample p <DID>   # stop continuous sampling for sensor <DID>\n\r"
                "  sample w <rate>  # set sampling time (in seconds)");
        return -EINVAL;
    }

    /* If the first parameter is "w", set sampling rate */
    if (strcmp(argv[1], "w") == 0)
    {
        /* Validate that argv[2] is a number */
        for (int i = 0, n = strlen(argv[2]); i < n; i++)
        {
            if (!isdigit((int)argv[2][i]))
            {
                LOG_ERR("Invalid rate, must be an integer");
                return -EINVAL;
            }
        }
        int rate_seconds = atoi(argv[2]);
        if (rate_seconds <= 0)
        {
            LOG_ERR("Rate must be positive");
            return -EINVAL;
        }
        global_sampling_rate_ms = rate_seconds * 1000;
        LOG_INF("Global sampling time set to %d seconds (%d ms)", rate_seconds, global_sampling_rate_ms);
        return 0;
    }

    /* Otherwise, we expect either "s" (start) or "p" (stop) as the first argument */
    if (strcmp(argv[1], "s") == 0)
    {
        /* Start continuous sampling for the sensor indicated by argv[2] */
        if (strcmp(argv[2], "0") == 0)
        {
            sensor_type = 0;
        }
        else if (strcmp(argv[2], "1") == 0)
        {
            sensor_type = 1;
        }
        else if (strcmp(argv[2], "2") == 0)
        {
            sensor_type = 2;
        }
        else if (strcmp(argv[2], "5") == 0)
        {
            sensor_type = 5;
        }
        else if (strcmp(argv[2], "15") == 0)
        {
            /* Start threads for all sensors */
            for (int type = 0; type <= 5; type++)
            {
                if (type == 3 || type == 4) // Skip invalid sensor types
                    continue;

                /* Check if continuous sampling for this sensor is already running */
                if (sensor_tid_by_type[type] != NULL)
                {
                    LOG_ERR("Sensor type %d is already sampling continuously", type);
                    continue;
                }

                /* Allocate memory for thread parameters */
                params = k_malloc(sizeof(*params));
                if (!params)
                {
                    LOG_ERR("Memory allocation failed for sensor type %d", type);
                    continue;
                }

                params->sensor_type = type;
                params->rate = global_sampling_rate_ms;

                /* Select a thread slot cyclically */
                slot = atomic_inc(&sensor_thread_slot) % MAX_SENSOR_THREADS;
                k_tid_t tid = k_thread_create(&sensor_threads[slot],
                                              sensor_stacks[slot],
                                              SENSOR_STACK_SIZE,
                                              sensor_continuous_thread,
                                              params, NULL, NULL,
                                              SENSOR_THREAD_PRIORITY,
                                              0,
                                              K_NO_WAIT);

                if (tid == NULL)
                {
                    LOG_ERR("Thread creation failed for sensor type %d", type);
                    k_free(params);
                    continue;
                }

                sensor_tid_by_type[type] = tid;
                // LOG_INF("Started continuous sampling for sensor type %d in slot %d", type, slot);
            }
            return 0;
        }
        else
        {
            LOG_ERR("Invalid sensor type. Options: 0 (temp), 1 (hum), 2 (pressure), 5 (light/UV), 15 (all)");
            return -EINVAL;
        }

        /* Check if continuous sampling for this sensor is already running */
        if (sensor_tid_by_type[sensor_type] != NULL)
        {
            LOG_ERR("Sensor type %d is already sampling continuously", sensor_type);
            return -EINVAL;
        }

        /* Allocate memory for thread parameters */
        params = k_malloc(sizeof(*params));
        if (!params)
        {
            LOG_ERR("Memory allocation failed");
            return -ENOMEM;
        }

        params->sensor_type = sensor_type;
        params->rate = global_sampling_rate_ms; // Just in case we need it

        /* Select a thread slot cyclically */
        slot = atomic_inc(&sensor_thread_slot) % MAX_SENSOR_THREADS;
        k_tid_t tid = k_thread_create(&sensor_threads[slot],
                                      sensor_stacks[slot],
                                      SENSOR_STACK_SIZE,
                                      sensor_continuous_thread,
                                      params, NULL, NULL,
                                      SENSOR_THREAD_PRIORITY,
                                      0,
                                      K_NO_WAIT);

        if (tid == NULL)
        {
            LOG_ERR("Thread creation failed");
            k_free(params);
            return -ENOMEM;
        }
        sensor_tid_by_type[sensor_type] = tid;
        // LOG_INF("Started continuous sampling for sensor type %d in slot %d", sensor_type, slot);
        return 0;
    }
    else if (strcmp(argv[1], "p") == 0)
    {
        /* Stop continuous sampling for the specified sensor */
        if (strcmp(argv[2], "0") == 0)
        {
            sensor_type = 0;
        }
        else if (strcmp(argv[2], "1") == 0)
        {
            sensor_type = 1;
        }
        else if (strcmp(argv[2], "2") == 0)
        {
            sensor_type = 2;
        }
        else if (strcmp(argv[2], "5") == 0)
        {
            sensor_type = 5;
        }
        else if (strcmp(argv[2], "15") == 0)
        {
            /* Stop threads for all sensors */
            for (int type = 0; type <= 5; type++)
            {
                if (type == 3 || type == 4) // Skip invalid sensor types
                    continue;

                if (sensor_tid_by_type[type] != NULL)
                {
                    k_thread_abort(sensor_tid_by_type[type]);
                    sensor_tid_by_type[type] = NULL;
                    // LOG_INF("Stopped continuous sampling for sensor type %d", type);
                }
            }
            return 0;
        }
        else
        {
            LOG_ERR("Invalid sensor type. Options: 0 (temp), 1 (hum), 2 (pressure), 5 (light/UV), 15 (all)");
            return -EINVAL;
        }

        if (sensor_tid_by_type[sensor_type] == NULL)
        {
            LOG_ERR("No continuous sampling running for sensor type %d", sensor_type);
            return -EINVAL;
        }
        k_thread_abort(sensor_tid_by_type[sensor_type]);
        sensor_tid_by_type[sensor_type] = NULL;
        // LOG_INF("Stopped continuous sampling for sensor type %d", sensor_type);
        return 0;
    }
    else
    {
        LOG_ERR("Invalid command option. Use 's' to start, 'p' to stop, or 'w' to set sampling rate");
        return -EINVAL;
    }
}

/*
 * @brief Task to display sensor data from ring buffers.
 *        Continuously reads data and logs it.
 */
void sensor_display_task(void) {
    uint8_t data[sizeof(uint8_t)];
    uint8_t q31data[sizeof(struct sensor_q31_data)]; // Space for one sensor_q31_data

    uint8_t light, uvi;
    struct sensor_q31_data temp, press, hum;

    while (1)
    {
        if (k_mutex_lock(&ring_mutex, K_MSEC(500)) == 0)
        {
            // Clear the variables before processing (get rid of those annoying bugs)
            light = 0;
            uvi = 0;
            memset(&temp, 0, sizeof(temp));
            memset(&press, 0, sizeof(press));
            memset(&hum, 0, sizeof(hum));

            // Read light sensor data
            if (ring_buf_get(&ringLight, data, sizeof(data)) != 0)
            {

                memcpy(&light, data, sizeof(light)); // Copy to struct

                if (ring_buf_get(&ringUV, data, sizeof(data)) != 0)
                {
                    memcpy(&uvi, data, sizeof(uvi)); // Copy to struct
                }

                // Print the sensor readings
                LOG_INF("Lux: %d, UV: %d\n", light, uvi);

                // Reset the ring buffer to remove any erroneous data
                ring_buf_reset(&ringLight);
                ring_buf_reset(&ringUV);
            }

            // Read temperature
            if (ring_buf_get(&ringTemp, q31data, sizeof(q31data)) >= sizeof(struct sensor_q31_data))
            {
                memcpy(&temp, q31data, sizeof(struct sensor_q31_data));
                LOG_INF("Temperature: %s%d.%d °C\n",
                        PRIq_arg(temp.readings[0].temperature, 3, temp.shift));

                // Reset the ring buffer to remove any erroneous data
                ring_buf_reset(&ringTemp);
            }

            // Read humidity
            if (ring_buf_get(&ringHum, q31data, sizeof(q31data)) >= sizeof(struct sensor_q31_data))
            {
                memcpy(&hum, q31data, sizeof(struct sensor_q31_data));
                LOG_INF("Humidity: %s%d.%d %%\n",
                        PRIq_arg(hum.readings[0].humidity, 3, hum.shift));

                // Reset the ring buffer to remove any erroneous data
                ring_buf_reset(&ringHum);
            }

            // Read pressure
            if (ring_buf_get(&ringPress, q31data, sizeof(q31data)) >= sizeof(struct sensor_q31_data))
            {
                memcpy(&press, q31data, sizeof(struct sensor_q31_data));
                LOG_INF("Pressure: %s%d.%d hPa\n",
                        PRIq_arg(press.readings[0].pressure * 10, 3, press.shift));

                // Reset the ring buffer to remove any erroneous data
                ring_buf_reset(&ringPress);
            }

            k_mutex_unlock(&ring_mutex);

            k_msleep(25);
        }

        else
        {
            LOG_ERR("Cannot lock ring mutex\n");
        }
    }
}