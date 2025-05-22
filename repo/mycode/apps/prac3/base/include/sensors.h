#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdio.h>

static const struct device *check_bme280_device(void);

#define BUFFER_SIZE 128

typedef struct
{
    uint8_t lux;
    uint8_t uv;
} light_sensor_output;

// int all_sensors_continuous(void);

/*
 * Shell command script to retrieve sensor values
 */
int cmd_sensors(const struct shell *shell, size_t argc, char *argv[]);
int cmd_sensors_sample(const struct shell *shell, size_t argc, char *argv[]);
void sensor_switch_sampling_mode(void);

void sensor_display_task(void);

struct sensor_q31_data temperature_sensor(bool silent);
struct sensor_q31_data pressure_sensor(bool silent);
struct sensor_q31_data humidity_sensor(bool silent);
light_sensor_output lightUV_sensor(bool silent);

#endif