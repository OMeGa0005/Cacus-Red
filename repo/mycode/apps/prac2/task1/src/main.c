#include <stdio.h>
#include "task1.c"
#include "task2.c"
#include "task3.c"
#include "task5.c"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>


#define SLEEP_TIME_MS   1000
#define STACK_SIZE 1024
#define RNG_PRIORITY 7
#define DISPLAY_PRIORITY 7
#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4
const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));

//Creates command subset

SHELL_STATIC_SUBCMD_SET_CREATE(
    custom_cmds,
    SHELL_CMD_ARG(rtc, NULL, "set time", rtc_time, 2, 6),
    SHELL_CMD_ARG(sample, NULL, "sample", sample_cmd, 2, 6),
    //SHELL_CMD_ARG(setTime, NULL, "set time", set_date_time, 0, 7),
    //SHELL_CMD_ARG(getTime, NULL, "get time", get_date_time, 0, 0),
    SHELL_CMD_ARG(sensor, NULL, "get sensor", sensor_main, 3, 0),
    SHELL_CMD_ARG(file, NULL, "file stuff", file_handler, 2, 0),
    SHELL_SUBCMD_SET_END
);
//SHELL_CMD_REGISTER(setTime, NULL, "", set_date_time); //determines what function will run based on the command
//SHELL_CMD_REGISTER(getTime, NULL, "", get_date_time);
SHELL_CMD_REGISTER(rtc, NULL, "", rtc_time);
SHELL_CMD_REGISTER(sensor, NULL, "", sensor_main);
SHELL_CMD_REGISTER(sample, NULL, "", sample_cmd);
SHELL_CMD_REGISTER(file, NULL, "", file_handler);

int main(){
    //set_date_time(2025, 3, 28, 3, 39, 0);
    while (get_date_time() == 0) {
		k_sleep(K_MSEC(1000));
	};
    return 0;
}