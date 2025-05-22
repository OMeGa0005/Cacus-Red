#ifndef TASK1_H
#define TASK1_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>


//const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));
static int rtc_time(const struct shell *shell, size_t argc, char *argv[]);
static int set_date_time(const struct shell *shell, size_t argc, char *argv[]);
static int get_date_time();
extern const struct device *const rtc;
static char* get_date_time_json();

#endif 