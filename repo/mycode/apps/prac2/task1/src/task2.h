#ifndef TASK2_H
#define TASK2_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/sensor.h>
//#include <sys/ring_buffer.h>

#include <zephyr/sys/mpsc_pbuf.h>
#include <zephyr/sys/mpsc_packet.h>

//const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));
void sensor_main(const struct shell *shell, size_t argc, char *argv[]);
char* get_mag();
char* get_tmp();
char* get_hum();
char* get_pressure();
void display_task(); 

#endif 