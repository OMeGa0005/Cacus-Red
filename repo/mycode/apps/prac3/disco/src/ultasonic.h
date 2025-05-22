#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>

int ultrasonic_init(void);
uint8_t read_distance(int select);



#endif 