#ifndef BLE_SEND_H
#define BLE_SEND_H

/*
 * Bluetooth Low Energy (BLE) header file
 * This file contains function declarations and includes necessary headers
 * for BLE functionality.
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>

int init_ble(void);
void send_value(uint8_t value);
void set_val1(uint8_t value);
void update_sensor_values(uint8_t coordinates[2]);
void restart_advertising(uint8_t coordinates[2]);

#endif