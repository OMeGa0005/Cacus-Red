/*
 * ble_receive.h
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#ifndef BLE_RECEIVE_H
#define BLE_RECEIVE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include "bluetooth_sensor_set.h"

void scan_thread(void);

BluetoothSensorSet get_sensor_set(void);

#endif // BLE_RECEIVE_H