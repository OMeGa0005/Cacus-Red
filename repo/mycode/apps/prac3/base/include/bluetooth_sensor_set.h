/*
 * Bluetooth Sensor Set Header File
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#ifndef BLUETOOTH_SENSOR_SET_H
#define BLUETOOTH_SENSOR_SET_H

#include <stdint.h>
#include "bluetooth_node.h"

typedef struct
{
    BluetoothNode ultrasonic[2]; // ultrasonic1 and ultrasonic2
    BluetoothNode ble_nodes[10]; // 10 BLE nodes
} BluetoothSensorSet;

void set_ultrasonic(BluetoothSensorSet *sensor_set, int index, uint8_t distance);

void set_ble_node(BluetoothSensorSet *sensor_set, int index, const char *name, int8_t rssi);

void get_ultrasonic(const BluetoothSensorSet *sensor_set, int index, uint8_t *distance);

void get_ble_node(const BluetoothSensorSet *sensor_set, int index, char *name, int8_t *rssi);
#endif // BLUETOOTH_NODE_H