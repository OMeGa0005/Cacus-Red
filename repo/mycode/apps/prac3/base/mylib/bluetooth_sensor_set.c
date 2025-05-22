/*
 * Bluetooth Sensor Set
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#include <stdint.h>
#include <string.h>
#include "bluetooth_node.h"

typedef struct
{
    uint8_t ultrasonic[2]; // ultrasonic1 and ultrasonic2
    BluetoothNode ble_nodes[8]; // 13 total BLE nodes
} BluetoothSensorSet;

void set_ultrasonic(BluetoothSensorSet *sensor_set, int index, uint8_t distance)
{
    if (index < 0 || index >= 2)
    {
        return; // Invalid index
    }
    sensor_set->ultrasonic[index] = distance; // Set the distance in cm
}

void set_ble_node(BluetoothSensorSet *sensor_set, int index, const char *name, int8_t rssi)
{
    if (index < 0 || index >= 10)
    {
        return; // Invalid index
    }
    set_node_name(&sensor_set->ble_nodes[index], name);
    set_rssi(&sensor_set->ble_nodes[index], rssi);
}

void get_ultrasonic(const BluetoothSensorSet *sensor_set, int index, uint8_t *distance)
{
    if (index < 0 || index >= 2)
    {
        return; // Invalid index
    }
    *distance = sensor_set->ultrasonic[index]; // Retrieve the distance in cm
}

void get_ble_node(const BluetoothSensorSet *sensor_set, int index, char *name, int8_t *rssi)
{
    if (index < 0 || index >= 10)
    {
        return; // Invalid index
    }
    strncpy(name, get_node_name(&sensor_set->ble_nodes[index]), MAX_NODE_NAME_LENGTH);
    *rssi = get_rssi(&sensor_set->ble_nodes[index]);
}