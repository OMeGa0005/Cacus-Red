/*
 * BLE Receive Module
 * Receives BLE data from iBeacon nodes and ultrasonic sensors.
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#include "ble_receive.h"
#include "bluetooth_sensor_set.h"
#include "ibeacon_commands.h"
#include "kalman.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

// Message Queue Size
#define MSGQ_MAX 10
#define MSGQ_SIZE 4
#define MSGQ_ALIGNMENT 4

K_MSGQ_DEFINE(ble_beacon_msgq, sizeof(BluetoothSensorSet), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(ultrasonic_beacon_msgq, sizeof(BluetoothSensorSet), MSGQ_SIZE, MSGQ_ALIGNMENT);

// // Define the ibeacon_data structure
// struct ibeacon_data_mobile {
//     bt_addr_le_t addr;  // iBeacon address
//     int8_t rssi;        // Received Signal Strength Indicator (RSSI)
//     uint8_t ultrasonic1; // Ultrasonic sensor 1
//     uint8_t ultrasonic2; // Ultrasonic sensor 2
//     uint16_t ble1;      // BLE data 1
//     uint16_t ble2;      // BLE data 2
//     uint16_t ble3;      // BLE data 3
//     uint16_t ble4;      // BLE data 4
//     uint16_t ble5;      // BLE data 5
//     uint16_t ble6;      // BLE data 6
//     uint16_t ble7;      // BLE data 7
//     uint16_t ble8;      // BLE data 8
//     uint16_t ble9;      // BLE data 9
//     uint16_t ble10;     // BLE data 10
// };

// Define the ibeacon_data structure
struct ibeacon_data_ultrasonic
{
    bt_addr_le_t addr;   // iBeacon address
    int8_t rssi;         // Received Signal Strength Indicator (RSSI)
    uint16_t ultrasonic1; // Ultrasonic sensor 1
    uint16_t ultrasonic2; // Ultrasonic sensor 2
};

struct ibeacon_data_mobile
{
    bt_addr_le_t addr;   // iBeacon address
    int8_t mobile_node_rssi;         // Received Signal Strength Indicator (Not for mobile node use, this is only for the base node)
    uint16_t address0;   // address0
    uint16_t address1;   // address1
    uint16_t address2;   // address2
    uint16_t major;      // major number
    uint16_t minor;      // minor number
    int8_t rssi_data;  // RSSI data about the node you have described above
};

static BluetoothSensorSet sensor_set;

// BLE Scan Callback
static void
scan_cb(const bt_addr_le_t *addr, int8_t rssi,
        uint8_t adv_type, struct net_buf_simple *buf)

{
    //printf("scan callback\n");
    if (!buf) {
        return;
    }

    // Only process known iBeacons
    // Mobile node
    // printf("%0x:%0x\n", buf->data[5], 6);
    
    if (buf->data[5] == 0x88 && buf->data[6] == 0x88)
    {
        // printf("Found Mobile Node\n");
        struct ibeacon_data_mobile beacon;
        beacon.addr = *addr;
        beacon.mobile_node_rssi = rssi;

        // Check if the payload matches the expected iBeacon format
        if (true) {  // Use sizeof(struct ibeacon_data) for validation

            // Parse the payload
            // beacon.address0 = sys_get_be16(&buf->data[7]);
            // beacon.address1 = sys_get_be16(&buf->data[9]);
            // beacon.address2 = sys_get_be16(&buf->data[11]);
            beacon.major = buf->data[14] << 8 | buf->data[13];
            beacon.minor = buf->data[16] << 8 | buf->data[15];
            beacon.rssi_data = (int8_t)buf->data[17]; // RSSI data about the node you have described above

            // uint64_t ble_address = ((beacon.address0 << 16) | (beacon.address1 << 8) | (beacon.address2));

            // Create a BluetoothSensorSet to hold the parsed data
            // BLE Sensor Set

            char mac_address[18];
            snprintf(mac_address, sizeof(mac_address), "%02X:%02X:%02X:%02X:%02X:%02X",
                     (uint8_t)(buf->data[7]),
                     (uint8_t)(buf->data[8]),
                     (uint8_t)(buf->data[9]),
                     (uint8_t)(buf->data[10]),
                     (uint8_t)(buf->data[11]),
                     (uint8_t)(buf->data[12]));

            // Log the parsed data
            // printf("iBeacon Data:\n");
            // printf("  Address: %s\n", mac_address);
            // printf("  Major: %u\n", beacon.major);
            // printf("  Minor: %u\n", beacon.minor);
            // printf("  RSSI Data: %d\n", beacon.rssi_data);

            if (ibeacon_node_exists(mac_address))
            {
                int x;
                int y;
                get_coordinates_by_mac(mac_address, &x, &y);
                add_node_position(get_ble_name_by_mac(mac_address), x, y, beacon.rssi_data);
            }
            else
            {
                // printf("Node not found in the list: %s\n", mac_address);
            }
        } else {
            printf("Invalid iBeacon payload\n");
        }
    }

    else if (buf->data[5] == 0x33 && buf->data[6] == 0x77)
    {
        // printf("found node\n");
        struct ibeacon_data_ultrasonic beacon;
        beacon.addr = *addr;
        beacon.rssi = rssi;

        // Check if the payload matches the expected iBeacon format
        if (true)
        { // Use sizeof(struct ibeacon_data) for validation

            // Parse the payload
            beacon.ultrasonic1 = buf->data[7] * 5;
            beacon.ultrasonic2 = buf->data[8] * 5;

            // Log the parsed data
            // printf("iBeacon Data:\n");
            // printf("  Ultrasonic1: %u\n", buf->data[7] * 5);
            // printf("  Ultrasonic2: %u\n", buf->data[8] * 5);

            set_ultrasonic1(beacon.ultrasonic1);
            set_ultrasonic2(beacon.ultrasonic2);
        }
        else
        {
            printf("Invalid iBeacon payload\n");
        }
    }
}

BluetoothSensorSet get_sensor_set(void)
{
    BluetoothSensorSet sensor_set;
    if (k_msgq_get(&ble_beacon_msgq, &sensor_set, K_NO_WAIT) == 0) {
        return sensor_set;
    } else {
        // Return an empty sensor set if the queue is empty
        memset(&sensor_set, 0, sizeof(sensor_set));
        return sensor_set;
    }
}


// Scanning thread
void scan_thread(void)
{
    // printf("STARTJFING THE TDHEADS\n");
    int err;

    struct bt_le_scan_param scan_param = {
        .type     = BT_HCI_LE_SCAN_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };
    // printf("STARTJFING THE TDHEADS\n");
    bt_enable(NULL);
    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        printf("Scanning failed to start (err %d)", err);
        return;
    }
    // printf("Scanning started");

    // Keep the thread alive
    while(1) {
        // printf("running blescan");
        k_msleep(5000);
    }
}