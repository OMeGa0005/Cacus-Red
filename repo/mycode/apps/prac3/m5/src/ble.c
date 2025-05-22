#include "ble.h"
#include "display.c"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

// Message Queue Size
#define MSGQ_MAX 10
#define MSGQ_SIZE 4
#define MSGQ_ALIGNMENT 4

K_MSGQ_DEFINE(beacon_msgq, sizeof(uint16_t), MSGQ_SIZE, MSGQ_ALIGNMENT);

// Define the ibeacon_data structure
struct ibeacon_data {
    bt_addr_le_t addr;  // iBeacon address
    int8_t rssi;        // Received Signal Strength Indicator (RSSI)
    uint8_t ultrasonic1; // Ultrasonic sensor 1
    uint8_t ultrasonic2; // Ultrasonic sensor 2
    uint16_t ble1;      // BLE data 1
    uint16_t ble2;      // BLE data 2
    uint16_t ble3;      // BLE data 3
    uint16_t ble4;      // BLE data 4
    uint16_t ble5;      // BLE data 5
    uint16_t ble6;      // BLE data 6
    uint16_t ble7;      // BLE data 7
    uint16_t ble8;      // BLE data 8
    uint16_t ble9;      // BLE data 9
    uint16_t ble10;     // BLE data 10
    uint8_t x;
    uint8_t y;
};

// BLE Scan Callback
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
    uint8_t adv_type, struct net_buf_simple *buf)
    
{
    //printf("scan callback\n");
    if (!buf) {
        return;
    }
    //printf("Payload: ");
//for (int i = 0; i < buf->len; i++) {
  //  printf("%02x ", buf->data[i]);
//}
//printf("\n");
    // Only process known iBeacons
    if(buf->data[5]==0x73 && buf->data[6]==0x73){
        printf("found node\n");
        struct ibeacon_data beacon;
        beacon.addr = *addr;
        beacon.rssi = rssi;

        // Check if the payload matches the expected iBeacon format
        //if (buf->len >= sizeof(struct ibeacon_data)) {  // Use sizeof(struct ibeacon_data) for validation

            // Parse the payload
            beacon.ultrasonic1 = buf->data[7]*5;
            beacon.ultrasonic2 = buf->data[8]*5;
            beacon.ble1 = sys_get_be16(&buf->data[9]);
            beacon.ble2 = sys_get_be16(&buf->data[11]);
            beacon.ble3 = sys_get_be16(&buf->data[13]);
            beacon.ble4 = sys_get_be16(&buf->data[15]);
            beacon.ble5 = sys_get_be16(&buf->data[17]);
            beacon.ble6 = sys_get_be16(&buf->data[19]);
            beacon.ble7 = sys_get_be16(&buf->data[21]);
            beacon.ble8 = sys_get_be16(&buf->data[23]);
            beacon.ble9 = sys_get_be16(&buf->data[25]);
            beacon.ble10 = sys_get_be16(&buf->data[27]);
            float x = buf->data[29];
            x=x/10;
            float y = buf->data[30];
            y=y/10;

            place_red_dot(x,y);


            // Log the parsed data
            printf("iBeacon Data:\n");
            printf("  Ultrasonic1: %u\n", beacon.ultrasonic1);
            printf("  Ultrasonic2: %u\n", beacon.ultrasonic2);
            printf("  BLE1: %u\n", beacon.ble1);
            printf("  BLE2: %u\n", beacon.ble2);
            printf("  BLE3: %u\n", beacon.ble3);
            printf("  BLE4: %u\n", beacon.ble4);
            printf("  BLE5: %u\n", beacon.ble5);
            printf("  BLE6: %u\n", beacon.ble6);
            printf("  BLE7: %u\n", beacon.ble7);
            printf("  BLE8: %u\n", beacon.ble8);
            printf("  BLE9: %u\n", beacon.ble9);
            printf("  BLE10: %u\n", beacon.ble10);
            printf("  x: %u\n", buf->data[29]);
            printf("  y: %u\n", sys_get_be16(&buf->data[30]));

            // Store the parsed data in the beacon structure (if needed)
            // Example: beacon.ultrasonic1 = ultrasonic1;

            // Add the beacon to the message queue
            //if (k_msgq_put(&beacon_msgq, &beacon, K_NO_WAIT) != 0) {
            //    printf("Message queue full, dropping data\n");
            //}
        //} else {
        //    printf("Invalid iBeacon payload\n");
        //}
    }
}


// Scanning thread
void scan_thread(void)
{
    printf("STARTJFING THE TDHEADS\n");
    int err;

    struct bt_le_scan_param scan_param = {
        .type     = BT_HCI_LE_SCAN_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };
    printf("STARTJFING THE TDHEADS\n");
    int ret = bt_enable(NULL);
    if (ret) {
        printf("Bluetooth initialization failed (err %d)\n", err);
        return;
    }
    printf("STARTJFING THE TDHEADS\n");
    err = bt_le_scan_start(&scan_param, scan_cb);
    printf("STARTJFING THE TDHEADS\n");
    if (err) {
        printf("Scanning failed to start (err %d)", err);
        return;
    }
    printf("Scanning started");

    // Keep the thread alive
    while(1) {
        printf("running blescan");
        k_msleep(5000);
    }
}