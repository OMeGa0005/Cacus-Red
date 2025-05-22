#include "ble_send.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

// iBeacon payload format
static uint8_t ibeacon_payload[] = {
    0x73, 0x73, // iBeacon prefix
    0x00,       // Ultrasonic1
    0x00,       // Ultrasonic2
    0x00, 0x00, // ble 1
    0x00, 0x00, // ble 2
    0x00, 0x00, // ble 3
    0x00, 0x00, // ble 4
    0x00, 0x00, // ble 5
    0x00, 0x00, // ble 6
    0x00, 0x00, // ble 7
    0x00, 0x00, // ble 8
    0x00, 0x00, // ble 9
    0x00, 0x00, // ble 10
    0x00, 0x00, // coordinates x,y
};

// // iBeacon payload format
// static uint8_t ibeacon_payload_in[] = {
//     0x37, 0x73, // iBeacon prefix
//     0x00, 0x00, // address0
//     0x00, 0x00, // address1
//     0x00, 0x00, // address2
//     0x00, 0x00, // major number
//     0x00, 0x00, // minor number
//     0x00, // RSSI data about the node you have described above
// };

// Define advertisement data
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_FLAGS, &(uint8_t){BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR}, 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload)),
};

void update_sensor_values(uint8_t coordinates[2])
{
    // static uint8_t ultrasonic[2] = {0, 0}; // ultrasonic1 and ultrasonic2
    // static uint8_t ble_node[10][2] = {  {0, 11},
    //                                     {1, 12},
    //                                     {2, 13},
    //                                     {3, 14},
    //                                     {4, 15},
    //                                     {5, 16},
    //                                     {6, 17},
    //                                     {7, 18},
    //                                     {8, 19},
    //                                     {9, 20}}; // 10 nodes, each with 2 bytes

    // // Given ultrasonic1 is at payload index 22 and 23 if major/minor are fixed update the value
    // ibeacon_payload[2] = ultrasonic[0]; // ultrasonic1
    // ibeacon_payload[3] = ultrasonic[1]; // ultrasonic2
    // // Update the ble node values
    // for (int i = 0; i < 10; i++)
    // {
    //     ibeacon_payload[4 + (i * 2)] = ble_node[i][0];   // ble node 1
    //     ibeacon_payload[5 + (i * 2)] = ble_node[i][1];   // ble node 2
    // }

    ibeacon_payload[24] = coordinates[0]; // x coordinate
    ibeacon_payload[25] = coordinates[1]; // y coordinate
}

void restart_advertising(uint8_t coordinates[2])
{
    update_sensor_values(coordinates);

    struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, &(uint8_t){BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR}, 1),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload)),
    };

    bt_le_adv_stop();
    int ret = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret)
    {
        printf("=== Failed to restart advertising (err %d) ===\n", ret);
    }
    else
    {
        // printf("Advertising restarted successfully\n");
    }
}

// Initialize BLE and start advertising iBeacon data
int init_ble(void)
{
    printf("Initialising BLE as iBeacon\n");

    int ret = bt_enable(NULL);
    if (ret)
    {
        printf("Bluetooth init failed (err %d)\n", ret);
        return ret;
    }

    ret = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret)
    {
        printf("Advertising start failed (err %d)\n", ret);
    }
    else
    {
        printf("iBeacon advertising started successfully\n");
    }

    return ret;
}

// reading bt
#define IBEACON_MANUF_DATA_MIN_LEN 24

static bool adv_parse_cb(struct bt_data *data, void *user_data)
{
    if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= IBEACON_MANUF_DATA_MIN_LEN)
    {
        const uint8_t *d = data->data;
        // Check for the iBeacon prefix (first 2 bytes: 0x02, 0x15)
        if (d[0] == 0x02 && d[1] == 0x15)
        {
            uint16_t major = (d[18] << 8) | d[19];
            uint16_t minor = (d[20] << 8) | d[21];
            int8_t tx_power = d[22];    // TX Power (signed 8-bit)
            uint8_t ultrasonic = d[23]; // Custom field "ultrasonic1"

            printk("iBeacon detected:\n");
            printk("  Major: %u\n", major);
            printk("  Minor: %u\n", minor);
            printk("  TX Power: %d\n", tx_power);
            printk("  Ultrasonic1: %u\n", ultrasonic);
        }
    }
    /* Returning true to continue processing any further fields */
    return true;
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
    printk("Advertisement from %s (RSSI %d)\n", addr_str, info->rssi);

    // Parse the advertisement data.
    bt_data_parse(buf, adv_parse_cb, NULL);
}
