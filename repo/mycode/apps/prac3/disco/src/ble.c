#include "ble.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>


#define STACK_SIZE 1024
#define BLE_PRIORITY 7

// iBeacon payload format
static uint8_t ibeacon_payload[] = {
    0x33, 0x77, // iBeacon prefix
    0x00,       // Ultrasonic1
    0x00,       // Ultrasonic2   
};

// Define advertisement data
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_FLAGS, &(uint8_t){BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR}, 1),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload)),
};

void update_sensor_values(uint16_t ultrasonic)
{
    uint8_t ultrasonic1 = ultrasonic & 0xFF; // Extract the lower byte
    uint8_t ultrasonic2 = (ultrasonic >> 8) & 0xFF; // Extract the upper byte

    // Given ultrasonic1 is at payload index 22 and 23 if major/minor are fixed update the value
    ibeacon_payload[2] = ultrasonic1; // ultrasonic1
    ibeacon_payload[3] = ultrasonic2; // ultrasonic2

}

void restart_advertising(uint16_t ultrasonic)
{
    update_sensor_values(ultrasonic);

    struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, &(uint8_t){BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR}, 1),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, ibeacon_payload, sizeof(ibeacon_payload)),
    };

    bt_le_adv_stop();
    int ret = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret)
    {
        printf("Failed to restart advertising (err %d)\n", ret);
    }
}

// Initialize BLE and start advertising iBeacon data
int init_ble(void)
{
    printf("Initializing BLE as iBeacon\n");

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


void ble_task(void) {
    init_ble();
    // Start scanning for iBeacon advertisements
    uint16_t combinedDistance = 0;

    while (1)
    {
        k_msgq_get(&my_msgq, &combinedDistance, K_FOREVER) == 0;
        restart_advertising(combinedDistance);
    }
}
K_THREAD_DEFINE(ble, STACK_SIZE, ble_task, NULL, NULL, BLE_PRIORITY,0,0,0);