#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>

// Thread Stack Size and Priorities
#define STACK_SIZE 2048
#define SCAN_PRIORITY 4
#define COMM_PRIORITY 5

// Message Queue Size
#define MSGQ_MAX 10
#define MSGQ_SIZE sizeof(struct ibeacon_data)

// Scan/Comm Timing
#define SCAN_INTERVAL 1000 // ms
#define COMM_INTERVAL 10   // ms

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// iBeacon Data Structure
struct ibeacon_data {
    bt_addr_le_t addr; // BLE address
    int8_t rssi;       // RSSI value
    uint16_t major;    // Major value
    uint16_t minor;    // Minor value
};

// Message Queue for sharing iBeacon data between threads
K_MSGQ_DEFINE(beacon_msgq, MSGQ_SIZE, MSGQ_MAX, 4);

// Stack and thread control blocks
K_THREAD_STACK_DEFINE(scan_area, STACK_SIZE);
K_THREAD_STACK_DEFINE(comm_area, STACK_SIZE);
struct k_thread scan_data;
struct k_thread comm_data;

// Known iBeacon addresses - filter on these
static const char *ibeacon_known[] = {
    "F5:75:FE:85:34:67 (random)", // 4011-A
    "E5:73:87:06:1E:86 (random)", // 4011-B
    "CA:99:9E:FD:98:B1 (random)", // 4011-C
    "CB:1B:89:82:FF:FE (random)", // 4011-D
    "D4:D2:A0:A4:5C:AC (random)", // 4011-E
    "C1:13:27:E9:B7:7C (random)", // 4011-F
    "F1:04:48:06:39:A0 (random)", // 4011-G
    "CA:0C:E0:DB:CE:60 (random)", // 4011-H
};

// Helper function to check if the address is known
static int is_ibeacon(const bt_addr_le_t *addr)
{
    // Convert address to string for comparison
    char ad[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, ad, sizeof(ad));

    // Check if the address is in the known list
    for (int i = 0; i < ARRAY_SIZE(ibeacon_known); i++) {
        if (strcmp(ad, ibeacon_known[i]) == 0) {
            return true;
        }
    }
    return false;
}

// BLE Scan Callback
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *buf)
{   
    // Only process known iBeacons
    if (is_ibeacon(addr)) {
        struct ibeacon_data beacon;
        beacon.addr = *addr;
        beacon.rssi = rssi;
        beacon.major = sys_get_be16(&buf->data[20]);
        beacon.minor = sys_get_be16(&buf->data[22]);
        
        if (k_msgq_put(&beacon_msgq, &beacon, K_NO_WAIT) != 0) {
            LOG_WRN("Message queue full, dropping data");
        }
    }

    k_sleep(K_MSEC(50)); // Sleep to avoid flooding the queue
}

// Scanning thread
void scan_thread(void *, void *, void *)
{
    int err;

    struct bt_le_scan_param scan_param = {
        .type     = BT_HCI_LE_SCAN_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return;
    }
    LOG_INF("Scanning started");

    // Keep the thread alive
    while(1) {
        k_sleep(K_MSEC(SCAN_INTERVAL));
    }
}

// Communication thread
void comm_thread(void *, void *, void *)
{
    struct ibeacon_data beacon;
    int err;

    while(1) {
        if (k_msgq_get(&beacon_msgq, &beacon, K_FOREVER) == 0) {
            // Payload
            uint8_t payload[14] = {0};

            // iBeacon Prefix
            payload[0] = 0x88;
            payload[1] = 0x88;

            // BLE address (6 bytes)
            payload[2] = beacon.addr.a.val[5];
            payload[3] = beacon.addr.a.val[4];
            payload[4] = beacon.addr.a.val[3];
            payload[5] = beacon.addr.a.val[2];
            payload[6] = beacon.addr.a.val[1];
            payload[7] = beacon.addr.a.val[0];

            // Major
            payload[8] = beacon.major & 0xFF;
            payload[9] = (beacon.major >> 8) & 0xFF;

            // Minor
            payload[10] = beacon.minor & 0xFF;
            payload[11] = (beacon.minor >> 8) & 0xFF;

            // RSSI
            payload[12] = (uint8_t)beacon.rssi;

            // Fill in advertising data
            struct bt_data ad[] = {
                BT_DATA(BT_DATA_FLAGS, (uint8_t[]){BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR}, 1),
                BT_DATA(BT_DATA_MANUFACTURER_DATA, payload, sizeof(payload)),
            };

            // Stop advertising
            err = bt_le_adv_stop();
            if (err) {
                LOG_ERR("Advertising failed to stop (err %d)", err);
            }

            // Start new advertising
            err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
            if (err) {
                LOG_ERR("Advertising failed to start (err %d)", err);
            } else {
                LOG_INF("Advertising RSSI=%d", beacon.rssi);
            }

            // Advertising window
            k_sleep(K_MSEC(COMM_INTERVAL));
        }
    }
}

// Main
int main(void)
{
    int err;

    LOG_INF("Starting Mobile Node");

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    LOG_INF("Bluetooth initialized");

    // Create scanning thread
    k_thread_create(&scan_data, scan_area,
                    K_THREAD_STACK_SIZEOF(scan_area),
                    scan_thread, NULL, NULL, NULL,
                    SCAN_PRIORITY, 0, K_NO_WAIT);
    
    // Create communication thread
    k_thread_create(&comm_data, comm_area,
                    K_THREAD_STACK_SIZEOF(comm_area),
                    comm_thread, NULL, NULL, NULL,
                    COMM_PRIORITY, 0, K_NO_WAIT);

    return 0;
}