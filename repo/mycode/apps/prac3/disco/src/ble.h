#ifndef BLE_H
#define BLE_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>


int init_ble(void);
void send_value(uint8_t value);
void set_val1(uint8_t value);


extern struct k_msgq my_msgq;

#endif 