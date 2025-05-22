#include <stdio.h>
#include "ble.c"
#include "display.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>



#define SLEEP_TIME_MS   1000
#define STACK_SIZE 4096
#define BLE_PRIORITY 7
#define BLE_PRIORITY 7
#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4

K_THREAD_DEFINE(BLE, STACK_SIZE, scan_thread, NULL, NULL, NULL, BLE_PRIORITY,0,0);
K_THREAD_DEFINE(DIS, STACK_SIZE, display_thread, NULL, NULL, NULL, BLE_PRIORITY,0,0);


/*
int main(){
    uint8_t value = 0;
    init_display();

    printf("started\n");
    while(1){
        //place_red_dot(1,2);
        lv_timer_handler();
        //printf("running");
        //printf("Sending value: %d\n", value);
        //restart_advertising(value++);
        k_msleep(5);
        //printf("Advertising restarted with value: %d\n", value);
    }
    return 0;
}*/