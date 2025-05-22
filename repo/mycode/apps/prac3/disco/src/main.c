#include <stdio.h>
#include "ble.c"
#include "ultrasonic.c"

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


#define SLEEP_TIME_MS   1000
#define STACK_SIZE 1024
#define RNG_PRIORITY 7
#define DISPLAY_PRIORITY 7
#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4



/*
int main(){
    uint8_t arr[2] = {1, 2};
    uint8_t len = 2;
    uint8_t value = 0;
    ultrasonic_init();
    bt_le_scan_start(BT_LE_SCAN_PASSIVE, scan_recv);


    printf("Advertising started\n");
    while(1){
        //read_distance(1);
        read_distance(2);
        //set_val1(55); // Set the value of val1 to 55
        k_msleep(2000);
    }
    return 0;
}*/