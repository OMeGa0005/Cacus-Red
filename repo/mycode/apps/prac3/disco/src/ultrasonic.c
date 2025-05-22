#include <limits.h>
#include "ultasonic.h"
#include "ble.h"
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>

#define STACK_SIZE 2048
#define ULTRASONIC_PRIORITY 7

static const struct gpio_dt_spec trig_pin1 = GPIO_DT_SPEC_GET(DT_ALIAS(trig), gpios);
static const struct gpio_dt_spec echo_pin1 = GPIO_DT_SPEC_GET(DT_ALIAS(echo), gpios);
static const struct gpio_dt_spec trig_pin2 = GPIO_DT_SPEC_GET(DT_ALIAS(trig2), gpios);
static const struct gpio_dt_spec echo_pin2 = GPIO_DT_SPEC_GET(DT_ALIAS(echo2), gpios);


#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4
K_MSGQ_DEFINE(my_msgq, sizeof(uint16_t), MSGQ_SIZE, MSGQ_ALIGNMENT);

int ultrasonic_init(void) {
    // Configure trigger pin as output
    int ret;
    ret = gpio_pin_configure(trig_pin1.port, trig_pin1.pin, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure trigger pin\n");
        return ret;
    }

    // Configure echo pin as input
    ret = gpio_pin_configure(echo_pin1.port, echo_pin1.pin, GPIO_INPUT);
    if (ret < 0) {
        printk("Failed to configure echo pin\n");
        return ret;
    }
    ret = gpio_pin_configure(trig_pin2.port, trig_pin2.pin, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure trigger pin\n");
        return ret;
    }

    // Configure echo pin as input
    ret = gpio_pin_configure(echo_pin2.port, echo_pin2.pin, GPIO_INPUT);
    if (ret < 0) {
        printk("Failed to configure echo pin\n");
        return ret;
    }

    return 0;
}

uint8_t read_distance(int select) {
    uint64_t start_time, end_time, timeout_start;
    // Wait for echo pin to go high
    if(select == 1){
        while (gpio_pin_get(echo_pin1.port, echo_pin1.pin) == 1) {
        }
        k_usleep(2);
        gpio_pin_set_dt(&trig_pin1, 1);
        k_usleep(10);
        gpio_pin_set_dt(&trig_pin1, 0);
    
    
        // Wait for the echo pin to go low
        timeout_start = k_cycle_get_64();
        while (gpio_pin_get(echo_pin1.port, echo_pin1.pin) == 0) {
            if (k_cyc_to_us_floor64(k_cycle_get_64()-timeout_start) > 60000) {
                printf("Sensor %d: Timeout waiting for echo to go high\n", select);
                return 0b11111111; // Return maximum distance (2000 cm or 20m)
            }
        }
        start_time = k_cycle_get_64();
        while (gpio_pin_get(echo_pin1.port, echo_pin1.pin) == 1) {
        }
    }
    else if(select == 2){
        while (gpio_pin_get(echo_pin2.port, echo_pin2.pin) == 1) {
        } 
        k_usleep(2);
        gpio_pin_set_dt(&trig_pin2, 1);
        k_usleep(10);
        gpio_pin_set_dt(&trig_pin2, 0);
    
    
        // Wait for the echo pin to go low
        timeout_start = k_cycle_get_64();
        while (gpio_pin_get(echo_pin2.port, echo_pin2.pin) == 0) {
            if (k_cyc_to_us_floor64(k_cycle_get_64() - timeout_start) > 60000) {
                printf("Sensor %d: Timeout waiting for echo to go high\n", select);
                return 0b11111111; // Return maximum distance (2000 cm or 20m)
            }
        }
        start_time = k_cycle_get_64();
        while (gpio_pin_get(echo_pin2.port, echo_pin2.pin) == 1) {
        }
    }

    end_time = k_cycle_get_64();
    // Calculate the duration of the echo pulse
    uint64_t pulse_duration_us = k_cyc_to_us_floor64(end_time - start_time);
    printf("end time is %llu\n", end_time);
    printf("difference is %llu\n", pulse_duration_us);

    // Calculate the distance in cm
    const float SPEED_OF_SOUND_CM_PER_US = 0.0343; // Speed of sound in cm/us
    uint32_t distance = (pulse_duration_us * SPEED_OF_SOUND_CM_PER_US) / 2;
    distance=distance/5;
    if(distance>255){
        distance=255;
    }
    uint8_t return_val=distance;


    printf("Sensor %d: Distance: %u cm\n",select, distance*5);

    return return_val;
}

void ultrasonic_task(void) {
    ultrasonic_init();
    uint16_t combinedDistance = 0;
    while (1) {
        combinedDistance = 0;
        combinedDistance = read_distance(1);
        k_msleep(100);
        combinedDistance |= read_distance(2)<<8;
        if (k_msgq_put(&my_msgq, &combinedDistance, K_NO_WAIT) != 0) {
            printf("Message queue is full, dropping value");
        }
        k_msleep(400);
    }
}
K_THREAD_DEFINE(ultra, STACK_SIZE, ultrasonic_task, NULL, NULL, ULTRASONIC_PRIORITY,0,0,0);