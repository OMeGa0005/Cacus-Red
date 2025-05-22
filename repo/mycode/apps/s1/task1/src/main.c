#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>


#define SLEEP_TIME_MS   1000
#define STACK_SIZE 1024
#define RNG_PRIORITY 7
#define DISPLAY_PRIORITY 7
#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4


//Creates queue
K_MSGQ_DEFINE(my_msgq, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);


//Creates random number and passes it to display thread
void rngThread(void){
    uint32_t rand;
    for(;;){
        rand=sys_rand32_get();
        if (k_msgq_put(&my_msgq, &rand, K_NO_WAIT) != 0) {
            printf("Message queue is full, dropping value");
        }
        k_msleep(1000);
    }
}
//prints 8 digits of random number 
void displayThread(void){
    uint32_t received_value;
    for(;;){
        //Receive the random number from the message queue 
        if (k_msgq_get(&my_msgq, &received_value, K_FOREVER) == 0) {
            printf("%08u\n", (received_value & 0x5F5E0FF));
        }
    }
}
//Creates threads in compile time
K_THREAD_DEFINE(rng, STACK_SIZE, rngThread, NULL, NULL, NULL, RNG_PRIORITY,0,0);
K_THREAD_DEFINE(display, STACK_SIZE, displayThread, NULL, NULL, NULL, DISPLAY_PRIORITY, 0, 0);