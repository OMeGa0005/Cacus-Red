#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>

#define SLEEP_TIME_MS   1000
#define STACK_SIZE 1024
#define SHELL_PRIORITY 7
#define ARGB_PRIORITY 7
#define ARGB_SIZE 10
#define ARGB_ALIGNMENT 4

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)


K_MSGQ_DEFINE(argb_colour, sizeof(uint8_t), ARGB_SIZE, ARGB_ALIGNMENT);
static const struct gpio_dt_spec data_spec = GPIO_DT_SPEC_GET(DT_ALIAS(datargb), gpios);
static const struct gpio_dt_spec clk_spec = GPIO_DT_SPEC_GET(DT_ALIAS(clkrgb), gpios);

static inline void my_digital_write(const struct gpio_dt_spec *spec, int value){
    gpio_pin_set_dt(spec, value);
}
void clk_pulse(void) {//pulese clock low then back to high
    gpio_pin_set_dt(&clk_spec, 0);
    k_usleep(30);
    gpio_pin_set_dt(&clk_spec, 1);
    k_usleep(30);
}
static void sendByte(uint8_t b){
    for (uint8_t i = 0; i < 8; i++) {
        // Advance to the next bit to send
        b <<= 1;
        /* Write MSB first */
        my_digital_write(&data_spec, (b & 0x80) ? 1 : 0);
        clk_pulse(); //pulses clock pin to signify data sent
        b <<= 1;
    }
}

void argbLedThread(void){ //sets the colour based on queue from control thread
    gpio_pin_configure_dt(&clk_spec, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&data_spec, GPIO_OUTPUT_ACTIVE);
    uint8_t colour;
    for(;;){

        if (k_msgq_get(&argb_colour, &colour, K_FOREVER) == 0) {
            sendByte(0x00);//leading 0 bytes
            sendByte(0x00);
            sendByte(0x00);
            sendByte(0x00);
            sendByte(0xC0); //data bytes
            switch (colour%8)
            {
            case 0:
                sendByte(0x00);
                sendByte(0x00);
                sendByte(0x00);
                break;

            case 1:
                sendByte(0xFF);
                sendByte(0x00);
                sendByte(0x00);
                break;   

            case 2:
                sendByte(0x00);
                sendByte(0xFF);
                sendByte(0x00);
                break;     
            
            case 3:
                sendByte(0xFF);
                sendByte(0xFF);
                sendByte(0x00);
                break; 
                
            case 4:
                sendByte(0x00);
                sendByte(0x00);
                sendByte(0xFF);
                break;  
            
            case 5:
                sendByte(0xFF);
                sendByte(0x00);
                sendByte(0xFF);
                break; 
            
            case 6:
                sendByte(0x00);
                sendByte(0xFF);
                sendByte(0xFF);
                break; 
                
            case 7:
                sendByte(0xFF);
                sendByte(0xFF);
                sendByte(0xFF);
                break;  
            default:
                break;
            }
            sendByte(0x00); //trailing 0 bytes
            sendByte(0x00);
            sendByte(0x00);
            sendByte(0x00);
        }
    }

}

void argbControlThread(void){ //Determines the colour and passes a queue to the setter thread
    uint8_t colour=0;
	for(;;){
        if (k_msgq_put(&argb_colour, &colour, K_NO_WAIT) != 0) {
            printf("Message queue is full, dropping value");
        }
        colour++;
        k_msleep(2000);
	}
}
//Create Threads
K_THREAD_DEFINE(argb, STACK_SIZE, argbLedThread, NULL, NULL, NULL, ARGB_PRIORITY, 0, 0);
K_THREAD_DEFINE(argbControl, STACK_SIZE, argbControlThread, NULL, NULL, NULL, ARGB_PRIORITY, 0, 0);