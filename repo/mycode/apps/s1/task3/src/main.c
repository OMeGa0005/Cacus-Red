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
#define SHELL_PRIORITY 7

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);



LOG_MODULE_REGISTER(led_module, LOG_LEVEL_DBG);


//Sets up comamnd to print system uptime in seconds

void inv_command(){
	LOG_ERR("Invalid Command");
}

static int cmd_time_get(const struct shell *shell, size_t argc, char *argv[]) {
    if (argc == 1) { //print time in seconds
        LOG_INF("System uptime: %d seconds\n", (int)(k_uptime_get()/1000));
    } //else check if arguments valid and print time formated if f arg
    else if (argc > 2){
        LOG_ERR("invalid number or arguments");
    }
	else if (strcmp(argv[1], "f")==0){
		LOG_INF("System uptime: %d hours, %d minutes, %d seconds\n", (int)(k_uptime_get()/(1000*60*60)),(int)(k_uptime_get()/(1000*60)),(int)(k_uptime_get()/(1000)));
	}
    else{
        LOG_ERR("invalid argument");
    }

    return 0;
}

static int cmd_led_toggle(const struct shell *shell, size_t argc, char *argv[]) {
	//will only configure the leds once 
	static bool is_configured = false;
    if (!is_configured) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
        is_configured = true;
    }
	//If setting leds
    if (strcmp(argv[1], "s")==0){
		//check which led is being set, return inv_command if invalid or warning if already set 
        if (argv[2][0] == '0') {
			if (gpio_pin_get_dt(&led)==0) {
                LOG_WRN("led 2 is already off");
            }
			else{
				LOG_INF("led 2 is off");
            	gpio_pin_set_dt(&led1, 0);
			}
        }
		else if(argv[2][0] != '1'){
			inv_command(); //non valid character
            return 1;
		}
        else {
			if (gpio_pin_get_dt(&led1)==1) {
                LOG_WRN("led 2 is already on");
            }
			else{
				LOG_INF("led 2 is on");
				gpio_pin_set_dt(&led1, 1);
			}
        }
        if (argv[2][1] == '0') {
			if (gpio_pin_get_dt(&led)==0) {
                LOG_WRN("led 1 is already off");
            }
            else{
                LOG_INF("led 1 is off");
                gpio_pin_set_dt(&led, 0);
            }
        }
		else if(argv[2][1] != '1'){
			inv_command(); //non valid character
            return 1;
		}
        else {
            if (gpio_pin_get_dt(&led)==1) {
                LOG_WRN("led 1 is already on");
            }
            gpio_pin_set_dt(&led, 1);
        }
    }
    else if (strcmp(argv[1], "t")==0){
        if (argv[2][0] == '1') {
            LOG_INF("led 2 is toggled");
            gpio_pin_toggle_dt(&led1);
        }
		else if(argv[2][0] != '0'){
			inv_command(); //non valid character
            return 1;
		}
        if (argv[2][1] == '1') {
            LOG_INF("led 1 is toggled");
            gpio_pin_toggle_dt(&led);
        }
		else if(argv[2][1] != '0'){
			inv_command(); //non valid character
            return 1;
		}
    }
	else{
		inv_command();
        return 1;
	}
    if(argc<2){ //if not enough arguments
        LOG_ERR("Invalid Command: not enough arguments");
        return 1;
    }
    return 0;
}
//Creates command subset
SHELL_STATIC_SUBCMD_SET_CREATE(
    custom_cmds,
    SHELL_CMD_ARG(time, NULL, "possible options: f", cmd_time_get, 1, 1),
    SHELL_CMD_ARG(led, NULL, "possible options: s(set), t(toggle)", cmd_led_toggle, 3, 0),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(time, NULL, "", cmd_time_get); //determines what function will run based on the command
SHELL_CMD_REGISTER(led, NULL, "", cmd_led_toggle);