#include "task3.h"
#include "task2.h"
#include "task5.h"
#include <zephyr/data/json.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>


#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_PRIORITY 6
#define STACK_SIZE 4096

#define MSGQ_SIZE 10
#define MSGQ_ALIGNMENT 4

static int on=0;

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios); // GPIO pin for button
static struct gpio_callback button_cb_data;

struct json_struct {
    int DID;
    char* time;
    char* value;
};
// JSON descriptors for struct json_struct
static const struct json_obj_descr json_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct json_struct, DID, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct json_struct, time, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct json_struct, value, JSON_TOK_STRING),
};

//Creates queue
K_MSGQ_DEFINE(Q_on_mag, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_on_tmp, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_on_hum, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_on_prs, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_on_all, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_rate, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);
K_MSGQ_DEFINE(Q_device, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);

//button function
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int on_val = 1;
    int off_val = 0;
    if(on == 1){
        k_msgq_put(&Q_on_mag, &off_val, K_NO_WAIT);
        k_msgq_put(&Q_on_prs, &off_val, K_NO_WAIT);
        k_msgq_put(&Q_on_all, &off_val, K_NO_WAIT);
        k_msgq_put(&Q_on_tmp, &off_val, K_NO_WAIT);
        k_msgq_put(&Q_on_hum, &off_val, K_NO_WAIT);
    }
    else{
        k_msgq_put(&Q_on_tmp, &on_val, K_NO_WAIT);
    }
}

//takes cmd params
void sample_cmd(const struct shell *shell, size_t argc, char *argv[]){
    int on_val = 1;
    int off_val = 0;
    if(argv[1][0]=='s'){
        if(argv[2][0]=='4'){
            k_msgq_put(&Q_on_mag, &on_val, K_NO_WAIT);
        }
        else if(strcmp(argv[2],"15")==0){
            k_msgq_put(&Q_on_mag, &off_val, K_NO_WAIT);
            k_msgq_put(&Q_on_tmp, &off_val, K_NO_WAIT);
            k_msgq_put(&Q_on_hum, &off_val, K_NO_WAIT);
            k_msgq_put(&Q_on_prs, &off_val, K_NO_WAIT);
            k_msgq_put(&Q_on_all, &on_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='0'){
            k_msgq_put(&Q_on_tmp, &on_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='1'){
            k_msgq_put(&Q_on_hum, &on_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='2'){
            k_msgq_put(&Q_on_prs, &on_val, K_NO_WAIT);
        }
        k_msgq_put(&Q_device, argv[2], K_NO_WAIT);
    }
    else if(argv[1][0]=='p'){
        if(argv[2][0]=='4'){
            k_msgq_put(&Q_on_mag, &off_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='2'){
            k_msgq_put(&Q_on_prs, &off_val, K_NO_WAIT);
        }
        else if(strcmp(argv[2],"15")==0){
            k_msgq_put(&Q_on_all, &off_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='0'){
            k_msgq_put(&Q_on_tmp, &off_val, K_NO_WAIT);
        }
        else if(argv[2][0]=='1'){
            k_msgq_put(&Q_on_hum, &off_val, K_NO_WAIT);
        }
        k_msgq_put(&Q_device, argv[2], K_NO_WAIT);
    }
    else if(argv[1][0]=='w'){
        uint32_t rate = (uint32_t)atoi(argv[2]);
        k_msgq_put(&Q_rate, &rate, K_NO_WAIT);
    }
}

void sample_mag(){
    struct json_struct mag_data = {
        .DID = 4, // Device ID for magnetometer
        .time = get_date_time_json(), // Function to get the current timestamp
        .value = get_mag(0) // Function to get magnetometer data
    };
    char json_output[SAMPLE_BUFFER_SIZE];
    int ret = json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &mag_data, json_output, sizeof(json_output));
    printf("%s\n", json_output);

}
void sample_tmp(){
    //sets up json object
    struct json_struct tmp_data = {
        .DID = 0, // Device ID for temperature sensor
        .time = get_date_time_json(), // Function to get the current timestamp
        .value = get_tmp(0) // Function to get temperature data
    };
    char json_output[SAMPLE_BUFFER_SIZE];
    int ret = json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &tmp_data, json_output, sizeof(json_output));
    printf("%s\n", json_output);
}
void sample_hum(){
    //sets up json object
    struct json_struct hum_data = {
        .DID = 1, // Device ID for humidity sensor
        .time = get_date_time_json(), // Function to get the current timestamp
        .value = get_hum(0) // Function to get humidity data
    };
    char json_output[SAMPLE_BUFFER_SIZE];
    int ret = json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &hum_data, json_output, sizeof(json_output));
    printf("%s\n", json_output);
}
void sample_prs(){
    //sets up json object
    struct json_struct prs_data = {
        .DID = 2, // Device ID for pressure sensor
        .time = get_date_time_json(), // Function to get the current timestamp
        .value = get_pressure(0) // Function to get pressure data
    };
    char json_output[SAMPLE_BUFFER_SIZE];
    int ret = json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &prs_data, json_output, sizeof(json_output));
    printf("%s\n", json_output);
}
void sample_all(){
        // Gets all indervidual sensor values
        char* hum_value = get_hum(0); 
        char* tmp_value = get_tmp(0); 
        char* prs_value = get_pressure(0); 
        char* mag_value = get_mag(0); 
        // Combine the values into a single string
        static char combined_value[SAMPLE_BUFFER_SIZE];
        snprintf(combined_value, sizeof(combined_value),"%s, %s, %s, %s",hum_value, tmp_value, prs_value, mag_value);

    //sets up json object
    struct json_struct prs_data = {
        .DID = 15, // Device ID for pressure sensor
        .time = get_date_time_json(), // Function to get the current timestamp
        .value = combined_value // Function to get pressure data
    };
    char json_output[SAMPLE_BUFFER_SIZE];
    int ret = json_obj_encode_buf(json_descr, ARRAY_SIZE(json_descr), &prs_data, json_output, sizeof(json_output));
    printf("%s\n", json_output);
}

//Task to sample data
void sample_task(){
    int on_mag = 0;
    int on_tmp = 0;
    int on_hum = 0;
    int on_prs = 0;
    int on_all = 0;
    int rate = 5;
    int device = 0;
    
    init_file_system(); //INITIALIZE FILE SYSTEM FOR TASK 5

    gpio_pin_configure_dt(&button, GPIO_INPUT); //configure as input
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE); //configure interrupt
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin)); //initialize callback
    gpio_add_callback(button.port, &button_cb_data); //add callback
    while(1){
        //takes params from cmd
        k_msgq_get(&Q_on_mag, &on_mag, K_MSEC(3));
        k_msgq_get(&Q_on_tmp, &on_tmp, K_MSEC(3));
        k_msgq_get(&Q_on_hum, &on_hum, K_MSEC(3));
        k_msgq_get(&Q_on_prs, &on_prs, K_MSEC(3));
        k_msgq_get(&Q_on_all, &on_all, K_MSEC(3));
        k_msgq_get(&Q_rate, &rate, K_MSEC(3));
        k_msgq_get(&Q_device, &device, K_MSEC(3));

        //runs samples of on sensors
        if(on_mag==1){
            on = 1;
            sample_mag();
        }
        if(on_tmp==1){
            on = 1;
            sample_tmp();
        }
        if(on_hum==1){
            on = 1;
            sample_hum();
        }
        if(on_prs==1){
            on = 1;
            sample_prs();
        }
        if(on_all==1){
            on = 1;
            sample_all();
        }
        if((on_mag || on_tmp || on_hum || on_prs || on_all)==0) {
            on = 0;
        }
        k_msleep(rate*1000);
    }

}
K_THREAD_DEFINE(sampleT, STACK_SIZE, sample_task, NULL, NULL, NULL, SAMPLE_PRIORITY, 0, 0); //starts task