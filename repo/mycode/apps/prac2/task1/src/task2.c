#include "task2.h"

#define RING_BUFFER_SIZE 128
#define DISPLAY_PRIORITY 7
#define STACK_SIZE 1024

//Gets nodes
const struct device *const mag = DEVICE_DT_GET(DT_ALIAS(mag));
const struct device *const env = DEVICE_DT_GET(DT_ALIAS(env));
const struct device *const prs = DEVICE_DT_GET(DT_ALIAS(prs));

RING_BUF_DECLARE(ringMag1, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringMag2, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringMag3, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringTmp, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringHum, RING_BUFFER_SIZE);
RING_BUF_DECLARE(ringPrs, RING_BUFFER_SIZE);





//Decides which sensor to return 
void sensor_main(const struct shell *shell, size_t argc, char *argv[]){
	if(strcmp(argv[1], "mag")==0){
		get_mag(1);
	}
	else if(strcmp(argv[1], "tmp")==0){
		get_tmp(1);
	}
	else if(strcmp(argv[1], "hum")==0){
		get_hum(1);
	}
	else if(strcmp(argv[1], "prs")==0){
		get_pressure(1);
	}
}

//Returns mag sensor
char* get_mag(int useRing){
	static char mag_value[64]; // Static buffer to hold the formatted string
	struct sensor_value x, y, z;
	uint8_t data1[sizeof(x)];
	uint8_t data2[sizeof(y)];
	uint8_t data3[sizeof(z)];
	//fetch sample from sensor
	int ret = sensor_sample_fetch(mag);
	if (ret) {
		printf("Sensor sample fetch error: %d\n", ret);
	 	return 1;
	}
	
	//retrieve values 
	ret = sensor_channel_get(mag, SENSOR_CHAN_MAGN_X, &x);
	ret |= sensor_channel_get(mag, SENSOR_CHAN_MAGN_Y, &y);
	ret |= sensor_channel_get(mag, SENSOR_CHAN_MAGN_Z, &z);
	memcpy(data1, &x, sizeof(x));  // Copy struct x into the data array
	memcpy(data2, &y, sizeof(y));  // Copy struct y into the data array
	memcpy(data3, &z, sizeof(z));  // Copy struct z into the data array
	if (ret) {
		printf("Sensor channel get error: %d\n", ret);
		k_sleep(K_MSEC(1000));
		return 1;
	}
	//print readings
	if(useRing == 1){
		if (ring_buf_put(&ringMag1, &data1, sizeof(data1)) == 0) {
		}
		if (ring_buf_put(&ringMag2, &data2, sizeof(data2)) == 0) {
		}
		if (ring_buf_put(&ringMag3, &data3, sizeof(data3)) == 0) {
		}
	}
	// Format the magnetometer values into a string
    snprintf(mag_value, sizeof(mag_value), "X: %d.%06d, Y: %d.%06d, Z: %d.%06d",
             x.val1, x.val2, y.val1, y.val2, z.val1, z.val2);
    return mag_value;	
}

char* get_tmp(int useRing) {
	static char tmp_value[32]; // Static buffer to hold the formatted string
    struct sensor_value temp;
    uint8_t data[sizeof(struct sensor_value)];  
    int ret = sensor_sample_fetch(env);

    if (ret) {
        printf("Error fetching temperature sample: %d\n", ret);
        return 1;
    }

    // Get temperature from the sensor
    ret = sensor_channel_get(env, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (ret) {
        printf("Error getting temperature channel: %d\n", ret);
        return 1;
    }

    // Copy the temperature value into the data buffer
    memcpy(data, &temp, sizeof(temp));

    // Write the data to the ring buffer (temperature)
	if(useRing == 1){
		if (ring_buf_put(&ringTmp, data, sizeof(data)) == 0) {
		}
	}	
    // Format the temperature value into a string
    snprintf(tmp_value, sizeof(tmp_value), "%d.%06d °C", temp.val1, temp.val2);
    return tmp_value;
}

char* get_hum(int useRing) {
	static char hum_value[32]; // Static buffer to hold the formatted string
    struct sensor_value hum;
	uint8_t data[sizeof(struct sensor_value)];
	//fetch new sample from sensor
    int ret = sensor_sample_fetch(env);

    if (ret) {
        printf("Error fetching humidity sample: %d\n", ret);
        return ret;
    }

    // Get the humidity data from the sensor
    ret = sensor_channel_get(env, SENSOR_CHAN_HUMIDITY, &hum);
    if (ret) {
        printf("Error getting humidity channel: %d\n", ret);
        return ret;
    }
	memcpy(data, &hum, sizeof(hum));
	//prints humidity
	if(useRing == 1){
		if (ring_buf_put(&ringHum, data, sizeof(data)) == 0) {
		}
	}
	// Format the humidity value into a string
    snprintf(hum_value, sizeof(hum_value), "%d.%06d %%", hum.val1, hum.val2);
    return hum_value;
}

char* get_pressure(int useRing) {
	static char prs_value[32]; // Static buffer to hold the formatted string
    struct sensor_value pressure;
	uint8_t data[sizeof(struct sensor_value)];
    int ret;

    // Fetch a new sample from the pressure sensor 
    ret = sensor_sample_fetch(prs);
    if (ret) {
        printf("Error fetching pressure sample: %d\n", ret);
        return 1;
    }

    // gets pressure from sensor
    ret = sensor_channel_get(prs, SENSOR_CHAN_PRESS, &pressure);
    if (ret) {
        printf("Error getting pressure channel: %d\n", ret);
        return 1;
    }
    pressure.val1 = pressure.val1 * 1000; // Convert integer part to Pa
    pressure.val2 = pressure.val2 * 1000; // Adjust fractional part
	memcpy(data, &pressure, sizeof(pressure));
	//prints pressure
	if(useRing == 1){
    	if (ring_buf_put(&ringPrs, data, sizeof(data)) == 0) {
    	}
	}
    // Format the pressure value into a string
    snprintf(prs_value, sizeof(prs_value), "%d.%06d Pa", pressure.val1, pressure.val2);
    return prs_value;
}

void display_task(int useRing) {
    uint8_t data[sizeof(struct sensor_value)];  // Space for one sensor_value
    struct sensor_value x, y, z, hum, pressure, temp;

    while (1) {
        // Read and display magnetometer data
		if (ring_buf_get(&ringMag3, data, sizeof(data)) != 0) {
            // Deserialize the data back into the sensor_value structs
            memcpy(&z, data, sizeof(z));  // Copy data into struct x
			if (ring_buf_get(&ringMag2, data, sizeof(data)) != 0) {
            	memcpy(&y, data, sizeof(y));  // Copy data into struct y3
				if (ring_buf_get(&ringMag1, data, sizeof(data)) != 0) {
            		memcpy(&x, data, sizeof(x));  // Copy data into struct z
				}
		}

            // Print the sensor readings
			printf("X: %d.%06d, Y: %d.%06d, Z: %d.%06d\n",
				x.val1, x.val2, y.val1, y.val2, z.val1, z.val2);
        }

        // Read and display temperature data
        if (ring_buf_get(&ringTmp, data, sizeof(data)) > 1) {
            memcpy(&temp, data, sizeof(temp));
            printf("%d.%06d °C\n", temp.val1, temp.val2);
        }

        // Read and display humidity data
        if (ring_buf_get(&ringHum, data, sizeof(data)) != 0) {
            memcpy(&hum, data, sizeof(hum));
            printf("%d.%06d %%\n", hum.val1, hum.val2);
        }

        // Read and display pressure data
        if (ring_buf_get(&ringPrs, data, sizeof(data)) != 0) {
            memcpy(&pressure, data, sizeof(pressure));
            printf("%d.%06d Pa\n", pressure.val1, pressure.val2);
        }

        k_msleep(50);
    }
}
K_THREAD_DEFINE(displayT, STACK_SIZE, display_task, NULL, NULL, NULL, DISPLAY_PRIORITY, 0, 0);