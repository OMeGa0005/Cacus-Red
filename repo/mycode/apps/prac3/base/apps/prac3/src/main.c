/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 * 
 * Main application file for iBeacon project.
 * This file contains the main function and initialises the iBeacon
 * commands and the BLE advertising.
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <time.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/device.h>
#include <string.h>
#include "ble_send.h"
#include "ble_receive.h"
#include "ibeacon_commands.h"
#include "kalman.h"

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/* 2000 msec = 2 sec */
#define SLEEP_TIME_MS 2000
#define STACK_SIZE 2048

#define MSGQ_SIZE 3
#define MSGQ_ALIGNMENT 4 // Each message is 32 bits => 32/8 = 4

#define SHELL_PRIORITY 7
#define SENSORS_PRIORITY 6

// Message queue for multithreading
K_MSGQ_DEFINE(my_msgq, sizeof(uint32_t), MSGQ_SIZE, MSGQ_ALIGNMENT);

// We will register logging to the "main" module
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// LOG_INF("Information log");
// LOG_WRN("Warning log");
// LOG_ERR("Error log");
// LOG_DBG("Debug log");


/*
 * @brief Main, duh
 */
void main(void)
{
	uint8_t coordinates[2] = {0, 0};
	if (init_ble())
	{
		return 1;
	}

	ibeacon_list_init();

	// printf("Advertising started\n");
	// while (1)
	// {
	// 	printf("Sending coordinates: %d, %d\n", coordinates[0], coordinates[1]);
	// 	restart_advertising(coordinates);
	// 	k_msleep(5000);
	// 	printf("Advertising restarted with coordinates: %d, %d\n", coordinates[0], coordinates[1]);
	// }

	// Create iBeacon command set
	SHELL_STATIC_SUBCMD_SET_CREATE(
		ibeacon_cmds,
		SHELL_CMD_ARG(add, NULL,
					  "Add an iBeacon node. Usage: add <BLE Name> <BLE MAC> <Major> <Minor> <X> <Y> <Left Neighbour> <Right Neighbour>",
					  cmd_add_ibeacon, 9, 0),
		SHELL_CMD_ARG(remove, NULL,
					  "Remove an iBeacon node. Usage: remove <BLE Name|BLE MAC>",
					  cmd_remove_ibeacon, 2, 0),
		SHELL_CMD_ARG(view, NULL,
					  "View iBeacon node details. Usage: view <BLE Name|BLE MAC> or view -a (to view all nodes)",
					  cmd_view_ibeacon, 2, 0),
		SHELL_SUBCMD_SET_END);

	// Register the iBeacon commands
	SHELL_CMD_REGISTER(add, &ibeacon_cmds, "Add an iBeacon node", cmd_add_ibeacon);
	SHELL_CMD_REGISTER(remove, &ibeacon_cmds, "Remove an iBeacon node", cmd_remove_ibeacon);
	SHELL_CMD_REGISTER(view, &ibeacon_cmds, "View iBeacon node details", cmd_view_ibeacon);

	LOG_INF("\r\nCommands:\r\n"
			"\tibeacon: Manage iBeacon nodes\n\r"
			"\t  add <BLE Name> <BLE MAC> <Major> <Minor> <X> <Y> <Left Neighbour> <Right Neighbour>	: Add an iBeacon node\n\r"
			"\t  remove <BLE Name|BLE MAC>	: Remove an iBeacon node\n\r"
			"\t  view <BLE Name|BLE MAC> or view -a		: View iBeacon node details or all nodes\n\r");

	return;
}


K_THREAD_DEFINE(main_thread, STACK_SIZE * 3, main, NULL, NULL, NULL, SHELL_PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_scan_thread, STACK_SIZE * 3, scan_thread, NULL, NULL, NULL, SENSORS_PRIORITY, 0, 0);
K_THREAD_DEFINE(kalman_math_thread, STACK_SIZE * 3, kalman_thread, NULL, NULL, NULL, SENSORS_PRIORITY, 0, 0);
