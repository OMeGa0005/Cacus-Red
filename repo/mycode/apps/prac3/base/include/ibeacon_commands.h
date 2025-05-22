/*
 * ibeacon commands header file
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#ifndef IBEACON_COMMANDS_H
#define IBEACON_COMMANDS_H

#include <zephyr/shell/shell.h>

// Function prototypes for iBeacon commands
int cmd_add_ibeacon(const struct shell *shell, size_t argc, char **argv);
int cmd_remove_ibeacon(const struct shell *shell, size_t argc, char **argv);
int cmd_view_ibeacon(const struct shell *shell, size_t argc, char **argv);
bool ibeacon_node_exists(const char *ble_mac);
const char *get_ble_name_by_mac(const char *ble_mac);
bool get_coordinates_by_mac(const char *ble_mac, int *x_coord, int *y_coord);

#endif // IBEACON_COMMANDS_H