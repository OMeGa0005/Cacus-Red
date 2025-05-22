/*
 * ibeacon commands
 * This file contains the implementation of iBeacon commands for managing iBeacon nodes.
 * It includes functions to add, remove, and view iBeacon nodes
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#include <zephyr/shell/shell.h>
#include <zephyr/sys/dlist.h>
#include <stdio.h>
#include <string.h>
#include "ibeacon_commands.h"
#include <stdbool.h>

// Define a maximum number of iBeacon nodes
#define MAX_IBEACON_NODES 10

// Data structure to store iBeacon node information
typedef struct {
    sys_dnode_t node;       // Doubly linked list node
    char ble_name[10];      // BLE Name (e.g., 4011-A)
    char ble_mac[18];       // BLE MAC Address (e.g., F5:75:FE:85:34:67)
    uint16_t major;         // Major number
    uint16_t minor;         // Minor number
    int x_coord;            // Fixed X coordinate
    int y_coord;            // Fixed Y coordinate
    char left_neighbour[10]; // Left neighbour BLE Name
    char right_neighbour[10]; // Right neighbour BLE Name
} iBeaconNode;

// Statically allocate memory for iBeacon nodes
static iBeaconNode ibeacon_nodes[MAX_IBEACON_NODES];
static bool ibeacon_node_used[MAX_IBEACON_NODES] = { false };

// Function to allocate a new iBeacon node
static iBeaconNode *allocate_ibeacon_node(void) {
    for (int i = 0; i < MAX_IBEACON_NODES; i++) {
        if (!ibeacon_node_used[i]) {
            ibeacon_node_used[i] = true;
            return &ibeacon_nodes[i];
        }
    }
    return NULL; // No available nodes
}

// Function to free an iBeacon node
static void free_ibeacon_node(iBeaconNode *node) {
    for (int i = 0; i < MAX_IBEACON_NODES; i++) {
        if (&ibeacon_nodes[i] == node) {
            ibeacon_node_used[i] = false;
            return;
        }
    }
}

// Doubly linked list to store iBeacon nodes
static sys_dlist_t ibeacon_list;

// Initialize the iBeacon list
void ibeacon_list_init(void) {
    sys_dlist_init(&ibeacon_list);
}

// Function to check if a node with a given BLE MAC address exists
bool ibeacon_node_exists(const char *ble_mac) {
    sys_dnode_t *node;

    SYS_DLIST_FOR_EACH_NODE(&ibeacon_list, node) {
        iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
        if (strcmp(ibeacon->ble_mac, ble_mac) == 0) {
            return true; // Node with the given BLE MAC address exists
        }
    }

    return false; // Node not found
}

// Function to get the BLE name for a given BLE MAC address
const char *get_ble_name_by_mac(const char *ble_mac) {
    sys_dnode_t *node;

    SYS_DLIST_FOR_EACH_NODE(&ibeacon_list, node) {
        iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
        if (strcmp(ibeacon->ble_mac, ble_mac) == 0) {
            return ibeacon->ble_name; // Return the BLE name if MAC matches
        }
    }

    return NULL; // Return NULL if no matching node is found
}

// Function to get the X and Y coordinates for a given BLE MAC address
bool get_coordinates_by_mac(const char *ble_mac, int *x_coord, int *y_coord) {
    sys_dnode_t *node;

    SYS_DLIST_FOR_EACH_NODE(&ibeacon_list, node) {
        iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
        if (strcmp(ibeacon->ble_mac, ble_mac) == 0) {
            *x_coord = ibeacon->x_coord; // Set the X coordinate
            *y_coord = ibeacon->y_coord; // Set the Y coordinate
            return true; // Coordinates found
        }
    }

    return false; // No matching node found
}

// Add an iBeacon node
int cmd_add_ibeacon(const struct shell *shell, size_t argc, char **argv) {
    iBeaconNode *new_node = allocate_ibeacon_node();
    if (!new_node) {
        shell_print(shell, "Error: No available memory for new iBeacon node.");
        return -1;
    }

    strncpy(new_node->ble_name, argv[1], sizeof(new_node->ble_name) - 1);
    strncpy(new_node->ble_mac, argv[2], sizeof(new_node->ble_mac) - 1);
    new_node->major = atoi(argv[3]);
    new_node->minor = atoi(argv[4]);
    new_node->x_coord = atoi(argv[5]);
    new_node->y_coord = atoi(argv[6]);
    strncpy(new_node->left_neighbour, argv[7], sizeof(new_node->left_neighbour) - 1);
    strncpy(new_node->right_neighbour, argv[8], sizeof(new_node->right_neighbour) - 1);

    sys_dlist_append(&ibeacon_list, &new_node->node);
    shell_print(shell, "iBeacon node added successfully.");
    return 0;
}

// Remove an iBeacon node
int cmd_remove_ibeacon(const struct shell *shell, size_t argc, char **argv) {
    sys_dnode_t *node, *next_node;
    SYS_DLIST_FOR_EACH_NODE_SAFE(&ibeacon_list, node, next_node) {
        iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
        if (strcmp(ibeacon->ble_name, argv[1]) == 0 || strcmp(ibeacon->ble_mac, argv[1]) == 0) {
            sys_dlist_remove(node);
            free_ibeacon_node(ibeacon);
            shell_print(shell, "iBeacon node removed successfully.");
            return 0;
        }
    }

    shell_print(shell, "Error: iBeacon node not found.");
    return -1;
}

// View iBeacon node details
int cmd_view_ibeacon(const struct shell *shell, size_t argc, char **argv) {
    sys_dnode_t *node; // Declare the node variable

    if (strcmp(argv[1], "-a") == 0) {
        shell_print(shell, "Listing all iBeacon nodes:");
        SYS_DLIST_FOR_EACH_NODE(&ibeacon_list, node) {
            iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
            shell_print(shell, "BLE Name: %s, BLE MAC: %s, Major: %d, Minor: %d, X: %d, Y: %d, Left: %s, Right: %s",
                        ibeacon->ble_name, ibeacon->ble_mac, ibeacon->major, ibeacon->minor,
                        ibeacon->x_coord, ibeacon->y_coord, ibeacon->left_neighbour, ibeacon->right_neighbour);
        }
    } else {
        SYS_DLIST_FOR_EACH_NODE(&ibeacon_list, node) {
            iBeaconNode *ibeacon = CONTAINER_OF(node, iBeaconNode, node);
            if (strcmp(ibeacon->ble_name, argv[1]) == 0 || strcmp(ibeacon->ble_mac, argv[1]) == 0) {
                shell_print(shell, "BLE Name: %s, BLE MAC: %s, Major: %d, Minor: %d, X: %d, Y: %d, Left: %s, Right: %s",
                            ibeacon->ble_name, ibeacon->ble_mac, ibeacon->major, ibeacon->minor,
                            ibeacon->x_coord, ibeacon->y_coord, ibeacon->left_neighbour, ibeacon->right_neighbour);
                return 0;
            }
        }
        shell_print(shell, "Error: iBeacon node not found.");
    }
    return 0;
}