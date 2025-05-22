#include <stdint.h>
#include <string.h>

#define MAX_NODE_NAME_LENGTH 50

typedef struct {
    char node_name[MAX_NODE_NAME_LENGTH];
    int8_t rssi;
} BluetoothNode;

void set_node_name(BluetoothNode *node, const char *name) {
    strncpy(node->node_name, name, MAX_NODE_NAME_LENGTH - 1);
    node->node_name[MAX_NODE_NAME_LENGTH - 1] = '\0';
}

const char* get_node_name(const BluetoothNode *node) {
    return node->node_name;
}

void set_rssi(BluetoothNode *node, int8_t rssi) {
    node->rssi = rssi;
}

int8_t get_rssi(const BluetoothNode *node) {
    return node->rssi;
}