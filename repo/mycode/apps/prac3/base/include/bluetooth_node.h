#ifndef BLUETOOTH_NODE_H
#define BLUETOOTH_NODE_H

#include <stdint.h>

#define MAX_NODE_NAME_LENGTH 50

typedef struct {
    char node_name[MAX_NODE_NAME_LENGTH];
    int8_t rssi;
} BluetoothNode;

void set_node_name(BluetoothNode *node, const char *name);
const char* get_node_name(const BluetoothNode *node);
void set_rssi(BluetoothNode *node, int8_t rssi);
int8_t get_rssi(const BluetoothNode *node);

#endif // BLUETOOTH_NODE_H