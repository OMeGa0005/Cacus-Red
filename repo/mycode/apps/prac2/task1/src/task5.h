#ifndef TASK5_H
#define TASK5_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>

#include <zephyr/storage/flash_map.h>

void file_handler(const struct shell *shell, size_t argc, char *argv[]);
static void init_file_system(void);
int write_to_file(const char *filename);
#endif 