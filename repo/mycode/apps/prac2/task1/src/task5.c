#include "task5.h"
#include "task2.h"
#include <zephyr/data/json.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h> // Include LittleFS header

#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>


#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_PRIORITY 6
#define STACK_SIZE 1024

#define MOUNT_POINT "/lfs" //little fs as mount point

//set to max size
#define MAX_PATH_LEN 255
#define TEST_FILE_SIZE 547

LOG_MODULE_REGISTER(task5, LOG_LEVEL_DBG);

static uint8_t file_test_pattern[TEST_FILE_SIZE];

void file_handler(const struct shell *shell, size_t argc, char *argv[]){
    if (argv[1][0] == '0'){
        write_to_file("tmp.txt");
    }
    else if (argv[1][0] == '1'){
        write_to_file("hum.txt");
    }
    else if (argv[1][0] == '2'){
        write_to_file("prs.txt");
    }
    else if (argv[1][0] == '4'){
        write_to_file("mag.txt");
    }
}


int write_to_file(const char *filename) {
    char filepath[MAX_PATH_LEN];
    char data[SAMPLE_BUFFER_SIZE];
    struct fs_file_t file;
    int ret;
    const char *retrieved;
    printf("writing to file %s\n", filename);

    // Construct the full file path
    snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, filename);

    // Retrieve the data
    if(strcmp(filename, "tmp.txt") == 0) {
        // Get temperature data
        retrieved = get_tmp(0);
    } else if (strcmp(filename, "hum.txt") == 0) {
        // Get humidity data
        retrieved = get_hum(0);
    } else if (strcmp(filename, "prs.txt") == 0) {
        // Get pressure data
        retrieved = get_pressure(0);
    } else if (strcmp(filename, "mag.txt") == 0) {
        // Get magnetometer data
        retrieved = get_mag(0);
    } else {
        printf("Unknown file type: %s\n", filename);
        return -1;
    }
    // Format the temperature data
    snprintf(data, sizeof(data), "%s", retrieved);

    // Initialize the file structure
    fs_file_t_init(&file);
    // Open the file for writing (create if it doesn't exist)
    ret = fs_open(&file, filepath, FS_O_CREATE | FS_O_WRITE);
    if (ret < 0) {
        printf("Error opening file %s [%d]\n", filepath, ret);
        return ret;
    }

    // Write the temperature data to the file
    ret = fs_write(&file, data, strlen(data));
    if (ret < 0) {
        printf("Error writing to file %s [%d]\n", filepath, ret);
        fs_close(&file);
        return ret;
    }
    
    printf("data written to %s\n", filepath);

    // Close the file
    fs_close(&file);

    return 0;
}

static int littlefs_flash_erase(unsigned int id)
{
	const struct flash_area *pfa;
	int rc;

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		LOG_ERR("FAIL: unable to find flash area %u: %d\n",
			id, rc);
		return rc;
	}

	LOG_PRINTK("Area %u at 0x%x on %s for %u bytes\n",
		   id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
		   (unsigned int)pfa->fa_size);

	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		rc = flash_area_flatten(pfa, 0, pfa->fa_size);
		LOG_ERR("Erasing flash area ... %d", rc);
	}

	flash_area_close(pfa);
	return rc;
}
static struct fs_littlefs littlefs_storage;
static void init_file_system(void) { //MUST BE CALLED FROM A TASK currently calling from task3
    static struct fs_mount_t my_fs_mount = {
        .type = FS_LITTLEFS,          // File system type
        .fs_data = &littlefs_storage,         // File system data
        .mnt_point = MOUNT_POINT,     // Mount point path
        .storage_dev = (void *)FIXED_PARTITION_ID(spi_storage_partition), // Storage partition
    };
    littlefs_flash_erase((uintptr_t)my_fs_mount.storage_dev);
    printf("attempting to mount file system\n");
    int ret = fs_mount(&my_fs_mount);
    printf("mounted file system\n");
    if (ret == 0) {
        printf("File system mounted at %s\n", MOUNT_POINT);
    } else {
        printf("Failed to mount file system: %d\n", ret);
    }
    printf("init complete\n");
}
