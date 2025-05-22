#include <zephyr/kernel.h>
#include <lvgl.h>
#include <stdio.h>
#include "display.h"
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>

static const struct device * display_dev;
static lv_obj_t *current_dot = NULL; // Keep track of the current red dot

void place_red_dot(float x, float y) {
    lv_obj_t *scr = lv_scr_act();

    // Define grid dimensions and unit size
    const int unit_size = 40; // Each grid cell is 40x40 pixels
    const int grid_width = 3 * unit_size;
    const int grid_height = 4 * unit_size;

    // Get screen dimensions
    int screen_width = lv_obj_get_width(scr);
    int screen_height = lv_obj_get_height(scr);

    // Calculate grid origin (bottom-left corner)
    int grid_start_x = (screen_width - grid_width) / 2;
    int grid_start_y = (screen_height - grid_height) / 2;

    // Calculate the position of the dot
    int dot_x = grid_start_x + x * unit_size; // Center the dot in the cell
    int dot_y = grid_start_y + y * unit_size; // Center the dot in the cell

    // Remove the old dot if it exists
    if (current_dot != NULL) {
        lv_obj_del(current_dot); // Delete the old dot
        current_dot = NULL;      // Reset the pointer
    }

    // Create the new red dot
    current_dot = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(current_dot); // Remove default styles
    lv_obj_set_size(current_dot, 10, 10); // Set dot size (10x10 pixels)
    lv_obj_set_style_bg_color(current_dot, lv_color_hex(0xFF0000), 0); // Set background color to red
    lv_obj_set_style_bg_opa(current_dot, LV_OPA_COVER, 0); // Ensure the background is opaque
    lv_obj_set_style_radius(current_dot, LV_RADIUS_CIRCLE, 0); // Make the dot circular
    lv_obj_set_pos(current_dot, dot_x - 5, dot_y - 5); // Position the dot (adjust for its size)
}

void init_display(void) {
    display_dev = device_get_binding(DT_NODE_FULL_NAME(DT_CHOSEN(zephyr_display)));
    lv_init();

    // Get the active screen
    lv_obj_t *scr = lv_scr_act();

    // Define grid dimensions
    const int unit_size = 40; // Each grid cell is 60x60 pixels
    const int grid_width = 3 * unit_size;
    const int grid_height = 4 * unit_size;

    int screen_width = lv_obj_get_width(scr);
    int screen_height = lv_obj_get_height(scr);
    
    int grid_start_x = (screen_width - grid_width) / 2;
    int grid_start_y = (screen_height - grid_height) / 2;

    // Draw vertical grid lines
    for (int i = 0; i <= 3; i++) { // 3 vertical lines for 3 columns
        lv_obj_t *line = lv_line_create(scr);
        static lv_point_t points[2];
        points[0].x = grid_start_x + i * unit_size;
        points[0].y = grid_start_y;
        points[1].x = grid_start_x + i * unit_size;
        points[1].y = grid_start_y + grid_height;

        lv_line_set_points(line, points, 2); // Set line points
        lv_obj_set_style_line_color(line, lv_color_hex(0x000000), 0); // Black color
        lv_obj_set_style_line_width(line, 2, 0); // Line width
    }

    // Draw horizontal grid lines
    for (int i = 0; i <= 4; i++) { // 4 horizontal lines for 4 rows
        lv_obj_t *line = lv_line_create(scr);
        static lv_point_t points[2];
        points[0].x = grid_start_x;
        points[0].y = grid_start_y + i * unit_size;
        points[1].x = grid_start_x + grid_width;
        points[1].y = grid_start_y + i * unit_size;

        lv_line_set_points(line, points, 2); // Set line points
        lv_obj_set_style_line_color(line, lv_color_hex(0x000000), 0); // Black color
        lv_obj_set_style_line_width(line, 2, 0); // Line width
    }

    // Add axis labels
    for (int i = 0; i <= 3; i++) { // X-axis labels
        lv_obj_t *label = lv_label_create(scr);
        char text[4];
        snprintf(text, sizeof(text), "%d", i);
        lv_label_set_text(label, text);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, grid_start_x + i * unit_size - 5, grid_start_y - 20);
    }

    for (int i = 0; i <= 4; i++) { // Y-axis labels
        lv_obj_t *label = lv_label_create(scr);
        char text[4];
        snprintf(text, sizeof(text), "%d", i);
        lv_label_set_text(label, text);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, grid_start_x - 30, grid_start_y + i * unit_size - 10);    }

    lv_obj_t *label = lv_label_create(scr);
    if (label == NULL) {
        printf("Failed to create label.\n");
        return;
    }
    lv_label_set_text(label, "Position of Mobile Node"); // Set the label text
    lv_obj_set_style_text_color(label, lv_color_hex(0x0000FF), 0); // Set text color to blue
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0); // Set font (optional)
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10); // Align the label at the top center with some offset

    display_blanking_off(display_dev);

}

// Scanning thread
void display_thread(void)
{
    uint8_t value = 0;
    init_display();

    printf("started\n");
    while(1){
        //place_red_dot(1,2);
        lv_timer_handler();
        //printf("running");
        //printf("Sending value: %d\n", value);
        //restart_advertising(value++);
        k_msleep(5);
        //printf("Advertising restarted with value: %d\n", value);
    }
    return 0;
}
