/*
 * Kalman filter implementation for localisation using RSSI and ultrasonic data.
 * This code estimates the position of a node based on received signal strength
 * indicator (RSSI) values from multiple beacons and ultrasonic distance measurements.
 * It uses a Kalman filter to smooth the position estimates and multilateration to calculate position
 *
 * Author: Theodore Al-Shami, 48008932
 * Completed: 02/05/2025
 * BAUD Rate: 115200
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "bluetooth_node.h"
#include "bluetooth_sensor_set.h"
#include "ble_receive.h"
#include "kalman.h"

// Change grid height to 4.0 to cover y values ranging from 0 to 4.
#define NUM_NODES 8
#define GRID_WIDTH 3.0
#define GRID_HEIGHT 4.0
// #define RSSI_REF -10.0
// #define RSSI_N 12.0
// #define INITIAL_X 1.5
// #define INITIAL_Y 2.0
// #define PROCESS_ERROR 0.2
// #define MEASUREMENT_ERROR 5.0
// #define VELOCITY_WINDOW 10

// --------------------- Node Dictionary ---------------------
// (Definition of NodePosition, add_node_position, and get_node_position remain unchanged)
static NodePosition *node_positions = NULL; // head of linked list

void add_node_position(const char *name, double x, double y, int8_t rssi)
{
    NodePosition *current = node_positions;
    while (current)
    {
        if (strcmp(current->name, name) == 0)
        {
            // Found node; update its values.
            current->x = x;
            current->y = y;
            current->rssi = rssi;
            return;
        }
        current = current->next;
    }
    NodePosition *new_node = (NodePosition *)malloc(sizeof(NodePosition));
    if (!new_node)
    {
        fprintf(stderr, "Error: Memory allocation failed\n");
        return;
    }
    strncpy(new_node->name, name, sizeof(new_node->name) - 1);
    new_node->name[sizeof(new_node->name) - 1] = '\0';
    new_node->x = x;
    new_node->y = y;
    new_node->rssi = rssi;
    new_node->next = node_positions;
    node_positions = new_node;
}

NodePosition *get_node_position(const char *name)
{
    NodePosition *current = node_positions;
    while (current)
    {
        if (strcmp(current->name, name) == 0)
            return current;
        current = current->next;
    }
    return NULL;
}

// ------------------- RSSI-Based Localization -------------------
double rssi_to_distance(int8_t rssi)
{
    if (abs(rssi) < abs(RSSI_REF) + 3) {
        rssi = RSSI_REF - 3; // Clamp RSSI to the reference value with a small margin
    }
    double rssi_diff = RSSI_REF - rssi;
    double rssi_denom = 10.0 * RSSI_N;
    double exponent = rssi_diff / rssi_denom;
    return pow(10.0, exponent);
}

// -------------------- Multilateration -------------------
// This function estimates the position using multilateration based on RSSI values.
// It takes the coordinates of the beacons (xs, ys) and their distances (ds) as input.
int estimate_position_from_rssi(double *xs, double *ys, double *ds, int count, double *est_x, double *est_y)
{
    if (count < 3)
        return -1; // Need at least 3 beacons

    int m = count - 1;
    double M00 = 0.0, M01 = 0.0, M11 = 0.0;
    double V0 = 0.0, V1 = 0.0;
    for (int i = 1; i < count; i++)
    {
        double A = 2.0 * (xs[0] - xs[i]);
        double B = 2.0 * (ys[0] - ys[i]);
        double C = (ds[i] * ds[i] - ds[0] * ds[0]) - ((xs[i] * xs[i] - xs[0] * xs[0]) + (ys[i] * ys[i] - ys[0] * ys[0]));
        M00 += A * A;
        M01 += A * B;
        M11 += B * B;
        V0 += A * C;
        V1 += B * C;
    }
    double det = M00 * M11 - M01 * M01;
    if (fabs(det) < 1e-6)
        return -1; // Singular matrix
    *est_x = (M11 * V0 - M01 * V1) / det;
    *est_y = (M00 * V1 - M01 * V0) / det;
    return 0;
}

// ------------------- Ultrasonic Data -------------------
// For these test values, we want the computed ultrasonic observation to equal 4.0.
// Using the formula: obs_ultra_y = (GRID_HEIGHT - bottom_ultra_m + top_ultra_m) / 2,
// if we set ultrasonic1 = 0 (0 cm) and ultrasonic2 = 400 (400 cm = 4 m), then:
// obs_ultra_y = (4 - 0 + 4)/2 = 4.
uint16_t ultrasonic1 = 700;   // Top ultrasonic reading in cm
uint16_t ultrasonic2 = 700; // Bottom ultrasonic reading in cm

uint16_t get_ultrasonic1(void) { return ultrasonic1; }
void set_ultrasonic1(uint16_t value) { ultrasonic1 = value; }
uint16_t get_ultrasonic2(void) { return ultrasonic2; }
void set_ultrasonic2(uint16_t value) { ultrasonic2 = value; }

// ------------------- Kalman Filter Functions -------------------
void kalman_init(KalmanFilter *kf, double x_init, double y_init, double proc_err, double meas_err)
{
    kf->x_hat[0] = x_init;
    kf->x_hat[1] = y_init;
    kf->cov[0][0] = 1.0;
    kf->cov[0][1] = 0.0;
    kf->cov[1][0] = 0.0;
    kf->cov[1][1] = 1.0;
    kf->Q[0][0] = proc_err;
    kf->Q[0][1] = 0.0;
    kf->Q[1][0] = 0.0;
    kf->Q[1][1] = proc_err;
    kf->R = meas_err;

    // Node positions
    // const double node_positions[NUM_NODES][2] = {
    //     {0.0, 0.0}, // Node 0 = A
    //     {1.5, 0.0}, // Node 1 = B
    //     {2.0, 0.0}, // Node 2 = C
    //     {0.0, 2.0}, // Node 3 = H
    //     {3.0, 2.0}, // Node 4 = D
    //     {0.0, 4.0}, // Node 5 = G
    //     {1.5, 4.0}, // Node 6 = F
    //     {3.0, 4.0}  // Node 7 = E
    // };

    //! --- TEST NODE POSITIONS ---
    // The following eight nodes represent beacon nodes.
    // Adjusted RSSI values (in dBm) are now negative.
    // The strongest signal is "4011-E" at (3,4) with rssi = -40,
    // so its computed distance will be 10^(((-40)-(-40))/(20)) = 1 m.
    // add_node_position("4011-A", 0.0, 0.0, -90);
    // add_node_position("4011-B", 1.5, 0.0, -80);
    // add_node_position("4011-C", 2.0, 0.0, -75);
    // add_node_position("4011-H", 0.0, 2.0, -85);
    // add_node_position("4011-D", 3.0, 2.0, -65);
    // add_node_position("4011-G", 0.0, 4.0, -80);
    // add_node_position("4011-F", 1.5, 4.0, -70);
    // add_node_position("4011-E", 3.0, 4.0, -10);

    // add_node_position("4011-A", 0.0, 0.0, -10); // Now strongest signal at (0,0)
    // add_node_position("4011-B", 1.5, 0.0, -80);
    // add_node_position("4011-C", 2.0, 0.0, -75);
    // add_node_position("4011-H", 0.0, 2.0, -85);
    // add_node_position("4011-D", 3.0, 2.0, -65);
    // add_node_position("4011-G", 0.0, 4.0, -80);
    // add_node_position("4011-F", 1.5, 4.0, -70);
    // add_node_position("4011-E", 3.0, 4.0, -90); // Now the weakest signal at (3,4)

    // add_node_position("4011-A", 0.0, 0.0, -50); // equal, to center
    // add_node_position("4011-B", 1.5, 0.0, -50);
    // add_node_position("4011-C", 2.0, 0.0, -50);
    // add_node_position("4011-H", 0.0, 2.0, -50);
    // add_node_position("4011-D", 3.0, 2.0, -50);
    // add_node_position("4011-G", 0.0, 4.0, -50);
    // add_node_position("4011-F", 1.5, 4.0, -50);
    // add_node_position("4011-E", 3.0, 4.0, -50);

    // add_node_position("4011-A", 0.0, 0.0, -90);
    // add_node_position("4011-B", 1.5, 0.0, -90);
    // add_node_position("4011-C", 2.0, 0.0, -90);
    // add_node_position("4011-H", 0.0, 2.0, -90);
    // add_node_position("4011-D", 3.0, 2.0, -90);
    // add_node_position("4011-G", 0.0, 4.0, -90);
    // add_node_position("4011-F", 1.5, 4.0, -90);
    // add_node_position("4011-E", 3.0, 4.0, -10);
}

// ------------------- Kalman Filter Update -------------------
// This function updates the Kalman filter with new observations.
// It takes the Kalman filter structure and the observed x and y coordinates as input.
void kalman_update(KalmanFilter *kf, double obs_x, double obs_y)
{
    double x_hat_est[2] = {kf->x_hat[0], kf->x_hat[1]};
    double cov_est[2][2] = {
        {kf->cov[0][0] + kf->Q[0][0], kf->cov[0][1] + kf->Q[0][1]},
        {kf->cov[1][0] + kf->Q[1][0], kf->cov[1][1] + kf->Q[1][1]}};

    double error_x = obs_x - x_hat_est[0];
    double error_y = obs_y - x_hat_est[1];

    double Sx = cov_est[0][0] + kf->R;
    double Sy = cov_est[1][1] + kf->R;
    double Kx = cov_est[0][0] / Sx;
    double Ky = cov_est[1][1] / Sy;

    kf->x_hat[0] = x_hat_est[0] + Kx * error_x;
    kf->x_hat[1] = x_hat_est[1] + Ky * error_y;

    kf->cov[0][0] = (1.0 - Kx) * cov_est[0][0];
    kf->cov[1][1] = (1.0 - Ky) * cov_est[1][1];
    kf->cov[0][1] = kf->cov[1][0] = 0;
}

// ------------------- Kalman Thread -----------------------
// This function runs in a separate thread and continuously estimates the position using the Kalman filter.
void kalman_thread(void)
{
    KalmanFilter kf;
    kalman_init(&kf, INITIAL_X, INITIAL_Y, PROCESS_ERROR, MEASUREMENT_ERROR);

    double velocities[VELOCITY_WINDOW] = {0.0};
    int velocity_index = 0;
    double prev_x = kf.x_hat[0];
    double prev_y = kf.x_hat[1];
    uint64_t prev_time = 0;

    uint8_t coordinates[2] = {0, 0};
    double total_distance = 0.0; // Initialize total distance

    k_msleep(1000);

    while (1)
    {
        // Arrays to hold beacon data for multilateration.
        double xs[16], ys[16], ds[16];
        int beacon_count = 0;

        NodePosition *current = node_positions;
        char rssi_list[1024] = ""; // Buffer to store RSSI values as a list
        strcat(rssi_list, "[");   // Start the list

        while (current && beacon_count < 16)
        {
            if (current->rssi < 0)
            { // only include nodes with valid (negative) dBm values
                xs[beacon_count] = current->x;
                ys[beacon_count] = current->y;
                ds[beacon_count] = rssi_to_distance(current->rssi);

                // Append the RSSI value to the list
                char rssi_entry[128];
                snprintf(rssi_entry, sizeof(rssi_entry), "{\"node\":\"%s\",\"rssi\":%d}", current->name, current->rssi);
                strcat(rssi_list, rssi_entry);

                if (beacon_count < 15 && current->next != NULL)
                {
                    strcat(rssi_list, ","); // Add a comma if not the last entry
                }

                beacon_count++;
            }
            current = current->next;
        }

        strcat(rssi_list, "]"); // Close the list

        double pos_rssi_x = kf.x_hat[0];
        double pos_rssi_y = kf.x_hat[1];
        if (beacon_count >= 3)
        {
            if (estimate_position_from_rssi(xs, ys, ds, beacon_count, &pos_rssi_x, &pos_rssi_y) < 0)
            {
                // On failure, keep previous estimate.
            }
        }

        uint16_t top_ultrasonic_cm = get_ultrasonic1();
        uint16_t bottom_ultrasonic_cm = get_ultrasonic2();
        double top_ultra_m = top_ultrasonic_cm / 100.0;
        double bottom_ultra_m = bottom_ultrasonic_cm / 100.0;
        double obs_ultra_y = (GRID_HEIGHT - bottom_ultra_m + top_ultra_m) / 2.0;

        // Weights for RSSI and ultrasonic data
        double distance_from_center = fabs(pos_rssi_x - 1.5);
        double weight_ultra = (bottom_ultra_m < 5 && top_ultra_m < 5) ? 0.9 : 0.0;
        double weight_rssi = 1.0 - weight_ultra;

        double obs_x = pos_rssi_x;
        double obs_y = weight_ultra * obs_ultra_y + weight_rssi * pos_rssi_y;

        if (obs_x < 0.0) {
            obs_x = 0.0;
        }
        else if (obs_x > GRID_WIDTH)
        {
            obs_x = GRID_WIDTH;
        }

        if (obs_y < 0.0)
        {
            obs_y = 0.0;
        }
        else if (obs_y > GRID_HEIGHT)
        {
            obs_y = GRID_HEIGHT;
        }

        kalman_update(&kf, obs_x, obs_y);

        // Calculate velocity
        uint64_t current_time = k_uptime_get();
        double time_diff = (current_time - prev_time) / 1000.0; // Convert to seconds
        double dx = kf.x_hat[0] - prev_x;
        double dy = kf.x_hat[1] - prev_y;
        double distance = sqrt(dx * dx + dy * dy); // Distance traveled in this iteration
        total_distance += distance; // Accumulate total distance
        double velocity = distance / time_diff;

        if (prev_time == 0)
        {
            velocity = 0.0; // No previous time, so no velocity
            total_distance = 0.0; // Reset total distance
        }

        // Update velocity history
        velocities[velocity_index] = velocity;
        velocity_index = (velocity_index + 1) % VELOCITY_WINDOW;

        // Calculate average velocity
        double avg_velocity = 0.0;
        for (int i = 0; i < VELOCITY_WINDOW; i++)
        {
            avg_velocity += velocities[i];
        }
        avg_velocity /= VELOCITY_WINDOW;

        // Update previous position and time
        prev_x = kf.x_hat[0];
        prev_y = kf.x_hat[1];
        prev_time = current_time;

        // Print position, velocity, total distance, RSSI list, and ultrasonic readings
        printf("[%d] -> Pos: x=%.1f, y=%.1f; Vel: %.3f m/s; Total Dist: %.3f m; RSSI: %s; Ultrasonic: top = %.2f m, bottom = %.2f m\n", 
               velocity_index, kf.x_hat[0], kf.x_hat[1], avg_velocity, total_distance, rssi_list, top_ultra_m, bottom_ultra_m);
        // printf("Sending coordinates: %d, %d\n", coordinates[0], coordinates[1]);

        coordinates[0] = kf.x_hat[0] * 10;
        coordinates[1] = kf.x_hat[1] * 10;
        // Restart advertising with new coordinates
        restart_advertising(coordinates);

        k_msleep(250);
    }
}