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

#ifndef KALMAN_H
#define KALMAN_H

#include "bluetooth_node.h"

// Kalman filter structure
typedef struct {
    double x_hat[2];      // State estimate (x, y)
    double cov[2][2];     // Covariance matrix
    double Q[2][2];       // Process noise covariance
    double R;             // Measurement noise covariance
} KalmanFilter;

typedef struct NodePosition
{
    char name[32];             // Node name (e.g., "4011-A")
    double x, y;               // Coordinates
    int8_t rssi;               // RSSI value
    struct NodePosition *next; // Pointer to the next node
} NodePosition;

//! Constants
//! These constants are used for the Kalman filter and RSSI to distance conversion
//! They can be adjusted based on the specific application and environment.
#define VELOCITY_WINDOW 3       // Number of iterations to average velocity
#define PROCESS_ERROR 0.1       // Process error for Kalman filter
#define MEASUREMENT_ERROR 3.5   // Measurement error for Kalman filter
#define INITIAL_X 1.5           // Initial x-coordinate for Kalman filter
#define INITIAL_Y 2.0           // Initial y-coordinate for Kalman filter
#define RSSI_REF -60.0          // Reference RSSI value (measured approx -50 dBm at 1 meter)
#define RSSI_N 2.0              // Path loss exponent for RSSI to distance conversion
//! End of constants

void add_node_position(const char *name, double x, double y, int8_t rssi);
NodePosition *get_node_position(const char *name);

uint16_t get_ultrasonic1(void);
void set_ultrasonic1(uint16_t value);
uint16_t get_ultrasonic2(void);
void set_ultrasonic2(uint16_t value);

// Function declarations
void kalman_init(KalmanFilter *kf, double x_init, double y_init, double proc_err, double meas_err);
void kalman_update(KalmanFilter *kf, double obs_x, double obs_y);
double calculate_distance(double cP_tx, double P_recv, double alpha);
void triangulate_position(BluetoothNode *nodes, int num_nodes, double cP_tx, double alpha, KalmanFilter *kf, double *result_x, double *result_y);
void estimate_position(BluetoothNode *nodes, int num_nodes, double cP_tx, double alpha, double *result_x, double *result_y);
void kalman_thread(void);

#endif // KALMAN_H