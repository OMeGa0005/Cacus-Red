# CSSE4011 Prac 3: iBeacon Multilateration and Kalman Filtering for Bluetooth Based Localisation

## Student Information

- **First Name:** Theodore  
- **Last Name:** Al-Shami  
- **Student ID:** 48008932
- **Partner:** B (using the **nrf52840dk** board)

## Overview

...

## iBeacon Shell Commands

### Add an iBeacon Node
- **Command**: `ibeacon add <UUID> <Major> <Minor> <RSSI>`
- **Description**: Adds a new iBeacon node with the specified UUID, Major, Minor, and RSSI values.

### Remove an iBeacon Node
- **Command**: `ibeacon remove <UUID>`
- **Description**: Removes an iBeacon node identified by its UUID.

### View iBeacon Node Details
- **Command**: `ibeacon view <UUID>`: Displays details of a specific iBeacon node.
- **Command**: `ibeacon view -a`: Displays details of all iBeacon nodes.

## Restrictions
- **ultrasonic sensors**: If an ultrasonic reading of more than 5 meters occurs, the sensor fusion will temporarily exclude ultrasonic data from multilateration

## Repository Structure

The repository is organised using the provided `mycode` template and contains a dedicated folder for this practical. The structure is as follows:

---

## Folder Structure

```
repo
|___mycode
    |___apps
    |   |___prac3
    |       |   CMakeLists.txt
    |       |   prj.conf
    |       |   README.md
    |       |   vnd,i2c-device.yaml
    |       |
    |       |___boards
    |       |   |   nrf52840dk_nrf52840.overlay
    |       |
    |       |___build
    |       |___src
    |           |   main.c
    |           |   init_serial_test.py
    |
    |___mylib
    |   |   rtc.c
    |   |   sensors.c
    |   |   si1133.c
    |   |   kalman.c
    |   |   ibeacon_commands.c
    |   |   bluetooth_sensor_set.c
    |   |   bluetooth_node.c
    |   |   ble_send.c
    |   |   ble_receive.c
    |
    |___include
        |   rtc.h
        |   sensors.h
        |   si1133.h
        |   kalman.h
        |   ibeacon_commands.h
        |   bluetooth_sensor_set.h
        |   bluetooth_node.h
        |   ble_send.h
        |   ble_receive.h
```
---

## References

- Zephyr Project Documentation [https://docs.zephyrproject.org/latest/]
- Took inspiration from example code provided by zephyr documentation.
- "Dynamic Fine-Grained Localization in Ad-Hoc Networks of Sensors" Andreas Savvides et al.
- Course slides and assignment instructions, as well as example code provided by the course

---

## Instructions Regarding Source Code

- **Building the Project:**  
  Navigate to repo directory  
   ```west build -b nrf52840dk/nrf52840 mycode/apps/prac3 --pristine```  
   ```west flash```


---

## User Instructions

Compile and flash the board. Once powered, the board will initialise the kalman filter with default multilateration values.
To begin multilateration, use the **iBeacon** commands mentioned previously to add the appropriate iBeacon nodes.
Once at least 3 iBeacons have been added and their sensor data has been received, the estimated positions will begin outputting
correct results.
