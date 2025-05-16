# Cacus-Red
4011

Benjamin Redpath 48034865,
Theodore Al-Shami 48008932,
Caleb Clarke 47428364

# Project Description
The project's purpose is to develop a centrally controlled fire/smoke alarm system. This would be useful for multistory houses or public spaces, i.e. a hospital or office building where independently operating fire alarms would not suffice in warning the entire building of the imminent threat. A sensor node will collect both temperature and gas data. This will be transmitted to a base node which will use intelligent algorithms and data fusion to determine whether there may be a fire, this will then be passed to an online viewer as well as a handheld viewer device, this handheld device will also play an alarm when there is a fire and display the temperature when there is not. It will also be possible to add more viewer nodes which would work independently from each other just reading from the base node

# Project Block Diagram
![nodes block diagream](https://github.com/OMeGa0005/Cacus-Red/blob/f0d8078754da5a4b0f84b8656d2536053679cf7a/Nodes%20block%20diagram.png)

# DIKW Pyramid Abstraction
```
                       ▲
                      / \
                     /   \
                    /     \
                   /       \
                  /         \
                 /           \
                /    Data     \
               / Visualisation \
              / and Alarm Layer \
             /-------------------\
            /                     \
           /    Processing and     \
          /     Decision Layer      \
         / (Data Fusion and Kalman)  \
        /-----------------------------\
       /                               \
      /   Device Communication Layer    \
     /             (BLE MQTT)            \
    /-------------------------------------\
   /                                       \
  /          Data Collection Layer          \
 /    (using temperature and gas sensors)    \
/---------------------------------------------\
```

# System Integration
This page describes the integration of each component in the fire/smoke alarm system.\
The system is designed to detect potential fire hazards in a multistory building or public space and send alerts for the safety of occupants.

## 4.1 System Overview
There are several key components in the fire/smoke alarm system:
* **Sensor Nodes**: These are nodes that are distributed throughout the building and measure the temperature and gas values. They continuously monitor the environment and transmit their data to the base node via Bluetooth for processing.
* **Base Node**: This is a centrally located device that collects all the data from each sensor node. It uses data fusion and intelligent algorithms to analyse incoming data and detect potential fire hazards.
* **Handheld Viewer**: This is a portable device that displays the current temperature readings if there is no fire detected and sends an audible alarm with an accommodating warning message if a fire is detected.
* **Online Viewer**: A web-hosted dashboard that displays received data from the base node, providing real-time monitoring of the building and allowing for detailed locating of fire outbreaks.

![System Overview Diagram](https://raw.githubusercontent.com/OMeGa0005/Cacus-Red/refs/heads/main/System%20Overview%20Diagram.drawio.png)

## 4.2 System Communication
This is how data flows through the system:
1. **Data Collection**: Each of the sensor nodes measures the temperature and gas concentrations of their environment periodically.
2. **Bluetooth Transmission**: The sensor nodes transmit their collected data to the base node using Bluetooth Low Energy.
3. **Data Processing**: The base node receives and processes the data using data fusion and accurately detects potential fire hazards. If a fire is detected, then an alert is generated.
4. **Alert Messaging**: If the base node generates an alert for a fire, the alert is sent to the handheld viewer and online viewer. The handheld device triggers an alarm and the online viewer displays the alert.

![System Communication Diagram](https://raw.githubusercontent.com/OMeGa0005/Cacus-Red/refs/heads/main/System%20Communication%20Diagram.drawio.png)

## 4.3 Component Interaction
These are some key interactions between the components of the system:
* **Sensor Node to Base Node**: Bluetooth Low Energy is used to communicate between devices wirelessly, where each sensor node frequently transmits its environmental reading to the base node.
* **Base Node to Online Viewer**: A network connection (Wi-Fi or Ethernet) is used to send the real-time data from the base node to the online viewer.
* **Base Node to Handheld Viewer**: A Bluetooth connection is used to send information and alerts.

![System Interaction Diagram](https://raw.githubusercontent.com/OMeGa0005/Cacus-Red/refs/heads/main/System%20Interaction%20Diagram.drawio.png)

# Wireless Network Communications
For wireless communication between the Sensor Node and the base node BLE GATT will be used. For each communication procedure one device will be the server and one will be the receiver. This will suffice as there no two devices need back and forth communication to complete the desired task.  


For communicating between devices JSON formatting will be used.  


Firstly the sensor node must communicate the sensor values to the base node:
data  

{  
 "time": "double",  
"temperature": "float",  
 "CO2 reading": "integer"   //this is measured in ppm  
}  

Then for communication between the base node and the display node the following will be used:  
{  
 "temperature": "float",  
 "isFire": "int"     //a boolean  
}  


For communication with the Grafana display this will be done differently rather than using GATT a UART wired connection will be established, also using JSON  

{  
 "time": "double",  
 "temperature": "float",  
 "CO2 reading": "int",  
 "Likleyhood of fire": "int",  
 "isFire": "int"  
}  


![](https://github.com/OMeGa0005/Cacus-Red/blob/f070f0e5934a350ee4e4effe273a35aaccc80cff/netwrok%20diagram.png)

# Deliverables and Key Performance Criteria
1. Sensor Data Accuracy
What: The temperature and gas sensors must work simultaneously and each achieve a relative accuracy such that when processed by the base node a fire can be accurately detected, whilst also not causing a false alarm.


How to Measure: Temperature sensor reading can be compared to that of the room's air-condition temperature reading from the control unit, to know whether it and the gas reading are accurate enough, if the system can detect a fire and not set off a false alarm this must be true.

2. Data Fusion and Intelligent Decision Accuracy
What: The base node's use of the calman filter and data fusion from both the temp and gas sensor is correctly implemented such that it will ALWAYS detect a fire and set off no false alarm in the test.

How to Measure: Place the mobile node near a lighter and see if it can detect the fire, place the sensor inside a box with a lighter nearby for if it can detect the fire using the gas sensor, if no false alarm occurs yet it detects the fire this is a success. 

3. Communication and Alert Latency
What: The system must complete data transfer—from a sensor node’s reading to final part of chain(M5Core2) in less than 2 seconds, as time is very important during a fire, however the matter of milliseconds negligible to a human which is why the unit is in seconds.

How to Measure: This can be measured by eye as the units are in seconds by printing a debug log when the sensors are read and when they are received by the M5.

4. Online viewer
What: The online viewer should in a very user friendly manner show the temperature and gas readings history. 

How to Measure: The online viewer works as expected and requires no explaining for the tutor to read the graphs. 

5. Handheld viewer and speaker
What: The hand held viewer will show the temperature when no fire and display that there is a fire and play an alarm when there is.

How to Measure: If the correct temperature is shown when there is no fire, and it is very clear to the tutor when a fire is occuring, the viewer should always update when there is a fire, missing a fire would be a failure of this criteria. 
