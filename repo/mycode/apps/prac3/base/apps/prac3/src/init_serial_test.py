import socket
import sys
import serial
import json
from datetime import datetime
import time
import queue
from threading import Thread

#  * Author: Theodore Al-Shami, 48008932
#  * Completed: 02/05/2025
#  * BAUD Rate: 115200

# ----- Configuration -----
SERIAL_PORT = '/dev/ttyACM1'  # Update this if your device is on a different port
BAUD_RATE = 115200

def send_serial_test():
    print("Starting serial communication...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to serial port: {SERIAL_PORT} at {BAUD_RATE} baud")

        # add_node_position("4011-A", 0.0, 0.0, -90);
        # add_node_position("4011-B", 1.5, 0.0, -80);
        # add_node_position("4011-C", 2.0, 0.0, -75);
        # add_node_position("4011-H", 0.0, 2.0, -85);
        # add_node_position("4011-D", 3.0, 2.0, -65);
        # add_node_position("4011-G", 0.0, 4.0, -80);
        # add_node_position("4011-F", 1.5, 4.0, -70);
        # add_node_position("4011-E", 3.0, 4.0, -10);

        commands = ["remove 4011-A",
                    "remove 4011-B",
                    "remove 4011-C",
                    "remove 4011-D",
                    "remove 4011-E",
                    "remove 4011-F",
                    "remove 4011-G",
                    "remove 4011-H",
                    "remove 4011-I",
                    "remove 4011-J",
                    "remove 4011-K",
                    "remove 4011-L",
                    "remove 4011-M",
                    "add 4011-A F5:75:FE:85:34:67 2753 32998 0 0 0 4011-B",
                    "add 4011-B E5:73:87:06:1E:86 32975 20959 1.5 0 4011-A 4011-C",
                    "add 4011-C CA:99:9E:FD:98:B1 26679 40363 3 0 4011-B 0",
                    "add 4011-H CA:0C:E0:DB:CE:60 57395 28931 0 2 0 4011-D",
                    "add 4011-D CB:1B:89:82:FF:FE 41747 38800 3 2 4011-H 0",
                    "add 4011-G F1:04:48:06:39:A0 30525 30544 0 4 0 4011-F",
                    "add 4011-F C1:13:27:E9:B7:7C 6195 18394 1.5 4 4011-G 4011-E",
                    "add 4011-E D4:D2:A0:A4:5C:AC 30679 51963 3 4 4011-F 0"]

        # commands = ["sample s 15"]
        for cmd in commands:
            ser.write((cmd + '\n').encode('utf-8'))
            time.sleep(0.05)  # Short delay between commands

        print("Initialisation commands sent.")

        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                print(line)  # Print the raw data for debugging
            except Exception as e:
                print("Error reading from serial:", e)
                time.sleep(1)

    except Exception as e:
        print("Unable to open serial port:", e)
        sys.exit(1)

# ----- Main Program -----
if __name__ == "__main__":
    # Create a thread-safe queue for passing data from the serial thread to the update function.
    q = queue.Queue()

    # Start the background thread that reads the serial port.
    serial_thread = Thread(target=send_serial_test, daemon=True)
    serial_thread.start()
    while True:
        time.sleep(0.01)