# references this script was co developed by Ben, Theo and Caleb which is why it looks
# famililar to the rest of the team
# This is Ben's version
 
import sys
import serial
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import time
import queue
import socket
from threading import Thread
import matplotlib.ticker as mticker

# ----- Configuration -----
SERIAL_PORT = '/dev/ttyACM0'  # Update this if your device is on a different port
BAUD_RATE = 115200


# BEN FROM HERE

# ----- Data Storage -----
ben_sensor_data = {
    0: {"x": [], "y": []},  # Temperature
    1: {"x": [], "y": []},  # Humidity
    2: {"x": [], "y": []},  # Pressure
    4: {"x": [], "y": {"X": [], "Y": [], "Z": []}},  # Magnetometer (X, Y, Z)
}

theo_sensor_data = {
    0: {"x": [], "y": []},
    1: {"x": [], "y": []},
    2: {"x": [], "y": []},
    5: {"x": [], "y0": [], "y1": []}
}

caleb_sensor_data = {
    0: {"x": [], "y": []},
    1: {"x": [], "y": []},
    2: {"x": [], "y": []},
    3: {"x": [], "y": []}
}

# ----- Global Variables -----
start_time = None
data_queue = queue.Queue()

# ----- Serial Reading Thread -----
def ben_read_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to serial port: {SERIAL_PORT} at {BAUD_RATE} baud")
    except Exception as e:
        print(f"Unable to open serial port: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    data = json.loads(line)
                    data_queue.put(data)
                except json.JSONDecodeError as e:
                    print(f"Error parsing JSON: {e}")
        except Exception as e:
            print(f"Error reading from serial: {e}")

# ----- Plot Update Function -----
def ben_update_plot(frame):
    global start_time

    # Process all data in the queue
    while not data_queue.empty():
        data = data_queue.get()
        sensor_id = data.get("DID")
        timestamp_str = data.get("time")
        value_str = data.get("value")

        try:
            # Parse the timestamp
            timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
            if start_time is None:
                start_time = timestamp
            x_val = (timestamp - start_time).total_seconds()

            # Parse the value
            if sensor_id == 4:  # Magnetometer
                try:
                    # Extract X, Y, Z values from the string
                    components = value_str.split(", ")
                    mag_values = {}
                    for component in components:
                        axis, value = component.split(": ")
                        value = value.replace("0.-", "-0.")  # Normalize invalid float format
                        mag_values[axis.strip()] = float(value.strip())

                    # Update the magnetometer data
                    ben_sensor_data[4]["x"].append(x_val)
                    ben_sensor_data[4]["y"]["X"].append(mag_values.get("X", 0.0))
                    ben_sensor_data[4]["y"]["Y"].append(mag_values.get("Y", 0.0))
                    ben_sensor_data[4]["y"]["Z"].append(mag_values.get("Z", 0.0))

                    # Keep only the last 100 data points
                    if len(ben_sensor_data[4]["x"]) > 100:
                        ben_sensor_data[4]["x"].pop(0)
                        ben_sensor_data[4]["y"]["X"].pop(0)
                        ben_sensor_data[4]["y"]["Y"].pop(0)
                        ben_sensor_data[4]["y"]["Z"].pop(0)
                except Exception as e:
                    print(f"Error processing magnetometer data: {e}")
                    print(f"Raw magnetometer value_str: {value_str}")
            elif sensor_id == 15:  # Combined data
                try:
                    # Normalize invalid float format
                    value_str = value_str.replace("0.-", "-0.")  # Fix invalid float formats like 0.-101578

                    # Split the combined value string into individual sensor values
                    components = value_str.split(", ")

                    # Process Humidity
                    humidity = float(components[0].split()[0])  # Extract numeric value
                    ben_sensor_data[1]["x"].append(x_val)
                    ben_sensor_data[1]["y"].append(round(humidity, 2))

                    # Process Temperature
                    temperature = float(components[1].split()[0])  # Extract numeric value
                    ben_sensor_data[0]["x"].append(x_val)
                    ben_sensor_data[0]["y"].append(round(temperature, 2))

                    # Process Pressure
                    pressure = float(components[2].split()[0])  # Extract numeric value
                    ben_sensor_data[2]["x"].append(x_val)
                    ben_sensor_data[2]["y"].append(round(pressure, 2))

                    # Process Magnetometer (X, Y, Z)
                    mag_values = {"X": 0.0, "Y": 0.0, "Z": 0.0}
                    for component in components[3:]:
                        axis, value = component.split(": ")
                        axis = axis.strip()
                        value = float(value.strip())
                        mag_values[axis] = round(value, 2)

                    ben_sensor_data[4]["x"].append(x_val)
                    ben_sensor_data[4]["y"]["X"].append(mag_values["X"])
                    ben_sensor_data[4]["y"]["Y"].append(mag_values["Y"])
                    ben_sensor_data[4]["y"]["Z"].append(mag_values["Z"])

                    # Ensure all magnetometer axes have the same length as x
                    for axis in ["X", "Y", "Z"]:
                        while len(ben_sensor_data[4]["y"].get(axis, [])) < len(ben_sensor_data[4]["x"]):
                            ben_sensor_data[4]["y"].setdefault(axis, []).append(0.0)

                    # Keep only the last 100 data points for each sensor
                    for sid in [0, 1, 2]:
                        if len(ben_sensor_data[sid]["x"]) > 100:
                            ben_sensor_data[sid]["x"].pop(0)
                            ben_sensor_data[sid]["y"].pop(0)
                    for axis in ["X", "Y", "Z"]:
                        if len(ben_sensor_data[4]["x"]) > 100:
                            ben_sensor_data[4]["x"].pop(0)
                            ben_sensor_data[4]["y"][axis].pop(0)
                except Exception as e:
                    print(f"Error processing combined sensor data: {e}")
                    print(f"Raw combined value_str: {value_str}")
            else:
                # For other sensors, extract the numeric value
                value = float(value_str.split()[0])  # Extract numeric value
                ben_sensor_data[sensor_id]["x"].append(x_val)
                ben_sensor_data[sensor_id]["y"].append(value)

                # Keep only the last 100 data points
                if len(ben_sensor_data[sensor_id]["x"]) > 100:
                    ben_sensor_data[sensor_id]["x"].pop(0)
                    ben_sensor_data[sensor_id]["y"].pop(0)

        except Exception as e:
            print(f"Error processing data: {e}")

    # Clear and update each subplot
    ax0.clear()
    ax1.clear()
    ax2.clear()
    ax3.clear()

    # Plot Temperature (DID = 0)
    if ben_sensor_data[0]["x"]:
        ax0.plot(ben_sensor_data[0]["x"], ben_sensor_data[0]["y"], label="Temperature (째C)", color="red")
        ax0.set_title("Temperature")
        ax0.set_xlabel("Time (s)")
        ax0.set_ylabel("째C")
        ax0.legend()

    # Plot Humidity (DID = 1)
    if ben_sensor_data[1]["x"]:
        ax1.plot(ben_sensor_data[1]["x"], ben_sensor_data[1]["y"], label="Humidity (%)", color="blue")
        ax1.set_title("Humidity")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("%")
        ax1.legend()

    # Plot Pressure (DID = 2)
    if ben_sensor_data[2]["x"]:
        ax2.plot(ben_sensor_data[2]["x"], ben_sensor_data[2]["y"], label="Pressure (Pa)", color="green")
        ax2.set_title("Pressure")
        ax2.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.2f'))
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Pa")
        ax2.legend()

    # Plot Magnetometer (DID = 4)
    if ben_sensor_data[4]["x"]:
        ax3.plot(ben_sensor_data[4]["x"], ben_sensor_data[4]["y"]["X"], label="Magnetometer X", color="purple")
        ax3.plot(ben_sensor_data[4]["x"], ben_sensor_data[4]["y"]["Y"], label="Magnetometer Y", color="orange")
        ax3.plot(ben_sensor_data[4]["x"], ben_sensor_data[4]["y"]["Z"], label="Magnetometer Z", color="cyan")
        ax3.set_title("Magnetometer")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Value")
        ax3.legend()

    plt.tight_layout()


#THEO FROM HERE

def theo_read_serial():
    """
    This function continuously reads from the serial port and pushes parsed JSON data
    into a thread-safe queue.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to serial port: {SERIAL_PORT} at {BAUD_RATE} baud")

        # commands = ["sample s 0", "sample s 1", "sample s 2", "sample s 5"]
        commands = ["sample s 15"]
        for cmd in commands:
            ser.write((cmd + '\n').encode('utf-8'))
            time.sleep(0.2)  # Short delay between commands

        print("Initialization commands sent.")

    except Exception as e:
        print("Unable to open serial port:", e)
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:  # only process non-empty lines
                try:
                    data = json.loads(line)
                    q.put(data)
                except Exception as e:
                    print("Error parsing JSON:", e)
        except Exception as e:
            print("Error reading from serial:", e)
            time.sleep(1)

# ----- Plot Update Function -----
def theo_update_plot(frame):
    """
    Processes new data in the queue and updates the sensor plots.
    """
    global start_time, ax0, ax1, ax2, ax3, theo_sensor_data, q

    while not q.empty():
        data = q.get()
        sensor_id = data.get("DID")
        sensor_time_str = data.get("time")

        try:
            sensor_time = datetime.strptime(sensor_time_str, "%Y-%m-%d %H:%M:%S")
        except Exception:
            sensor_time = datetime.now()

        if start_time is None:
            start_time = sensor_time

        x_val = (sensor_time - start_time).total_seconds()

        if sensor_id in [0, 1, 2]:
            try:
                value = float(data.get("value"))
            except Exception as e:
                print("Error converting value to float:", e)
                continue

            if ((sensor_id == 0 or sensor_id == 2) and value == 0):
                print(f"Sensor {sensor_id} value is invalid, skipping...")
                continue

            theo_sensor_data[sensor_id]["x"].append(x_val)
            theo_sensor_data[sensor_id]["y"].append(value)

            print(f"Converted sensor {sensor_id} value: {value}")
        elif sensor_id == 5:
            value_list = data.get("value")
            if isinstance(value_list, list) and len(value_list) == 2:
                theo_sensor_data[5]["x"].append(x_val)
                theo_sensor_data[5]["y0"].append(value_list[0])
                theo_sensor_data[5]["y1"].append(value_list[1])

    # ---- Update the plots ----
    # Sensor 0
    ax0.clear()
    ax0.plot(theo_sensor_data[0]["x"], theo_sensor_data[0]["y"], label="Sensor 0", color='blue')
    ax0.legend(loc="upper right")
    ax0.set_title("Sensor 0 (Temperature 째C)")
    ax0.set_xlabel("Time (s)")
    ax0.set_ylabel("Temperature (째C)")
    ax0.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.2f'))
    # ax0.set_ylim(20, 40)  # Set y-axis range for Sensor 0

    # Sensor 1
    ax1.clear()
    ax1.plot(theo_sensor_data[1]["x"], theo_sensor_data[1]["y"], label="Sensor 1", color='red')
    ax1.legend(loc="upper right")
    ax1.set_title("Sensor 1 (Humidity %)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Humidity (%)")
    ax1.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.2f'))
    # ax1.set_ylim(0, 100)  # Set y-axis range for Sensor 1

    # Sensor 2
    ax2.clear()
    ax2.plot(theo_sensor_data[2]["x"], theo_sensor_data[2]["y"], label="Sensor 2", color='green')
    ax2.legend(loc="upper right")
    ax2.set_title("Sensor 2 (Pressure hPa)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Pressure (hPa)")
    ax2.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.2f'))
    # ax2.set_ylim(950, 1100)  # Set y-axis range for Sensor 2

    # Sensor 5 (adaptive y-axis)
    ax3.clear()
    ax3.plot(theo_sensor_data[5]["x"], theo_sensor_data[5]["y0"], label="Sensor 5 - Channel 0", color='purple')
    ax3.plot(theo_sensor_data[5]["x"], theo_sensor_data[5]["y1"], label="Sensor 5 - Channel 1", color='orange')
    ax3.legend(loc="upper right")
    ax3.set_title("Sensor 5 (Light and UV)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Light and UV (lux)")
    ax3.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.2f'))
    # No fixed ylim for Sensor 5, keeping it adaptive

#CALEB FROM HERE

def caleb_read_serial():
    try:
        rtt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rtt.connect((HOST, PORT))
        print(f"Connected to J-Link RTT Server at {HOST}:{PORT}")
    except Exception as e:
        print("Unable to connect to RTT socket:", e)
        return
    
    rtt.sendall("p2 sample s 15\n".encode())
    time.sleep(1)
    _ = rtt.recv(1024)
    
    buffer = ""
    try:
        while True:
            data = rtt.recv(1024)
            if not data:
                break

            buffer += data.decode()

            while True:
                start_idx = buffer.find('{')
                end_idx = buffer.find('}', start_idx) + 1

                if start_idx != -1 and end_idx != -1:
                    json_str = buffer[start_idx:end_idx]
                    buffer = buffer[end_idx:]

                    try:
                        json_data = json.loads(json_str)
                        data_queue.put(json_data)
                    except json.JSONDecodeError as e:
                        print("Failed to decode JSON:", json_str)
                else:
                    break
    finally:
        rtt.close()

# ----- Plot Update Function -----
def caleb_update_plot(frame):
    global start_time

    # Process all data in the queue
    while not data_queue.empty():
        data = data_queue.get()
        sensor_id = data.get("did")
        timestamp_str = data.get("time")
        value_str = data.get("values")

        try:
            x_val = timestamp_str

            # Parse the value
            if sensor_id == 15:  # Gas
                values = [float(value_str[0]), float(value_str[1]), float(value_str[2]), value_str[3]]

                for did in [0, 1, 2, 3]:
                    caleb_sensor_data[did]["x"].append(x_val)
                    caleb_sensor_data[did]["y"].append(values[did])

                    if len(caleb_sensor_data[did]["x"]) > 100:
                        caleb_sensor_data[did]["x"].pop(0)
                        caleb_sensor_data[did]["y"].pop(0)
            else:
                if sensor_id == 3:
                    value = value_str[0]
                else:
                    # For other sensors, extract the numeric value
                    value = float(value_str[0])  # Extract numeric value
                caleb_sensor_data[sensor_id]["x"].append(x_val)
                caleb_sensor_data[sensor_id]["y"].append(value)

                # Keep only the last 100 data points
                if len(caleb_sensor_data[sensor_id]["x"]) > 100:
                    caleb_sensor_data[sensor_id]["x"].pop(0)
                    caleb_sensor_data[sensor_id]["y"].pop(0)

        except Exception as e:
            print(f"Error processing data: {e}")

    # Clear and update each subplot
    ax0.clear()
    ax1.clear()
    ax2.clear()
    ax3.clear()

    # Plot Temperature (DID = 0)
    if caleb_sensor_data[0]["x"]:
        ax0.plot(caleb_sensor_data[0]["x"], caleb_sensor_data[0]["y"], label="Temperature (C)", color="red")
        ax0.set_title("Temperature")
        ax0.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.1f'))
        ax0.set_xlabel("Time (s)")
        ax0.set_ylabel("C")
        ax0.legend()

    # Plot Humidity (DID = 1)
    if caleb_sensor_data[1]["x"]:
        ax1.plot(caleb_sensor_data[1]["x"], caleb_sensor_data[1]["y"], label="Humidity (%)", color="blue")
        ax1.set_title("Humidity")
        ax1.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.1f'))
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("%")
        ax1.legend()

    # Plot Pressure (DID = 2)
    if caleb_sensor_data[2]["x"]:
        ax2.plot(caleb_sensor_data[2]["x"], caleb_sensor_data[2]["y"], label="Pressure (Pa)", color="green")
        ax2.set_title("Pressure")
        ax2.yaxis.set_major_formatter(mticker.FormatStrFormatter('%.1f'))
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Pa")
        ax2.legend()

    # Plot Magnetometer (DID = 3)
    if caleb_sensor_data[3]["x"]:
        ax3.plot(caleb_sensor_data[3]["x"], caleb_sensor_data[3]["y"], label="Gas (ppb)", color="purple")
        ax3.set_title("Gas")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("ppb")
        ax3.legend()

    plt.tight_layout()

# ----- Main Program -----
if __name__ == "__main__":
    person = input("who's board is it: ")
    if (person == "ben"):
        # Start the serial reading thread
        serial_thread = Thread(target=ben_read_serial, daemon=True)
        serial_thread.start()

        # Set up the Matplotlib figure and subplots
        fig, ((ax0, ax1), (ax2, ax3)) = plt.subplots(2, 2, figsize=(12, 8))

        # Create an animation that updates the plots
        ani = FuncAnimation(fig, ben_update_plot, interval=1000)

        # Show the plot
        plt.show()
    elif (person == "theo"):
            # Create a thread-safe queue for passing data from the serial thread to the update function.
            q = queue.Queue()

            # Start the background thread that reads the serial port.
            serial_thread = Thread(target=theo_read_serial, daemon=True)
            serial_thread.start()

            # Set up four subplots in a 2x2 grid.
            fig, ((ax0, ax1), (ax2, ax3)) = plt.subplots(2, 2, figsize=(15, 10))
            fig.canvas.manager.set_window_title("Sensor Data (SLTB004A)")

            # Create an animation that updates every 100 milliseconds.
            ani = FuncAnimation(fig, theo_update_plot, interval=100)

            # plt.tight_layout()
            plt.show()
    elif (person == "caleb"):
        # Create a thread-safe queue for passing data from the serial thread to the update function.
        HOST = 'localhost'
        PORT = 19021
        
        # Start the serial reading thread
        serial_thread = Thread(target=caleb_read_serial, daemon=True)
        serial_thread.start()

        # Set up the Matplotlib figure and subplots
        fig, ((ax0, ax1), (ax2, ax3)) = plt.subplots(2, 2, figsize=(12, 8))

        # Create an animation that updates the plots
        ani = FuncAnimation(fig, caleb_update_plot, interval=1000)

        # Show the plot
        plt.show()