from influxdb_client import InfluxDBClient, Point, WriteOptions
import serial
import re
import json

# Connect to InfluxDB
client = InfluxDBClient(
    url="http://localhost:8086",
    token="supersecrettoken",
    org="my-org"
)
write_api = client.write_api(write_options=WriteOptions(batch_size=1))

# Connect to base node (via UART)
ser = serial.Serial('/dev/ttyACM0', baudrate=115200)

print("Listening for beacon data...")

# Regex patterns
velocity_re = re.compile(r'Vel:\s*([\d.]+)\s*m/s')
distance_re = re.compile(r'Total Dist:\s*([\d.]+)\s*m')
rssi_re = re.compile(r'RSSI:\s*(\[.*?\])')

while True:
    line = ser.readline().decode('utf-8').strip()
    
    try:
        # Extract values from the line
        velocity_match = velocity_re.search(line)
        distance_match = distance_re.search(line)
        rssi_match = rssi_re.search(line)

        if not (velocity_match and distance_match and rssi_match):
            print("Incomplete data, skipping...")
            continue

        velocity = float(velocity_match.group(1))
        distance = float(distance_match.group(1))
        rssi_list = json.loads(rssi_match.group(1)) # List of {node, rssi}

        # Write the beacon metrics (Velocity and Distance)
        metrics_point = (
            Point("beacon_metrics")
            .field("velocity", velocity)
            .field("distance", distance)
        )
        write_api.write(bucket="beacon_data", record=metrics_point)

        # Write the RSSI values for each node
        for entry in rssi_list:
            node = entry["node"]
            rssi = entry["rssi"]
            rssi_point = (
                Point("beacon_rssi")
                .tag("node", node)
                .field("rssi", rssi)
            )
            write_api.write(bucket="beacon_data", record=rssi_point)

        print(f"Written: velocity={velocity}, distance={distance}, RSSI from {len(rssi_list)} nodes")

    except Exception as e:
        print(f"Error: {e}")