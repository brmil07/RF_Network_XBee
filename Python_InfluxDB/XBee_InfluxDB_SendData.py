from datetime import datetime
from digi.xbee.devices import *

import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# Initialize influxDB configurations
org = "FH"
url = "http://127.0.0.1:8086"
token = "Wfwp8kjSQpSaGsP2ZqXB7rEqmsjtGfaj-8nr6LQzIWmr8RbWcz4S_xBbB_484MTjmM0Y8F38WGIG4ve_1ubB1g=="
client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
bucket = "fh_bucket"
write_api = client.write_api(write_options=SYNCHRONOUS)

# Initialize XBEE Device
device = XBeeDevice("COM12", 115200)
# remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A200418E85F0"))
# remote = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20041BDFF8D"))

# Initialize variables
device_id = []

time.sleep(0.01)

# Open serial device
device.open()

def send_data():
    remote_num = xbee_message.remote_device
    data = xbee_message.data
    data = data.decode('utf-8')
    is_broadcast = xbee_message.is_broadcast
    timestamp = xbee_message.timestamp
    timestamp = datetime.utcfromtimestamp(timestamp)

    print(remote_num)
    print(data)
    print(is_broadcast)
    print(timestamp, "\n")

    device_id = str(remote_num)

    # Split the data into individual sensor readings
    sensor_readings = data.split(',')

    # Create a dictionary to store the parsed values
    parsed_values = {}

    # Iterate through each sensor reading and extract the values
    for reading in sensor_readings:
        # Split each reading into sensor type and value
        sensor_type, value = reading.split(':')

        # Convert the value to float and store it in the dictionary
        parsed_values[sensor_type] = float(value)

    # Print the parsed values
    light = parsed_values.get("Light")
    temp = parsed_values.get("Temperature")
    altitude = parsed_values.get("Altitude")
    pressure = parsed_values.get("Pressure")
    battery = parsed_values.get("Battery")

    point = (
        Point("Experiment")
        .tag("Location", "Soest")
        .tag("Device_ID", device_id[:16])
        .field("Light", light)
        .field("Temperature", temp)
        .field("Altitude", altitude)
        .field("Pressure", pressure)
        .field("Battery", battery)
    )
    write_api.write(bucket=bucket, org="FH", record=point)

    print(f"Light: {light}")
    print(f"Temperature: {temp}")
    print(f"Altitude: {altitude}")
    print(f"Pressure: {pressure}")
    print(f"Battery: {battery}")

try:
    while True:
        xbee_message = device.read_data()

        if xbee_message is not None:
            send_data()

except KeyboardInterrupt:
    device.close()
    print("Finished")
    pass