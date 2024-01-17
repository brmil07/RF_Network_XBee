from datetime import datetime
import urllib3
import logging
from digi.xbee.devices import *
from influxdb_client import Point
from influxdb_client.client.write_api import SYNCHRONOUS
import influxdb_client


def establish_influxdb_connection():
    # Establish a connection to InfluxDB
    # Returns a tuple containing InfluxDB client and write API
    while True:
        try:
            client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
            write_api = client.write_api(write_options=SYNCHRONOUS)
            return client, write_api
        except urllib3.exceptions.NewConnectionError:
            print("InfluxDB is not available. Retrying in 5 seconds...")
            time.sleep(5)


def send_data(xbee_message, write_api):
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


def main():
    # Initialize InfluxDB configurations
    global org, url, token, bucket
    org = "FH"
    url = "http://127.0.0.1:8086"
    token = "Wfwp8kjSQpSaGsP2ZqXB7rEqmsjtGfaj-8nr6LQzIWmr8RbWcz4S_xBbB_484MTjmM0Y8F38WGIG4ve_1ubB1g=="
    bucket = "fh_bucket"

    # Establish InfluxDB connection
    client, write_api = establish_influxdb_connection()

    # Initialize XBEE Device
    device = XBeeDevice("COM14", 115200)

    # Open serial device
    device.open()

    try:
        while True:
            try:
                xbee_message = device.read_data()

                if xbee_message is not None:
                    send_data(xbee_message, write_api)

            except Exception as e:
                print(f"An error occurred while processing XBee data: {str(e)}")
                break

    except KeyboardInterrupt:
        print("Script terminated by user")
        logging.info("Script terminated by user.")

    finally:
        device.close()
        print("Script successfully terminated")
        logging.info("Script successfully terminated")


if __name__ == "__main__":
    main()