import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

from random import seed
from random import randint
seed(1)

org = "FH"
url = "http://127.0.0.1:8086"
token = "Wfwp8kjSQpSaGsP2ZqXB7rEqmsjtGfaj-8nr6LQzIWmr8RbWcz4S_xBbB_484MTjmM0Y8F38WGIG4ve_1ubB1g=="

write_client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)

bucket = "my_bucket"

write_api = write_client.write_api(write_options=SYNCHRONOUS)

for value in range(360):
    point = (
        Point("Experiment")
        .tag("Location", "Bad Sassendorf")
        .field("Temperature", randint(0,30))
        .field("Humidity", randint(50,100))
    )
    write_api.write(bucket=bucket, org="FH", record=point)
    time.sleep(10)  # separate points by 1 second