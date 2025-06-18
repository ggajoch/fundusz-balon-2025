#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''

.\influxdb3.exe create database db_balon_b2

.\influxdb3 serve --node-id my_node_name --object-store file --data-dir ~/.influxdb3_data --without-auth

http://localhost:3000/d/0bc3a2a9-8321-46cb-83ce-f6510830d07d/falenty-2025-balon?orgId=1&from=now-30m&to=now&timezone=browser&refresh=5s


python.exe .\groundstation_grafana.py --com-port "COM3" --balon-name "b2"
'''

import serial
import serial_mock
import simplekml
import matplotlib.pyplot as plt
from drawnow import *
import re

import datetime
import argparse



# tablice na dane
coordinates = []
alt = []
all_pressure = []
time = []
hum = []
temp = []


from influxdb_client_3 import InfluxDBClient3, Point

def write_to_influxdb3(client, data: dict, database: str, host: str, token: str, org: str = None, measurement: str = "default"):
    """
    Writes a key-value dict to a measurement in InfluxDB 3.

    :param data: dict of field key-values to write
    :param database: existing InfluxDB database name
    :param host: URL to InfluxDB instance, e.g., http://localhost:8181
    :param token: auth token (empty string if auth disabled)
    :param org: your InfluxDB org (optional for self-hosted)
    :param measurement: name of the measurement to write to
    """


    point = Point(measurement)
    for key, value in data.items():
        point.field(key, value)

    # Optional: add timestamp manually if needed
    point.time(datetime.datetime.now(datetime.UTC))#  ← if using a specific timestamp

    client.write(record=point)
    print(f"✅ Wrote to '{database}' → measurement='{measurement}': {data}")




import math

def calculate_azimuth(lat1, lon1, lat2, lon2):
    """
    Calculate azimuth (bearing) from start point (lat1, lon1) to end point (lat2, lon2)
    Inputs in decimal degrees.
    Returns azimuth in degrees [0..360)
    """
    # Convert degrees to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    diff_long = math.radians(lon2 - lon1)

    x = math.sin(diff_long) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long)

    initial_bearing = math.atan2(x, y)
    # Convert from radians to degrees and normalize to 0-360
    initial_bearing_deg = (math.degrees(initial_bearing) + 360) % 360

    return initial_bearing_deg


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate great-circle distance between two points on Earth surface.
    Returns distance in meters.
    """
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

def calculate_elevation(lat1, lon1, alt1, lat2, lon2, alt2):
    """
    Calculate elevation angle from start point to target point in degrees.
    """
    horizontal_distance = haversine_distance(lat1, lon1, lat2, lon2)
    altitude_diff = alt2 - alt1

    elevation_rad = math.atan2(altitude_diff, horizontal_distance)
    elevation_deg = math.degrees(elevation_rad)

    return elevation_deg



def drawFigure(): # Create a function that makes our desired plot

    plt.subplot(2, 2, 1)
    plt.title('Altitude') # Plot the title
    plt.grid(True) # Turn the grid on
    plt.ylabel('Altitude [m]') # Set ylabels

    plt.plot(time, alt, 'r-', label='Altitude [m]') # plot the altitude

    plt.subplot(2, 2, 2)
    plt.title('Pressure')  # Plot the title
    plt.grid(True)  # Turn the grid on
    plt.ylabel('Pressure [Pa]')  # Set ylabels

    plt.plot(time, all_pressure, 'r-', label='Pressure [Pa]')  # plot the altitude

    plt.subplot(2, 2, 3)
    plt.title('Humidity')  # Plot the title
    plt.grid(True)  # Turn the grid on
    plt.ylabel('Humidity [%]')  # Set ylabels

    plt.plot(time, hum, 'r-', label='Humidity [%]')  # plot the altitude

    plt.subplot(2, 2, 4)
    plt.title('Temperature')  # Plot the title
    plt.grid(True)  # Turn the grid on
    plt.ylabel('Temperature [C]')  # Set ylabels

    plt.plot(time, temp, 'r-', label='Temperature [C]')  # plot the altitude



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process COM port and balon name.")
    parser.add_argument("--com-port", type=str, required=True, help="COM port string, e.g. COM3")
    parser.add_argument("--balon-name", type=str, required=True, help="Balon name string")
    parser.add_argument("--mock", action='store_true', help="Enable mock mode (boolean flag)")

    args = parser.parse_args()


    # konfiguracja
    PORT = args.com_port
    kml_file_name = f"position-{args.balon_name}.kml"
    output_file_name = f"output-{args.balon_name}.csv"


    

    if args.mock:
        ser = serial_mock.SerialMock('..\\mock\\gs-data-feeder\\test_data.csv', 0.01)
    else:
        ser = serial.Serial(PORT)
        print('not mock')

    # create/append output.csv + header
    with open(output_file_name, 'a') as output:
        output.write("gs_timestamp;timestamp;received_packets_qty;rssi;satellites;longitude;latitude;altitude;temperature;pressure;humidity\n")

    # kml file settings
    skml = simplekml.Kml()
    ls = skml.newlinestring(name='FlightPath')
    ls.extrude = 1
    ls.altitudemode = simplekml.AltitudeMode.relativetoground
    ls.style.linestyle.width = 5
    ls.style.linestyle.color = '7f00ffff'
    ls.style.polystyle.color = '7f00ff00'

    # clear buffer and ongoing transmition
    ser.flush()
    ser.readline()


    database=f"db_balon_{args.balon_name}"
    host="http://localhost:8181"
    token=""  # leave empty if auth is disabled
    org=None
    measurement="readings"

    client = InfluxDBClient3(
        host=host,
        token=token,
        org=org,
        database=database
    )

    while True:

        # odczytaj jedną linię danych z Serial portu
        if args.mock:
            line = ser.readline()
        else:
            line = ser.readline().decode('ascii')

        print(line)

        # weź datę i godzinę odczytania danych
        gs_timestamp = str(datetime.datetime.now())

        # wyświetl dla użytkownika dane w terminalu - to zachowamy w finalnym programie, reszte usuniemy
        line_with_timestamp = gs_timestamp + ";" + line
        print(line_with_timestamp)

        # zapisz linię do pliku tekstowego w celu backupu
        with open(output_file_name, 'a') as output:
            output.write(line_with_timestamp)


        # usuń znaki nowej linii z końca `strip`
        stripped_line = line.strip()
        print(stripped_line)

        # sprawdź czy linia zawiera tylko dobre znaki
        if re.match('^[0-9\.\-\;]*$', stripped_line):

            # podziel linię na poszczególne pola danych
            data = stripped_line.split(";")
            print(data)

            # sprawdź czy jest odpowiednia ilość pól
            if len(data) == 10:

                # wszystkie inne krzaki
                try:
                    # przekonwertuj wartości
                    timestamp =             float(data[0])/1000.0 # konwersja z milisekund na sekundy
                    received_packets_qty =  int(data[1])
                    rssi =                  int(data[2])
                    satellites =            int(data[3])
                    longitude =             float(data[4])
                    latitude =              float(data[5])
                    altitude =              float(data[6])
                    temperature =           float(data[7])/10.0 # konwersja decy stopnie Celsjusza na stopnie Celsjusza
                    pressure =              float(data[8])
                    humidity =              int(data[9])

                    # dodaj koordynaty GPS do pliku KLM dla Google Earth
                    new_coordinates_point = (longitude, latitude, altitude)
                    coordinates.append(new_coordinates_point)
                    ls.coords = coordinates
                    skml.save(kml_file_name)


                    # dodaj aktualną wysokość i czas do tablicy z danymi
                    time.append(timestamp)
                    alt.append(altitude)
                    all_pressure.append(pressure)
                    hum.append(humidity)
                    temp.append(temperature)

                    start_long = 20.921160924098206
                    start_lat = 52.13768408696847
                    start_alt = 100.0  # meters
                    #start_long = 21.797351
                    #start_lat = 50.44721
                    target_long = longitude
                    target_lat = latitude
                    target_alt = altitude


                    azimuth = calculate_azimuth(start_lat, start_long, target_lat, target_long)
                    elevation = calculate_elevation(start_lat, start_long, start_alt, target_lat, target_long, target_alt)

                    print(f"Azimuth: {azimuth:.2f}°")
                    print(f"Elevation: {elevation:.2f}°")


                    #drawnow(drawFigure)

                    data_dict = {
                        "timestamp": timestamp,
                        "received_packets_qty": received_packets_qty,
                        "rssi": rssi,
                        "satellites": satellites,
                        "longitude": longitude,
                        "latitude": latitude,
                        "altitude": altitude,
                        "temperature": temperature,
                        "pressure": pressure,
                        "humidity": humidity,
                        "azimuth" : azimuth,
                        "elevation" : elevation
                    }
                    
                    write_to_influxdb3(client,
                        data=data_dict,
                        database="balon_test",
                        host="http://localhost:8181",
                        token="",  # leave empty if auth is disabled
                        org=None,
                        measurement="readings"
                    )




                except Exception as e:
                    print(e)
            else:
                print("[error] zla liczba pol")
        else:
            print("[error] pojawily sie zle znaki")
