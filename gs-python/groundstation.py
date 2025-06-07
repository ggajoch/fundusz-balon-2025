#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import serial_mock
import simplekml
import matplotlib.pyplot as plt
from drawnow import *
import re

import datetime

# konfiguracja
PORT = 'COM9'
kml_file_name = "position.kml"
output_file_name = "output.csv"

# tablice na dane
coordinates = []
alt = []
all_pressure = []
time = []
hum = []
temp = []


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


ser = serial.Serial(PORT)
#ser = serial_mock.SerialMock('..\\mock\\gs-data-feeder\\test_data.csv', 0.1)

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

while True:

    # odczytaj jedną linię danych z Serial portu
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
                skml.save("position.kml")


                # dodaj aktualną wysokość i czas do tablicy z danymi
                time.append(timestamp)
                alt.append(altitude)
                all_pressure.append(pressure)
                hum.append(humidity)
                temp.append(temperature)

                drawnow(drawFigure)

            except Exception as e:
                print(e)
        else:
            print("[error] zla liczba pol")
    else:
        print("[error] pojawily sie zle znaki")
